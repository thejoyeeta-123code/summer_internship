#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" // Added for SemaphoreHandle_t
#include "driver/gpio.h"
#include <errno.h>
#include "bno085.hpp"
#include "audio_pin_config.h"

// --- NimBLE & ESP-Hosted Headers ---
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "bleprph.h"
#include "esp_hosted.h"

extern "C" {
    #include "audio_driver.h"
    #include "microphone.h"
}

static const char *TAG = "data_streamer_ble";

// ── Pin assignments (I2C - IMU) ──────────────────────────────────────────────
#define PIN_INT  4
#define PIN_SDA  3
#define PIN_SCL  2
#define PIN_RST  27

// --- Metadata Variables ---
const char* SUBJECT_ID = "4";
const char* CLASS_ID   = "7";
const char* ACTIVITY   = "THROW";
const char* EMOTION    = "FRUSTRATED";
const int LOG_INTERVAL_MS = 20; // 50Hz Logging

// --- Global Objects & Queues ---
BNO085 *imu = nullptr; 
QueueHandle_t streamQueue;
uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Exposed from gatt_svr.c
extern "C" uint16_t gatt_svr_chr_val_handle;

typedef struct {
    char data[256];
} LogItem_t;

// --- Audio Feature Cache ---
typedef struct {
    double sum_sq;
    uint32_t sample_count;
    int32_t peak;
    uint32_t zcr;
    int16_t last_val;
} AudioMetrics_t;

AudioMetrics_t audio_metrics = {0, 0, 0, 0, 0};
portMUX_TYPE audio_mux = portMUX_INITIALIZER_UNLOCKED;

// --- IMU Interrupt Setup ---
SemaphoreHandle_t imu_int_sem = NULL;


static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

static void IRAM_ATTR imu_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(imu_int_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// =========================================================================
// ========================== NIMBLE BLE SETUP =============================
// =========================================================================

static void bleprph_advertise(void)
{
    uint8_t own_addr_type;

    ble_hs_id_infer_auto(0, &own_addr_type);

    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = "ESP32_Data_Stream";

    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);

    ESP_LOGI(TAG, "adv_set_fields rc=%d", rc);

    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
            own_addr_type,
            NULL,
            BLE_HS_FOREVER,
            &adv_params,
            bleprph_gap_event,
            NULL);

    ESP_LOGI(TAG, "adv_start rc=%d", rc);
}

static int bleprph_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "✅ BLE Client Connected!");
            } else {
                bleprph_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "❌ BLE Client Disconnected. Re-advertising...");
            ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            bleprph_advertise();
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "📲 BLE Client Subscribed to Notifications!");
            break;
    }
    return 0;
}

static void bleprph_on_sync(void) 
{
    bleprph_advertise();
}

void bleprph_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =========================================================================
// ============================ RTOS TASKS =================================
// =========================================================================

// --- Core 0: BLE Stream Writer ---
void ble_stream_task(void *pvParameters) {
    ESP_LOGI(TAG, "BLE Stream Task Started");
    LogItem_t item;
    for (;;) {
        if (xQueueReceive(streamQueue, &item, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Queue received");

            ESP_LOGI(TAG, "Connection Handle = %d", ble_conn_handle);

            // Push CSV string via BLE Notification
            if (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGI(TAG, "Sending notification");
                struct os_mbuf *om = ble_hs_mbuf_from_flat(item.data, strlen(item.data));
                int rc = ble_gatts_notify_custom(ble_conn_handle, gatt_svr_chr_val_handle, om);

                if (rc != 0)
                {
                    ESP_LOGE(TAG, "Notify failed rc=%d", rc);
                }
            }
        }
    }
}

// --- Core 1: Continuous Audio Processing ---
void audio_processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "Audio Task Started");
    int16_t *buf = (int16_t *)malloc(AUDIO_BUFFER_SIZE);
    if (!buf) { ESP_LOGE(TAG, "Audio buffer allocation failed!"); vTaskDelete(NULL); }

    for (;;) {
        size_t bytes_read = microphone_read(buf, AUDIO_BUFFER_SIZE);
        if (bytes_read > 0) {
            int num_samples = bytes_read / sizeof(int16_t);
            
            portENTER_CRITICAL(&audio_mux);
            for (int i = 0; i < num_samples; i += 2) { 
                int16_t val = buf[i]; 
                
                audio_metrics.sum_sq += ((double)val * val);
                if (abs(val) > audio_metrics.peak) audio_metrics.peak = abs(val);
                
                if ((val >= 0 && audio_metrics.last_val < 0) || (val < 0 && audio_metrics.last_val >= 0)) {
                    audio_metrics.zcr++;
                }
                
                audio_metrics.last_val = val;
                audio_metrics.sample_count++;
            }
            portEXIT_CRITICAL(&audio_mux);
        }
    }
}

// --- Core 1: Synchronous Sensor Poller (Interrupt-Driven) ---
void sensor_log_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started");
    float prev_ax = 0, prev_ay = 0, prev_az = 0; 
    
    static float ax_cpy = 0, ay_cpy = 0, az_cpy = 0;
    static float gx_cpy = 0, gy_cpy = 0, gz_cpy = 0;
    static float qi_cpy = 0, qj_cpy = 0, qk_cpy = 0, qr_cpy = 1;

    static int last_audio_rms = 0;
    static long last_peak = 0;
    static unsigned long last_zcr = 0;
    static float last_crest_factor = 0.0f;

    bno085_accel_t acc {};
    bno085_gyro_t  gyr {};
    bno085_quat_t  quat {};

    for (;;) {
        // Wait up to 100ms for the interrupt edge
        BaseType_t int_triggered = xSemaphoreTake(imu_int_sem, pdMS_TO_TICKS(100));
        //ESP_LOGI(TAG, "Semaphore=%d  INT=%d", int_triggered, gpio_get_level((gpio_num_t)PIN_INT));

        // IF the interrupt fired, OR if we timed out but the pin is physically stuck LOW:
        if (int_triggered == pdTRUE || gpio_get_level((gpio_num_t)PIN_INT) == 0) {
            
            unsigned long current_time_ms = (unsigned long)(esp_timer_get_time() / 1000);

            // This is the magic key: reading the IMU clears the buffer 
            // and allows the BNO085 to pull the INT pin back HIGH.
            imu->service(); 

            if (imu->read_accelerometer(acc)) {
                ax_cpy = acc.x; ay_cpy = acc.y; az_cpy = acc.z;
            }
            if (imu->read_gyroscope(gyr)) {
                gx_cpy = gyr.x; gy_cpy = gyr.y; gz_cpy = gyr.z;
            }
            if (imu->read_quaternion(quat)) {
                qi_cpy = quat.i; qj_cpy = quat.j; qk_cpy = quat.k; qr_cpy = quat.real;
            }

            double sum_sq_copy = 0; uint32_t count_copy = 0; int32_t peak_copy = 0; uint32_t zcr_copy = 0;
            
            portENTER_CRITICAL(&audio_mux);
            sum_sq_copy = audio_metrics.sum_sq; count_copy = audio_metrics.sample_count;
            peak_copy = audio_metrics.peak; zcr_copy = audio_metrics.zcr;
            
            audio_metrics.sum_sq = 0; audio_metrics.sample_count = 0; 
            audio_metrics.peak = 0; audio_metrics.zcr = 0;
            portEXIT_CRITICAL(&audio_mux);

            if (count_copy > 0) {
                last_audio_rms = sqrt(sum_sq_copy / count_copy);
                last_peak = peak_copy;
                last_zcr = zcr_copy;
                last_crest_factor = (last_audio_rms > 0) ? ((float)last_peak / last_audio_rms) : 0;
            }

            float accel_mag = sqrt((ax_cpy * ax_cpy) + (ay_cpy * ay_cpy) + (az_cpy * az_cpy));
            float gyro_mag  = sqrt((gx_cpy * gx_cpy) + (gy_cpy * gy_cpy) + (gz_cpy * gz_cpy));
            
            float dt_s = LOG_INTERVAL_MS / 1000.0f;
            float jerk_x = (ax_cpy - prev_ax) / dt_s;
            float jerk_y = (ay_cpy - prev_ay) / dt_s;
            float jerk_z = (az_cpy - prev_az) / dt_s;
            float jerk = sqrt((jerk_x * jerk_x) + (jerk_y * jerk_y) + (jerk_z * jerk_z));
            
            prev_ax = ax_cpy; prev_ay = ay_cpy; prev_az = az_cpy;

            LogItem_t item;
            snprintf(item.data, sizeof(item.data),
                     "%lu,%s,%s,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%ld,%lu,%.2f\r\n",
                     current_time_ms, SUBJECT_ID, CLASS_ID, ACTIVITY, EMOTION,
                     ax_cpy, ay_cpy, az_cpy, gx_cpy, gy_cpy, gz_cpy, accel_mag, gyro_mag, jerk, qi_cpy, qj_cpy, qk_cpy, qr_cpy,
                     last_audio_rms, last_peak, last_zcr, last_crest_factor);

            ESP_LOGI(TAG, "Sending packet");
            // Send to BLE task
            xQueueSend(streamQueue, &item, 0);
        }
    }
}

// =========================================================================
// ============================== APP MAIN =================================
// =========================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- Initializing System ---");

    // 1. Initialize NVS (Required by NimBLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize ESP-Hosted VHCI Co-Processor (ESP32P4 requirement)
    ret = esp_hosted_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to init ESP Hosted");
        return;
    }

    // 3. Initialize NimBLE
    nimble_port_init();
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    gatt_svr_init();
    nimble_port_freertos_init(bleprph_host_task);

    // 4. Initialize Audio Driver
    if (audio_driver_init() != ESP_OK) return;

    // 5. Initialize BNO085 
    ESP_LOGW(TAG, "Forcing BNO085 Hardware Reset to clear I2C bus...");
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&rst_conf);
    gpio_set_level((gpio_num_t)PIN_RST, 0); // Pull LOW to hard reset
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)PIN_RST, 1); // Pull HIGH to wake up
    vTaskDelay(pdMS_TO_TICKS(200));         // Wait for sensor to fully boot

    bno085_i2c_cfg_t cfg {};
    cfg.sda      = PIN_SDA; 
    cfg.scl      = PIN_SCL; 
    cfg.rst      = PIN_RST; 
    cfg.i2c_port = 1;       
    cfg.clk_hz   = 100000;  // Dropped to 100kHz for maximum stability
    cfg.addr     = 0x4A;

    imu = new BNO085(cfg);

    if (imu->init() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize BNO085 via I2C! Check wiring.");
        return;
    }

    ESP_LOGI(TAG, "Draining BNO085 startup buffer...");
    for (int i = 0; i < 30; i++) {
        imu->service();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "✅ BNO085 initialized on I2C bus.");

    // Enable sensors AFTER the buffer is fully drained
    imu->enable_accelerometer(20000);
    imu->enable_gyroscope(20000);
    imu->enable_game_rotation_vector(20000);

    // 6. Configure Interrupts (Semaphore & GPIO)
    imu_int_sem = xSemaphoreCreateBinary();
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // BNO085 asserts LOW when data is ready
    };
    gpio_config(&io_conf);
    
    // Install ISR service (ignoring error if already installed by another driver)
    gpio_install_isr_service(0); 
    gpio_isr_handler_add((gpio_num_t)PIN_INT, imu_isr_handler, NULL);

    // Clear any pending triggers from startup before launching the task
    xSemaphoreTake(imu_int_sem, 0); 

    // 7. Create Queue and Launch Tasks
    streamQueue = xQueueCreate(150, sizeof(LogItem_t));
    
    BaseType_t rc;

    rc = xTaskCreatePinnedToCore(ble_stream_task, "ble_task", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG,"BLE task rc=%d", rc);

    rc = xTaskCreatePinnedToCore(audio_processing_task, "audio_task", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG,"Audio task rc=%d", rc);

    rc = xTaskCreatePinnedToCore(sensor_log_task, "sensor_task", 8192, NULL, 15, NULL, 1);
    ESP_LOGI(TAG,"Sensor task rc=%d", rc);
    
    ESP_LOGI(TAG, "✅ System running. Streaming Unified Data at 50Hz via Native BLE Module.");
}