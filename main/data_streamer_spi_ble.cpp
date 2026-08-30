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
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <errno.h>

// --- Myles Parfeniuk's Library ---
#include "BNO08x.hpp" 

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

// ── Pin assignments (SPI - IMU - Left Row) ───────────────────────────────────
#define PIN_CS   GPIO_NUM_28
#define PIN_MOSI GPIO_NUM_29 // DI on IMU (Master Out, Slave In)
#define PIN_MISO GPIO_NUM_30 // SDA on IMU (Master In, Slave Out)
#define PIN_SCK  GPIO_NUM_31 // SCL on IMU
#define PIN_INT  GPIO_NUM_51
#define PIN_RST  GPIO_NUM_52

const uint32_t LOG_INTERVAL_US = 20000; // 20ms = 50Hz Logging

// --- Global Objects & Queues ---
BNO08x *imu = nullptr; 
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


// =========================================================================
// ========================== NIMBLE BLE SETUP =============================
// =========================================================================

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

static void bleprph_advertise(void)
{
    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = "ESP32_Data_Stream";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                      &adv_params, bleprph_gap_event, NULL);
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
    }
    return 0;
}

static void bleprph_on_sync(void) 
{
    bleprph_advertise();
}

void bleprph_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =========================================================================
// ============================ RTOS TASKS =================================
// =========================================================================

// --- Core 0: BLE Stream Writer ---
void ble_stream_task(void *pvParameters) {
    LogItem_t item;
    for (;;) {
        if (xQueueReceive(streamQueue, &item, portMAX_DELAY)) {
            if (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                struct os_mbuf *om = ble_hs_mbuf_from_flat(item.data, strlen(item.data));
                ble_gatts_notify_custom(ble_conn_handle, gatt_svr_chr_val_handle, om);
            }
        }
    }
}

// --- Core 1: Continuous Audio Processing ---
void audio_processing_task(void *pvParameters) {
    int16_t *buf = (int16_t *)malloc(AUDIO_BUFFER_SIZE);
    if (!buf) { ESP_LOGE(TAG, "Audio buffer alloc failed!"); vTaskDelete(NULL); }

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

// --- Core 1: Synchronous Sensor Poller (Strict 50Hz FreeRTOS Timer) ---
void sensor_log_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started");
    
    float prev_ax = 0, prev_ay = 0, prev_az = 0; 
    float ax_cpy = 0, ay_cpy = 0, az_cpy = 0;
    float gx_cpy = 0, gy_cpy = 0, gz_cpy = 0;
    float qi_cpy = 0, qj_cpy = 0, qk_cpy = 0, qr_cpy = 1;

    int last_audio_rms = 0;
    long last_peak = 0;
    unsigned long last_zcr = 0;
    float last_crest_factor = 0.0f;

    // --- NEW: FreeRTOS Strict Timing Variables ---
    const TickType_t xFrequency = pdMS_TO_TICKS(LOG_INTERVAL_US / 1000); // Exactly 20ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 1. Block this task precisely until 20ms have passed since the last unblock.
        // This yields the CPU so the BNO085's background tasks can process incoming SPI packets.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
        unsigned long current_time_ms = (unsigned long)(esp_timer_get_time() / 1000);

        // 2. Fetch the latest cached data from the background SPI tasks
        // We ignore data_available() entirely. The background tasks update these structs automatically.
        bno08x_accel_t acc = imu->rpt.accelerometer.get();
        ax_cpy = acc.x; ay_cpy = acc.y; az_cpy = acc.z;
        
        bno08x_gyro_t gyr = imu->rpt.cal_gyro.get();
        gx_cpy = gyr.x; gy_cpy = gyr.y; gz_cpy = gyr.z;
        
        bno08x_quat_t quat = imu->rpt.rv_game.get_quat();
        qi_cpy = quat.i; qj_cpy = quat.j; qk_cpy = quat.k; qr_cpy = quat.real; 

        // 3. Sync and process audio snapshot
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

        // 4. Math 
        float accel_mag = sqrt((ax_cpy * ax_cpy) + (ay_cpy * ay_cpy) + (az_cpy * az_cpy));
        float gyro_mag  = sqrt((gx_cpy * gx_cpy) + (gy_cpy * gy_cpy) + (gz_cpy * gz_cpy));
        
        // Because we are using vTaskDelayUntil, dt_s is a flawless 0.02 every loop
        float dt_s = (float)LOG_INTERVAL_US / 1000000.0f; 
        float jerk_x = (ax_cpy - prev_ax) / dt_s;
        float jerk_y = (ay_cpy - prev_ay) / dt_s;
        float jerk_z = (az_cpy - prev_az) / dt_s;
        float jerk = sqrt((jerk_x * jerk_x) + (jerk_y * jerk_y) + (jerk_z * jerk_z));
        
        prev_ax = ax_cpy; prev_ay = ay_cpy; prev_az = az_cpy;

        // 5. Log
        LogItem_t item;
        snprintf(item.data, sizeof(item.data),
                    "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%ld,%lu,%.2f\r\n",
                    current_time_ms, ax_cpy, ay_cpy, az_cpy, gx_cpy, gy_cpy, gz_cpy, accel_mag, gyro_mag,
					jerk, qi_cpy, qj_cpy, qk_cpy, qr_cpy, last_audio_rms, last_peak, last_zcr, last_crest_factor);

        xQueueSend(streamQueue, &item, 0);
    }
}

// =========================================================================
// ============================== APP MAIN =================================
// =========================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- Initializing System ---");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = esp_hosted_init();
    if (ret != ESP_OK) return;

    nimble_port_init();
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    gatt_svr_init();
    nimble_port_freertos_init(bleprph_host_task);

    if (audio_driver_init() != ESP_OK) return;

    // 5. Initialize BNO08x using Myles' Library API
    bno08x_config_t imu_config = {};
    
    // Pass in our left-row custom pinout
    imu_config.io_mosi = PIN_MOSI; 
    imu_config.io_miso = PIN_MISO; 
    imu_config.io_sclk = PIN_SCK; 
    imu_config.io_cs   = PIN_CS; 
    imu_config.io_int  = PIN_INT; 
    imu_config.io_rst  = PIN_RST; 

    // --- BULLETPROOFING CONFIGURATIONS ---
    imu_config.sclk_speed = 1000000;               // Drop SPI speed to 1MHz
    gpio_set_pull_mode(PIN_INT, GPIO_PULLUP_ONLY); // Prevent floating INT pin
    // -------------------------------------

    // Instantiate and initialize the IMU object
    imu = new BNO08x(imu_config);

    if (!imu->initialize()) {
        ESP_LOGE(TAG, "❌ Failed to initialize BNO085! Check wiring and P0/P1 pins.");
        return;
    }

    ESP_LOGI(TAG, "✅ BNO085 initialized via esp32_BNO08x.");

    // 6. Enable desired reports at 50Hz (20,000 microseconds)
    imu->rpt.accelerometer.enable(LOG_INTERVAL_US);
    imu->rpt.cal_gyro.enable(LOG_INTERVAL_US);
    imu->rpt.rv_game.enable(LOG_INTERVAL_US);

    // 7. Create Queue and Launch Tasks
    streamQueue = xQueueCreate(150, sizeof(LogItem_t));
    
    xTaskCreatePinnedToCore(ble_stream_task, "ble_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(audio_processing_task, "audio_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(sensor_log_task, "sensor_task", 8192, NULL, 15, NULL, 1);
    
    ESP_LOGI(TAG, "✅ System running. Streaming Unified Data at 50Hz.");
}