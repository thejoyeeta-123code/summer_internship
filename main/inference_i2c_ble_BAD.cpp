#include <stdio.h>
#include <string.h>
#include <math.h>
#include <algorithm> // Required for std::sort
#include <sys/unistd.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <errno.h>
#include "bno085.hpp"
#include "audio_pin_config.h"
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
    #include "rf_model.h"
}

static const char *TAG = "edge_inference_ble";

// ── Pin assignments (I2C - IMU) ──────────────────────────────────────────────
#define PIN_SDA  3
#define PIN_SCL  2
#define PIN_RST  27

// --- ML Windowing Constants ---
const int LOG_INTERVAL_MS = 20;  
const int WINDOW_SIZE = 100;     
const int NUM_BASE_FEATURES = 10; // Assuming Orientation-Independent features
const int NUM_ROLLED_FEATURES = 70; // 10 * 7

// --- Global Objects & Queues ---
BNO085 *imu = nullptr; 
QueueHandle_t streamQueue;
uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Exposed from gatt_svr.c
extern "C" uint16_t gatt_svr_chr_val_handle;

typedef struct {
    char data[64];
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

static void bleprph_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);
    
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"ESP32_AI_Stream";
    fields.name_len = strlen("ESP32_AI_Stream");
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    
    ESP_LOGI(TAG, "BLE Advertising Started...");
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

static void bleprph_on_sync(void) {
    uint8_t own_addr_type; // Create a variable to hold the address type

    // Pass the memory address of the variable so NimBLE can write to it safely
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type, error code: %d", rc);
        return;
    }
    
    // Override the default advertisement with our event handler
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    // Use the dynamically inferred own_addr_type here
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, bleprph_gap_event, NULL);
}

void bleprph_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =========================================================================
// ========================== PANDAS MATH ENGINE ===========================
// =========================================================================

void compute_pandas_stats(float *data, int n, float &mean, float &stddev, float &min_val, float &max_val, float &median, float &skew, float &kurt) {
    if (n == 0) return;

    min_val = data[0]; max_val = data[0]; double sum = 0;
    for (int i = 0; i < n; i++) {
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
        sum += data[i];
    }
    mean = sum / n;

    double m2 = 0, m3 = 0, m4 = 0;
    for (int i = 0; i < n; i++) {
        double diff = data[i] - mean;
        double diff2 = diff * diff;
        m2 += diff2; m3 += diff2 * diff; m4 += diff2 * diff2;
    }

    m2 /= n; m3 /= n; m4 /= n;

    double sample_variance = (m2 * n) / (n - 1);
    stddev = sqrt(sample_variance);

    if (m2 <= 1e-8) { skew = 0; kurt = 0; } else {
        double g1 = m3 / pow(m2, 1.5);
        skew = (sqrt(n * (n - 1)) / (n - 2)) * g1; 
        double g2 = (m4 / (m2 * m2)) - 3.0;
        kurt = ((n - 1.0) / ((n - 2.0) * (n - 3.0))) * ((n + 1.0) * g2 + 6.0); 
    }

    float temp[WINDOW_SIZE];
    memcpy(temp, data, n * sizeof(float));
    std::sort(temp, temp + n);
    if (n % 2 == 0) median = (temp[n/2 - 1] + temp[n/2]) / 2.0f;
    else median = temp[n/2];
}

// =========================================================================
// ============================ RTOS TASKS =================================
// =========================================================================

// --- Core 0: BLE Stream Writer ---
void ble_stream_task(void *pvParameters) {
    LogItem_t item;
    for (;;) {
        if (xQueueReceive(streamQueue, &item, portMAX_DELAY)) {
            printf("%s", item.data); // Print to serial monitor
            
            // If a Web BLE client is connected, push the string via Notification
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

// --- Core 1: Synchronous 50Hz Sensor Poller & Inference Engine ---
void sensor_log_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOG_INTERVAL_MS);
    
    float prev_ax = 0, prev_ay = 0, prev_az = 0; 
    
    static float ax_cpy = 0, ay_cpy = 0, az_cpy = 0;
    static float gx_cpy = 0, gy_cpy = 0, gz_cpy = 0;

    static int last_audio_rms = 0;
    static long last_peak = 0;
    static unsigned long last_zcr = 0;
    static float last_crest_factor = 0.0f;

    bno085_accel_t acc {};
    bno085_gyro_t  gyr {};

    static float window_data[NUM_BASE_FEATURES][WINDOW_SIZE];
    static int window_idx = 0;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        imu->service(); 

        if (imu->read_accelerometer(acc)) {
            ax_cpy = acc.x; ay_cpy = acc.y; az_cpy = acc.z;
        }
        if (imu->read_gyroscope(gyr)) {
            gx_cpy = gyr.x; gy_cpy = gyr.y; gz_cpy = gyr.z;
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

        // Populate the 2D window buffer
        window_data[0][window_idx]  = gx_cpy;
        window_data[1][window_idx]  = gy_cpy;
        window_data[2][window_idx]  = gz_cpy;
        window_data[3][window_idx]  = accel_mag;
        window_data[4][window_idx]  = gyro_mag;
        window_data[5][window_idx]  = jerk;
        window_data[6][window_idx]  = (float)last_audio_rms;
        window_data[7][window_idx]  = (float)last_peak;
        window_data[8][window_idx]  = (float)last_zcr;
        window_data[9][window_idx]  = last_crest_factor;

        window_idx++;

        if (window_idx >= WINDOW_SIZE) {
            float rf_features[NUM_ROLLED_FEATURES];

            for (int i = 0; i < NUM_BASE_FEATURES; i++) {
                float f_mean, f_std, f_min, f_max, f_med, f_skew, f_kurt;
                compute_pandas_stats(window_data[i], WINDOW_SIZE, f_mean, f_std, f_min, f_max, f_med, f_skew, f_kurt);
                
                rf_features[0 * NUM_BASE_FEATURES + i] = f_mean;
                rf_features[1 * NUM_BASE_FEATURES + i] = f_std;
                rf_features[2 * NUM_BASE_FEATURES + i] = f_min;
                rf_features[3 * NUM_BASE_FEATURES + i] = f_max;
                rf_features[4 * NUM_BASE_FEATURES + i] = f_med;
                rf_features[5 * NUM_BASE_FEATURES + i] = f_skew;
                rf_features[6 * NUM_BASE_FEATURES + i] = f_kurt;
            }

            // Run Edge Inference
            int prediction = predict_random_forest(rf_features);

            LogItem_t item;
            snprintf(item.data, sizeof(item.data), "Prediction Class: %d\n", prediction);
            xQueueSend(streamQueue, &item, 0);

            window_idx = 0;
        }
    }
}

// =========================================================================
// ============================== APP MAIN =================================
// =========================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- Initializing Edge Inference System ---");

    // 1. Initialize NVS (Required by NimBLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize ESP-Hosted VHCI Co-Processor
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
    bno085_i2c_cfg_t cfg {};
    cfg.sda      = PIN_SDA; 
    cfg.scl      = PIN_SCL; 
    cfg.rst      = PIN_RST; 
    cfg.i2c_port = 1;       
    cfg.clk_hz   = 400'000;
    cfg.addr     = 0x4A;

    imu = new BNO085(cfg);

    if (imu->init() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize BNO085 via I2C!");
        return;
    }
    
    ESP_LOGI(TAG, "Draining BNO085 startup buffer...");
    for (int i = 0; i < 30; i++) {
        imu->service();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "✅ BNO085 initialized on I2C Port 1.");

    imu->enable_accelerometer(20000);
    imu->enable_gyroscope(20000);

    // 6. Launch RTOS Tasks
    streamQueue = xQueueCreate(10, sizeof(LogItem_t)); 
    
    xTaskCreatePinnedToCore(ble_stream_task, "ble_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(audio_processing_task, "audio_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(sensor_log_task, "sensor_task", 8192, NULL, 15, NULL, 1);

    ESP_LOGI(TAG, "✅ System running. Waiting for Web BLE connection...");
}