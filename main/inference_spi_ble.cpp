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
    #include "rf_model.h" // Edge Inference Model
}

static const char *TAG = "edge_inference_ble";

// ── Pin assignments (SPI - IMU - Left Row) ───────────────────────────────────
#define PIN_CS   GPIO_NUM_28
#define PIN_MOSI GPIO_NUM_29 // DI on IMU (Master Out, Slave In)
#define PIN_MISO GPIO_NUM_30 // SDA on IMU (Master In, Slave Out)
#define PIN_SCK  GPIO_NUM_31 // SCL on IMU
#define PIN_INT  GPIO_NUM_51
#define PIN_RST  GPIO_NUM_52

const uint32_t LOG_INTERVAL_US = 20000; // 20ms = 50Hz Logging

// --- ML Windowing Constants ---
const int WINDOW_SIZE = 100;     
const int NUM_BASE_FEATURES = 17;   // Orientation-dependent (Accel, Gyro, Mag, Jerk, Quats, Audio)
const int NUM_ROLLED_FEATURES = 119; // 17 * 7

// --- Global Objects & Queues ---
BNO08x *imu = nullptr; 
QueueHandle_t streamQueue;
uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Exposed from gatt_svr.c
extern "C" uint16_t gatt_svr_chr_val_handle;

typedef struct {
    char data[64]; // Shrunk from 256, only sending prediction string
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
    const char *name = "ESP32_AI_Stream"; // Renamed for AI Stream
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

// --- Core 1: Synchronous Sensor Poller & Inference Engine ---

void sensor_log_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor & ML Task Started");
    
    float prev_ax = 0, prev_ay = 0, prev_az = 0; 
    float ax_cpy = 0, ay_cpy = 0, az_cpy = 0;
    float gx_cpy = 0, gy_cpy = 0, gz_cpy = 0;
    float qi_cpy = 0, qj_cpy = 0, qk_cpy = 0, qr_cpy = 1;

    int last_audio_rms = 0;
    long last_peak = 0;
    unsigned long last_zcr = 0;
    float last_crest_factor = 0.0f;
    bool first_run = true; 

    // ML Windowing Buffers
    static float window_data[NUM_BASE_FEATURES][WINDOW_SIZE];
    static int window_idx = 0;

    // FreeRTOS Strict Timing Variables
    const TickType_t xFrequency = pdMS_TO_TICKS(LOG_INTERVAL_US / 1000); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
            
        // FIX 1: Fetch cached data directly. Do not guard with has_new_data().
        // (Matches the working data-streamer: always read, no has_new_data() gate,
        // since that gate was starving the inference path of fresh samples.)
        bno08x_accel_t acc = imu->rpt.accelerometer.get();
        ax_cpy = acc.x; ay_cpy = acc.y; az_cpy = acc.z;
        
        bno08x_gyro_t gyr = imu->rpt.cal_gyro.get();
        gx_cpy = gyr.x; gy_cpy = gyr.y; gz_cpy = gyr.z;
        
        bno08x_quat_t quat = imu->rpt.rv_game.get_quat();
        qi_cpy = quat.i; qj_cpy = quat.j; qk_cpy = quat.k; qr_cpy = quat.real; 

        // Sync and process audio snapshot
        double sum_sq_copy = 0; uint32_t count_copy = 0; int32_t peak_copy = 0; uint32_t zcr_copy = 0;
        
        portENTER_CRITICAL(&audio_mux);
        sum_sq_copy = audio_metrics.sum_sq; count_copy = audio_metrics.sample_count;
        peak_copy = audio_metrics.peak; zcr_copy = audio_metrics.zcr;
        
        audio_metrics.sum_sq = 0; audio_metrics.sample_count = 0; 
        audio_metrics.peak = 0; audio_metrics.zcr = 0;
        portEXIT_CRITICAL(&audio_mux);

        // FIX 2: Explicitly clear audio values if the buffer was empty,
        // instead of silently repeating stale values from a previous tick.
        if (count_copy > 0) {
            last_audio_rms = sqrt(sum_sq_copy / count_copy);
            last_peak = peak_copy;
            last_zcr = zcr_copy;
            last_crest_factor = (last_audio_rms > 0) ? ((float)last_peak / last_audio_rms) : 0;
        } else {
            last_audio_rms = 0;
            last_peak = 0;
            last_zcr = 0;
            last_crest_factor = 0.0f;
        }

        // Derived Math Features 
        float accel_mag = sqrt((ax_cpy * ax_cpy) + (ay_cpy * ay_cpy) + (az_cpy * az_cpy));
        float gyro_mag  = sqrt((gx_cpy * gx_cpy) + (gy_cpy * gy_cpy) + (gz_cpy * gz_cpy));
        
        float dt_s = (float)LOG_INTERVAL_US / 1000000.0f; 
        float jerk_x = 0, jerk_y = 0, jerk_z = 0, jerk = 0;

        // FIX 3: Skip jerk calculation on the very first loop iteration, since
        // prev_ax/ay/az start at 0 and would otherwise produce one fake spike
        // at startup (diff against zero, not against a real previous sample).
        if (!first_run) {
            jerk_x = (ax_cpy - prev_ax) / dt_s;
            jerk_y = (ay_cpy - prev_ay) / dt_s;
            jerk_z = (az_cpy - prev_az) / dt_s;
            jerk = sqrt((jerk_x * jerk_x) + (jerk_y * jerk_y) + (jerk_z * jerk_z));
        } else {
            first_run = false;
        }
        
        prev_ax = ax_cpy; prev_ay = ay_cpy; prev_az = az_cpy;

        // Populate the 2D window buffer in the EXACT order your RF model expects (17 features)
        window_data[0][window_idx]  = ax_cpy;
        window_data[1][window_idx]  = ay_cpy;
        window_data[2][window_idx]  = az_cpy;
        window_data[3][window_idx]  = gx_cpy;
        window_data[4][window_idx]  = gy_cpy;
        window_data[5][window_idx]  = gz_cpy;
        window_data[6][window_idx]  = accel_mag;
        window_data[7][window_idx]  = gyro_mag;
        window_data[8][window_idx]  = jerk;
        window_data[9][window_idx]  = qi_cpy;
        window_data[10][window_idx] = qj_cpy;
        window_data[11][window_idx] = qk_cpy;
        window_data[12][window_idx] = qr_cpy;
        window_data[13][window_idx] = (float)last_audio_rms;
        window_data[14][window_idx] = (float)last_peak;
        window_data[15][window_idx] = (float)last_zcr;
        window_data[16][window_idx] = last_crest_factor;

        window_idx++;

        if (window_idx >= WINDOW_SIZE) {
            float rf_features[NUM_ROLLED_FEATURES];

            // Roll the stats (Creates blocks of means, then stds, then mins, etc.)
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

            // Blast prediction to Queue
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
    
    ESP_LOGI(TAG, "✅ System running. Streaming AI Inference at 0.5Hz.");
}