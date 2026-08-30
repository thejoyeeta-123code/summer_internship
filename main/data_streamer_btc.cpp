#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <errno.h>
#include "bno085.hpp"
#include "audio_pin_config.h"

extern "C" {
    #include "audio_driver.h"
    #include "microphone.h"
}

static const char *TAG = "data_streamer";

// ── Pin assignments (I2C - IMU) ──────────────────────────────────────────────
#define PIN_SDA  3
#define PIN_SCL  2
#define PIN_RST  27

// ── Pin assignments (UART - External BT Module) ──────────────────────────────
#define BT_UART_NUM      UART_NUM_1
#define BT_UART_TX_PIN   4  // Connect to Module RX
#define BT_UART_RX_PIN   5  // Connect to Module TX
#define BT_UART_BAUD     115200

// --- Metadata Variables ---
const char* SUBJECT_ID = "4";
const char* CLASS_ID   = "7";
const char* ACTIVITY   = "THROW";
const char* EMOTION    = "FRUSTRATED";
const int LOG_INTERVAL_MS = 20; // 50Hz Logging

// --- Global Objects & Queues ---
BNO085 *imu = nullptr; 
QueueHandle_t streamQueue;

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
// ============================ RTOS TASKS =================================
// =========================================================================

// --- Core 0: UART BT Stream Writer ---
void bt_stream_task(void *pvParameters) {
    LogItem_t item;
    for (;;) {
        if (xQueueReceive(streamQueue, &item, portMAX_DELAY)) {
            // Blast the CSV string out of the UART port to the BT Module
            uart_write_bytes(BT_UART_NUM, item.data, strlen(item.data));
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

// --- Core 1: Synchronous 50Hz Sensor Poller & Logger ---
void sensor_log_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOG_INTERVAL_MS);
    
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
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned long current_time_ms = (unsigned long)(esp_timer_get_time() / 1000);

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

        xQueueSend(streamQueue, &item, 0);
    }
}

// =========================================================================
// ============================== APP MAIN =================================
// =========================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- Initializing System ---");

    // 1. Initialize UART for BT Module
    uart_config_t uart_config = {
        .baud_rate = BT_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(BT_UART_NUM, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BT_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BT_UART_NUM, BT_UART_TX_PIN, BT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "✅ UART configured for BT Module on TX:%d RX:%d", BT_UART_TX_PIN, BT_UART_RX_PIN);

    // 2. Initialize Audio Driver
    if (audio_driver_init() != ESP_OK) return;

    // 3. Initialize BNO085 
    bno085_i2c_cfg_t cfg {};
    cfg.sda      = PIN_SDA; 
    cfg.scl      = PIN_SCL; 
    cfg.rst      = PIN_RST; 
    cfg.i2c_port = 1;       
    cfg.clk_hz   = 400'000;
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

    imu->enable_accelerometer(20000);
    imu->enable_gyroscope(20000);
    imu->enable_game_rotation_vector(20000);

    // 4. Create Queue and Launch Tasks
    streamQueue = xQueueCreate(150, sizeof(LogItem_t));
    
    xTaskCreatePinnedToCore(bt_stream_task, "bt_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(audio_processing_task, "audio_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(sensor_log_task, "sensor_task", 4096, NULL, 15, NULL, 1);

    ESP_LOGI(TAG, "✅ System running. Streaming Unified Data at 50Hz via UART BT module.");
}