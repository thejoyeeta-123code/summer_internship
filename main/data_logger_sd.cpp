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
#include <errno.h>
#include "bno085.hpp"
#include "audio_pin_config.h"

extern "C" {
    #include "audio_driver.h"
    #include "microphone.h"
    #include "sdcard.h"
}

static const char *TAG = "data_logger";

// ── Pin assignments (I2C - DEDICATED BUS) ────────────────────────────────────
#define PIN_SDA  3
#define PIN_SCL  2
#define PIN_RST  27

// --- Metadata Variables ---
const char* SUBJECT_ID = "4";
const char* CLASS_ID   = "7";
const char* ACTIVITY   = "THROW";
const char* EMOTION    = "FRUSTRATED";
const int LOG_INTERVAL_MS = 20; // 50Hz Logging

char logFileName[64];
FILE *dataFile = NULL;

// --- Global Objects & Queues ---
BNO085 *imu = nullptr; 
QueueHandle_t sdQueue;

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

// --- Core 0: SD Card Writer ---
void sd_write_task(void *pvParameters) {
    LogItem_t item;
    uint8_t flush_counter = 0;
    
    for (;;) {
        if (xQueueReceive(sdQueue, &item, portMAX_DELAY)) {
            printf("%s\n", item.data);

            if (dataFile) {
                fprintf(dataFile, "%s\n", item.data);
                flush_counter++;
                    
                if (flush_counter >= 50) {  
                    fflush(dataFile); 
                    fsync(fileno(dataFile));
                    flush_counter = 0;
                }
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

// --- Core 1: Synchronous 50Hz Sensor Poller & Logger ---
void sensor_log_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOG_INTERVAL_MS);
    
    float prev_ax = 0, prev_ay = 0, prev_az = 0; 
    
    // IMU Static Cache
    static float ax_cpy = 0, ay_cpy = 0, az_cpy = 0;
    static float gx_cpy = 0, gy_cpy = 0, gz_cpy = 0;
    static float qi_cpy = 0, qj_cpy = 0, qk_cpy = 0, qr_cpy = 1;

    // AUDIO Static Cache
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

        imu->service(); // Poll the BNO085 for new data

        // 2. Fetch the latest cached data from the wrapper
        if (imu->read_accelerometer(acc)) {
            ax_cpy = acc.x; ay_cpy = acc.y; az_cpy = acc.z;
        }
        if (imu->read_gyroscope(gyr)) {
            gx_cpy = gyr.x; gy_cpy = gyr.y; gz_cpy = gyr.z;
        }
        if (imu->read_quaternion(quat)) {
            qi_cpy = quat.i; qj_cpy = quat.j; qk_cpy = quat.k; qr_cpy = quat.real;
        }

        // 3. Safely grab and reset the Audio accumulator variables
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

        // IMU Magnitude and Jerk calculations
        float accel_mag = sqrt((ax_cpy * ax_cpy) + (ay_cpy * ay_cpy) + (az_cpy * az_cpy));
        float gyro_mag  = sqrt((gx_cpy * gx_cpy) + (gy_cpy * gy_cpy) + (gz_cpy * gz_cpy));
        
        float dt_s = LOG_INTERVAL_MS / 1000.0f;
        float jerk_x = (ax_cpy - prev_ax) / dt_s;
        float jerk_y = (ay_cpy - prev_ay) / dt_s;
        float jerk_z = (az_cpy - prev_az) / dt_s;
        float jerk = sqrt((jerk_x * jerk_x) + (jerk_y * jerk_y) + (jerk_z * jerk_z));
        
        prev_ax = ax_cpy; prev_ay = ay_cpy; prev_az = az_cpy;

        // 4. String Formatting & Queuing
        LogItem_t item;
        snprintf(item.data, sizeof(item.data),
                 "%lu,%s,%s,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%ld,%lu,%.2f",
                 current_time_ms, SUBJECT_ID, CLASS_ID, ACTIVITY, EMOTION,
                 ax_cpy, ay_cpy, az_cpy, gx_cpy, gy_cpy, gz_cpy, accel_mag, gyro_mag, jerk, qi_cpy, qj_cpy, qk_cpy, qr_cpy,
                 last_audio_rms, last_peak, last_zcr, last_crest_factor);

        xQueueSend(sdQueue, &item, 0);
    }
}

// =========================================================================
// ============================== APP MAIN =================================
// =========================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- Initializing System ---");
    snprintf(logFileName, sizeof(logFileName), MOUNT_POINT "/%s_%s.csv", SUBJECT_ID, ACTIVITY);
    ESP_LOGI(TAG, "Attempting to create log file at: %s", logFileName);

    if (sd_card_mount() != ESP_OK) return;
    
    // Audio Driver initializes on its own I2C port (Port 0)
    if (audio_driver_init() != ESP_OK) return;

    // 1. Initialize BNO085 on a completely separate I2C Port (Port 1)
    bno085_i2c_cfg_t cfg {};
    cfg.sda      = PIN_SDA; // GPIO 2
    cfg.scl      = PIN_SCL; // GPIO 3
    cfg.rst      = PIN_RST; // GPIO 27
    cfg.i2c_port = 1;       // <--- CRITICAL: DEDICATED I2C HARDWARE!
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

    // 2. Enable Background Reports (20,000 us = 50Hz)
    imu->enable_accelerometer(20000);
    imu->enable_gyroscope(20000);
    imu->enable_game_rotation_vector(20000);

    // 3. Create and Prepare CSV File
    dataFile = fopen(logFileName, "w");
    if (!dataFile) {
        ESP_LOGE(TAG, "❌ Failed to create CSV file: %s (errno=%d: %s)",
                logFileName, errno, strerror(errno));
        return;
    }

    fprintf(dataFile, "timestamp_ms,subject_id,class_id,activity,emotion,ax,ay,az,gx,gy,gz,accel_mag,gyro_mag,jerk,q_i,q_j,q_k,q_r,audio_rms,audio_peak,audio_zcr,crest_factor\n");
    fflush(dataFile);

    // 4. Create Queue and Launch Tasks
    sdQueue = xQueueCreate(150, sizeof(LogItem_t));
    
    xTaskCreatePinnedToCore(sd_write_task, "sd_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(audio_processing_task, "audio_task", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(sensor_log_task, "sensor_task", 4096, NULL, 15, NULL, 1);

    ESP_LOGI(TAG, "✅ System running. Logging Unified Data at 50Hz.");
}