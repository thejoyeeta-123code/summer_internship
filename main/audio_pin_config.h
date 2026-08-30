#pragma once
#include"driver/gpio.h"
#include"es8311.h" 

// --- Audio Configuration ---
#define SAMPLE_RATE             (16000)
#define AUDIO_BUFFER_SIZE       (1024)  // Bytes
#define RECORD_TIME_SEC         (20)    // Duration to record

// --- I2C Configuration (Control Plane) ---
#define I2C_NUM                 (0)
#define I2C_SCL_IO              (GPIO_NUM_8)
#define I2C_SDA_IO              (GPIO_NUM_7)

// --- I2S Configuration (Audio Data Plane) ---
#define I2S_NUM                 (0)
#define I2S_MCK_IO              (GPIO_NUM_13)
#define I2S_BCK_IO              (GPIO_NUM_12)
#define I2S_WS_IO               (GPIO_NUM_10)
#define I2S_DO_IO               (GPIO_NUM_9)
#define I2S_DI_IO               (GPIO_NUM_11)

// --- Power Amplifier Control ---
#define GPIO_PA_ENABLE          (GPIO_NUM_53)

// --- Codec Settings ---
#define MCLK_MULTIPLE           (384)
#define MCLK_FREQ_HZ            (SAMPLE_RATE * MCLK_MULTIPLE)
#define VOICE_VOLUME            (70)
#define MIC_GAIN                (ES8311_MIC_GAIN_18DB) 

// --- SD Card (SDMMC) Configuration --- 

#define MOUNT_POINT             "/sdcard"
