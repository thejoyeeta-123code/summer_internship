#pragma once
#include "esp_err.h"
#include "driver/i2s_std.h"

// Initialize I2C, Codec, and I2S
esp_err_t audio_driver_init(void);

// Get Handles
i2s_chan_handle_t audio_driver_get_rx_handle(void);
i2s_chan_handle_t audio_driver_get_tx_handle(void);
