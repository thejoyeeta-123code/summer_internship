#include "microphone.h"
#include "audio_driver.h"
#include "esp_log.h"

size_t microphone_read(int16_t *buffer, size_t buffer_size) {
    size_t bytes_read = 0;
    i2s_chan_handle_t rx = audio_driver_get_rx_handle();
    if (!rx) return 0;

    i2s_channel_read(rx, buffer, buffer_size, &bytes_read, 1000);
    return bytes_read;
}
