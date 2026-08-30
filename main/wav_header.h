#pragma once
#include <stdint.h>
#include <string.h>

typedef struct {
    char riff_tag[4]; uint32_t riff_len; char wave_tag[4];
    char fmt_tag[4];  uint32_t fmt_len;  uint16_t audio_format;
    uint16_t num_channels; uint32_t sample_rate; uint32_t byte_rate;
    uint16_t block_align; uint16_t bits_per_sample;
    char data_tag[4]; uint32_t data_len;
} __attribute__((packed)) wav_header_t;

static inline void generate_wav_header(wav_header_t *header, uint32_t data_len, uint32_t sample_rate) {
    memcpy(header->riff_tag, "RIFF", 4);
    memcpy(header->wave_tag, "WAVE", 4);
    memcpy(header->fmt_tag, "fmt ", 4);
    memcpy(header->data_tag, "data", 4);

    header->riff_len = data_len + sizeof(wav_header_t) - 8;
    header->fmt_len = 16;
    header->audio_format = 1; // PCM
    header->num_channels = 2; // Stereo
    header->sample_rate = sample_rate;
    header->bits_per_sample = 16;
    header->byte_rate = sample_rate * 2 * 2;
    header->block_align = 4;
    header->data_len = data_len;
}
