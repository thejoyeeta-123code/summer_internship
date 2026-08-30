#pragma once

/**
 * @file bno085.hpp
 * @brief Lightweight BNO085 IMU driver for ESP-IDF v5.4+ / ESP32-P4.
 *
 * No ETL dependency. Pure ESP-IDF (C++17/23 compatible).
 * Uses the official Hillcrest SH-2 C library directly.
 *
 * Wiring (I2C, no INT pin):
 *   SDA  → cfg.sda   (4.7 kΩ pull-up to 3.3 V required)
 *   SCL  → cfg.scl   (4.7 kΩ pull-up to 3.3 V required)
 *   RST  → cfg.rst   (active-low; drive high to run)
 *   VIN  → 3.3 V
 *   GND  → GND
 *
 * INT/HINT pin is NOT connected.
 * Data-ready is detected by polling: hal_read() attempts an I2C read and
 * inspects the returned SHTP header length field.  A length of 0 means the
 * sensor has nothing ready; a non-zero length means a packet is available.
 *
 * I2C address:
 *   PS1=0, PS0=0 → 0x4A  (default, SA0 low)
 *   PS1=0, PS0=1 → 0x4B  (SA0 high)
 */

#include <cmath>
#include <cstdint>
#include <functional>

#include "esp_err.h"
#include "sh2.h"

// ── Public data types ────────────────────────────────────────────────────────

struct bno085_accel_t { float x, y, z; };  ///< m/s²
struct bno085_gyro_t  { float x, y, z; };  ///< rad/s
struct bno085_mag_t   { float x, y, z; };  ///< µTesla

struct bno085_quat_t {
    float i, j, k, real;
    float accuracy;   ///< heading accuracy [rad]; 0 for game rotation vector
};

struct bno085_euler_t {
    float yaw;    ///< heading [°]
    float pitch;  ///< tilt    [°]
    float roll;   ///< roll    [°]
};

// ── I2C configuration ────────────────────────────────────────────────────────

struct bno085_i2c_cfg_t {
    int     sda;       ///< I2C data GPIO
    int     scl;       ///< I2C clock GPIO
    int     rst;       ///< Reset GPIO (active-low)
    int     i2c_port;  ///< I2C_NUM_0 or I2C_NUM_1 (default 0)
    int     clk_hz;    ///< I2C clock Hz (default 400 000)
    uint8_t addr;      ///< 7-bit I2C address: 0x4A or 0x4B (default 0x4A)
};

// ── Driver class ─────────────────────────────────────────────────────────────

class BNO085 {
public:
    explicit BNO085(const bno085_i2c_cfg_t &cfg);
    ~BNO085();

    BNO085(const BNO085 &)            = delete;
    BNO085 &operator=(const BNO085 &) = delete;

    esp_err_t init();
    esp_err_t reset();

    // ── Enable individual reports ────────────────────────────────────────────

    /** @param interval_us Report period in microseconds (e.g. 10000 = 100 Hz). */
    esp_err_t enable_accelerometer       (uint32_t interval_us = 10000);
    esp_err_t enable_gyroscope           (uint32_t interval_us = 10000);
    esp_err_t enable_magnetometer        (uint32_t interval_us = 10000);
    esp_err_t enable_rotation_vector     (uint32_t interval_us = 10000);
    esp_err_t enable_game_rotation_vector(uint32_t interval_us = 10000);

    // ── Poll / service ───────────────────────────────────────────────────────

    /**
     * @brief Service the SH-2 stack — call from your task loop.
     *
     * Internally calls sh2_service() which invokes hal_read().
     * hal_read() polls the sensor over I2C; if no packet is ready it
     * returns immediately so this call never blocks longer than one
     * I2C transaction (~100 µs at 400 kHz for a max-size packet).
     *
     * @return true if new sensor data was decoded this call.
     */
    bool service();

    // ── Read latest cached values ────────────────────────────────────────────

    bool read_accelerometer(bno085_accel_t &out) const;
    bool read_gyroscope    (bno085_gyro_t  &out) const;
    bool read_magnetometer (bno085_mag_t   &out) const;
    bool read_quaternion   (bno085_quat_t  &out) const;
    bool read_euler        (bno085_euler_t &out) const;

    // ── Optional data-ready callback ─────────────────────────────────────────

    static constexpr uint32_t REPORT_ACCEL = (1u << 0);
    static constexpr uint32_t REPORT_GYRO  = (1u << 1);
    static constexpr uint32_t REPORT_MAG   = (1u << 2);
    static constexpr uint32_t REPORT_RV    = (1u << 3);

    using DataCallback = std::function<void(uint32_t report_mask)>;
    void set_data_callback(DataCallback cb) { data_cb_ = std::move(cb); }

private:
    // SH-2 HAL vtable (plain C function pointers)
    static int      hal_open   (struct sh2_Hal_s *self);
    static void     hal_close  (struct sh2_Hal_s *self);
    static int      hal_read   (struct sh2_Hal_s *self, uint8_t *buf, unsigned len, uint32_t *t_us);
    static int      hal_write  (struct sh2_Hal_s *self, uint8_t *buf, unsigned len);
    static uint32_t hal_time_us(struct sh2_Hal_s *self);

    // SH-2 event / sensor callbacks
    static void event_cb (void *cookie, sh2_AsyncEvent_t *event);
    static void sensor_cb(void *cookie, sh2_SensorEvent_t *event);

    // Internal helpers
    esp_err_t sh2_open_internal();
    void      hw_reset();
    int       i2c_read_packet (uint8_t *buf, unsigned max_len);
    int       i2c_write_packet(const uint8_t *buf, unsigned len);

    // ── Members ──────────────────────────────────────────────────────────────
    bno085_i2c_cfg_t cfg_;

    struct sh2_Hal_s *sh2_hal_ {nullptr};
    uint8_t           sh2_hal_buf_[sizeof(void*) * 8] {};

    bno085_accel_t    accel_ {};
    bno085_gyro_t     gyro_  {};
    bno085_mag_t      mag_   {};
    bno085_quat_t     quat_  {};

    volatile uint32_t received_mask_ {0};

    DataCallback      data_cb_;

    static BNO085    *instance_;
};
