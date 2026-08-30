/**
 * @file bno085.cpp
 * @brief BNO085 I2C driver using ESP-IDF Legacy I2C API (<driver/i2c.h>).
 * Features maximized I2C timeouts for BNO085 clock stretching.
 */

#include "bno085.hpp"

#include <driver/gpio.h>
#include <driver/i2c.h> // Legacy driver
#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_hal.h"
#include "sh2_err.h"
}

#include <cmath>
#include <cstring>

static const char *TAG = "BNO085";

// I2C blocking timeout for a single transaction
static constexpr int I2C_TIMEOUT_MS = 50;

// ─── static instance ─────────────────────────────────────────────────────────

BNO085 *BNO085::instance_ = nullptr;

// ─── constructor ─────────────────────────────────────────────────────────────

BNO085::BNO085(const bno085_i2c_cfg_t &cfg) : cfg_(cfg)
{
    instance_ = this;
    static_assert(sizeof(sh2_hal_buf_) >= sizeof(sh2_Hal_t),
                  "sh2_hal_buf_ too small");
    sh2_hal_ = reinterpret_cast<sh2_Hal_t *>(sh2_hal_buf_);
}

// ─── destructor ──────────────────────────────────────────────────────────────

BNO085::~BNO085()
{
    sh2_close();
    instance_ = nullptr;
}

// ─── init() ──────────────────────────────────────────────────────────────────

esp_err_t BNO085::init()
{
    // 1. RST pin — output, start HIGH (sensor running)
    gpio_config_t io {};
    io.pin_bit_mask = (1ULL << cfg_.rst);
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst), 1);

    // 2. I2C bus (Legacy API)
    i2c_port_t i2c_port = static_cast<i2c_port_t>(cfg_.i2c_port);
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = cfg_.sda;
    conf.scl_io_num = cfg_.scl;
    
    // Safety Net: Enable internal pullups just in case external ones are weak
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; 
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = (cfg_.clk_hz > 0) ? cfg_.clk_hz : 400000;
    
    i2c_param_config(i2c_port, &conf);
    
    // Install driver. If audio driver already installed it on this port, 
    // this will return ESP_ERR_INVALID_STATE. We safely ignore that error.
    esp_err_t err = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    // CRITICAL FIX: The BNO085 utilizes significant I2C clock stretching.
    // The default ESP-IDF legacy timeout is too short and will abort the read, 
    // causing SHTP packets to be dropped. 0xFFFFF is the maximum timeout.
    i2c_set_timeout(i2c_port, 31);

    // 4. SH-2 HAL vtable
    sh2_hal_->open      = hal_open;
    sh2_hal_->close     = hal_close;
    sh2_hal_->read      = hal_read;
    sh2_hal_->write     = hal_write;
    sh2_hal_->getTimeUs = hal_time_us;

    // 5. Open SH-2
    return sh2_open_internal();
}

esp_err_t BNO085::reset()
{
    sh2_close();
    hw_reset();
    return sh2_open_internal();
}

esp_err_t BNO085::sh2_open_internal()
{
    int rc = sh2_open(sh2_hal_, event_cb, this);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open failed: %d", rc);
        return ESP_FAIL;
    }
    sh2_setSensorCallback(sensor_cb, this);
    ESP_LOGI(TAG, "BNO085 I2C (Legacy API) initialised OK");
    return ESP_OK;
}

// ─── enable helpers ───────────────────────────────────────────────────────────

static esp_err_t enable_report(sh2_SensorId_t id, uint32_t us)
{
    sh2_SensorConfig_t cfg {};
    cfg.reportInterval_us = us;
    if (sh2_setSensorConfig(id, &cfg) != SH2_OK) {
        ESP_LOGE("BNO085", "setSensorConfig(%d) failed", id);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t BNO085::enable_accelerometer(uint32_t us)
    { return enable_report(SH2_ACCELEROMETER,             us); }
esp_err_t BNO085::enable_gyroscope(uint32_t us)
    { return enable_report(SH2_GYROSCOPE_CALIBRATED,      us); }
esp_err_t BNO085::enable_magnetometer(uint32_t us)
    { return enable_report(SH2_MAGNETIC_FIELD_CALIBRATED, us); }
esp_err_t BNO085::enable_rotation_vector(uint32_t us)
    { return enable_report(SH2_ROTATION_VECTOR,           us); }
esp_err_t BNO085::enable_game_rotation_vector(uint32_t us)
    { return enable_report(SH2_GAME_ROTATION_VECTOR,      us); }

// ─── service() ───────────────────────────────────────────────────────────────

bool BNO085::service()
{
    received_mask_ = 0; 
    sh2_service();
    if (received_mask_ != 0 && data_cb_) data_cb_(received_mask_);
    return (received_mask_ != 0);
}

// ─── read cached values ──────────────────────────────────────────────────────

bool BNO085::read_accelerometer(bno085_accel_t &out) const {
    if (!(received_mask_ & REPORT_ACCEL)) return false;
    out = accel_; return true;
}
bool BNO085::read_gyroscope(bno085_gyro_t &out) const {
    if (!(received_mask_ & REPORT_GYRO)) return false;
    out = gyro_; return true;
}
bool BNO085::read_magnetometer(bno085_mag_t &out) const {
    if (!(received_mask_ & REPORT_MAG)) return false;
    out = mag_; return true;
}
bool BNO085::read_quaternion(bno085_quat_t &out) const {
    if (!(received_mask_ & REPORT_RV)) return false;
    out = quat_; return true;
}
bool BNO085::read_euler(bno085_euler_t &out) const
{
    if (!(received_mask_ & REPORT_RV)) return false;

    const float qi = quat_.i, qj = quat_.j, qk = quat_.k, qr = quat_.real;

    const float sinr = 2.0f * (qr*qi + qj*qk);
    const float cosr = 1.0f - 2.0f * (qi*qi + qj*qj);
    out.roll  = std::atan2(sinr, cosr) * (180.0f / M_PI);

    const float sinp = 2.0f * (qr*qj - qk*qi);
    out.pitch = (std::fabs(sinp) >= 1.0f)
                ? std::copysign(90.0f, sinp)
                : std::asin(sinp) * (180.0f / M_PI);

    const float siny = 2.0f * (qr*qk + qi*qj);
    const float cosy = 1.0f - 2.0f * (qj*qj + qk*qk);
    out.yaw   = std::atan2(siny, cosy) * (180.0f / M_PI);

    return true;
}

// ─── SH-2 event callback ─────────────────────────────────────────────────────

void BNO085::event_cb(void * /*cookie*/, sh2_AsyncEvent_t *event)
{
    if (event && event->eventId == SH2_RESET)
        ESP_LOGW(TAG, "BNO085 unexpected reset");
}

// ─── SH-2 sensor data callback ───────────────────────────────────────────────

void BNO085::sensor_cb(void *cookie, sh2_SensorEvent_t *event)
{
    BNO085 *self = static_cast<BNO085 *>(cookie);
    if (!self || !event) return;

    sh2_SensorValue_t val {};
    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) return;

    switch (val.sensorId) {
    case SH2_ACCELEROMETER:
        self->accel_ = {val.un.accelerometer.x, val.un.accelerometer.y, val.un.accelerometer.z};
        self->received_mask_ |= REPORT_ACCEL; break;
    case SH2_GYROSCOPE_CALIBRATED:
        self->gyro_ = {val.un.gyroscope.x, val.un.gyroscope.y, val.un.gyroscope.z};
        self->received_mask_ |= REPORT_GYRO; break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        self->mag_ = {val.un.magneticField.x, val.un.magneticField.y, val.un.magneticField.z};
        self->received_mask_ |= REPORT_MAG; break;
    case SH2_ROTATION_VECTOR:
        self->quat_ = {val.un.rotationVector.i, val.un.rotationVector.j, val.un.rotationVector.k, val.un.rotationVector.real, val.un.rotationVector.accuracy};
        self->received_mask_ |= REPORT_RV; break;
    case SH2_GAME_ROTATION_VECTOR:
        self->quat_ = {val.un.gameRotationVector.i, val.un.gameRotationVector.j, val.un.gameRotationVector.k, val.un.gameRotationVector.real, 0.0f};
        self->received_mask_ |= REPORT_RV; break;
    default: break;
    }
}

// ─── SH-2 HAL: open ──────────────────────────────────────────────────────────

int BNO085::hal_open(sh2_Hal_t * /*self*/)
{
    instance_->hw_reset();
    return SH2_OK;
}

// ─── SH-2 HAL: close ─────────────────────────────────────────────────────────

void BNO085::hal_close(sh2_Hal_t * /*self*/)
{
}

// ─── SH-2 HAL: read ──────────────────────────────────────────────────────────

int BNO085::hal_read(sh2_Hal_t * /*self*/, uint8_t *buf, unsigned len, uint32_t *t_us)
{
    BNO085 *self = instance_;

    int n = self->i2c_read_packet(buf, len);
    if (n < 4) return 0;

    uint16_t pkt_sz = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    pkt_sz &= ~0x8000U;

    if (pkt_sz == 0 || pkt_sz > static_cast<uint16_t>(len)) {
        return 0;
    }

    if (t_us) *t_us = static_cast<uint32_t>(esp_timer_get_time() & 0xFFFFFFFFU);

    return static_cast<int>(pkt_sz);
}

// ─── SH-2 HAL: write ─────────────────────────────────────────────────────────

int BNO085::hal_write(sh2_Hal_t * /*self*/, uint8_t *buf, unsigned len)
{
    return instance_->i2c_write_packet(buf, len);
}

// ─── SH-2 HAL: getTimeUs ─────────────────────────────────────────────────────

uint32_t BNO085::hal_time_us(sh2_Hal_t * /*self*/)
{
    return static_cast<uint32_t>(esp_timer_get_time() & 0xFFFFFFFFU);
}

// ─── hw_reset() ──────────────────────────────────────────────────────────────

void BNO085::hw_reset()
{
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(cfg_.rst), 1);
    
    // Give BNO085 a full 400ms to boot up and format its 
    // internal advertisement packet before polling begins.
    vTaskDelay(pdMS_TO_TICKS(400));
}

// ─── i2c_read_packet() (Legacy) ──────────────────────────────────────────────

int BNO085::i2c_read_packet(uint8_t *buf, unsigned max_len)
{
    i2c_port_t port = static_cast<i2c_port_t>(cfg_.i2c_port);
    esp_err_t err = i2c_master_read_from_device(port, cfg_.addr, buf, max_len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return 0;
    return static_cast<int>(max_len);
}

// ─── i2c_write_packet() (Legacy) ─────────────────────────────────────────────

int BNO085::i2c_write_packet(const uint8_t *buf, unsigned len)
{
    i2c_port_t port = static_cast<i2c_port_t>(cfg_.i2c_port);
    esp_err_t err = i2c_master_write_to_device(port, cfg_.addr, buf, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return 0;
    return static_cast<int>(len);
}