#pragma once

#include "as3935.h"
#include "esp_err.h"
#include "driver/gpio.h"

#define AS3935_I2C_FREQ_MAX_HZ 380000     // 380 kHz

#define AS3935_I2C_ADDRESS_A0_A1 0x06
#define AS3935_I2C_ADDRESS_A0 0x02
#define AS3935_I2C_ADDRESS_A1 0x04

#define AS3935_I2C_MODE_MASK 1

esp_err_t as3935_i2c_init(i2c_port_t i2c_port, uint8_t device_address, gpio_num_t gpio_num_sda, gpio_num_t gpio_num_scl, uint32_t clock_hz, as3935_t *dev);
esp_err_t as3935_i2c_free(as3935_t *dev);

esp_err_t as3935_i2c_write_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length);
esp_err_t as3935_i2c_read_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length);

esp_err_t as3935_i2c_write_byte(const as3935_t *dev, uint8_t address, uint8_t data);
esp_err_t as3935_i2c_read_byte(const as3935_t *dev, uint8_t address, uint8_t *data);
