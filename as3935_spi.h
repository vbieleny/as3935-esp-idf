#pragma once

#include "as3935.h"
#include "esp_err.h"

#define AS3935_SPI_MASTER_FREQ_MAX 2000000     // 2 MHz

#define AS3935_SPI_MODE_WRITE 0
#define AS3935_SPI_MODE_READ 1

esp_err_t as3935_spi_init(spi_host_device_t host_id, int cs, int clock_hz, as3935_t *dev);
esp_err_t as3935_spi_free(as3935_t *dev);

esp_err_t as3935_spi_write_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length);
esp_err_t as3935_spi_read_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length);

esp_err_t as3935_spi_write_byte(const as3935_t *dev, uint8_t address, uint8_t data);
esp_err_t as3935_spi_read_byte(const as3935_t *dev, uint8_t address, uint8_t *data);
