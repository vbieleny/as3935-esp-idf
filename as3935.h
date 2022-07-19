#pragma once

#include <stdbool.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"

#define AS3935_DISTANCE_OUT_OF_RANGE -1
#define AS3935_DISTANCE_INVALID -2

typedef enum
{
    PROTOCOL_I2C,
    PROTOCOL_SPI,
    PROTOCOL_NONE,
} protocol_t;

typedef struct
{
    protocol_t protocol;
    i2c_port_t i2c_port;
    uint8_t device_address;
    spi_device_handle_t spi_handle;
    spi_host_device_t host_id;
} as3935_t;

typedef enum
{
    AS3935_AF_INDOOR,
    AS3935_AF_OUTDOOR,
} as3935_analog_frontend_e;

typedef enum
{
    AS3935_NOISE_LEVEL_390_28,
    AS3935_NOISE_LEVEL_630_45,
    AS3935_NOISE_LEVEL_860_62,
    AS3935_NOISE_LEVEL_1100_78,
    AS3935_NOISE_LEVEL_1140_95,
    AS3935_NOISE_LEVEL_1570_112,
    AS3935_NOISE_LEVEL_1800_130,
    AS3935_NOISE_LEVEL_2000_146,
} as3935_noise_level_e;

typedef enum
{
    AS3935_INT_NOISE,
    AS3935_INT_DISTURBER,
    AS3935_INT_LIGHTNING,
    AS3935_INT_NONE,
} as3935_interrupt_reason_e;

typedef enum
{
    AS3935_MIN_LIGHTNING_1,
    AS3935_MIN_LIGHTNING_5,
    AS3935_MIN_LIGHTNING_9,
    AS3935_MIN_LIGHTNING_16,
} as3935_min_lightning_e;

typedef enum
{
    AS3935_FREQ_DIV_RATIO_16,
    AS3935_FREQ_DIV_RATIO_32,
    AS3935_FREQ_DIV_RATIO_64,
    AS3935_FREQ_DIV_RATIO_128,
} as3935_frequency_division_ratio_e;

typedef enum
{
    AS3935_RCO_CALIBRATION_SUCCESSFUL,
    AS3935_RCO_CALIBRATION_UNSUCCESSFUL,
    AS3935_RCO_CALIBRATION_INCOMPLETE,
} as3935_rco_calibration_status_e;

typedef enum
{
    AS3935_OSCILLATOR_ANTENNA_LC,
    AS3935_OSCILLATOR_TIMER_RC,
    AS3935_OSCILLATOR_SYSTEM_RC,
} as3935_oscillator_e;

esp_err_t as3935_reset_to_defaults(const as3935_t *dev);
esp_err_t as3935_calibrate_rco(const as3935_t *dev);
esp_err_t as3935_clear_lightning_statistics(const as3935_t *dev);

esp_err_t as3935_set_power_down(const as3935_t *dev, bool power_down);
esp_err_t as3935_set_watchdog_threshold(const as3935_t *dev, int watchdog_threshold);
esp_err_t as3935_set_analog_frontend(const as3935_t *dev, as3935_analog_frontend_e frontend);
esp_err_t as3935_set_noise_floor_level(const as3935_t *dev, as3935_noise_level_e level);
esp_err_t as3935_set_spike_rejection(const as3935_t *dev, int spike_rejection);
esp_err_t as3935_set_minimum_lightnings(const as3935_t *dev, as3935_min_lightning_e lightnings);
esp_err_t as3935_set_disturber_detection(const as3935_t *dev, bool enabled);
esp_err_t as3935_set_frequency_division_ratio(const as3935_t *dev, as3935_frequency_division_ratio_e ratio);
esp_err_t as3935_set_display_oscillator_on_irq(const as3935_t *dev, as3935_oscillator_e oscillator, bool enabled);
esp_err_t as3935_set_internal_capacitors(const as3935_t *dev, int value);

esp_err_t as3935_get_power_down(const as3935_t *dev, bool *power_down);
esp_err_t as3935_get_watchdog_threshold(const as3935_t *dev, int *watchdog_threshold);
esp_err_t as3935_get_analog_frontend(const as3935_t *dev, as3935_analog_frontend_e *frontend);
esp_err_t as3935_get_noise_floor_level(const as3935_t *dev, as3935_noise_level_e *level);
esp_err_t as3935_get_spike_rejection(const as3935_t *dev, int *spike_rejection);
esp_err_t as3935_get_minimum_lightnings(const as3935_t *dev, as3935_min_lightning_e *lightnings);
esp_err_t as3935_get_disturber_detection(const as3935_t *dev, bool *enabled);
esp_err_t as3935_get_frequency_division_ratio(const as3935_t *dev, as3935_frequency_division_ratio_e *ratio);
esp_err_t as3935_get_display_oscillator_on_irq(const as3935_t *dev, as3935_oscillator_e oscillator, bool *enabled);
esp_err_t as3935_get_internal_capacitors(const as3935_t *dev, int *value);

esp_err_t as3935_get_interrupt_reason(const as3935_t *dev, as3935_interrupt_reason_e *reason);
esp_err_t as3935_get_srco_calibration_status(const as3935_t *dev, as3935_rco_calibration_status_e *status);
esp_err_t as3935_get_trco_calibration_status(const as3935_t *dev, as3935_rco_calibration_status_e *status);
esp_err_t as3935_get_distance(const as3935_t *dev, int *distance);
esp_err_t as3935_get_lightning_energy(const as3935_t *dev, uint32_t *energy);
