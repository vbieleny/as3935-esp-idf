#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "as3935.h"
#include "as3935_spi.h"
#include "as3935_err.h"

esp_err_t as3935_reset_to_defaults(const as3935_t *dev)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x3C, 0x96));
    return ESP_OK;
}

esp_err_t as3935_calibrate_rco(const as3935_t *dev)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x3D, 0x96));

    AS3935_CHECK(as3935_set_display_oscillator_on_irq(dev, AS3935_OSCILLATOR_SYSTEM_RC, true));
    vTaskDelay(pdMS_TO_TICKS(2));
    AS3935_CHECK(as3935_set_display_oscillator_on_irq(dev, AS3935_OSCILLATOR_SYSTEM_RC, false));

    return ESP_OK;
}

esp_err_t as3935_clear_lightning_statistics(const as3935_t *dev)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x02, &data));

    data |= 1 << 6;
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x02, data));
    data &= ~(1 << 6);
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x02, data));
    data |= 1 << 6;
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x02, data));

    return ESP_OK;
}

esp_err_t as3935_set_power_down(const as3935_t *dev, bool power_down)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x00, &data));

    if (power_down)
        data |= 1;
    else
        data &= ~1;
    
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x00, data));
    
    if (!power_down)
        AS3935_CHECK(as3935_calibrate_rco(dev));

    return ESP_OK;
}

esp_err_t as3935_set_watchdog_threshold(const as3935_t *dev, int watchdog_threshold)
{
    AS3935_CHECK_ARG(dev);
    
    if (watchdog_threshold < 0 || watchdog_threshold > 15)
        return ESP_ERR_INVALID_ARG;
    
    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x01, &data));

    data &= ~0b1111;
    data |= watchdog_threshold;

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x01, data));

    return ESP_OK;
}

esp_err_t as3935_set_analog_frontend(const as3935_t *dev, as3935_analog_frontend_e frontend)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x00, &data));

    data &= ~(0b11111 << 1);

    switch (frontend)
    {
        case AS3935_AF_INDOOR:
            data |= 0b10010 << 1;
            break;
        case AS3935_AF_OUTDOOR:
            data |= 0b01110 << 1;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x00, data));

    return ESP_OK;
}

esp_err_t as3935_set_noise_floor_level(const as3935_t *dev, as3935_noise_level_e level)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x01, &data));

    data &= ~(0b111 << 4);

    switch (level)
    {
        case AS3935_NOISE_LEVEL_390_28:
            data |= 0b000 << 4;
            break;
        case AS3935_NOISE_LEVEL_630_45:
            data |= 0b001 << 4;
            break;
        case AS3935_NOISE_LEVEL_860_62:
            data |= 0b010 << 4;
            break;
        case AS3935_NOISE_LEVEL_1100_78:
            data |= 0b011 << 4;
            break;
        case AS3935_NOISE_LEVEL_1140_95:
            data |= 0b100 << 4;
            break;
        case AS3935_NOISE_LEVEL_1570_112:
            data |= 0b101 << 4;
            break;
        case AS3935_NOISE_LEVEL_1800_130:
            data |= 0b110 << 4;
            break;
        case AS3935_NOISE_LEVEL_2000_146:
            data |= 0b111 << 4;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    AS3935_CHECK(as3935_spi_write_byte(dev, 0x01, data));

    return ESP_OK;
}

esp_err_t as3935_set_spike_rejection(const as3935_t *dev, int spike_rejection)
{
    AS3935_CHECK_ARG(dev);
    
    if (spike_rejection < 0 || spike_rejection > 15)
        return ESP_ERR_INVALID_ARG;
    
    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x02, &data));

    data &= ~0b1111;
    data |= spike_rejection;

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x02, data));

    return ESP_OK;
}

esp_err_t as3935_set_minimum_lightnings(const as3935_t *dev, as3935_min_lightning_e lightnings)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x02, &data));

    data &= ~(0b11 << 4);

    switch (lightnings)
    {
        case AS3935_MIN_LIGHTNING_1:
            data |= 0b00 << 4;
            break;
        case AS3935_MIN_LIGHTNING_5:
            data |= 0b01 << 4;
            break;
        case AS3935_MIN_LIGHTNING_9:
            data |= 0b10 << 4;
            break;
        case AS3935_MIN_LIGHTNING_16:
            data |= 0b11 << 4;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x02, data));

    return ESP_OK;
}

esp_err_t as3935_set_disturber_detection(const as3935_t *dev, bool enabled)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x03, &data));

    data &= ~(1 << 5);
    data |= (enabled ? 0 : 1) << 5;

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x03, data));

    return ESP_OK;
}

esp_err_t as3935_set_frequency_division_ratio(const as3935_t *dev, as3935_frequency_division_ratio_e ratio)
{
    AS3935_CHECK_ARG(dev);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x03, &data));

    data &= ~(0b11 << 6);

    switch (ratio)
    {
        case AS3935_FREQ_DIV_RATIO_16:
            data |= 0b00 << 6;
            break;
        case AS3935_FREQ_DIV_RATIO_32:
            data |= 0b01 << 6;
            break;
        case AS3935_FREQ_DIV_RATIO_64:
            data |= 0b10 << 6;
            break;
        case AS3935_FREQ_DIV_RATIO_128:
            data |= 0b11 << 6;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x03, data));

    return ESP_OK;
}

esp_err_t as3935_set_display_oscillator_on_irq(const as3935_t *dev, as3935_oscillator_e oscillator, bool enabled)
{
    AS3935_CHECK_ARG(dev);

    uint8_t shift;
    switch (oscillator)
    {
        case AS3935_OSCILLATOR_ANTENNA_LC:
            shift = 7;
            break;
        case AS3935_OSCILLATOR_SYSTEM_RC:
            shift = 6;
            break;
        case AS3935_OSCILLATOR_TIMER_RC:
            shift = 5;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x08, &data));

    data &= ~(1 << shift);
    data |= (enabled ? 1 : 0) << shift;

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x08, data));

    return ESP_OK;
}

esp_err_t as3935_set_internal_capacitors(const as3935_t *dev, int value)
{
    AS3935_CHECK_ARG(dev);
    
    if (value < 0 || value > 15)
        return ESP_ERR_INVALID_ARG;
    
    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x08, &data));

    data &= ~0b1111;
    data |= value;

    AS3935_CHECK(as3935_spi_write_byte(dev, 0x08, data));

    return ESP_OK;
}

esp_err_t as3935_get_power_down(const as3935_t *dev, bool *power_down)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(power_down);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x00, &data));

    *power_down = data & 1;

    return ESP_OK;
}

esp_err_t as3935_get_watchdog_threshold(const as3935_t *dev, int *watchdog_threshold)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(watchdog_threshold);
    
    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x01, &data));

    *watchdog_threshold = data & 0b1111;

    return ESP_OK;
}

esp_err_t as3935_get_analog_frontend(const as3935_t *dev, as3935_analog_frontend_e *frontend)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(frontend);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x00, &data));

    data &= 0b11111 << 1;
    data >>= 1;

    switch (data)
    {
        case 0b10010:
            *frontend = AS3935_AF_INDOOR;
            break;
        case 0b01110:
            *frontend = AS3935_AF_OUTDOOR;
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t as3935_get_noise_floor_level(const as3935_t *dev, as3935_noise_level_e *level)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(level);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x01, &data));

    data &= 0b111 << 4;
    data >>= 4;

    switch (data)
    {
        case 0b000:
            *level = AS3935_NOISE_LEVEL_390_28;
            break;
        case 0b001:
            *level = AS3935_NOISE_LEVEL_630_45;
            break;
        case 0b010:
            *level = AS3935_NOISE_LEVEL_860_62;
            break;
        case 0b011:
            *level = AS3935_NOISE_LEVEL_1100_78;
            break;
        case 0b100:
            *level = AS3935_NOISE_LEVEL_1140_95;
            break;
        case 0b101:
            *level = AS3935_NOISE_LEVEL_1570_112;
            break;
        case 0b110:
            *level = AS3935_NOISE_LEVEL_1800_130;
            break;
        case 0b111:
            *level = AS3935_NOISE_LEVEL_2000_146;
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t as3935_get_spike_rejection(const as3935_t *dev, int *spike_rejection)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(spike_rejection);
    
    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x02, &data));

    *spike_rejection = data & 0b1111;

    return ESP_OK;
}

esp_err_t as3935_get_minimum_lightnings(const as3935_t *dev, as3935_min_lightning_e *lightnings)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(lightnings);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x02, &data));

    data &= 0b11 << 4;
    data >>= 4;

    switch (data)
    {
        case 0b00:
            *lightnings = AS3935_MIN_LIGHTNING_1;
            break;
        case 0b01:
            *lightnings = AS3935_MIN_LIGHTNING_5;
            break;
        case 0b10:
            *lightnings = AS3935_MIN_LIGHTNING_9;
            break;
        case 0b11:
            *lightnings = AS3935_MIN_LIGHTNING_16;
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t as3935_get_disturber_detection(const as3935_t *dev, bool *enabled)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(enabled);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x03, &data));

    *enabled = !(data & (1 << 5));

    return ESP_OK;
}

esp_err_t as3935_get_frequency_division_ratio(const as3935_t *dev, as3935_frequency_division_ratio_e *ratio)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(ratio);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x03, &data));

    data &= 0b11 << 6;
    data >>= 6;

    switch (data)
    {
        case 0b00:
            *ratio = AS3935_FREQ_DIV_RATIO_16;
            break;
        case 0b01:
            *ratio = AS3935_FREQ_DIV_RATIO_32;
            break;
        case 0b10:
            *ratio = AS3935_FREQ_DIV_RATIO_64;
            break;
        case 0b11:
            *ratio = AS3935_FREQ_DIV_RATIO_128;
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t as3935_get_display_oscillator_on_irq(const as3935_t *dev, as3935_oscillator_e oscillator, bool *enabled)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(enabled);

    uint8_t shift;
    switch (oscillator)
    {
        case AS3935_OSCILLATOR_ANTENNA_LC:
            shift = 7;
            break;
        case AS3935_OSCILLATOR_SYSTEM_RC:
            shift = 6;
            break;
        case AS3935_OSCILLATOR_TIMER_RC:
            shift = 5;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x08, &data));

    *enabled = data & (1 << shift);

    return ESP_OK;
}

esp_err_t as3935_get_internal_capacitors(const as3935_t *dev, int *value)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(value);
    
    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x08, &data));

    *value = data & 0b1111;

    return ESP_OK;
}

esp_err_t as3935_get_interrupt_reason(const as3935_t *dev, as3935_interrupt_reason_e *reason)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(reason);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x03, &data));

    data &= 0b1111;

    switch (data)
    {
        case 0b0001:
            *reason = AS3935_INT_NOISE;
            break;
        case 0b0100:
            *reason = AS3935_INT_DISTURBER;
            break;
        case 0b1000:
            *reason = AS3935_INT_LIGHTNING;
            break;
        case 0b0000:
            *reason = AS3935_INT_NONE;
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t as3935_get_srco_calibration_status(const as3935_t *dev, as3935_rco_calibration_status_e *status)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(status);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x3B, &data));

    bool done = data & (1 << 7);
    bool nok = data & (1 << 6);

    if (!done && !nok)
        *status = AS3935_RCO_CALIBRATION_INCOMPLETE;
    else if (done && !nok)
        *status = AS3935_RCO_CALIBRATION_SUCCESSFUL;
    else if (!done && nok)
        *status = AS3935_RCO_CALIBRATION_UNSUCCESSFUL;
    else
        return ESP_ERR_INVALID_STATE;

    return ESP_OK;
}

esp_err_t as3935_get_trco_calibration_status(const as3935_t *dev, as3935_rco_calibration_status_e *status)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(status);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x3A, &data));

    bool done = data & (1 << 7);
    bool nok = data & (1 << 6);

    if (!done && !nok)
        *status = AS3935_RCO_CALIBRATION_INCOMPLETE;
    else if (done && !nok)
        *status = AS3935_RCO_CALIBRATION_SUCCESSFUL;
    else if (!done && nok)
        *status = AS3935_RCO_CALIBRATION_UNSUCCESSFUL;
    else
        return ESP_ERR_INVALID_STATE;

    return ESP_OK;
}

esp_err_t as3935_get_distance(const as3935_t *dev, int *distance)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(distance);

    uint8_t data;
    AS3935_CHECK(as3935_spi_read_byte(dev, 0x07, &data));
    switch (data)
    {
        case 0b111111:
            *distance = AS3935_DISTANCE_OUT_OF_RANGE;
            break;
        case 0b101000:
            *distance = 40;
            break;
        case 0b100101:
            *distance = 37;
            break;
        case 0b100010:
            *distance = 34;
            break;
        case 0b011111:
            *distance = 31;
            break;
        case 0b011011:
            *distance = 27;
            break;
        case 0b011000:
            *distance = 24;
            break;
        case 0b010100:
            *distance = 20;
            break;
        case 0b010001:
            *distance = 17;
            break;
        case 0b001110:
            *distance = 14;
            break;
        case 0b001100:
            *distance = 12;
            break;
        case 0b001010:
            *distance = 10;
            break;
        case 0b001000:
            *distance = 8;
            break;
        case 0b000110:
            *distance = 6;
            break;
        case 0b000101:
            *distance = 5;
            break;
        case 0b000001:
            *distance = 0;
            break;
        default:
            *distance = AS3935_DISTANCE_INVALID;
            break;
    }
    return ESP_OK;
}

esp_err_t as3935_get_lightning_energy(const as3935_t *dev, uint32_t *energy)
{
    AS3935_CHECK_ARG(dev);
    AS3935_CHECK_ARG(energy);

    uint8_t data[3];
    AS3935_CHECK(as3935_spi_read_bytes(dev, 0x04, data, 3));

    data[2] &= 0b11111;

    *energy = 0;
    for (size_t i = 0; i < 3; ++i)
        *energy |= data[i] << (i * 8);

    return ESP_OK;
}
