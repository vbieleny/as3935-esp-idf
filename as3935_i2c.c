#include "as3935_i2c.h"
#include "as3935_err.h"

#define I2C_TRANSMISSION_WAIT_TIME 300

esp_err_t as3935_i2c_init(i2c_port_t i2c_port, uint8_t device_address, gpio_num_t gpio_num_sda, gpio_num_t gpio_num_scl, uint32_t clock_hz, as3935_t *dev)
{
    i2c_config_t i2c_config =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = gpio_num_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = gpio_num_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clock_hz,
    };
    AS3935_CHECK(i2c_param_config(i2c_port, &i2c_config));
    AS3935_CHECK(i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0));

    dev->protocol = PROTOCOL_I2C;
    dev->i2c_port = i2c_port;
    dev->device_address = device_address;
    return ESP_OK;
}

esp_err_t as3935_i2c_free(as3935_t *dev)
{
    AS3935_CHECK(i2c_driver_delete(dev->i2c_port));
    dev->protocol = PROTOCOL_NONE;
    dev->i2c_port = -1;
    dev->device_address = 0;
    return ESP_OK;
}

esp_err_t as3935_i2c_write_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    AS3935_CHECK(i2c_master_start(cmd_handle));
    AS3935_CHECK(i2c_master_write_byte(cmd_handle, dev->device_address & ~AS3935_I2C_MODE_MASK, true));
    AS3935_CHECK(i2c_master_write_byte(cmd_handle, address, true));
    AS3935_CHECK(i2c_master_write(cmd_handle, data, length, true));
    AS3935_CHECK(i2c_master_stop(cmd_handle));

    AS3935_CHECK(i2c_master_cmd_begin(dev->i2c_port, cmd_handle, pdMS_TO_TICKS(I2C_TRANSMISSION_WAIT_TIME)));

    i2c_cmd_link_delete(cmd_handle);

    return ESP_OK;
}

esp_err_t as3935_i2c_read_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    AS3935_CHECK(i2c_master_start(cmd_handle));
    AS3935_CHECK(i2c_master_write_byte(cmd_handle, dev->device_address & ~AS3935_I2C_MODE_MASK, true));
    AS3935_CHECK(i2c_master_write_byte(cmd_handle, address, true));
    AS3935_CHECK(i2c_master_start(cmd_handle));
    AS3935_CHECK(i2c_master_write_byte(cmd_handle, dev->device_address | AS3935_I2C_MODE_MASK, true));
    for (size_t i = 0; i < length; i++)
        AS3935_CHECK(i2c_master_read_byte(cmd_handle, data + i, i < length - 1));
    AS3935_CHECK(i2c_master_stop(cmd_handle));

    AS3935_CHECK(i2c_master_cmd_begin(dev->i2c_port, cmd_handle, pdMS_TO_TICKS(I2C_TRANSMISSION_WAIT_TIME)));

    i2c_cmd_link_delete(cmd_handle);
    return ESP_OK;
}

esp_err_t as3935_i2c_write_byte(const as3935_t *dev, uint8_t address, uint8_t data)
{
    return as3935_i2c_write_bytes(dev, address, &data, 1);
}

esp_err_t as3935_i2c_read_byte(const as3935_t *dev, uint8_t address, uint8_t *data)
{
    return as3935_i2c_read_bytes(dev, address, data, 1);
}
