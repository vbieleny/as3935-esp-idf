#include "as3935_spi.h"
#include "as3935_err.h"

esp_err_t as3935_spi_init(spi_host_device_t host_id, int cs, int clock_hz, as3935_t *dev)
{
    const spi_device_interface_config_t dev_config = {
        .command_bits = 2,
        .address_bits = 6,
        .mode = 1,
        .clock_speed_hz = clock_hz,
        .spics_io_num = cs,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    spi_device_handle_t handle;
    AS3935_CHECK(spi_bus_add_device(host_id, &dev_config, &handle));

    dev->spi_handle = handle;
    dev->host_id = host_id;
    return ESP_OK;
}

esp_err_t as3935_spi_free(as3935_t *dev)
{
    AS3935_CHECK(spi_bus_remove_device(dev->spi_handle));
    AS3935_CHECK(spi_bus_free(dev->host_id));
    dev->spi_handle = NULL;
    return ESP_OK;
}

esp_err_t as3935_spi_write_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length)
{
    if (length > 4)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t transaction =
    {
        .cmd = AS3935_SPI_MODE_WRITE,
        .addr = address,
        .length = length * sizeof(uint8_t) * 8,
        .flags = SPI_TRANS_USE_TXDATA
    };

    for (size_t i = 0; i < length; ++i)
        transaction.tx_data[i] = data[i];

    AS3935_CHECK(spi_device_polling_transmit(dev->spi_handle, &transaction));
    return ESP_OK;
}

esp_err_t as3935_spi_read_bytes(const as3935_t *dev, uint8_t address, uint8_t *data, size_t length)
{
    if (length > 4)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t transaction =
    {
        .cmd = AS3935_SPI_MODE_READ,
        .addr = address,
        .length = length * sizeof(uint8_t) * 8,
        .rxlength = length * sizeof(uint8_t) * 8,
        .flags = SPI_TRANS_USE_RXDATA
    };
    AS3935_CHECK(spi_device_polling_transmit(dev->spi_handle, &transaction));

    for (size_t i = 0; i < length; ++i)
        data[i] = transaction.rx_data[i];

    return ESP_OK;
}

esp_err_t as3935_spi_write_byte(const as3935_t *dev, uint8_t address, uint8_t data)
{
    return as3935_spi_write_bytes(dev, address, &data, 1);
}

esp_err_t as3935_spi_read_byte(const as3935_t *dev, uint8_t address, uint8_t *data)
{
    return as3935_spi_read_bytes(dev, address, data, 1);
}


