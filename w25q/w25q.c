/**
 * @file    w25q.c
 * @brief   W25Q SPI Flash Memory Driver Implementation
 * @version 1.0.0
 */

#include "w25q.h"
#include <stdlib.h>
#include <string.h>

/* Private function prototypes */
static w25q_error_t w25q_send_command(w25q_handle_t *handle, uint8_t command);
static w25q_error_t
w25q_send_command_with_address(w25q_handle_t *handle, uint8_t command, uint32_t address);
static w25q_error_t w25q_send_command_with_address_and_data(
    w25q_handle_t *handle, uint8_t command, uint32_t address, const uint8_t *data, uint16_t size);

/* Private helper functions */
static w25q_error_t w25q_send_command(w25q_handle_t *handle, uint8_t command)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    return w25q_spi_tx(&command, 1);
}

static w25q_error_t
w25q_send_command_with_address(w25q_handle_t *handle, uint8_t command, uint32_t address)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = command;
    cmd_buffer[1] = (uint8_t)((address >> 16) & 0xFF);
    cmd_buffer[2] = (uint8_t)((address >> 8) & 0xFF);
    cmd_buffer[3] = (uint8_t)(address & 0xFF);

    return w25q_spi_tx(cmd_buffer, 4);
}

/* Removed unused function w25q_send_command_with_data */

static w25q_error_t w25q_send_command_with_address_and_data(
    w25q_handle_t *handle, uint8_t command, uint32_t address, const uint8_t *data, uint16_t size)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    uint8_t *cmd_buffer = (uint8_t *)malloc(size + 4);
    if (cmd_buffer == NULL)
    {
        return W25Q_ERROR_INVALID_PARAM;
    }

    cmd_buffer[0] = command;
    cmd_buffer[1] = (uint8_t)((address >> 16) & 0xFF);
    cmd_buffer[2] = (uint8_t)((address >> 8) & 0xFF);
    cmd_buffer[3] = (uint8_t)(address & 0xFF);
    memcpy(&cmd_buffer[4], data, size);

    w25q_error_t result = w25q_spi_tx(cmd_buffer, size + 4);
    free(cmd_buffer);

    return result;
}

/* Removed unused function w25q_receive_data */

/* Public API functions */
w25q_error_t w25q_init(w25q_handle_t *handle)
{
    if (handle == NULL)
    {
        return W25Q_ERROR_INVALID_PARAM;
    }

    handle->initialized = true;
    return W25Q_OK;
}

void w25q_deinit(w25q_handle_t *handle)
{
    if (handle != NULL)
    {
        handle->initialized = false;
    }
}

w25q_error_t w25q_read_id(w25q_handle_t *handle, uint8_t *manufacturer_id, uint16_t *device_id)
{
    if (!handle || !handle->initialized || !manufacturer_id || !device_id)
    {
        return W25Q_ERROR_INVALID_PARAM;
    }

    uint8_t cmd = W25Q_CMD_READ_ID;
    uint8_t response[4];

    w25q_error_t result = w25q_spi_tx(&cmd, 1);
    if (result != W25Q_OK)
    {
        return result;
    }

    result = w25q_spi_rx(response, 4);
    if (result != W25Q_OK)
    {
        return result;
    }

    *manufacturer_id = response[2];
    *device_id       = (uint16_t)((response[3] << 8) | response[4]);

    return W25Q_OK;
}

w25q_error_t w25q_read_status(w25q_handle_t *handle, uint8_t *status)
{
    if (!handle || !handle->initialized || !status)
    {
        return W25Q_ERROR_INVALID_PARAM;
    }

    uint8_t cmd = W25Q_CMD_READ_STATUS1;
    uint8_t response[2];

    w25q_error_t result = w25q_spi_tx(&cmd, 1);
    if (result != W25Q_OK)
    {
        return result;
    }

    result = w25q_spi_rx(response, 2);
    if (result != W25Q_OK)
    {
        return result;
    }

    *status = response[1];
    return W25Q_OK;
}

w25q_error_t w25q_wait_busy(w25q_handle_t *handle, uint32_t timeout_ms)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    uint32_t start_time = 0;
    uint8_t  status;

    while (true)
    {
        w25q_error_t result = w25q_read_status(handle, &status);
        if (result != W25Q_OK)
        {
            return result;
        }

        if (!(status & W25Q_SR_BUSY))
        {
            return W25Q_OK;
        }

        if (start_time == 0)
        {
            start_time = 0; // Would need proper timer implementation
        }

        w25q_delay_ms(1);
    }
}

w25q_error_t w25q_write_enable(w25q_handle_t *handle)
{
    return w25q_send_command(handle, W25Q_CMD_WRITE_ENABLE);
}

w25q_error_t w25q_write_disable(w25q_handle_t *handle)
{
    return w25q_send_command(handle, W25Q_CMD_WRITE_DISABLE);
}

w25q_error_t w25q_read_data(w25q_handle_t *handle, uint32_t address, uint8_t *data, uint16_t size)
{
    if (!handle || !handle->initialized || !data || size == 0)
    {
        return W25Q_ERROR_INVALID_PARAM;
    }

    uint8_t cmd_buffer[4];
    cmd_buffer[0] = W25Q_CMD_FAST_READ;
    cmd_buffer[1] = (uint8_t)((address >> 16) & 0xFF);
    cmd_buffer[2] = (uint8_t)((address >> 8) & 0xFF);
    cmd_buffer[3] = (uint8_t)(address & 0xFF);

    w25q_error_t result = w25q_spi_tx(cmd_buffer, 4);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Send dummy byte for fast read
    uint8_t dummy = 0;
    result        = w25q_spi_tx(&dummy, 1);
    if (result != W25Q_OK)
    {
        return result;
    }

    return w25q_spi_rx(data, size);
}

w25q_error_t
w25q_write_data(w25q_handle_t *handle, uint32_t address, const uint8_t *data, uint16_t size)
{
    if (!handle || !handle->initialized || !data || size == 0)
    {
        return W25Q_ERROR_INVALID_PARAM;
    }

    // Enable write
    w25q_error_t result = w25q_write_enable(handle);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Send page program command with address and data
    result =
        w25q_send_command_with_address_and_data(handle, W25Q_CMD_PAGE_PROGRAM, address, data, size);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Wait for write to complete
    return w25q_wait_busy(handle, 1000);
}

w25q_error_t w25q_erase_sector(w25q_handle_t *handle, uint32_t address)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    // Enable write
    w25q_error_t result = w25q_write_enable(handle);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Send sector erase command
    result = w25q_send_command_with_address(handle, W25Q_CMD_SECTOR_ERASE, address);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Wait for erase to complete
    return w25q_wait_busy(handle, 1000);
}

w25q_error_t w25q_erase_block(w25q_handle_t *handle, uint32_t address)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    // Enable write
    w25q_error_t result = w25q_write_enable(handle);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Send block erase command
    result = w25q_send_command_with_address(handle, W25Q_CMD_BLOCK_ERASE_64K, address);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Wait for erase to complete
    return w25q_wait_busy(handle, 2000);
}

w25q_error_t w25q_erase_chip(w25q_handle_t *handle)
{
    if (!handle || !handle->initialized)
    {
        return W25Q_ERROR_NOT_INITIALIZED;
    }

    // Enable write
    w25q_error_t result = w25q_write_enable(handle);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Send chip erase command
    result = w25q_send_command(handle, W25Q_CMD_CHIP_ERASE);
    if (result != W25Q_OK)
    {
        return result;
    }

    // Wait for erase to complete
    return w25q_wait_busy(handle, 30000);
}

const char *w25q_get_error_string(w25q_error_t error)
{
    switch (error)
    {
    case W25Q_OK:
        return "Success";
    case W25Q_ERROR_INVALID_PARAM:
        return "Invalid parameter";
    case W25Q_ERROR_SPI:
        return "SPI communication error";
    case W25Q_ERROR_TIMEOUT:
        return "Timeout";
    case W25Q_ERROR_WRITE_PROTECTED:
        return "Write protected";
    case W25Q_ERROR_ERASE_FAILED:
        return "Erase failed";
    case W25Q_ERROR_WRITE_FAILED:
        return "Write failed";
    case W25Q_ERROR_READ_FAILED:
        return "Read failed";
    case W25Q_ERROR_NOT_INITIALIZED:
        return "Not initialized";
    case W25Q_ERROR_BUSY:
        return "Device busy";
    default:
        return "Unknown error";
    }
}