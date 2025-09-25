/**
 * @file    stm32boot.c
 * @brief   STM32 Bootloader Driver Implementation
 * @version 1.0.0
 */

#include "stm32boot.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Private function prototypes */
static stm32boot_error_t stm32boot_send_command(stm32boot_handle_t *handle, uint8_t command);
static stm32boot_error_t stm32boot_wait_ack(stm32boot_handle_t *handle, uint32_t timeout_ms);
static stm32boot_error_t stm32boot_send_ack(stm32boot_handle_t *handle);
static stm32boot_error_t stm32boot_verify_communication(stm32boot_handle_t *handle);

/* Private helper functions */
static stm32boot_error_t stm32boot_send_command(stm32boot_handle_t *handle, uint8_t command)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    return stm32boot_comm_tx(&command, 1);
}

/* Removed unused function stm32boot_send_command_with_data */

static stm32boot_error_t stm32boot_wait_ack(stm32boot_handle_t *handle, uint32_t timeout_ms)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    uint8_t  response;
    uint32_t start_time = 0;

    while (true)
    {
        stm32boot_error_t result = stm32boot_comm_rx(&response, 1);
        if (result != STM32BOOT_OK)
        {
            return result;
        }

        if (response == STM32_BOOTLOADER_ACK)
        {
            return STM32BOOT_OK;
        }

        if (response == STM32_BOOTLOADER_NACK)
        {
            return STM32BOOT_ERROR_NOT_ACK;
        }

        if (start_time == 0)
        {
            start_time = 0; // Would need proper timer implementation
        }

        stm32boot_delay_ms(1);
    }
}

static stm32boot_error_t stm32boot_send_ack(stm32boot_handle_t *handle)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    uint8_t ack = STM32_BOOTLOADER_ACK;
    return stm32boot_comm_tx(&ack, 1);
}

static stm32boot_error_t stm32boot_verify_communication(stm32boot_handle_t *handle)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    /* Send GET command to verify bootloader is responding */
    stm32boot_error_t result = stm32boot_send_command(handle, STM32_BOOTLOADER_GET);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send ACK back */
    return stm32boot_send_ack(handle);
}

/* Public API functions */
stm32boot_error_t stm32boot_init(stm32boot_handle_t *handle, uint8_t boot0_pin, uint8_t reset_pin)
{
    if (handle == NULL)
    {
        return STM32BOOT_ERROR_INVALID_PARAM;
    }

    handle->boot0_pin   = boot0_pin;
    handle->reset_pin   = reset_pin;
    handle->initialized = true;

    return STM32BOOT_OK;
}

void stm32boot_deinit(stm32boot_handle_t *handle)
{
    if (handle != NULL)
    {
        handle->initialized = false;
    }
}

stm32boot_error_t stm32boot_enter_bootloader(stm32boot_handle_t *handle)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    /* Step 1: Set STM32 BOOT0 pin high to enable bootloader mode */
    stm32boot_gpio_set_high(handle->boot0_pin);

    /* Step 2: Reset the STM32 */
    stm32boot_gpio_set_low(handle->reset_pin);
    stm32boot_delay_ms(10); /* 10ms low pulse */
    stm32boot_gpio_set_high(handle->reset_pin);

    /* Step 3: Wait for STM32 to enter bootloader mode */
    stm32boot_delay_ms(100);

    /* Step 4: Verify bootloader communication */
    stm32boot_error_t result = stm32boot_verify_communication(handle);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    return STM32BOOT_OK;
}

stm32boot_error_t
stm32boot_erase_pages(stm32boot_handle_t *handle, uint8_t start_page, uint8_t num_pages)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    if (start_page + num_pages > STM32_FLASH_TOTAL_PAGES)
    {
        return STM32BOOT_ERROR_INVALID_PARAM;
    }

    /* Send erase command */
    stm32boot_error_t result = stm32boot_send_command(handle, STM32_BOOTLOADER_ERASE);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send number of pages to erase */
    uint8_t num_pages_byte = num_pages - 1; /* STM32 expects (N-1) */
    result                 = stm32boot_comm_tx(&num_pages_byte, 1);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send page numbers */
    for (uint8_t i = 0; i < num_pages; i++)
    {
        uint8_t page = start_page + i;
        result       = stm32boot_comm_tx(&page, 1);
        if (result != STM32BOOT_OK)
        {
            return result;
        }
    }

    /* Wait for final ACK */
    return stm32boot_wait_ack(handle, 5000); /* Erase can take time */
}

stm32boot_error_t stm32boot_write_data(stm32boot_handle_t *handle,
                                       uint32_t            address,
                                       const uint8_t      *data,
                                       uint16_t            size)
{
    if (!handle || !handle->initialized || !data || size == 0)
    {
        return STM32BOOT_ERROR_INVALID_PARAM;
    }

    /* Send write command */
    stm32boot_error_t result = stm32boot_send_command(handle, STM32_BOOTLOADER_WRITE);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send address (3 bytes, big-endian) */
    uint8_t addr_bytes[3];
    addr_bytes[0] = (uint8_t)((address >> 16) & 0xFF);
    addr_bytes[1] = (uint8_t)((address >> 8) & 0xFF);
    addr_bytes[2] = (uint8_t)(address & 0xFF);

    result = stm32boot_comm_tx(addr_bytes, 3);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send data length */
    uint8_t length = size - 1; /* STM32 expects (N-1) */
    result         = stm32boot_comm_tx(&length, 1);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send data */
    result = stm32boot_comm_tx(data, size);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for final ACK */
    return stm32boot_wait_ack(handle, 1000);
}

stm32boot_error_t
stm32boot_read_data(stm32boot_handle_t *handle, uint32_t address, uint8_t *data, uint16_t size)
{
    if (!handle || !handle->initialized || !data || size == 0)
    {
        return STM32BOOT_ERROR_INVALID_PARAM;
    }

    /* Send read command */
    stm32boot_error_t result = stm32boot_send_command(handle, STM32_BOOTLOADER_READ);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send address (3 bytes, big-endian) */
    uint8_t addr_bytes[3];
    addr_bytes[0] = (uint8_t)((address >> 16) & 0xFF);
    addr_bytes[1] = (uint8_t)((address >> 8) & 0xFF);
    addr_bytes[2] = (uint8_t)(address & 0xFF);

    result = stm32boot_comm_tx(addr_bytes, 3);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Send number of bytes to read */
    uint8_t length = size - 1; /* STM32 expects (N-1) */
    result         = stm32boot_comm_tx(&length, 1);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = stm32boot_wait_ack(handle, 1000);
    if (result != STM32BOOT_OK)
    {
        return result;
    }

    /* Read data */
    return stm32boot_comm_rx(data, size);
}

stm32boot_error_t stm32boot_reset(stm32boot_handle_t *handle)
{
    if (!handle || !handle->initialized)
    {
        return STM32BOOT_ERROR_NOT_INITIALIZED;
    }

    /* Reset STM32 by pulling RESET pin low, then high */
    stm32boot_gpio_set_low(handle->reset_pin);
    stm32boot_delay_ms(10); /* 10ms low pulse */
    stm32boot_gpio_set_high(handle->reset_pin);

    return STM32BOOT_OK;
}

const char *stm32boot_get_error_string(stm32boot_error_t error)
{
    switch (error)
    {
    case STM32BOOT_OK:
        return "Success";
    case STM32BOOT_ERROR_INVALID_PARAM:
        return "Invalid parameter";
    case STM32BOOT_ERROR_COMM_FAILED:
        return "Communication failed";
    case STM32BOOT_ERROR_TIMEOUT:
        return "Timeout";
    case STM32BOOT_ERROR_NOT_ACK:
        return "Not acknowledged";
    case STM32BOOT_ERROR_ERASE_FAILED:
        return "Erase failed";
    case STM32BOOT_ERROR_WRITE_FAILED:
        return "Write failed";
    case STM32BOOT_ERROR_READ_FAILED:
        return "Read failed";
    case STM32BOOT_ERROR_NOT_INITIALIZED:
        return "Not initialized";
    case STM32BOOT_ERROR_BUSY:
        return "Device busy";
    case STM32BOOT_ERROR_VERIFY_FAILED:
        return "Verification failed";
    default:
        return "Unknown error";
    }
}
