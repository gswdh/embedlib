/**
 * @file    stm32boot.c
 * @brief   STM32 I²C Bootloader Driver Implementation (AN4221 compliant)
 * @version 2.0.0
 */

#include "stm32boot.h"
#include <stddef.h>
#include <stdint.h>

#ifdef STM32
#include "stm32g0xx_hal.h"
#endif

/* Calculate XOR checksum of buffer */
// static uint8_t xor_sum(const uint8_t *buf, uint16_t len)
// {
//     uint8_t checksum = 0;
//     for (uint16_t i = 0; i < len; i++)
//     {
//         checksum ^= buf[i];
//     }
//     return checksum;
// }

static stm32bl_status_t stm32bl_write_bytes(const uint8_t *buf, const uint16_t len)
{
    stm32bl_status_t result =
        stm32bl_i2c_write(STM32BL_DEFAULT_I2C_ADDR, buf, len, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
    if (result != STM32BL_OK)
    {
        return result;
    }

    return STM32BL_OK;
}

static stm32bl_status_t stm32bl_read_bytes(uint8_t *buf, const uint16_t len)
{
    stm32bl_status_t result =
        stm32bl_i2c_read(STM32BL_DEFAULT_I2C_ADDR, buf, len, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
    if (result != STM32BL_OK)
    {
        return result;
    }
    return STM32BL_OK;
}

static stm32bl_status_t stm32bl_get_ack(void)
{
    uint8_t          rx_byte = 0U;
    stm32bl_status_t result  = stm32bl_read_bytes(&rx_byte, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }
    else
    {
        if (rx_byte == STM32BL_NACK)
        {
            return STM32BL_ERR_NACK;
        }
        else if (rx_byte == STM32BL_BUSY)
        {
            return STM32BL_ERR_BUSY;
        }
        else if (rx_byte == STM32BL_ACK)
        {
            return STM32BL_OK;
        }
        else
        {
            return STM32BL_ERR_INVALID;
        }
    }

    return STM32BL_ERR_PROTO;
}

static stm32bl_status_t stm32bl_poll_for_ack(const uint32_t timeout_ms)
{
    const uint32_t start_time = stm32bl_get_tick();
    while (stm32bl_get_tick() - start_time < timeout_ms)
    {
        stm32bl_status_t result = stm32bl_get_ack();
        if (result == STM32BL_OK)
        {
            return STM32BL_OK;
        }

        stm32bl_delay(STM32BL_DEFAULT_BUSY_POLL_INTERVAL);
    }
    return STM32BL_ERR_TIMEOUT;
}

static stm32bl_status_t stm32bl_send_cmd(const uint8_t cmd)
{
    const uint8_t write_buf[2] = {cmd, (uint8_t)(~cmd)};
    return stm32bl_write_bytes(write_buf, 2);
}

static stm32bl_status_t stm32bl_send_mass_erase_special(void)
{
    /* The special byte values for mass erase are 0xFF, 0xFF and 0x00 is the XOR sum of the previous
     * two bytes */
    return stm32bl_write_bytes((uint8_t[]){0xFF, 0xFF, 0x00}, 3);
}

/* Public API Implementation */
stm32bl_status_t stm32bl_enable_bootloader(void)
{
#ifdef STM32
    FLASH_OBProgramInitTypeDef option_bytes = {0};
    option_bytes.WRPArea                    = OB_WRPAREA_ZONE_A;

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    HAL_FLASHEx_OBGetConfig(&option_bytes);

    // Modify only what you need (for example, set nBOOT_SEL and BOOT0)
    option_bytes.USERConfig &= ~FLASH_OPTR_nBOOT_SEL;
    HAL_FLASHEx_OBProgram(&option_bytes);

    HAL_FLASH_OB_Launch();

    return STM32BL_OK;
#else
    return STM32BL_ERR_INVALID;
#endif
}

stm32bl_status_t stm32bl_enter_bootloader(uint8_t boot0_pin, uint8_t reset_pin)
{
    /* Set BOOT0 high */
    stm32bl_gpio_set(boot0_pin, true);

    /* Reset STM32 */
    stm32bl_gpio_set(reset_pin, true);
    stm32bl_delay(10); /* 10ms low pulse */
    stm32bl_gpio_set(reset_pin, false);

    /* Wait for bootloader to start */
    stm32bl_delay(100);

    stm32bl_gpio_set(boot0_pin, false);

    /* Verify bootloader communication */
    uint8_t proto_ver = 0U;
    return stm32bl_get_version(&proto_ver);
}

stm32bl_status_t stm32bl_get_version(uint8_t *proto_ver)
{
    if (proto_ver == NULL)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    /* Send GET VERSION command */
    stm32bl_status_t result = STM32BL_OK;

    result = stm32bl_send_cmd(STM32BL_CMD_GET_VERSION);
    if (result != STM32BL_OK)
    {
        return result;
    }
    result = stm32bl_get_ack();
    if (result != STM32BL_OK)
    {
        return result;
    }

    result = stm32bl_read_bytes(proto_ver, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    result = stm32bl_get_ack();
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read final ACK */
    return STM32BL_OK;
}

stm32bl_status_t stm32bl_get_id(uint16_t *pid)
{
    if (pid == NULL)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    stm32bl_status_t result = STM32BL_OK;

    result = stm32bl_send_cmd(STM32BL_CMD_GET_ID);
    if (result != STM32BL_OK)
    {
        return result;
    }

    result = stm32bl_get_ack();
    if (result != STM32BL_OK)
    {
        return result;
    }

    uint8_t read_buf[2] = {0x00, 0x00};
    result              = stm32bl_read_bytes(read_buf, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* product ID */
    result = stm32bl_read_bytes((uint8_t *)pid, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* ACK byte */
    result = stm32bl_get_ack();
    if (result != STM32BL_OK)
    {
        return result;
    }

    return STM32BL_OK;
}

stm32bl_status_t stm32bl_erase_pages(const uint16_t *pages, uint16_t count, bool nostretch)
{
    (void)pages;
    (void)count;
    (void)nostretch;
    return STM32BL_ERR_COMM;
}

stm32bl_status_t stm32bl_erase_mass(void)
{
    stm32bl_status_t result = STM32BL_OK;

    result = stm32bl_send_cmd(STM32BL_CMD_ERASE);
    if (result != STM32BL_OK)
    {
        return result;
    }

    result = stm32bl_get_ack();
    if (result != STM32BL_OK)
    {
        return result;
    }

    result = stm32bl_send_mass_erase_special();
    if (result != STM32BL_OK)
    {
        return result;
    }

    result = stm32bl_poll_for_ack(STM32BL_DEFAULT_BUSY_POLL_TIMEOUT);
    if (result != STM32BL_OK)
    {
        return result;
    }

    return STM32BL_OK;
}

stm32bl_status_t stm32bl_go(uint32_t addr)
{
    (void)addr;
    return STM32BL_ERR_COMM;
}

const char *stm32bl_get_error_string(stm32bl_status_t status)
{
    switch (status)
    {
    case STM32BL_OK:
        return "Success";
    case STM32BL_ERR_INVALID_PARAM:
        return "Invalid parameter";
    case STM32BL_ERR_COMM:
        return "Communication failed";
    case STM32BL_ERR_TIMEOUT:
        return "Timeout";
    case STM32BL_ERR_NACK:
        return "Not acknowledged";
    case STM32BL_ERR_PROTO:
        return "Protocol error";
    case STM32BL_ERR_BUSY_TIMEOUT:
        return "Busy timeout";
    case STM32BL_ERR_VERIFY:
        return "Verification failed";
    case STM32BL_ERR_FLASH_UNLOCK:
        return "Flash unlock failed";
    case STM32BL_ERR_OB_UNLOCK:
        return "Option bytes unlock failed";
    case STM32BL_ERR_OB_GET_CONFIG:
        return "Get option bytes configuration failed";
    case STM32BL_ERR_OB_PROGRAM:
        return "Program option bytes failed";
    case STM32BL_ERR_OB_LAUNCH:
        return "Launch option bytes reload failed";
    default:
        return "Unknown error";
    }
}