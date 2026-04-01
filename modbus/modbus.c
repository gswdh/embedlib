#include "modbus.h"

#include <stdbool.h>
#include <stddef.h>

/* Modbus RTU CRC-16 polynomial (reflected form of 0x8005) */
#define MODBUS_CRC_POLY 0xA001U

typedef struct
{
    uint8_t           func_code;
    modbus_callback_t cb;
} modbus_cb_entry_t;

static modbus_cb_entry_t s_callbacks[MODBUS_MAX_CALLBACKS];
static uint8_t           s_n_callbacks = 0U;

uint16_t modbus_calc_crc(const uint8_t *const data, const uint32_t len)
{
    if (data == NULL)
    {
        return 0U;
    }

    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0U; i < len; i++)
    {
        crc = (uint16_t)(crc ^ (uint16_t)data[i]);

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ MODBUS_CRC_POLY);
            }
            else
            {
                crc = (uint16_t)(crc >> 1U);
            }
        }
    }

    return crc;
}

uint16_t modbus_get_crc(const uint8_t *const data, const uint32_t len)
{
    if ((data == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return 0U;
    }

    /* CRC is stored little-endian: low byte first, high byte second */
    const uint16_t crc_lo = (uint16_t)data[len - 2U];
    const uint16_t crc_hi = (uint16_t)data[len - 1U];

    return (uint16_t)(((uint16_t)crc_hi << 8U) | (uint16_t)crc_lo);
}

modbus_crc_integrity_t modbus_crc_integrity(const uint8_t *const data, const uint32_t len)
{
    if ((data == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return MODBUS_CRC_FAIL;
    }

    const uint16_t calculated = modbus_calc_crc(data, len - 2U);
    const uint16_t received   = modbus_get_crc(data, len);

    return (calculated == received) ? MODBUS_CRC_PASS : MODBUS_CRC_FAIL;
}

uint8_t modbus_get_address(const uint8_t *const data, const uint32_t len)
{
    if ((data == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return 0U;
    }

    return data[0];
}

uint8_t modbus_get_func_code(const uint8_t *const msg, const uint32_t len)
{
    if ((msg == NULL) || (len < 2U))
    {
        return 0U;
    }

    return msg[1];
}

const uint8_t *modbus_get_payload(const uint8_t *const msg, const uint32_t len)
{
    if ((msg == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return NULL;
    }

    return &msg[2];
}

uint32_t modbus_get_payload_len(const uint8_t *const msg, const uint32_t len)
{
    if ((msg == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return 0U;
    }

    return len - 4U;
}

uint16_t modbus_get_register_addr(const uint8_t *const msg, const uint32_t len)
{
    if ((msg == NULL) || (len < MODBUS_MIN_REG_REQ_LEN))
    {
        return 0U;
    }

    return (uint16_t)(((uint16_t)msg[2] << 8U) | (uint16_t)msg[3]);
}

uint16_t modbus_get_register_count(const uint8_t *const msg, const uint32_t len)
{
    if ((msg == NULL) || (len < MODBUS_MIN_REG_REQ_LEN))
    {
        return 0U;
    }

    return (uint16_t)(((uint16_t)msg[4] << 8U) | (uint16_t)msg[5]);
}

uint16_t modbus_get_register_value(const uint8_t *const msg, const uint32_t len)
{
    if ((msg == NULL) || (len < MODBUS_MIN_REG_REQ_LEN))
    {
        return 0U;
    }

    return (uint16_t)(((uint16_t)msg[4] << 8U) | (uint16_t)msg[5]);
}

modbus_error_t modbus_register_callback(const uint8_t func_code, const modbus_callback_t cb)
{
    if (cb == NULL)
    {
        return MODBUS_ERR_NULL;
    }

    for (uint8_t i = 0U; i < s_n_callbacks; i++)
    {
        if (s_callbacks[i].func_code == func_code)
        {
            s_callbacks[i].cb = cb;
            return MODBUS_OK;
        }
    }

    if (s_n_callbacks >= MODBUS_MAX_CALLBACKS)
    {
        return MODBUS_ERR_TABLE_FULL;
    }

    s_callbacks[s_n_callbacks].func_code = func_code;
    s_callbacks[s_n_callbacks].cb        = cb;
    s_n_callbacks++;

    return MODBUS_OK;
}

modbus_error_t modbus_process(const uint8_t *const msg, const uint32_t len)
{
    if (msg == NULL)
    {
        return MODBUS_ERR_NULL;
    }

    if (len < MODBUS_MIN_MSG_LEN)
    {
        return MODBUS_ERR_LENGTH;
    }

    if (modbus_crc_integrity(msg, len) != MODBUS_CRC_PASS)
    {
        return MODBUS_ERR_CRC;
    }

    const uint8_t func_code = modbus_get_func_code(msg, len);

    for (uint8_t i = 0U; i < s_n_callbacks; i++)
    {
        if (s_callbacks[i].func_code == func_code)
        {
            return s_callbacks[i].cb(msg, len);
        }
    }

    return MODBUS_ERR_NO_CALLBACK;
}
