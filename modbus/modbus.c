#include "modbus.h"

#include <stdbool.h>
#include <stddef.h>

/* Modbus RTU CRC-16 polynomial (reflected form of 0x8005) */
#define MODBUS_CRC_POLY 0xA001U

uint16_t modbus_calc_crc(const uint8_t *data, const uint32_t len)
{
    if (data == NULL)
    {
        return 0U;
    }

    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)data[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (crc >> 1U) ^ MODBUS_CRC_POLY;
            }
            else
            {
                crc = crc >> 1U;
            }
        }
    }

    return crc;
}

uint16_t modbus_get_crc(const uint8_t *data, const uint32_t len)
{
    if ((data == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return 0U;
    }

    /* CRC is stored little-endian: low byte first, high byte second */
    const uint16_t crc_lo = (uint16_t)data[len - 2U];
    const uint16_t crc_hi = (uint16_t)data[len - 1U];

    return (crc_hi << 8U) | crc_lo;
}

modbus_crc_integrity_t modbus_crc_integrity(const uint8_t *data, const uint32_t len)
{
    if ((data == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return MODBUS_CRC_FAIL;
    }

    /* Calculate CRC over data portion (excluding the 2-byte CRC at the end) */
    const uint16_t calculated = modbus_calc_crc(data, len - 2U);
    const uint16_t received   = modbus_get_crc(data, len);

    return (calculated == received) ? MODBUS_CRC_PASS : MODBUS_CRC_FAIL;
}

uint8_t modbus_get_address(const uint8_t *data, const uint32_t len)
{
    if ((data == NULL) || (len < MODBUS_MIN_MSG_LEN))
    {
        return 0U;
    }

    return data[0];
}