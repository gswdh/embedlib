#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MODBUS_CRC_PASS = 0U,
    MODBUS_CRC_FAIL,
} modbus_crc_integrity_t;

/* Minimum Modbus RTU message: 1 addr + 1 func + 2 CRC = 4 bytes */
#define MODBUS_MIN_MSG_LEN (4U)

/*
 * Calculate the Modbus RTU CRC-16 for the given data.
 *
 * @param data Pointer to the data buffer
 * @param len  Length of the data in bytes
 * @return     Calculated CRC-16 value
 */
uint16_t modbus_calc_crc(const uint8_t *data, const uint32_t len);

/*
 * Extract the CRC from a Modbus RTU message.
 * The CRC is stored in the last 2 bytes in little-endian order.
 *
 * @param data Pointer to the complete message buffer (including CRC)
 * @param len  Total length of the message in bytes (must be >= 2)
 * @return     Extracted CRC-16 value, or 0 if len < 2
 */
uint16_t modbus_get_crc(const uint8_t *data, const uint32_t len);

/*
 * Verify the CRC integrity of a Modbus RTU message.
 * Calculates CRC over the data portion and compares with the embedded CRC.
 *
 * @param data Pointer to the complete message buffer (including CRC)
 * @param len  Total length of the message in bytes (must be >= 3)
 * @return     MODBUS_CRC_PASS if CRC is valid, MODBUS_CRC_FAIL otherwise
 */
modbus_crc_integrity_t modbus_crc_integrity(const uint8_t *data, const uint32_t len);

/*
 * Extract the device address from a Modbus RTU message.
 * The address is the first byte of the message.
 *
 * @param data Pointer to the message buffer
 * @param len  Length of the message in bytes (must be >= 1)
 * @return     Device address (0-247), or 0 on error
 */
uint8_t modbus_get_address(const uint8_t *data, const uint32_t len);