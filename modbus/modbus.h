#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Minimum Modbus RTU message: 1 addr + 1 func + 2 CRC = 4 bytes */
#define MODBUS_MIN_MSG_LEN (4U)

/* Minimum register request: addr + func + reg_addr(2) + reg_count/value(2) + CRC(2) */
#define MODBUS_MIN_REG_REQ_LEN (8U)

/* Maximum number of registered callbacks */
#define MODBUS_MAX_CALLBACKS (16U)

typedef enum
{
    MODBUS_CRC_PASS = 0U,
    MODBUS_CRC_FAIL,
} modbus_crc_integrity_t;

typedef enum
{
    MODBUS_OK = 0,
    MODBUS_ERR_NULL,
    MODBUS_ERR_LENGTH,
    MODBUS_ERR_CRC,
    MODBUS_ERR_NO_CALLBACK,
    MODBUS_ERR_TABLE_FULL,
    MODBUS_ERR_ILLEGAL_FUNCTION,
    MODBUS_ERR_ILLEGAL_DATA_ADDRESS,
    MODBUS_ERR_ILLEGAL_DATA_VALUE,
} modbus_error_t;

typedef enum
{
    MODBUS_FC_READ_COILS               = 0x01U,
    MODBUS_FC_READ_DISCRETE_INPUTS     = 0x02U,
    MODBUS_FC_READ_HOLDING_REGISTERS   = 0x03U,
    MODBUS_FC_READ_INPUT_REGISTERS     = 0x04U,
    MODBUS_FC_WRITE_SINGLE_COIL        = 0x05U,
    MODBUS_FC_WRITE_SINGLE_REGISTER    = 0x06U,
    MODBUS_FC_WRITE_MULTIPLE_COILS     = 0x0FU,
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 0x10U,
} modbus_func_code_t;

/*
 * Callback invoked by modbus_process when a message with a matching function
 * code is received. msg and len include the full raw RTU frame (addr, func,
 * data, CRC). Use the modbus_get_* helpers to decode fields.
 */
typedef modbus_error_t (*modbus_callback_t)(const uint8_t *msg, const uint32_t len);

/*
 * Calculate the Modbus RTU CRC-16 for the given data.
 */
uint16_t modbus_calc_crc(const uint8_t *data, const uint32_t len);

/*
 * Extract the CRC from a Modbus RTU message (last 2 bytes, little-endian).
 */
uint16_t modbus_get_crc(const uint8_t *data, const uint32_t len);

/*
 * Verify the CRC integrity of a Modbus RTU message.
 */
modbus_crc_integrity_t modbus_crc_integrity(const uint8_t *data, const uint32_t len);

/*
 * Extract the device address (first byte) from a Modbus RTU message.
 */
uint8_t modbus_get_address(const uint8_t *data, const uint32_t len);

/*
 * Extract the function code (second byte) from a Modbus RTU message.
 */
uint8_t modbus_get_func_code(const uint8_t *msg, const uint32_t len);

/*
 * Return a pointer to the payload (bytes after addr and func, before CRC).
 * Returns NULL if msg is NULL or len < MODBUS_MIN_MSG_LEN.
 */
const uint8_t *modbus_get_payload(const uint8_t *msg, const uint32_t len);

/*
 * Return the payload length in bytes (len - 4: excludes addr, func, 2 CRC bytes).
 * Returns 0 if msg is NULL or len < MODBUS_MIN_MSG_LEN.
 */
uint32_t modbus_get_payload_len(const uint8_t *msg, const uint32_t len);

/*
 * Extract the register address from a register request (bytes 2-3, big-endian).
 * Valid for FC01-FC06, FC0F, FC10. Returns 0 on error.
 */
uint16_t modbus_get_register_addr(const uint8_t *msg, const uint32_t len);

/*
 * Extract the register count from a multi-register request (bytes 4-5, big-endian).
 * Valid for FC01-FC04, FC0F, FC10. Returns 0 on error.
 */
uint16_t modbus_get_register_count(const uint8_t *msg, const uint32_t len);

/*
 * Extract the register value from a single-register write request (bytes 4-5, big-endian).
 * Valid for FC05, FC06. Returns 0 on error.
 */
uint16_t modbus_get_register_value(const uint8_t *msg, const uint32_t len);

/*
 * Register a callback for the given Modbus function code. If a callback is
 * already registered for that function code it is replaced.
 *
 * @param func_code  Modbus function code (use modbus_func_code_t values)
 * @param cb         Callback to invoke when a matching message is processed
 * @return           MODBUS_OK, MODBUS_ERR_NULL, or MODBUS_ERR_TABLE_FULL
 */
modbus_error_t modbus_register_callback(const uint8_t func_code, const modbus_callback_t cb);

/*
 * Validate and dispatch a raw Modbus RTU message to its registered callback.
 * Checks length, verifies CRC, then calls the callback registered for the
 * message's function code.
 *
 * @param msg  Pointer to the raw RTU frame
 * @param len  Length of the frame in bytes
 * @return     MODBUS_OK on success, or an error code
 */
modbus_error_t modbus_process(const uint8_t *msg, const uint32_t len);
