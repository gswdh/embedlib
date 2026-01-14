/**
 * @file    stm32boot.h
 * @brief   STM32 I²C Bootloader Driver Interface (AN4221 compliant)
 * @version 2.0.0
 *
 * Platform-agnostic driver for STM32 I²C system memory bootloader operations.
 * Implements AN4221 protocol exactly with proper framing, checksums, and BUSY handling.
 *
 * @note MISRA-C:2012 Compliance
 * This module is designed to comply with MISRA-C:2012 guidelines.
 * Key compliance considerations:
 * - Rule 8.6: All external identifiers are declared in exactly one header file
 * - Rule 8.7: All objects and functions with external linkage are declared
 * - Rule 9.1: All automatic variables are assigned a value before use
 * - Rule 10.3: Value of an expression of integer type shall not be implicitly converted
 * - Rule 11.8: A cast shall not remove any const or volatile qualification
 * - Rule 14.4: The controlling expression of an if statement shall have boolean type
 * - Rule 15.3: The goto statement shall not be used
 * - Rule 16.1: All switch statements shall be well-formed
 * - Rule 17.7: The return value of a function having non-void return type shall be used
 * - Rule 18.1: A pointer resulting from arithmetic on a pointer operand shall address an element of
 * the same array
 * - Rule 21.6: The Standard Library input/output functions shall not be used
 */

#ifndef STM32BOOT_H
#define STM32BOOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define STM32BL_BTLDR_IS_ENABLED (*((uint32_t *)(0x1FFF7800)) == 0xDEFFE1AA)

/* AN4221 Protocol Constants */
#define STM32BL_ACK  (0x79u)
#define STM32BL_NACK (0x1Fu)
#define STM32BL_BUSY (0x76u)

/* Command opcodes per AN4221 */
#define STM32BL_CMD_GET          (0x00u)
#define STM32BL_CMD_GET_VERSION  (0x01u)
#define STM32BL_CMD_GET_ID       (0x02u)
#define STM32BL_CMD_READ_MEMORY  (0x11u)
#define STM32BL_CMD_GO           (0x21u)
#define STM32BL_CMD_WRITE_MEMORY (0x31u)
#define STM32BL_CMD_ERASE        (0x44u)
/* No-Stretch variants (if supported by target) */
#define STM32BL_CMD_NOSTRETCH_WRITE   (0x32u)
#define STM32BL_CMD_NOSTRETCH_ERASE   (0x45u)
#define STM32BL_CMD_NOSTRETCH_GET_CRC (0xA1u)

/* Configurable device/flash params (provide sane defaults for STM32G0) */
#ifndef STM32_FLASH_PAGE_SIZE
#define STM32_FLASH_PAGE_SIZE (2048u)
#endif
#ifndef STM32_FLASH_BASE
#define STM32_FLASH_BASE (0x08000000u)
#endif
#ifndef STM32_FLASH_SIZE
#define STM32_FLASH_SIZE (65536u) /* 64KB for STM32G030C8 */
#endif

/* Default I²C address for STM32G0 bootloader (from AN2606) */
#ifndef STM32BL_DEFAULT_I2C_ADDR
#define STM32BL_DEFAULT_I2C_ADDR (0x56u)
#endif

/* Default timeouts (ms) */
#ifndef STM32BL_DEFAULT_INTERFRAME_TIMEOUT
#define STM32BL_DEFAULT_INTERFRAME_TIMEOUT (100u)
#endif
#ifndef STM32BL_DEFAULT_BUSY_POLL_INTERVAL
#define STM32BL_DEFAULT_BUSY_POLL_INTERVAL (5u)
#endif
#ifndef STM32BL_DEFAULT_BUSY_POLL_TIMEOUT
#define STM32BL_DEFAULT_BUSY_POLL_TIMEOUT (500u)
#endif

/* Maximum data length per write command */
#define STM32BL_MAX_WRITE_LEN (256u)

/* Maximum erase items per command */
#define STM32BL_MAX_ERASE_ITEMS (512u)

    /* Status codes */
    typedef enum
    {
        STM32BL_OK = 0,
        STM32BL_ERR_BUSY,
        STM32BL_ERR_INVALID_PARAM,
        STM32BL_ERR_COMM,
        STM32BL_ERR_TIMEOUT,
        STM32BL_ERR_NACK,
        STM32BL_ERR_PROTO,
        STM32BL_ERR_BUSY_TIMEOUT,
        STM32BL_ERR_VERIFY,
        STM32BL_ERR_FLASH_UNLOCK,
        STM32BL_ERR_OB_UNLOCK,
        STM32BL_ERR_OB_GET_CONFIG,
        STM32BL_ERR_OB_PROGRAM,
        STM32BL_ERR_OB_LAUNCH,
        STM32BL_ERR_INVALID,
    } stm32bl_status_t;

    /* Platform-specific function prototypes - must be implemented by caller */
    /**
     * @brief Write data to I²C device
     * @param addr I²C device address (7-bit)
     * @param buf Pointer to data buffer (MISRA Rule 11.8: const preserved)
     * @param len Number of bytes to write (MISRA Rule 10.3: explicit conversion)
     * @param to_ms Timeout in milliseconds
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t
    stm32bl_i2c_write(uint8_t addr, const uint8_t *buf, uint16_t len, uint32_t to_ms);

    /**
     * @brief Read data from I²C device
     * @param addr I²C device address (7-bit)
     * @param buf Pointer to receive buffer (MISRA Rule 18.1: valid array access)
     * @param len Number of bytes to read (MISRA Rule 10.3: explicit conversion)
     * @param to_ms Timeout in milliseconds
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_i2c_read(uint8_t addr, uint8_t *buf, uint16_t len, uint32_t to_ms);

    /**
     * @brief Get system tick count
     * @return System tick count
     */
    uint32_t stm32bl_get_tick(void);

    /**
     * @brief Delay execution for specified time
     * @param ms Delay time in milliseconds (MISRA Rule 10.3: explicit conversion)
     */
    void stm32bl_delay(uint32_t ms);

    /**
     * @brief Set GPIO pin state
     * @param pin GPIO pin number (MISRA Rule 10.3: explicit conversion)
     * @param high Pin state: true=high, false=low (MISRA Rule 14.4: boolean type)
     */
    void stm32bl_gpio_set(uint8_t pin, bool high);

    /* Core protocol functions */
    /**
     * @brief Get bootloader protocol information
     * @param proto_ver Pointer to store protocol version (MISRA Rule 18.1: valid array access)
     * @param cmds Pointer to store supported commands (MISRA Rule 18.1: valid array access)
     * @param num_cmds Pointer to store number of commands (MISRA Rule 18.1: valid array access)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_get(uint8_t *proto_ver, uint8_t *cmds, uint8_t *num_cmds);

    /**
     * @brief Get bootloader version
     * @param proto_ver Pointer to store protocol version (MISRA Rule 18.1: valid array access)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_get_version(uint8_t *proto_ver);

    /**
     * @brief Get device product ID
     * @param pid Pointer to store product ID (MISRA Rule 18.1: valid array access)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_get_id(uint16_t *pid);

    /**
     * @brief Read memory from device
     * @param addr Memory address (MISRA Rule 10.3: explicit conversion)
     * @param dst Pointer to destination buffer (MISRA Rule 18.1: valid array access)
     * @param len Number of bytes to read (MISRA Rule 10.3: explicit conversion)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_read(uint32_t addr, uint8_t *dst, uint16_t len);

    /**
     * @brief Write memory to device
     * @param addr Memory address (MISRA Rule 10.3: explicit conversion)
     * @param src Pointer to source buffer (MISRA Rule 11.8: const preserved)
     * @param len Number of bytes to write (MISRA Rule 10.3: explicit conversion)
     * @param nostretch No-stretch mode flag (MISRA Rule 14.4: boolean type)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_write(uint32_t addr, const uint8_t *src, uint16_t len, bool nostretch);

    /**
     * @brief Erase specific flash pages
     * @param pages Pointer to page numbers array (MISRA Rule 11.8: const preserved)
     * @param count Number of pages to erase (MISRA Rule 10.3: explicit conversion)
     * @param nostretch No-stretch mode flag (MISRA Rule 14.4: boolean type)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_erase_pages(const uint16_t *pages, uint16_t count, bool nostretch);

    /**
     * @brief Erase entire flash memory
     * @param nostretch No-stretch mode flag (MISRA Rule 14.4: boolean type)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_erase_mass(void);

    /**
     * @brief Jump to specified address
     * @param addr Target address (MISRA Rule 10.3: explicit conversion)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_go(uint32_t addr);

    /**
     * @brief Enter bootloader mode via GPIO control
     * @param boot0_pin BOOT0 pin number (MISRA Rule 10.3: explicit conversion)
     * @param reset_pin Reset pin number (MISRA Rule 10.3: explicit conversion)
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_enter_bootloader(uint8_t boot0_pin, uint8_t reset_pin);

    /**
     * @brief Enable bootloader via option bytes configuration
     * @note This function configures STM32 option bytes to enable bootloader access.
     *       The MCU will reset after successful configuration (MISRA Rule 15.3: no goto used).
     * @return Status code (MISRA Rule 17.7: return value must be used)
     */
    stm32bl_status_t stm32bl_enable_bootloader(void);

    /* Utility functions */
    /**
     * @brief Get human-readable error string
     * @param status Error status code (MISRA Rule 10.3: explicit conversion)
     * @return Pointer to error string (MISRA Rule 8.6: external identifier declared)
     */
    const char *stm32bl_get_error_string(stm32bl_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* STM32BOOT_H */