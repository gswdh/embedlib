/**
 * @file    stm32boot.h
 * @brief   STM32 I²C Bootloader Driver Interface (AN4221 compliant)
 * @version 2.0.0
 *
 * Platform-agnostic driver for STM32 I²C system memory bootloader operations.
 * Implements AN4221 protocol exactly with proper framing, checksums, and BUSY handling.
 */

#ifndef STM32BOOT_H
#define STM32BOOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

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
#define STM32BL_DEFAULT_I2C_ADDR (0xACu)
#endif

/* Default timeouts (ms) */
#ifndef STM32BL_DEFAULT_INTERFRAME_TIMEOUT
#define STM32BL_DEFAULT_INTERFRAME_TIMEOUT (100u)
#endif
#ifndef STM32BL_DEFAULT_BUSY_POLL_INTERVAL
#define STM32BL_DEFAULT_BUSY_POLL_INTERVAL (5u)
#endif
#ifndef STM32BL_DEFAULT_BUSY_POLL_TIMEOUT
#define STM32BL_DEFAULT_BUSY_POLL_TIMEOUT (5000u)
#endif

/* Maximum data length per write command */
#define STM32BL_MAX_WRITE_LEN (256u)

/* Maximum erase items per command */
#define STM32BL_MAX_ERASE_ITEMS (512u)

    /* Status codes */
    typedef enum
    {
        STM32BL_OK = 0,
        STM32BL_ERR_INVALID_PARAM,
        STM32BL_ERR_COMM,
        STM32BL_ERR_TIMEOUT,
        STM32BL_ERR_NACK,
        STM32BL_ERR_PROTO,
        STM32BL_ERR_BUSY_TIMEOUT,
        STM32BL_ERR_VERIFY
    } stm32bl_status_t;

    /* Platform-specific function prototypes - must be implemented by caller */
    stm32bl_status_t
    stm32bl_i2c_write(uint8_t addr, const uint8_t *buf, uint16_t len, uint32_t to_ms);
    stm32bl_status_t stm32bl_i2c_read(uint8_t addr, uint8_t *buf, uint16_t len, uint32_t to_ms);
    void             stm32bl_delay(uint32_t ms);
    void             stm32bl_gpio_set(uint8_t pin, bool high);

    /* Core protocol functions */
    stm32bl_status_t stm32bl_get(uint8_t *proto_ver, uint8_t *cmds, uint8_t *num_cmds);
    stm32bl_status_t stm32bl_get_version(uint8_t *proto_ver);
    stm32bl_status_t stm32bl_get_id(uint16_t *pid);
    stm32bl_status_t stm32bl_read(uint32_t addr, uint8_t *dst, uint16_t len);
    stm32bl_status_t stm32bl_write(uint32_t addr, const uint8_t *src, uint16_t len, bool nostretch);
    stm32bl_status_t stm32bl_erase_pages(const uint16_t *pages, uint16_t count, bool nostretch);
    stm32bl_status_t stm32bl_erase_mass(bool nostretch);
    stm32bl_status_t stm32bl_go(uint32_t addr);
    stm32bl_status_t stm32bl_enter_bootloader(uint8_t boot0_pin, uint8_t reset_pin);

    /* Utility functions */
    const char *stm32bl_get_error_string(stm32bl_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* STM32BOOT_H */