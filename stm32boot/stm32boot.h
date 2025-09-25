/**
 * @file    stm32boot.h
 * @brief   STM32 Bootloader Driver Interface
 * @version 1.0.0
 *
 * Platform-agnostic driver for STM32 bootloader operations.
 * Requires communication functions to be assigned by the user program.
 */

#ifndef __STM32BOOT_H__
#define __STM32BOOT_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "misra_config.h"
#include <stdbool.h>

/* STM32 Bootloader Constants */
#define STM32_BOOTLOADER_ACK    0x79U
#define STM32_BOOTLOADER_NACK   0x1FU
#define STM32_BOOTLOADER_GET    0x00U
#define STM32_BOOTLOADER_GET_ID 0x02U
#define STM32_BOOTLOADER_READ   0x11U
#define STM32_BOOTLOADER_WRITE  0x31U
#define STM32_BOOTLOADER_ERASE  0x44U
#define STM32_BOOTLOADER_GO     0x21U

/* STM32 Flash Constants */
#define STM32_FLASH_PAGE_SIZE   2048U /* 2KB pages for STM32G030 */
#define STM32_FLASH_START_ADDR  0x08000000U
#define STM32_FLASH_END_ADDR    0x0800FFFFU /* 64KB for STM32G030C8 */
#define STM32_FLASH_TOTAL_PAGES 32U

    /* STM32 Bootloader Error Codes */
    typedef enum
    {
        STM32BOOT_OK = 0,
        STM32BOOT_ERROR_INVALID_PARAM,
        STM32BOOT_ERROR_COMM_FAILED,
        STM32BOOT_ERROR_TIMEOUT,
        STM32BOOT_ERROR_NOT_ACK,
        STM32BOOT_ERROR_ERASE_FAILED,
        STM32BOOT_ERROR_WRITE_FAILED,
        STM32BOOT_ERROR_READ_FAILED,
        STM32BOOT_ERROR_NOT_INITIALIZED,
        STM32BOOT_ERROR_BUSY,
        STM32BOOT_ERROR_VERIFY_FAILED
    } stm32boot_error_t;

    /* Communication Function Prototypes - User must implement these */
    stm32boot_error_t stm32boot_comm_tx(const uint8_t *data, uint16_t size);
    stm32boot_error_t stm32boot_comm_rx(uint8_t *data, uint16_t size);
    stm32boot_error_t stm32boot_comm_tx_rx(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
    void              stm32boot_delay_ms(uint32_t ms);

    /* GPIO Function Prototypes - User must implement these */
    void stm32boot_gpio_set_high(uint8_t pin);
    void stm32boot_gpio_set_low(uint8_t pin);

    /* Device Handle */
    typedef struct
    {
        bool    initialized;
        uint8_t boot0_pin;
        uint8_t reset_pin;
    } stm32boot_handle_t;

    /* Function Prototypes */

    /**
     * @brief Initialize STM32 bootloader driver
     * @param handle Pointer to device handle
     * @param boot0_pin GPIO pin for STM32 BOOT0 control
     * @param reset_pin GPIO pin for STM32 RESET control
     * @return STM32BOOT_OK if successful, error code otherwise
     */
    stm32boot_error_t
    stm32boot_init(stm32boot_handle_t *handle, uint8_t boot0_pin, uint8_t reset_pin);

    /**
     * @brief Deinitialize STM32 bootloader driver
     * @param handle Pointer to device handle
     */
    void stm32boot_deinit(stm32boot_handle_t *handle);

    /**
     * @brief Enter STM32 bootloader mode
     * @param handle Pointer to device handle
     * @return STM32BOOT_OK if successful, error code otherwise
     */
    stm32boot_error_t stm32boot_enter_bootloader(stm32boot_handle_t *handle);

    /**
     * @brief Erase STM32 flash pages
     * @param handle Pointer to device handle
     * @param start_page Starting page number
     * @param num_pages Number of pages to erase
     * @return STM32BOOT_OK if successful, error code otherwise
     */
    stm32boot_error_t
    stm32boot_erase_pages(stm32boot_handle_t *handle, uint8_t start_page, uint8_t num_pages);

    /**
     * @brief Write data to STM32 flash
     * @param handle Pointer to device handle
     * @param address Flash address to write to
     * @param data Pointer to data to write
     * @param size Number of bytes to write
     * @return STM32BOOT_OK if successful, error code otherwise
     */
    stm32boot_error_t stm32boot_write_data(stm32boot_handle_t *handle,
                                           uint32_t            address,
                                           const uint8_t      *data,
                                           uint16_t            size);

    /**
     * @brief Read data from STM32 flash
     * @param handle Pointer to device handle
     * @param address Flash address to read from
     * @param data Pointer to buffer to store read data
     * @param size Number of bytes to read
     * @return STM32BOOT_OK if successful, error code otherwise
     */
    stm32boot_error_t
    stm32boot_read_data(stm32boot_handle_t *handle, uint32_t address, uint8_t *data, uint16_t size);

    /**
     * @brief Reset STM32 to run new firmware
     * @param handle Pointer to device handle
     * @return STM32BOOT_OK if successful, error code otherwise
     */
    stm32boot_error_t stm32boot_reset(stm32boot_handle_t *handle);

    /**
     * @brief Get error string for error code
     * @param error Error code
     * @return Error string
     */
    const char *stm32boot_get_error_string(stm32boot_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* __STM32BOOT_H__ */
