/**
 * @file    w25q128jv.h
 * @brief   W25Q128JV SPI Flash Memory Driver
 * @version 1.0.0
 *
 * Platform-agnostic driver for Winbond W25Q128JV SPI flash memory.
 * Requires SPI functions to be assigned by the user program.
 */

#ifndef __W25Q_H__
#define __W25Q_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/* W25Q128JV Device Information */
#define W25Q_MANUFACTURER_ID 0xEF
#define W25Q_DEVICE_ID       0x4018
#define W25Q_PAGE_SIZE       256U
#define W25Q_SECTOR_SIZE     4096U
#define W25Q_BLOCK_SIZE      65536U
#define W25Q_TOTAL_SIZE      16777216U /* 16MB */
#define W25Q_TOTAL_SECTORS   4096U
#define W25Q_TOTAL_BLOCKS    256U

/* Status Register Bits */
#define W25Q_SR_BUSY (1U << 0) /* Write In Progress */
#define W25Q_SR_WEL  (1U << 1) /* Write Enable Latch */
#define W25Q_SR_BP0  (1U << 2) /* Block Protection Bit 0 */
#define W25Q_SR_BP1  (1U << 3) /* Block Protection Bit 1 */
#define W25Q_SR_BP2  (1U << 4) /* Block Protection Bit 2 */
#define W25Q_SR_TB   (1U << 5) /* Top/Bottom Protect */
#define W25Q_SR_SEC  (1U << 6) /* Sector Protect */
#define W25Q_SR_SRP  (1U << 7) /* Status Register Protect */

/* Command Definitions */
#define W25Q_CMD_WRITE_ENABLE    0x06
#define W25Q_CMD_WRITE_DISABLE   0x04
#define W25Q_CMD_READ_STATUS1    0x05
#define W25Q_CMD_READ_STATUS2    0x35
#define W25Q_CMD_READ_STATUS3    0x15
#define W25Q_CMD_WRITE_STATUS1   0x01
#define W25Q_CMD_WRITE_STATUS2   0x31
#define W25Q_CMD_WRITE_STATUS3   0x11
#define W25Q_CMD_READ_DATA       0x03
#define W25Q_CMD_FAST_READ       0x0B
#define W25Q_CMD_PAGE_PROGRAM    0x02
#define W25Q_CMD_SECTOR_ERASE    0x20
#define W25Q_CMD_BLOCK_ERASE_32K 0x52
#define W25Q_CMD_BLOCK_ERASE_64K 0xD8
#define W25Q_CMD_CHIP_ERASE      0xC7
#define W25Q_CMD_POWER_DOWN      0xB9
#define W25Q_CMD_RELEASE_PD      0xAB
#define W25Q_CMD_READ_ID         0x90
#define W25Q_CMD_READ_JEDEC_ID   0x9F
#define W25Q_CMD_READ_UNIQUE_ID  0x4B

    /* Error Codes */
    typedef enum
    {
        W25Q_OK = 0,
        W25Q_ERROR_INVALID_PARAM,
        W25Q_ERROR_SPI,
        W25Q_ERROR_TIMEOUT,
        W25Q_ERROR_WRITE_PROTECTED,
        W25Q_ERROR_ERASE_FAILED,
        W25Q_ERROR_WRITE_FAILED,
        W25Q_ERROR_READ_FAILED,
        W25Q_ERROR_NOT_INITIALIZED,
        W25Q_ERROR_BUSY
    } w25q_error_t;

    /* SPI Function Prototypes - User must implement these */
    w25q_error_t w25q_spi_init(void);
    w25q_error_t w25q_spi_tx(const uint8_t *data, uint16_t size);
    w25q_error_t w25q_spi_rx(uint8_t *data, uint16_t size);
    w25q_error_t w25q_spi_tx_rx(const uint8_t *tx_data,
                                const uint32_t tx_len,
                                uint8_t       *rx_data,
                                const uint32_t rx_len);
    void         w25q_delay_ms(uint32_t ms);

    /* Device Handle */
    typedef struct
    {
        bool initialised;
    } w25q_handle_t;

    /* Function Prototypes */

    /**
     * @brief Initialize W25Q128JV driver
     * @param handle Pointer to device handle
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_init(w25q_handle_t *handle);

    /**
     * @brief Deinitialize W25Q128JV driver
     * @param handle Pointer to device handle
     */
    void w25q_deinit(w25q_handle_t *handle);

    /**
     * @brief Read device ID
     * @param handle Pointer to device handle
     * @param manufacturer_id Pointer to store manufacturer ID
     * @param device_id Pointer to store device ID
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_read_id(w25q_handle_t *handle, uint8_t *manufacturer_id, uint16_t *device_id);

    /**
     * @brief Read status register
     * @param handle Pointer to device handle
     * @param status Pointer to store status register value
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_read_status(w25q_handle_t *handle, uint8_t *status);

    /**
     * @brief Wait for write operation to complete
     * @param handle Pointer to device handle
     * @param timeout_ms Timeout in milliseconds
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_wait_busy(w25q_handle_t *handle, uint32_t timeout_ms);

    /**
     * @brief Enable write operations
     * @param handle Pointer to device handle
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_write_enable(w25q_handle_t *handle);

    /**
     * @brief Disable write operations
     * @param handle Pointer to device handle
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_write_disable(w25q_handle_t *handle);

    /**
     * @brief Read data from flash memory
     * @param handle Pointer to device handle
     * @param address Memory address to read from
     * @param data Pointer to data buffer
     * @param size Number of bytes to read
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t
    w25q_read_data(w25q_handle_t *handle, uint32_t address, uint8_t *data, uint16_t size);

    /**
     * @brief Write data to flash memory (page program)
     * @param handle Pointer to device handle
     * @param address Memory address to write to
     * @param data Pointer to data buffer
     * @param size Number of bytes to write
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t
    w25q_write_data(w25q_handle_t *handle, uint32_t address, const uint8_t *data, uint16_t size);

    /**
     * @brief Erase a sector (4KB)
     * @param handle Pointer to device handle
     * @param address Sector address to erase
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_erase_sector(w25q_handle_t *handle, uint32_t address);

    /**
     * @brief Erase a block (64KB)
     * @param handle Pointer to device handle
     * @param address Block address to erase
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_erase_block(w25q_handle_t *handle, uint32_t address);

    /**
     * @brief Erase entire chip
     * @param handle Pointer to device handle
     * @return W25Q_OK if successful, error code otherwise
     */
    w25q_error_t w25q_erase_chip(w25q_handle_t *handle);

    /**
     * @brief Get error string
     * @param error Error code
     * @return Error string
     */
    const char *w25q_get_error_string(w25q_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q_H__ */
