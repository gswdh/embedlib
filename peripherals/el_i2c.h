#ifndef EL_I2C_H
#define EL_I2C_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * I2C — platform-agnostic interface.
 *
 * Interface only; each target supplies the implementation.  Nothing here
 * includes a vendor header.
 *
 * Buses are identified by a board-defined instance index (el_i2c_id_t):
 *
 *     #define BOARD_I2C_SENSORS (0U)
 *     #define BOARD_I2C_POWER   (1U)
 *
 * Device addresses are always 7-bit, right-aligned — 0x28, not 0x50.  The
 * read/write bit is the implementation's business and is never part of an
 * address passed across this interface.  This matters: half the I2C bugs in
 * embedded code come from datasheets that quote the shifted 8-bit form, and
 * one convention stated once here removes the ambiguity from every driver.
 * 10-bit addressing is available where the hardware supports it, selected by
 * the configuration rather than by a different address width in the calls.
 *
 * Most devices in this repository are register files, so the memory-access
 * pair (el_i2c_mem_read / el_i2c_mem_write) is the workhorse: it writes the
 * register pointer, issues a repeated START, and transfers the data as one
 * bus transaction that cannot be interrupted by another master.  Doing this
 * as a separate write then read is not equivalent.
 *
 * Conventions shared by every peripheral header in this directory:
 *   - Functions return <mod>_error_t; EL_I2C_OK is zero, all failures non-zero.
 *   - Timeouts are milliseconds; EL_I2C_TIMEOUT_NONE polls once without blocking
 *     and EL_I2C_TIMEOUT_FOREVER waits indefinitely.
 *   - Callbacks run in interrupt context.
 *   - An unsupported optional feature returns EL_I2C_ERR_UNSUPPORTED.
 * ------------------------------------------------------------------------- */

/** @brief Board-defined bus index. */
typedef uint32_t el_i2c_id_t;

#define EL_I2C_TIMEOUT_NONE    (0U)
#define EL_I2C_TIMEOUT_FOREVER (0xFFFFFFFFU)

/* Standard bus rates, for use as el_i2c_config_t.frequency_hz. */
#define EL_I2C_FREQ_STANDARD  (100000U)
#define EL_I2C_FREQ_FAST      (400000U)
#define EL_I2C_FREQ_FAST_PLUS (1000000U)

/* -------------------------------------------------------------------------
 * Error codes.  The NACK cases are kept apart because they mean different
 * things: an address NACK is usually a missing or unpowered device, while a
 * data NACK is usually a device that is present but busy or unhappy with the
 * transaction.
 * ------------------------------------------------------------------------- */
typedef enum
{
    EL_I2C_OK = 0,

    /* Configuration and arguments */
    EL_I2C_ERR_UNSUPPORTED,     /* feature not offered by this platform       */
    EL_I2C_ERR_NOT_INITIALISED, /* el_i2c_init() has not succeeded for this id   */
    EL_I2C_ERR_NULL_PARAM,      /* mandatory pointer argument was NULL        */
    EL_I2C_ERR_BAD_ID,          /* no such bus on this board                  */
    EL_I2C_ERR_BAD_ADDRESS,     /* address outside the 7- or 10-bit range     */
    EL_I2C_ERR_BAD_PARAM,       /* value outside the encodable range          */
    EL_I2C_ERR_BAD_LENGTH,      /* zero-length or oversized transfer          */

    /* Bus conditions */
    EL_I2C_ERR_NACK_ADDRESS,    /* no device acknowledged the address         */
    EL_I2C_ERR_NACK_DATA,       /* device stopped acknowledging mid-transfer  */
    EL_I2C_ERR_ARBITRATION,     /* lost arbitration to another master         */
    EL_I2C_ERR_BUS,             /* misplaced START/STOP, or SDA stuck low     */
    EL_I2C_ERR_OVERRUN,         /* slave-mode data lost                       */

    /* Transfers */
    EL_I2C_ERR_BUSY,            /* a transfer is already in progress          */
    EL_I2C_ERR_TIMEOUT,         /* deadline passed, or clock stretched too    */
                             /* long                                       */
    EL_I2C_ERR_TRANSFER         /* the controller reported a transfer fault   */
} el_i2c_error_t;

/* -------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

typedef enum
{
    EL_I2C_ADDR_7BIT = 0,
    EL_I2C_ADDR_10BIT
} el_i2c_addr_mode_t;

/**
 * @brief Width of the register pointer used by the memory-access calls.
 *
 * Chosen per call rather than per bus: a single bus commonly carries an
 * 8-bit-pointer sensor and a 16-bit-pointer EEPROM.
 */
typedef enum
{
    EL_I2C_MEM_ADDR_8BIT = 0,
    EL_I2C_MEM_ADDR_16BIT
} el_i2c_mem_addr_size_t;

/**
 * @brief Bus configuration.
 *
 * @c frequency_hz is a ceiling, as on SPI: the implementation picks the
 * fastest timing that does not exceed it.  @c own_address is only meaningful
 * when the controller will act as a slave; leave it zero otherwise.
 */
typedef struct
{
    uint32_t        frequency_hz;
    el_i2c_addr_mode_t addr_mode;
    uint16_t        own_address;
    bool            enable_clock_stretching;
} el_i2c_config_t;

/** @brief Common default: 100 kHz, 7-bit addressing, master only. */
#define EL_I2C_CONFIG_STANDARD()                                                                      \
    {                                                                                              \
        EL_I2C_FREQ_STANDARD, EL_I2C_ADDR_7BIT, 0U, true                                                 \
    }

/* -------------------------------------------------------------------------
 * Callbacks.  All run in interrupt context.
 * ------------------------------------------------------------------------- */

/**
 * @brief Signals that a background master transfer has finished.
 *
 * @param[in] id     Bus that completed.
 * @param[in] status EL_I2C_OK, or the error that ended the transfer.
 * @param[in] len    Bytes actually transferred.
 * @param[in] ctx    Opaque pointer supplied at registration.
 */
typedef void (*el_i2c_callback_t)(const el_i2c_id_t    id,
                               const el_i2c_error_t status,
                               const uint16_t    len,
                               void             *ctx);

/** @brief Slave-mode events. */
typedef enum
{
    EL_I2C_SLAVE_EVENT_ADDRESSED_WRITE = 0, /* master is about to write to us  */
    EL_I2C_SLAVE_EVENT_ADDRESSED_READ,      /* master is about to read from us */
    EL_I2C_SLAVE_EVENT_RX_COMPLETE,         /* the master's write finished     */
    EL_I2C_SLAVE_EVENT_TX_COMPLETE,         /* our reply finished              */
    EL_I2C_SLAVE_EVENT_STOP,                /* STOP seen; transaction over     */
    EL_I2C_SLAVE_EVENT_ERROR                /* bus fault during the exchange   */
} el_i2c_slave_event_t;

/**
 * @brief Reports a slave-mode event.
 *
 * On I2C_SLAVE_EVENT_ADDRESSED_* the handler is expected to install the
 * buffer for the coming phase with el_i2c_slave_set_tx_buffer() or
 * el_i2c_slave_set_rx_buffer() before returning.
 *
 * @param[in] id    Bus the event occurred on.
 * @param[in] event What happened.
 * @param[in] len   Bytes transferred, for the completion events; else zero.
 * @param[in] ctx   Opaque pointer supplied at registration.
 */
typedef void (*el_i2c_slave_callback_t)(const el_i2c_id_t          id,
                                     const el_i2c_slave_event_t event,
                                     const uint16_t          len,
                                     void                   *ctx);

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure and enable bus @p id.
 *
 * Idempotent for an identical configuration.
 *
 * @param[in] id  Bus index.
 * @param[in] cfg Bus configuration; must not be NULL.
 */
el_i2c_error_t el_i2c_init(const el_i2c_id_t id, const el_i2c_config_t *cfg);

/**
 * @brief Disable bus @p id and release its pins and clock.
 */
el_i2c_error_t el_i2c_deinit(const el_i2c_id_t id);

/**
 * @brief Recover a bus wedged by a slave holding SDA low.
 *
 * Clocks up to nine pulses on SCL until the slave releases SDA, then issues a
 * STOP.  The standard escape from a reset that interrupted a device mid-read;
 * without it the bus stays stuck until the slave is power-cycled.  Fails with
 * EL_I2C_ERR_BUS if SDA is still low afterwards.
 */
el_i2c_error_t el_i2c_recover(const el_i2c_id_t id);

/* -------------------------------------------------------------------------
 * Master transfers, blocking
 * ------------------------------------------------------------------------- */

/**
 * @brief Write @p len bytes to @p dev_addr, framed by START and STOP.
 *
 * @param[in] id         Bus index.
 * @param[in] dev_addr   7-bit device address, right-aligned.
 * @param[in] tx         Bytes to send; must not be NULL.
 * @param[in] len        Number of bytes; must not be zero.
 * @param[in] timeout_ms Deadline for the whole transaction.
 * @return EL_I2C_OK, or EL_I2C_ERR_NACK_ADDRESS if nothing answered.
 */
el_i2c_error_t el_i2c_write(const el_i2c_id_t id,
                      const uint16_t dev_addr,
                      const uint8_t *tx,
                      const uint16_t len,
                      const uint32_t timeout_ms);

/**
 * @brief Read @p len bytes from @p dev_addr, framed by START and STOP.
 */
el_i2c_error_t el_i2c_read(const el_i2c_id_t id,
                     const uint16_t dev_addr,
                     uint8_t       *rx,
                     const uint16_t len,
                     const uint32_t timeout_ms);

/**
 * @brief Write then read in one transaction, separated by a repeated START.
 *
 * No STOP is issued between the phases, so no other master can take the bus
 * and move the device's internal pointer in between.
 *
 * @param[in]  id         Bus index.
 * @param[in]  dev_addr   7-bit device address.
 * @param[in]  tx         Bytes to send first; must not be NULL.
 * @param[in]  tx_len     Number of bytes to send; must not be zero.
 * @param[out] rx         Destination for the read phase; must not be NULL.
 * @param[in]  rx_len     Number of bytes to read; must not be zero.
 * @param[in]  timeout_ms Deadline for the whole transaction.
 */
el_i2c_error_t el_i2c_write_read(const el_i2c_id_t id,
                           const uint16_t dev_addr,
                           const uint8_t *tx,
                           const uint16_t tx_len,
                           uint8_t       *rx,
                           const uint16_t rx_len,
                           const uint32_t timeout_ms);

/**
 * @brief Read a register block: pointer write, repeated START, data read.
 *
 * @param[in]  id            Bus index.
 * @param[in]  dev_addr      7-bit device address.
 * @param[in]  mem_addr      Register pointer value.
 * @param[in]  mem_addr_size Width of the pointer on the wire.
 * @param[out] rx            Destination buffer; must not be NULL.
 * @param[in]  len           Bytes to read; must not be zero.
 * @param[in]  timeout_ms    Deadline for the whole transaction.
 */
el_i2c_error_t el_i2c_mem_read(const el_i2c_id_t            id,
                         const uint16_t            dev_addr,
                         const uint16_t            mem_addr,
                         const el_i2c_mem_addr_size_t mem_addr_size,
                         uint8_t                  *rx,
                         const uint16_t            len,
                         const uint32_t            timeout_ms);

/**
 * @brief Write a register block: pointer, then data, in one transaction.
 *
 * Note that many devices impose a page size on writes; splitting a long write
 * at page boundaries is the caller's responsibility, not this layer's.
 */
el_i2c_error_t el_i2c_mem_write(const el_i2c_id_t            id,
                          const uint16_t            dev_addr,
                          const uint16_t            mem_addr,
                          const el_i2c_mem_addr_size_t mem_addr_size,
                          const uint8_t            *tx,
                          const uint16_t            len,
                          const uint32_t            timeout_ms);

/**
 * @brief Check whether a device acknowledges its address.
 *
 * Sends address-plus-write and a STOP, transferring no data.  Used for bus
 * scans and for polling an EEPROM through its write cycle, when it NACKs
 * until the internal write completes.
 *
 * @return EL_I2C_OK if the device answered, EL_I2C_ERR_NACK_ADDRESS if not.
 */
el_i2c_error_t el_i2c_probe(const el_i2c_id_t id, const uint16_t dev_addr, const uint32_t timeout_ms);

/* -------------------------------------------------------------------------
 * Master transfers, background
 * ------------------------------------------------------------------------- */

/**
 * @brief Start a write and return immediately.
 *
 * The caller must keep @p tx valid until the callback fires.
 *
 * @return EL_I2C_ERR_BUSY if a transfer is already in progress.
 */
el_i2c_error_t el_i2c_write_async(const el_i2c_id_t id,
                            const uint16_t dev_addr,
                            const uint8_t *tx,
                            const uint16_t len);

/**
 * @brief Start a read and return immediately.
 */
el_i2c_error_t
el_i2c_read_async(const el_i2c_id_t id, const uint16_t dev_addr, uint8_t *rx, const uint16_t len);

/**
 * @brief Abort the transfer in progress.
 */
el_i2c_error_t el_i2c_abort(const el_i2c_id_t id);

/**
 * @brief Install the completion callback for background master transfers.
 *
 * Passing NULL for @p callback removes the handler.
 */
el_i2c_error_t el_i2c_set_callback(const el_i2c_id_t id, const el_i2c_callback_t callback, void *ctx);

/* -------------------------------------------------------------------------
 * Slave mode
 *
 * Optional across the whole group: a master-only implementation returns
 * EL_I2C_ERR_UNSUPPORTED from all of it.
 * ------------------------------------------------------------------------- */

/**
 * @brief Begin answering to @p own_address as a slave.
 *
 * Events arrive through the slave callback, which must be registered first.
 *
 * @param[in] id           Bus index.
 * @param[in] own_address  7-bit address to answer to.
 */
el_i2c_error_t el_i2c_slave_listen(const el_i2c_id_t id, const uint16_t own_address);

/**
 * @brief Stop answering as a slave.
 */
el_i2c_error_t el_i2c_slave_stop(const el_i2c_id_t id);

/**
 * @brief Install the buffer the master's next read will be served from.
 *
 * Normally called from the callback on EL_I2C_SLAVE_EVENT_ADDRESSED_READ.  The
 * buffer must stay valid until the transaction completes.
 */
el_i2c_error_t el_i2c_slave_set_tx_buffer(const el_i2c_id_t id, const uint8_t *tx, const uint16_t len);

/**
 * @brief Install the buffer the master's next write will be stored into.
 */
el_i2c_error_t el_i2c_slave_set_rx_buffer(const el_i2c_id_t id, uint8_t *rx, const uint16_t len);

/**
 * @brief Install the slave event callback.  NULL removes it.
 */
el_i2c_error_t
el_i2c_set_slave_callback(const el_i2c_id_t id, const el_i2c_slave_callback_t callback, void *ctx);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/**
 * @brief Report whether a transfer currently owns the bus.
 *
 * @param[out] busy Destination; must not be NULL.
 */
el_i2c_error_t el_i2c_is_busy(const el_i2c_id_t id, bool *busy);

#ifdef __cplusplus
}
#endif

#endif /* EL_I2C_H */
