#ifndef EL_SPI_H
#define EL_SPI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * SPI master — platform-agnostic interface.
 *
 * Interface only; each target supplies the implementation.  Nothing here
 * includes a vendor header.
 *
 * Buses are identified by a board-defined instance index (el_spi_id_t) and chip
 * selects by a separate board-defined index (el_spi_cs_t).  Keeping the two
 * apart is what lets one bus carry several devices without the device driver
 * knowing which GPIO the select line lives on:
 *
 *     #define BOARD_SPI_SENSORS (0U)
 *     #define BOARD_CS_ADXL355  (0U)
 *     #define BOARD_CS_FLASH    (1U)
 *
 * The implementation asserts the named select before the transfer and
 * releases it afterwards.  Where a device needs the select held across
 * several transfers — a command byte followed by a burst read, or a flash
 * page program — bracket them with el_spi_select() and el_spi_deselect() and pass
 * EL_SPI_CS_NONE to the transfers in between.
 *
 * Only master mode is described here.  SPI slave is rare in this repository's
 * drivers and its framing differs enough that it deserves its own interface
 * rather than optional arguments on this one.
 *
 * Conventions shared by every peripheral header in this directory:
 *   - Functions return <mod>_error_t; EL_SPI_OK is zero, all failures non-zero.
 *   - Timeouts are milliseconds; EL_SPI_TIMEOUT_NONE polls once without blocking
 *     and EL_SPI_TIMEOUT_FOREVER waits indefinitely.
 *   - Callbacks run in interrupt context.
 *   - An unsupported optional feature returns EL_SPI_ERR_UNSUPPORTED.
 * ------------------------------------------------------------------------- */

/** @brief Board-defined bus index. */
typedef uint32_t el_spi_id_t;

/** @brief Board-defined chip-select index. */
typedef uint32_t el_spi_cs_t;

/** @brief Perform the transfer without touching any select line. */
#define EL_SPI_CS_NONE ((el_spi_cs_t)0xFFFFFFFFU)

#define EL_SPI_TIMEOUT_NONE    (0U)
#define EL_SPI_TIMEOUT_FOREVER (0xFFFFFFFFU)

/* -------------------------------------------------------------------------
 * Error codes
 * ------------------------------------------------------------------------- */
typedef enum
{
    EL_SPI_OK = 0,

    /* Configuration and arguments */
    EL_SPI_ERR_UNSUPPORTED,     /* feature not offered by this platform       */
    EL_SPI_ERR_NOT_INITIALISED, /* el_spi_init() has not succeeded for this id   */
    EL_SPI_ERR_NULL_PARAM,      /* both tx and rx were NULL                   */
    EL_SPI_ERR_BAD_ID,          /* no such bus on this board                  */
    EL_SPI_ERR_BAD_CS,          /* no such chip select on this board          */
    EL_SPI_ERR_BAD_PARAM,       /* value outside the encodable range          */
    EL_SPI_ERR_BAD_LENGTH,      /* zero-length or oversized transfer          */

    /* Transfers */
    EL_SPI_ERR_BUSY,            /* a background transfer is already running   */
    EL_SPI_ERR_TIMEOUT,         /* deadline passed before the transfer ended  */
    EL_SPI_ERR_TRANSFER,        /* the controller reported a transfer fault   */
    EL_SPI_ERR_OVERRUN          /* receive data was lost                      */
} el_spi_error_t;

/* -------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

/**
 * @brief Clock polarity and phase, in the conventional numbering.
 *
 *   mode 0: CPOL=0 CPHA=0 — idle low,  sample on the rising edge
 *   mode 1: CPOL=0 CPHA=1 — idle low,  sample on the falling edge
 *   mode 2: CPOL=1 CPHA=0 — idle high, sample on the falling edge
 *   mode 3: CPOL=1 CPHA=1 — idle high, sample on the rising edge
 */
typedef enum
{
    EL_SPI_MODE_0 = 0,
    EL_SPI_MODE_1,
    EL_SPI_MODE_2,
    EL_SPI_MODE_3
} el_spi_mode_t;

typedef enum
{
    EL_SPI_BIT_ORDER_MSB_FIRST = 0,
    EL_SPI_BIT_ORDER_LSB_FIRST
} el_spi_bit_order_t;

/**
 * @brief Bus configuration.
 *
 * @c frequency_hz is a ceiling, not an exact request: an implementation picks
 * the fastest prescaler that does not exceed it, because exceeding a device's
 * rated clock is a fault while running slower is merely slow.  Read the value
 * actually chosen back with el_spi_get_frequency() when it matters.
 *
 * @c cs_active_high covers the uncommon device that selects on a high level;
 * the default false gives the usual active-low behaviour.
 */
typedef struct
{
    uint32_t        frequency_hz;
    el_spi_mode_t      mode;
    el_spi_bit_order_t bit_order;
    uint8_t         data_bits;     /* usually 8; 16 where the part needs it */
    bool            cs_active_high;
} el_spi_config_t;

/** @brief Common default: 8-bit, MSB first, mode 0, active-low select. */
#define EL_SPI_CONFIG_MODE0(hz)                                                                       \
    {                                                                                              \
        (hz), EL_SPI_MODE_0, EL_SPI_BIT_ORDER_MSB_FIRST, 8U, false                                       \
    }

/**
 * @brief Signals that a background transfer has finished.
 *
 * Runs in interrupt context.  @p status is EL_SPI_OK, or the error that ended
 * the transfer early.
 */
typedef void (*el_spi_callback_t)(const el_spi_id_t id, const el_spi_error_t status, void *ctx);

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure and enable bus @p id.
 *
 * Idempotent for an identical configuration.  All selects on the bus are left
 * deasserted.
 *
 * @param[in] id  Bus index.
 * @param[in] cfg Bus configuration; must not be NULL.
 */
el_spi_error_t el_spi_init(const el_spi_id_t id, const el_spi_config_t *cfg);

/**
 * @brief Disable bus @p id and release its pins and clock.
 */
el_spi_error_t el_spi_deinit(const el_spi_id_t id);

/**
 * @brief Reconfigure a live bus, for a second device with different timing.
 *
 * Returns EL_SPI_ERR_BUSY rather than reconfiguring underneath a transfer.
 */
el_spi_error_t el_spi_configure(const el_spi_id_t id, const el_spi_config_t *cfg);

/**
 * @brief Read back the clock frequency the hardware actually settled on.
 *
 * @param[out] frequency_hz Destination; must not be NULL.
 */
el_spi_error_t el_spi_get_frequency(const el_spi_id_t id, uint32_t *frequency_hz);

/* -------------------------------------------------------------------------
 * Blocking transfers
 * ------------------------------------------------------------------------- */

/**
 * @brief Full-duplex transfer of @p len bytes.
 *
 * SPI always shifts both directions at once, so this is the primitive and
 * el_spi_write()/el_spi_read() are conveniences over it.  Exactly one of @p tx and
 * @p rx may be NULL: a NULL @p tx clocks out zeros, a NULL @p rx discards
 * what comes back.  @p tx and @p rx may be the same buffer, in which case the
 * received bytes overwrite the transmitted ones.
 *
 * @param[in]  id         Bus index.
 * @param[in]  cs         Chip select to assert, or EL_SPI_CS_NONE.
 * @param[in]  tx         Bytes to send, or NULL.
 * @param[out] rx         Destination for received bytes, or NULL.
 * @param[in]  len        Number of bytes to clock; must not be zero.
 * @param[in]  timeout_ms Deadline for the whole transfer.
 */
el_spi_error_t el_spi_transfer(const el_spi_id_t id,
                         const el_spi_cs_t cs,
                         const uint8_t *tx,
                         uint8_t       *rx,
                         const uint16_t len,
                         const uint32_t timeout_ms);

/**
 * @brief Send @p len bytes and discard what is shifted in.
 */
el_spi_error_t el_spi_write(const el_spi_id_t id,
                      const el_spi_cs_t cs,
                      const uint8_t *tx,
                      const uint16_t len,
                      const uint32_t timeout_ms);

/**
 * @brief Clock out zeros and keep @p len received bytes.
 */
el_spi_error_t el_spi_read(const el_spi_id_t id,
                     const el_spi_cs_t cs,
                     uint8_t       *rx,
                     const uint16_t len,
                     const uint32_t timeout_ms);

/**
 * @brief Send @p tx_len bytes, then read @p rx_len bytes, under one select.
 *
 * The half-duplex register access most devices use: a command or register
 * address out, then the answer back, without releasing the select between.
 * Bytes shifted in during the write phase are discarded, as are bytes shifted
 * out during the read phase.
 */
el_spi_error_t el_spi_write_read(const el_spi_id_t id,
                           const el_spi_cs_t cs,
                           const uint8_t *tx,
                           const uint16_t tx_len,
                           uint8_t       *rx,
                           const uint16_t rx_len,
                           const uint32_t timeout_ms);

/* -------------------------------------------------------------------------
 * Manual chip-select control
 *
 * For sequences that must hold the select across several calls.  Pass
 * EL_SPI_CS_NONE to the transfers in between so they do not toggle it.
 * ------------------------------------------------------------------------- */

/** @brief Assert @p cs and leave it asserted. */
el_spi_error_t el_spi_select(const el_spi_id_t id, const el_spi_cs_t cs);

/** @brief Release @p cs. */
el_spi_error_t el_spi_deselect(const el_spi_id_t id, const el_spi_cs_t cs);

/* -------------------------------------------------------------------------
 * Background transfers
 * ------------------------------------------------------------------------- */

/**
 * @brief Start a full-duplex transfer and return immediately.
 *
 * The caller must keep @p tx and @p rx valid until the callback fires; the
 * implementation does not copy them.  The select, if any, is released by the
 * implementation before the callback runs.
 *
 * @return EL_SPI_ERR_BUSY if a background transfer is already in progress.
 */
el_spi_error_t el_spi_transfer_async(const el_spi_id_t id,
                               const el_spi_cs_t cs,
                               const uint8_t *tx,
                               uint8_t       *rx,
                               const uint16_t len);

/**
 * @brief Abort a background transfer.  The select is released.
 */
el_spi_error_t el_spi_abort(const el_spi_id_t id);

/**
 * @brief Install the completion callback for background transfers.
 *
 * Passing NULL for @p callback removes the handler.
 */
el_spi_error_t el_spi_set_callback(const el_spi_id_t id, const el_spi_callback_t callback, void *ctx);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/**
 * @brief Report whether a transfer currently owns the bus.
 *
 * @param[out] busy Destination; must not be NULL.
 */
el_spi_error_t el_spi_is_busy(const el_spi_id_t id, bool *busy);

#ifdef __cplusplus
}
#endif

#endif /* EL_SPI_H */
