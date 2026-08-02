#ifndef EL_UART_H
#define EL_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * UART — platform-agnostic interface.
 *
 * Interface only; each target supplies the implementation.  Nothing here
 * includes a vendor header.
 *
 * Controllers are identified by a board-defined instance index of type
 * el_uart_id_t rather than by a hardware handle, so driver and application code
 * never names a USART peripheral directly.  A board publishes its instances
 * as named constants, e.g.
 *
 *     #define BOARD_UART_DEBUG  (0U)
 *     #define BOARD_UART_MOTOR  (1U)
 *
 * Two transfer styles are offered and may be mixed on different instances:
 *
 *   Blocking   el_uart_write() / el_uart_read() return when the transfer completes
 *              or the timeout expires.  Simplest, and enough for bring-up and
 *              for command/response protocols.
 *
 *   Background el_uart_write_async() and el_uart_rx_start() hand the transfer to an
 *              interrupt or DMA engine and report completion through
 *              callbacks.  el_uart_rx_start() runs continuously, which is how a
 *              line-oriented or framed protocol should be received.
 *
 * Conventions shared by every peripheral header in this directory:
 *   - Functions return <mod>_error_t; EL_UART_OK is zero, all failures non-zero.
 *   - Timeouts are milliseconds; EL_UART_TIMEOUT_NONE polls once without
 *     blocking and EL_UART_TIMEOUT_FOREVER waits indefinitely.
 *   - Callbacks run in interrupt context.
 *   - An unsupported optional feature returns EL_UART_ERR_UNSUPPORTED.
 * ------------------------------------------------------------------------- */

/** @brief Board-defined controller index. */
typedef uint32_t el_uart_id_t;

#define EL_UART_TIMEOUT_NONE    (0U)
#define EL_UART_TIMEOUT_FOREVER (0xFFFFFFFFU)

/* -------------------------------------------------------------------------
 * Error codes
 * ------------------------------------------------------------------------- */
typedef enum
{
    EL_UART_OK = 0,

    /* Configuration and arguments */
    EL_UART_ERR_UNSUPPORTED,     /* feature not offered by this platform      */
    EL_UART_ERR_NOT_INITIALISED, /* el_uart_init() has not succeeded for this id */
    EL_UART_ERR_NULL_PARAM,      /* mandatory pointer argument was NULL       */
    EL_UART_ERR_BAD_ID,          /* no such controller on this board          */
    EL_UART_ERR_BAD_PARAM,       /* value outside the encodable range         */
    EL_UART_ERR_BAD_LENGTH,      /* zero-length or oversized transfer         */

    /* Transfers */
    EL_UART_ERR_BUSY,            /* a background transfer is already running  */
    EL_UART_ERR_TIMEOUT,         /* deadline passed before the transfer ended */
    EL_UART_ERR_TRANSMIT,        /* the controller reported a transmit fault  */
    EL_UART_ERR_RECEIVE,         /* the controller reported a receive fault   */
    EL_UART_ERR_LINE             /* framing/parity/noise/overrun on the wire  */
} el_uart_error_t;

/**
 * @brief Line conditions, usable as a bit mask.
 *
 * Reported through the error callback and by el_uart_get_line_errors().  These
 * describe the wire, not the API call, and several can be set at once — an
 * overrun commonly follows a framing error at the wrong baud rate.
 */
typedef enum
{
    EL_UART_LINE_OK      = 0U,
    EL_UART_LINE_PARITY  = (1U << 0U),
    EL_UART_LINE_NOISE   = (1U << 1U),
    EL_UART_LINE_FRAMING = (1U << 2U),
    EL_UART_LINE_OVERRUN = (1U << 3U),
    EL_UART_LINE_BREAK   = (1U << 4U),
    EL_UART_LINE_DMA     = (1U << 5U)
} el_uart_line_t;

/* -------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

typedef enum
{
    EL_UART_DATA_BITS_7 = 7,
    EL_UART_DATA_BITS_8 = 8,
    EL_UART_DATA_BITS_9 = 9
} el_uart_data_bits_t;

typedef enum
{
    EL_UART_PARITY_NONE = 0,
    EL_UART_PARITY_EVEN,
    EL_UART_PARITY_ODD
} el_uart_parity_t;

typedef enum
{
    EL_UART_STOP_BITS_1 = 0,
    EL_UART_STOP_BITS_1_5,
    EL_UART_STOP_BITS_2
} el_uart_stop_bits_t;

typedef enum
{
    EL_UART_FLOW_NONE = 0,
    EL_UART_FLOW_RTS,
    EL_UART_FLOW_CTS,
    EL_UART_FLOW_RTS_CTS
} el_uart_flow_t;

/**
 * @brief Line configuration.
 *
 * @c data_bits counts data bits excluding parity, which is the way datasheets
 * describe the frame; a platform that configures word length including parity
 * adjusts internally.  Setting @c invert_rx or @c invert_tx is unusual and
 * platforms without inversion hardware return EL_UART_ERR_UNSUPPORTED rather
 * than pretending to honour it.
 */
typedef struct
{
    uint32_t         baudrate;
    el_uart_data_bits_t data_bits;
    el_uart_parity_t    parity;
    el_uart_stop_bits_t stop_bits;
    el_uart_flow_t      flow_control;
    bool             invert_rx;
    bool             invert_tx;
    bool             swap_rx_tx;
} el_uart_config_t;

/**
 * @brief Sensible 8N1 defaults at @p baud, for use as a struct initialiser.
 */
#define EL_UART_CONFIG_8N1(baud)                                                                      \
    {                                                                                              \
        (baud), EL_UART_DATA_BITS_8, EL_UART_PARITY_NONE, EL_UART_STOP_BITS_1, EL_UART_FLOW_NONE, false,       \
            false, false                                                                           \
    }

/* -------------------------------------------------------------------------
 * Callbacks.  All run in interrupt context: keep them short, do not block,
 * and do not call a blocking API from inside one.
 * ------------------------------------------------------------------------- */

/**
 * @brief Delivers received bytes from a background receive.
 *
 * @param[in] id   Controller that received.
 * @param[in] data Bytes received; valid only for the duration of the call.
 * @param[in] len  Number of valid bytes.
 * @param[in] ctx  Opaque pointer supplied at registration.
 */
typedef void (*el_uart_rx_callback_t)(const el_uart_id_t id,
                                   const uint8_t  *data,
                                   const uint16_t  len,
                                   void           *ctx);

/** @brief Signals that a el_uart_write_async() transfer has fully drained. */
typedef void (*el_uart_tx_callback_t)(const el_uart_id_t id, void *ctx);

/** @brief Reports a line condition; @p errors is a mask of el_uart_line_t. */
typedef void (*el_uart_error_callback_t)(const el_uart_id_t id, const uint32_t errors, void *ctx);

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure and enable controller @p id.
 *
 * Idempotent for an identical configuration; calling it with a different
 * configuration reconfigures the controller and discards any transfer in
 * progress.
 *
 * @param[in] id  Controller index.
 * @param[in] cfg Line configuration; must not be NULL.
 */
el_uart_error_t el_uart_init(const el_uart_id_t id, const el_uart_config_t *cfg);

/**
 * @brief Disable controller @p id and release its pins and clock.
 */
el_uart_error_t el_uart_deinit(const el_uart_id_t id);

/**
 * @brief Change baud rate without disturbing the rest of the configuration.
 *
 * Any transfer in progress is aborted first: changing the divisor mid-frame
 * corrupts the frame on the wire.
 */
el_uart_error_t el_uart_set_baudrate(const el_uart_id_t id, const uint32_t baudrate);

/* -------------------------------------------------------------------------
 * Blocking transfers
 * ------------------------------------------------------------------------- */

/**
 * @brief Send @p len bytes, returning once the last bit has left the shifter.
 *
 * @param[in] id         Controller index.
 * @param[in] data       Bytes to send; must not be NULL.
 * @param[in] len        Number of bytes; must not be zero.
 * @param[in] timeout_ms Deadline for the whole transfer.
 * @return EL_UART_OK, EL_UART_ERR_TIMEOUT if the deadline passed, or
 *         EL_UART_ERR_BUSY if a background transfer owns the controller.
 */
el_uart_error_t el_uart_write(const el_uart_id_t id,
                        const uint8_t  *data,
                        const uint16_t  len,
                        const uint32_t  timeout_ms);

/**
 * @brief Receive exactly @p len bytes.
 *
 * Returns EL_UART_ERR_TIMEOUT if fewer than @p len bytes arrive in time; the
 * bytes that did arrive are still written to @p data, and @p received says
 * how many.  Pass NULL for @p received if a short read is of no use.
 *
 * @param[in]  id         Controller index.
 * @param[out] data       Destination buffer; must not be NULL.
 * @param[in]  len        Number of bytes wanted; must not be zero.
 * @param[out] received   Bytes actually stored; may be NULL.
 * @param[in]  timeout_ms Deadline for the whole transfer.
 */
el_uart_error_t el_uart_read(const el_uart_id_t id,
                       uint8_t        *data,
                       const uint16_t  len,
                       uint16_t       *received,
                       const uint32_t  timeout_ms);

/**
 * @brief Block until the transmitter is idle, or the deadline passes.
 *
 * Call before cutting power or entering a low-power mode that stops the
 * peripheral clock, so a queued message is not truncated mid-frame.
 */
el_uart_error_t el_uart_flush(const el_uart_id_t id, const uint32_t timeout_ms);

/* -------------------------------------------------------------------------
 * Background transfers
 * ------------------------------------------------------------------------- */

/**
 * @brief Start sending @p len bytes and return immediately.
 *
 * The caller must keep @p data valid and unmodified until the transmit
 * callback fires; the implementation does not copy it.
 *
 * @return EL_UART_ERR_BUSY if a background transmit is already in progress.
 */
el_uart_error_t el_uart_write_async(const el_uart_id_t id, const uint8_t *data, const uint16_t len);

/**
 * @brief Begin continuous background reception into @p buffer.
 *
 * Received bytes are reported through the receive callback as they arrive;
 * reception continues until el_uart_rx_stop(). @p buffer is owned by the
 * implementation until then and must remain valid.
 *
 * @param[in] id   Controller index.
 * @param[in] buffer Storage for the implementation to fill; must not be NULL.
 * @param[in] size   Size of @p buffer in bytes; must not be zero.
 */
el_uart_error_t el_uart_rx_start(const el_uart_id_t id, uint8_t *buffer, const uint16_t size);

/**
 * @brief Stop continuous reception and release the buffer.
 */
el_uart_error_t el_uart_rx_stop(const el_uart_id_t id);

/**
 * @brief Abort any transfer in progress in both directions.
 */
el_uart_error_t el_uart_abort(const el_uart_id_t id);

/* -------------------------------------------------------------------------
 * Callback registration.  Passing NULL for @p callback removes the handler.
 * ------------------------------------------------------------------------- */

el_uart_error_t
el_uart_set_rx_callback(const el_uart_id_t id, const el_uart_rx_callback_t callback, void *ctx);

el_uart_error_t
el_uart_set_tx_callback(const el_uart_id_t id, const el_uart_tx_callback_t callback, void *ctx);

el_uart_error_t
el_uart_set_error_callback(const el_uart_id_t id, const el_uart_error_callback_t callback, void *ctx);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/**
 * @brief Report whether a background transfer currently owns @p id.
 *
 * @param[out] busy Destination; must not be NULL.
 */
el_uart_error_t el_uart_is_busy(const el_uart_id_t id, bool *busy);

/**
 * @brief Read and clear the accumulated line-condition mask.
 *
 * Useful when no error callback is registered.  The returned value is a mask
 * of el_uart_line_t; it is cleared by the read, so each condition is reported
 * once.
 *
 * @param[out] errors Destination; must not be NULL.
 */
el_uart_error_t el_uart_get_line_errors(const el_uart_id_t id, uint32_t *errors);

#ifdef __cplusplus
}
#endif

#endif /* EL_UART_H */
