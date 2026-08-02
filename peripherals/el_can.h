#ifndef EL_CAN_H
#define EL_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * CAN — platform-agnostic interface.
 *
 * Interface only; each target supplies the implementation.  Nothing here
 * includes a vendor header.
 *
 * Controllers are identified by a board-defined instance index of type
 * el_can_bus_t.  The instance is deliberately not called can_id_t: on CAN, "id"
 * means the arbitration identifier carried in every frame, and reusing the
 * word for the controller would make every call site ambiguous.
 *
 *     #define BOARD_CAN_VEHICLE (0U)
 *
 * Classic CAN and CAN FD share this interface.  A frame carries its own
 * format flags, so a bus configured for FD can still send classic frames, and
 * a classic-only controller rejects an FD frame with EL_CAN_ERR_UNSUPPORTED
 * rather than silently truncating it.
 *
 * Bring-up is deliberately two-stage — el_can_init() then el_can_start().  Hardware
 * acceptance filters can only be programmed while the controller is out of
 * the bus, so filters are installed between the two calls; a single init
 * would force every implementation to invent a re-entry into configuration
 * mode.
 *
 * Conventions shared by every peripheral header in this directory:
 *   - Functions return <mod>_error_t; EL_CAN_OK is zero, all failures non-zero.
 *   - Timeouts are milliseconds; EL_CAN_TIMEOUT_NONE polls once without blocking
 *     and EL_CAN_TIMEOUT_FOREVER waits indefinitely.
 *   - Callbacks run in interrupt context.
 *   - An unsupported optional feature returns EL_CAN_ERR_UNSUPPORTED.
 * ------------------------------------------------------------------------- */

/** @brief Board-defined controller index. */
typedef uint32_t el_can_bus_t;

#define EL_CAN_TIMEOUT_NONE    (0U)
#define EL_CAN_TIMEOUT_FOREVER (0xFFFFFFFFU)

/**
 * @brief Largest payload a el_can_frame_t can hold.
 *
 * Defaults to the CAN FD maximum.  A classic-only project can define it to 8
 * before including this header and save 56 bytes on every frame it stores,
 * which matters in a receive queue.
 */
#ifndef EL_CAN_MAX_PAYLOAD
#define EL_CAN_MAX_PAYLOAD (64U)
#endif

/** @brief Payload limit of a classic CAN frame. */
#define EL_CAN_CLASSIC_MAX_PAYLOAD (8U)

/** @brief Identifier limits. */
#define EL_CAN_STD_ID_MAX (0x7FFU)
#define EL_CAN_EXT_ID_MAX (0x1FFFFFFFU)

/* -------------------------------------------------------------------------
 * Error codes
 * ------------------------------------------------------------------------- */
typedef enum
{
    EL_CAN_OK = 0,

    /* Configuration and arguments */
    EL_CAN_ERR_UNSUPPORTED,     /* feature not offered by this platform       */
    EL_CAN_ERR_NOT_INITIALISED, /* el_can_init() has not succeeded for this bus  */
    EL_CAN_ERR_NOT_STARTED,     /* controller is configured but off the bus   */
    EL_CAN_ERR_ALREADY_STARTED, /* operation is only valid while stopped      */
    EL_CAN_ERR_NULL_PARAM,      /* mandatory pointer argument was NULL        */
    EL_CAN_ERR_BAD_BUS,         /* no such controller on this board           */
    EL_CAN_ERR_BAD_ID,          /* identifier outside the 11- or 29-bit range */
    EL_CAN_ERR_BAD_LENGTH,      /* payload longer than the format allows      */
    EL_CAN_ERR_BAD_PARAM,       /* value outside the encodable range          */
    EL_CAN_ERR_BAD_BITRATE,     /* no valid timing for the requested bitrate  */
    EL_CAN_ERR_NO_FILTER,       /* no acceptance filter slot left             */

    /* Traffic */
    EL_CAN_ERR_TX_FULL,         /* every transmit mailbox is occupied         */
    EL_CAN_ERR_RX_EMPTY,        /* nothing waiting to be received             */
    EL_CAN_ERR_OVERRUN,         /* a received frame was lost                  */
    EL_CAN_ERR_TIMEOUT,         /* deadline passed before the frame moved     */

    /* Bus condition */
    EL_CAN_ERR_BUS_OFF,         /* controller has taken itself off the bus    */
    EL_CAN_ERR_NO_ACK,          /* no other node acknowledged the frame       */
    EL_CAN_ERR_BUS              /* form, stuff, CRC or bit error on the wire  */
} el_can_error_t;

/* -------------------------------------------------------------------------
 * Frames
 * ------------------------------------------------------------------------- */

/**
 * @brief Frame format and attributes, usable as a bit mask.
 *
 * EL_CAN_FLAG_FD selects the FD frame format; EL_CAN_FLAG_BRS additionally switches
 * to the faster data bitrate for the payload and is meaningless without it.
 * A remote frame (EL_CAN_FLAG_RTR) has no payload and does not exist in FD.
 */
typedef enum
{
    EL_CAN_FLAG_NONE     = 0U,
    EL_CAN_FLAG_EXTENDED = (1U << 0U), /* 29-bit identifier                   */
    EL_CAN_FLAG_RTR      = (1U << 1U), /* remote transmission request         */
    EL_CAN_FLAG_FD       = (1U << 2U), /* CAN FD format                       */
    EL_CAN_FLAG_BRS      = (1U << 3U), /* FD bitrate switch for the payload   */
    EL_CAN_FLAG_ESI      = (1U << 4U)  /* FD error-state indicator, RX only   */
} el_can_flag_t;

/**
 * @brief One CAN frame.
 *
 * @c len is a byte count, not the wire DLC code.  FD payloads only exist in
 * the sizes 0-8, 12, 16, 20, 24, 32, 48 and 64; an implementation rounds a
 * length up to the next valid size and pads, since the wire format has no way
 * to express anything else.
 *
 * @c timestamp is filled on receive when the controller has a timer for it
 * and is left zero otherwise.  It is ignored on transmit.
 */
typedef struct
{
    uint32_t id;                     /* 11- or 29-bit, right-aligned       */
    uint32_t flags;                  /* mask of el_can_flag_t                 */
    uint8_t  len;                    /* payload bytes                      */
    uint8_t  data[EL_CAN_MAX_PAYLOAD];
    uint32_t timestamp;              /* receive time, implementation units */
} el_can_frame_t;

/* -------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

/**
 * @brief Controller operating mode.
 *
 * EL_CAN_MODE_LISTEN_ONLY never drives the bus, not even an acknowledge bit,
 * which is what makes it safe for passive monitoring of a live network.
 * The loopback modes are for self-test: internal loopback needs no
 * transceiver or partner node at all.
 */
typedef enum
{
    EL_CAN_MODE_NORMAL = 0,
    EL_CAN_MODE_LISTEN_ONLY,
    EL_CAN_MODE_LOOPBACK,          /* external: frames appear on the wire     */
    EL_CAN_MODE_LOOPBACK_INTERNAL  /* self-contained, bus untouched           */
} el_can_mode_t;

/**
 * @brief Bus configuration.
 *
 * @c sample_point_permille is the position of the sample point within the bit
 * time, in tenths of a percent — 875 means 87.5%, the value CiA recommends up
 * to 500 kbit/s.  It is a request: the implementation picks the segment
 * lengths that come closest with the clock it has.  Zero means "choose a
 * sensible default", which is the right answer for most boards.
 *
 * @c data_bitrate_bps and the FD fields are ignored unless @c fd_enabled.
 * The data bitrate must be at least the nominal bitrate.
 *
 * @c auto_retransmit off gives single-shot transmission: a frame that loses
 * arbitration or goes unacknowledged is abandoned rather than retried, which
 * is what a time-triggered or best-effort protocol wants.
 */
typedef struct
{
    uint32_t   bitrate_bps;
    uint32_t   data_bitrate_bps;
    uint16_t   sample_point_permille;
    uint16_t   data_sample_point_permille;
    el_can_mode_t mode;
    bool       fd_enabled;
    bool       auto_retransmit;
    bool       auto_bus_off_recovery;
} el_can_config_t;

/** @brief Classic CAN at @p bps, 87.5% sample point, normal mode. */
#define EL_CAN_CONFIG_CLASSIC(bps)                                                                    \
    {                                                                                              \
        (bps), 0U, 875U, 0U, EL_CAN_MODE_NORMAL, false, true, true                                    \
    }

/* -------------------------------------------------------------------------
 * Acceptance filters
 * ------------------------------------------------------------------------- */

/**
 * @brief One acceptance filter, in the id/mask form.
 *
 * A frame is accepted when (frame.id & mask) == (id & mask).  A mask of zero
 * therefore accepts everything, and a mask of EL_CAN_EXT_ID_MAX matches one
 * identifier exactly.
 *
 * With no filters installed, a controller accepts every frame on the bus.
 * That is the useful default for a monitor and the wrong one for a node on a
 * busy network, where unfiltered traffic can swamp the receive path.
 */
typedef struct
{
    uint32_t id;
    uint32_t mask;
    bool     extended;   /* match 29-bit identifiers rather than 11-bit    */
    bool     match_rtr;  /* also accept remote frames matching id/mask     */
} el_can_filter_t;

/** @brief Accept every standard-identifier frame. */
#define EL_CAN_FILTER_ACCEPT_ALL()                                                                    \
    {                                                                                              \
        0U, 0U, false, true                                                                        \
    }

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/**
 * @brief Error-confinement state, in the order severity increases.
 *
 * A node counts its own errors and steps back from the bus as they mount:
 * error-passive nodes stop sending dominant error flags, and a bus-off node
 * has stopped participating entirely.  Watching this transition is the
 * earliest warning of a wiring or termination fault.
 */
typedef enum
{
    EL_CAN_STATE_STOPPED = 0,
    EL_CAN_STATE_ERROR_ACTIVE,
    EL_CAN_STATE_ERROR_WARNING,
    EL_CAN_STATE_ERROR_PASSIVE,
    EL_CAN_STATE_BUS_OFF
} el_can_state_t;

typedef struct
{
    el_can_state_t state;
    uint8_t     tx_error_count;
    uint8_t     rx_error_count;
    uint32_t    rx_overruns;      /* frames dropped for want of room       */
    uint32_t    tx_pending;       /* frames queued but not yet on the wire */
    uint32_t    rx_pending;       /* frames received but not yet read      */
} el_can_status_t;

/* -------------------------------------------------------------------------
 * Callbacks.  All run in interrupt context.
 * ------------------------------------------------------------------------- */

/**
 * @brief Delivers a received frame.
 *
 * @p frame is valid only for the duration of the call; copy what is needed.
 */
typedef void (*el_can_rx_callback_t)(const el_can_bus_t bus, const el_can_frame_t *frame, void *ctx);

/** @brief Signals that a queued frame has been transmitted and acknowledged. */
typedef void (*el_can_tx_callback_t)(const el_can_bus_t bus, void *ctx);

/**
 * @brief Reports a bus error or a change of error-confinement state.
 *
 * @param[in] bus   Controller reporting.
 * @param[in] error What went wrong.
 * @param[in] state The state after the event.
 * @param[in] ctx   Opaque pointer supplied at registration.
 */
typedef void (*el_can_error_callback_t)(const el_can_bus_t   bus,
                                     const el_can_error_t error,
                                     const el_can_state_t state,
                                     void             *ctx);

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure controller @p bus and leave it off the bus.
 *
 * Install acceptance filters after this and before el_can_start().
 *
 * @param[in] bus Controller index.
 * @param[in] cfg Bus configuration; must not be NULL.
 * @return EL_CAN_OK, or EL_CAN_ERR_BAD_BITRATE if the requested timing cannot be
 *         produced from the available clock.
 */
el_can_error_t el_can_init(const el_can_bus_t bus, const el_can_config_t *cfg);

/**
 * @brief Release the controller and its pins and clock.
 */
el_can_error_t el_can_deinit(const el_can_bus_t bus);

/**
 * @brief Join the bus and begin transmitting and receiving.
 *
 * Returns once the controller is synchronised to the bus.
 */
el_can_error_t el_can_start(const el_can_bus_t bus);

/**
 * @brief Leave the bus, discarding anything queued.
 *
 * The configuration survives, so filters can be changed and el_can_start()
 * called again without a full re-init.
 */
el_can_error_t el_can_stop(const el_can_bus_t bus);

/**
 * @brief Recover from bus-off.
 *
 * Needed only when @c auto_bus_off_recovery was false.  Returns
 * EL_CAN_ERR_BAD_PARAM if the controller is not actually bus-off.  Recovery
 * still requires the standard 128 occurrences of 11 recessive bits, so the
 * bus must be healthy again for this to complete.
 */
el_can_error_t el_can_recover(const el_can_bus_t bus);

/* -------------------------------------------------------------------------
 * Acceptance filters
 * ------------------------------------------------------------------------- */

/**
 * @brief Install @p filter in slot @p index.
 *
 * Call while the controller is stopped; returns EL_CAN_ERR_ALREADY_STARTED
 * otherwise.  Slot count is platform-defined — EL_CAN_ERR_NO_FILTER means the
 * hardware has run out.
 *
 * @param[in] bus    Controller index.
 * @param[in] index  Filter slot, counting from zero.
 * @param[in] filter Filter to install; must not be NULL.
 */
el_can_error_t
el_can_set_filter(const el_can_bus_t bus, const uint32_t index, const el_can_filter_t *filter);

/**
 * @brief Remove every filter, returning the controller to accepting all
 *        frames.
 */
el_can_error_t el_can_clear_filters(const el_can_bus_t bus);

/* -------------------------------------------------------------------------
 * Traffic
 * ------------------------------------------------------------------------- */

/**
 * @brief Queue @p frame for transmission.
 *
 * Returns once the frame is accepted by a transmit mailbox, which is not the
 * same as it having reached the wire — CAN is arbitrated, so a low-priority
 * frame can wait indefinitely on a busy bus.  Register a transmit callback,
 * or watch el_can_get_status(), to know it actually went out.
 *
 * @param[in] bus        Controller index.
 * @param[in] frame      Frame to send; must not be NULL.
 * @param[in] timeout_ms How long to wait for a free mailbox.
 * @return EL_CAN_OK, EL_CAN_ERR_TX_FULL if no mailbox freed up in time, or
 *         EL_CAN_ERR_BUS_OFF if the controller is not on the bus.
 */
el_can_error_t
el_can_send(const el_can_bus_t bus, const el_can_frame_t *frame, const uint32_t timeout_ms);

/**
 * @brief Take the oldest frame from the receive queue.
 *
 * An alternative to the receive callback, for a polled main loop.  With
 * EL_CAN_TIMEOUT_NONE this returns EL_CAN_ERR_RX_EMPTY immediately when nothing is
 * waiting, which is the normal way to drain the queue.
 *
 * @param[in]  bus        Controller index.
 * @param[out] frame      Destination; must not be NULL.
 * @param[in]  timeout_ms How long to wait for a frame.
 */
el_can_error_t el_can_receive(const el_can_bus_t bus, el_can_frame_t *frame, const uint32_t timeout_ms);

/**
 * @brief Discard everything queued for transmission.
 */
el_can_error_t el_can_flush_tx(const el_can_bus_t bus);

/* -------------------------------------------------------------------------
 * Callback registration.  Passing NULL for @p callback removes the handler.
 * ------------------------------------------------------------------------- */

el_can_error_t el_can_set_rx_callback(const el_can_bus_t bus, const el_can_rx_callback_t callback, void *ctx);

el_can_error_t el_can_set_tx_callback(const el_can_bus_t bus, const el_can_tx_callback_t callback, void *ctx);

el_can_error_t
el_can_set_error_callback(const el_can_bus_t bus, const el_can_error_callback_t callback, void *ctx);

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/**
 * @brief Read the controller's error counters and confinement state.
 *
 * @param[out] status Destination; must not be NULL.
 */
el_can_error_t el_can_get_status(const el_can_bus_t bus, el_can_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* EL_CAN_H */
