#ifndef EL_GPIO_H
#define EL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * GPIO — platform-agnostic interface.
 *
 * This header declares an interface only.  There is no gpio.c in embedlib:
 * every target supplies its own implementation (STM32 HAL, ESP-IDF, a
 * register-level port, or a host stub for unit tests) and the drivers in this
 * repository are written against these prototypes alone.  Nothing here
 * includes a vendor header, so a driver that includes it stays portable.
 *
 * Pins are identified by a board-defined index of type el_gpio_pin_t, not by a
 * (port, mask) pair.  The mapping from index to physical pin lives with the
 * implementation — typically a table in a board file — which keeps board
 * wiring out of driver code.  A board is expected to publish its indices as
 * named constants, e.g.
 *
 *     #define BOARD_LED       (0U)
 *     #define BOARD_SENSOR_CS (1U)
 *
 * Conventions shared by every peripheral header in this directory:
 *   - Functions return <mod>_error_t; EL_GPIO_OK is zero, all failures non-zero.
 *   - Value parameters are const-qualified; outputs are trailing pointers.
 *   - el_gpio_init() is idempotent and safe to call more than once.
 *   - An implementation that cannot offer an optional feature returns
 *     EL_GPIO_ERR_UNSUPPORTED rather than failing silently.
 * ------------------------------------------------------------------------- */

/** @brief Board-defined pin index.  Meaning is owned by the implementation. */
typedef uint32_t el_gpio_pin_t;

/** @brief Sentinel for "no pin", e.g. an unpopulated optional signal. */
#define EL_GPIO_PIN_NONE ((el_gpio_pin_t)0xFFFFFFFFU)

/* -------------------------------------------------------------------------
 * Error codes
 * ------------------------------------------------------------------------- */
typedef enum
{
    EL_GPIO_OK = 0,

    EL_GPIO_ERR_UNSUPPORTED,      /* feature not offered by this platform     */
    EL_GPIO_ERR_NOT_INITIALISED,  /* el_gpio_init() has not succeeded yet        */
    EL_GPIO_ERR_NULL_PARAM,       /* mandatory pointer argument was NULL      */
    EL_GPIO_ERR_BAD_PIN,          /* pin index outside the board table        */
    EL_GPIO_ERR_BAD_PARAM,        /* value outside the encodable range        */
    EL_GPIO_ERR_WRONG_MODE,       /* operation invalid for the current mode   */
    EL_GPIO_ERR_NO_RESOURCE,      /* no interrupt line/slot left to allocate  */
    EL_GPIO_ERR_IO                /* the underlying port reported a failure   */
} el_gpio_error_t;

/* -------------------------------------------------------------------------
 * Pin configuration
 * ------------------------------------------------------------------------- */

/** @brief Logic level.  Electrical polarity is a board concern, not this. */
typedef enum
{
    EL_GPIO_LOW  = 0,
    EL_GPIO_HIGH = 1
} el_gpio_state_t;

typedef enum
{
    EL_GPIO_MODE_INPUT = 0,     /* high impedance, readable                   */
    EL_GPIO_MODE_OUTPUT_PP,     /* push-pull output                           */
    EL_GPIO_MODE_OUTPUT_OD,     /* open-drain output; needs a pull-up         */
    EL_GPIO_MODE_ALTERNATE,     /* owned by a peripheral (see cfg.alternate)  */
    EL_GPIO_MODE_ANALOG         /* digital buffer disabled, lowest leakage    */
} el_gpio_mode_t;

typedef enum
{
    EL_GPIO_PULL_NONE = 0,
    EL_GPIO_PULL_UP,
    EL_GPIO_PULL_DOWN
} el_gpio_pull_t;

/** @brief Requested slew rate.  Advisory: platforms map it onto whatever
 *         drive-strength or speed classes they actually have. */
typedef enum
{
    EL_GPIO_SPEED_LOW = 0,
    EL_GPIO_SPEED_MEDIUM,
    EL_GPIO_SPEED_HIGH,
    EL_GPIO_SPEED_VERY_HIGH
} el_gpio_speed_t;

/** @brief Interrupt trigger, usable as a bit mask. */
typedef enum
{
    EL_GPIO_EDGE_NONE    = 0U,
    EL_GPIO_EDGE_RISING  = (1U << 0U),
    EL_GPIO_EDGE_FALLING = (1U << 1U),
    EL_GPIO_EDGE_BOTH    = (EL_GPIO_EDGE_RISING | EL_GPIO_EDGE_FALLING)
} el_gpio_edge_t;

/**
 * @brief Static configuration of one pin.
 *
 * @c alternate is the platform's alternate-function selector and is ignored
 * unless @c mode is EL_GPIO_MODE_ALTERNATE.  @c initial is applied before the
 * pin is switched to an output, so a pin never glitches to the wrong level
 * during bring-up.
 */
typedef struct
{
    el_gpio_mode_t  mode;
    el_gpio_pull_t  pull;
    el_gpio_speed_t speed;
    el_gpio_state_t initial;
    uint32_t     alternate;
} el_gpio_config_t;

/**
 * @brief Edge-interrupt callback.
 *
 * Runs in interrupt context: keep it short, do not block, and do not call
 * back into a blocking peripheral API from it.
 *
 * @param[in] pin  Pin that triggered.
 * @param[in] edge Edge observed; EL_GPIO_EDGE_RISING or EL_GPIO_EDGE_FALLING.
 * @param[in] ctx  Opaque pointer supplied at el_gpio_attach_irq() time.
 */
typedef void (*el_gpio_irq_callback_t)(const el_gpio_pin_t pin, const el_gpio_edge_t edge, void *ctx);

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/**
 * @brief Bring up the GPIO subsystem and apply the board's default pin table.
 *
 * Enables port clocks and configures every pin the board declares.  Outputs
 * are driven to their configured initial level before being enabled.
 * Idempotent: a second call is a no-op that returns EL_GPIO_OK.
 */
el_gpio_error_t el_gpio_init(void);

/**
 * @brief Release the GPIO subsystem, returning pins to their reset state.
 */
el_gpio_error_t el_gpio_deinit(void);

/**
 * @brief Reconfigure a single pin at run time.
 *
 * Optional — a board that configures everything statically in el_gpio_init() may
 * return EL_GPIO_ERR_UNSUPPORTED.  Needed for pins that change role, such as a
 * bus line parked as an input while a device is powered down.
 *
 * @param[in] pin Pin index.
 * @param[in] cfg Configuration to apply; must not be NULL.
 */
el_gpio_error_t el_gpio_configure(const el_gpio_pin_t pin, const el_gpio_config_t *cfg);

/* -------------------------------------------------------------------------
 * Level access
 * ------------------------------------------------------------------------- */

/**
 * @brief Drive an output pin to @p state.
 * @return EL_GPIO_ERR_WRONG_MODE if the pin is not currently an output.
 */
el_gpio_error_t el_gpio_write(const el_gpio_pin_t pin, const el_gpio_state_t state);

/**
 * @brief Sample a pin.
 *
 * Reads the input register, so an open-drain output that is being held low
 * externally reads EL_GPIO_LOW even though it was written high.
 *
 * @param[in]  pin   Pin index.
 * @param[out] state Destination for the level; must not be NULL.
 */
el_gpio_error_t el_gpio_read(const el_gpio_pin_t pin, el_gpio_state_t *state);

/**
 * @brief Invert an output pin's level.
 */
el_gpio_error_t el_gpio_toggle(const el_gpio_pin_t pin);

/* -------------------------------------------------------------------------
 * Edge interrupts
 *
 * Optional across the whole group: a platform without pin interrupts returns
 * EL_GPIO_ERR_UNSUPPORTED from all four functions.
 * ------------------------------------------------------------------------- */

/**
 * @brief Install @p callback for edges on @p pin and enable the interrupt.
 *
 * One callback per pin; attaching again replaces the previous one.  Many
 * MCUs share an interrupt line between pins of the same number across ports,
 * so this may return EL_GPIO_ERR_NO_RESOURCE for a pin that is otherwise valid.
 *
 * @param[in] pin      Pin index; must be configured as an input.
 * @param[in] edge     Edge or edges to trigger on; EL_GPIO_EDGE_NONE is invalid.
 * @param[in] callback Handler, called in interrupt context; must not be NULL.
 * @param[in] ctx      Opaque pointer passed back to @p callback; may be NULL.
 */
el_gpio_error_t el_gpio_attach_irq(const el_gpio_pin_t          pin,
                             const el_gpio_edge_t         edge,
                             const el_gpio_irq_callback_t callback,
                             void                     *ctx);

/**
 * @brief Disable the interrupt on @p pin and forget its callback.
 */
el_gpio_error_t el_gpio_detach_irq(const el_gpio_pin_t pin);

/**
 * @brief Mask or unmask an already-attached interrupt without losing it.
 *
 * Use around a critical section instead of detach/attach, which would drop
 * the callback registration.
 */
el_gpio_error_t el_gpio_set_irq_enabled(const el_gpio_pin_t pin, const bool enabled);

/**
 * @brief Report whether @p pin currently has a handler attached.
 *
 * @param[in]  pin      Pin index.
 * @param[out] attached Destination; must not be NULL.
 */
el_gpio_error_t el_gpio_is_irq_attached(const el_gpio_pin_t pin, bool *attached);

#ifdef __cplusplus
}
#endif

#endif /* EL_GPIO_H */
