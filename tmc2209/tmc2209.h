/**
 * @file tmc2209.h
 * @brief Step/direction stepper driver for a TMC2209 (or any STEP/DIR driver)
 *        in standalone mode: ENABLE, STEP and DIRECTION only, no UART.
 *
 * Microstepping, current and standstill behaviour are set by the driver's
 * straps. The driver here turns turns and speeds into step counts and step
 * frequencies and hands them to the hardware interface below, which the
 * application implements on its timer and GPIO drivers.
 *
 * Moves:
 *  - tmc_start_rotate_angle(): an exact number of steps (a hardware-counted
 *    burst), ended by the hardware calling tmc_burst_done()
 *  - tmc_go() / tmc_start_ramp(): continuous rotation at a speed, ramped from
 *    tmc_poll()
 * Speeds are RPM of the motor shaft; the sign is the direction.
 */

#ifndef TMC2209_H
#define TMC2209_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TMC_OK = 0,
    TMC_ERROR_INIT,          /* tmc_hw_init() failed */
    TMC_ERROR_INVALID_PARAM, /* zero microsteps / steps per revolution, bad speed */
    TMC_ERROR_HW,            /* the hardware interface refused a step command */
} tmc_error_t;

/* Microsteps per full step, as strapped on MS1/MS2 */
typedef enum
{
    TMC_MICROSTEP_1   = 1,
    TMC_MICROSTEP_2   = 2,
    TMC_MICROSTEP_4   = 4,
    TMC_MICROSTEP_8   = 8,
    TMC_MICROSTEP_16  = 16,
    TMC_MICROSTEP_32  = 32,
    TMC_MICROSTEP_64  = 64,
    TMC_MICROSTEP_128 = 128,
    TMC_MICROSTEP_256 = 256,
} tmc_microstep_t;

/* ------------------------------------------------------------------------ */
/* Hardware interface: implemented by the application                        */
/* ------------------------------------------------------------------------ */

/** @brief Claim the pins and timers. Leave the driver disabled, STEP low. */
tmc_error_t tmc_hw_init(void);

/** @brief Driver enable (the TMC2209's ENN is active low: enable = ENN low) */
void tmc_hw_enable(const bool enable);

/** @brief DIR pin */
void tmc_hw_direction(const bool forward);

/**
 * @brief Continuous STEP pulses at step_hz, 0 stops them (STEP idle low).
 *        Called again with a new frequency while running.
 */
tmc_error_t tmc_hw_step_run(const float step_hz);

/**
 * @brief Exactly `steps` rising edges on STEP at step_hz, then STEP idle low,
 *        and tmc_burst_done() called when the last one is out (any context).
 */
tmc_error_t tmc_hw_step_burst(const uint32_t steps, const float step_hz);

/** @brief Stop any pulses now: STEP idle low, no tmc_burst_done() */
void tmc_hw_step_stop(void);

/** @brief Millisecond tick for the ramp */
uint32_t tmc_hw_tick_ms(void);

/* ------------------------------------------------------------------------ */
/* Provided by the driver, called by the hardware interface                  */
/* ------------------------------------------------------------------------ */

/** @brief The burst started with tmc_hw_step_burst() has ended */
void tmc_burst_done(void);

/* ------------------------------------------------------------------------ */
/* Driver                                                                    */
/* ------------------------------------------------------------------------ */

/**
 * @brief Initialise: hardware interface, geometry; driver disabled, no move.
 * @param microstepping Microsteps per full step as strapped
 * @param steps_per_rev Full steps per motor revolution
 */
tmc_error_t tmc_init(const tmc_microstep_t microstepping, const uint32_t steps_per_rev);

/**
 * @brief Rotate by an exact number of turns (steps rounded to nearest) at a
 *        speed, as a counted burst; the sign of `turns` is the direction and
 *        the driver is enabled for the move only. Ends any move in progress.
 */
tmc_error_t tmc_start_rotate_angle(const float turns, const float turns_per_second);

/**
 * @brief Rotate by an exact number of (micro)steps, signed.
 */
tmc_error_t tmc_move_steps(const int32_t steps, const float steps_per_second);

/**
 * @brief Continuous rotation at speed_rpm now (sign = direction); 0 stops and
 *        disables the driver. Ends any burst in progress.
 */
tmc_error_t tmc_go(const float speed_rpm);

/**
 * @brief Continuous rotation ramped linearly from the current speed to
 *        target_rpm at ramp_rpm_per_sec, from tmc_poll().
 */
tmc_error_t tmc_start_ramp(const float target_rpm, const float ramp_rpm_per_sec);

/** @brief Stop everything now and disable the driver */
void tmc_stop(void);

/** @brief Ramp engine; call every millisecond or so */
void tmc_poll(void);

/** @brief A burst or a continuous rotation is in progress */
bool tmc_is_moving(void);

/** @brief Speed of the continuous rotation (0 during a burst or at rest), RPM */
float tmc_speed_rpm(void);

/** @brief Microsteps per motor revolution, as configured */
uint32_t tmc_steps_per_rev(void);

#endif /* TMC2209_H */
