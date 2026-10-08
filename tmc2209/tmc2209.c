/**
 * @file tmc2209.c
 * @brief Step/direction stepper driver, standalone TMC2209 (see tmc2209.h)
 */

#include "tmc2209.h"

/* Geometry */
static uint32_t tmc_microsteps = 0U; /* per full step */
static uint32_t tmc_full_steps = 0U; /* per revolution */

/* Continuous rotation */
static float tmc_rpm = 0.0f; /* current speed, 0 at rest */

/* Burst in progress (set before the hardware starts it, cleared by the
 * hardware through tmc_burst_done(), from any context) */
static volatile bool tmc_bursting = false;

/* Linear ramp of the continuous speed */
static bool     tmc_ramp_active = false;
static float    tmc_ramp_from   = 0.0f;
static float    tmc_ramp_to     = 0.0f;
static uint32_t tmc_ramp_start  = 0U; /* ms */
static uint32_t tmc_ramp_length = 0U; /* ms */

static float tmc_abs(const float x) { return (x < 0.0f) ? -x : x; }

static float tmc_rpm_to_step_hz(const float rpm)
{
    return tmc_abs(rpm) * (float)tmc_full_steps * (float)tmc_microsteps / 60.0f;
}

static void tmc_drive(const float rpm)
{
    if (tmc_abs(rpm) < 0.001f)
    {
        tmc_hw_step_stop();
        tmc_hw_enable(false);
        tmc_rpm = 0.0f;
        return;
    }
    tmc_hw_direction(rpm >= 0.0f);
    tmc_hw_enable(true);
    (void)tmc_hw_step_run(tmc_rpm_to_step_hz(rpm));
    tmc_rpm = rpm;
}

/* Whatever is moving stops, nothing disabled yet */
static void tmc_end_move(void)
{
    tmc_ramp_active = false;
    if (tmc_bursting)
    {
        tmc_hw_step_stop();
        tmc_bursting = false;
    }
    tmc_rpm = 0.0f;
}

tmc_error_t tmc_init(const tmc_microstep_t microstepping, const uint32_t steps_per_rev)
{
    if ((microstepping == 0U) || (steps_per_rev == 0U))
    {
        return TMC_ERROR_INVALID_PARAM;
    }
    tmc_microsteps  = (uint32_t)microstepping;
    tmc_full_steps  = steps_per_rev;
    tmc_rpm         = 0.0f;
    tmc_bursting    = false;
    tmc_ramp_active = false;

    const tmc_error_t error = tmc_hw_init();
    if (error != TMC_OK)
    {
        return error;
    }
    tmc_hw_enable(false);
    return TMC_OK;
}

tmc_error_t tmc_move_steps(const int32_t steps, const float steps_per_second)
{
    if (steps_per_second <= 0.0f)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    tmc_end_move();
    tmc_hw_step_stop();
    if (steps == 0)
    {
        tmc_hw_enable(false);
        return TMC_OK;
    }

    const uint32_t count = (steps < 0) ? (uint32_t)(-steps) : (uint32_t)steps;
    tmc_hw_direction(steps >= 0);
    tmc_hw_enable(true);
    tmc_bursting = true;
    if (tmc_hw_step_burst(count, steps_per_second) != TMC_OK)
    {
        tmc_bursting = false;
        tmc_hw_enable(false);
        return TMC_ERROR_HW;
    }
    return TMC_OK;
}

tmc_error_t tmc_start_rotate_angle(const float turns, const float turns_per_second)
{
    const float steps_per_turn = (float)tmc_full_steps * (float)tmc_microsteps;
    const float steps          = turns * steps_per_turn;
    const int32_t count        = (int32_t)((steps < 0.0f) ? (steps - 0.5f) : (steps + 0.5f));
    return tmc_move_steps(count, tmc_abs(turns_per_second) * steps_per_turn);
}

void tmc_burst_done(void)
{
    /* The hardware has put out the last step: driver off */
    tmc_bursting = false;
    tmc_hw_enable(false);
}

tmc_error_t tmc_go(const float speed_rpm)
{
    tmc_end_move();
    tmc_drive(speed_rpm);
    return TMC_OK;
}

tmc_error_t tmc_start_ramp(const float target_rpm, const float ramp_rpm_per_sec)
{
    if (ramp_rpm_per_sec <= 0.0f)
    {
        return TMC_ERROR_INVALID_PARAM;
    }
    if (tmc_bursting)
    {
        tmc_end_move();
    }
    tmc_ramp_from   = tmc_rpm;
    tmc_ramp_to     = target_rpm;
    tmc_ramp_start  = tmc_hw_tick_ms();
    tmc_ramp_length = (uint32_t)((tmc_abs(target_rpm - tmc_rpm) / ramp_rpm_per_sec) * 1000.0f);
    tmc_ramp_active = true;
    if (tmc_ramp_length == 0U)
    {
        tmc_poll();
    }
    return TMC_OK;
}

void tmc_stop(void)
{
    tmc_end_move();
    tmc_hw_step_stop();
    tmc_hw_enable(false);
}

void tmc_poll(void)
{
    if (!tmc_ramp_active)
    {
        return;
    }
    const uint32_t elapsed = tmc_hw_tick_ms() - tmc_ramp_start;
    if (elapsed >= tmc_ramp_length)
    {
        tmc_ramp_active = false;
        tmc_drive(tmc_ramp_to);
        return;
    }
    const float progress = (float)elapsed / (float)tmc_ramp_length;
    tmc_drive(tmc_ramp_from + ((tmc_ramp_to - tmc_ramp_from) * progress));
}

bool tmc_is_moving(void) { return tmc_bursting || (tmc_abs(tmc_rpm) >= 0.001f); }

float tmc_speed_rpm(void) { return tmc_rpm; }

uint32_t tmc_steps_per_rev(void) { return tmc_full_steps * tmc_microsteps; }
