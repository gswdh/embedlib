/**
 * @file    tmc2209.c
 * @brief   TMC2209 stepper motor driver implementation
 * @version 2.0.0
 *
 * This file contains the complete implementation of the TMC2209 stepper motor
 * driver library, including UART communication, motor control, and speed ramping.
 *
 * The TMC2209 is a silent stepper motor driver with advanced features like
 * StealthChop, SpreadCycle, and stall detection.
 */

#include "tmc2209.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stats.h"

#define TMC2209_VACTUAL_SCALE_SHIFT (24U)
#define TMC2209_VACTUAL_MAX         (INT32_C(8388607))
#define TMC2209_FCLK_DEFAULT_HZ     (12000000UL)

/* ========================================================================== */
/* GLOBAL VARIABLES                                                           */
/* ========================================================================== */

/**
 * @brief Global TMC2209 ramping structure
 *
 * Contains all ramping parameters including state variables.
 */
static tmc_ramp_t tmc_ramp = {0};

/**
 * @brief Global TMC2209 node address
 */
static uint8_t tmc_node_address = 0;

static tmc_microstep_t tmc_microstepping = TMC_MICROSTEP_16;
static uint32_t        tmc_steps_per_rev = 400U;

static uint32_t tmc_step_count  = 0U;
static bool     tmc_is_stepping = false;

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

/* Implements the CRC8 polynomial 0x07 as specified in the TMC2209 datasheet */
static uint8_t tmc_uart_calc_crc(const uint8_t *datagram, const uint32_t len)
{
    uint8_t crc = 0;

    for (uint32_t i = 0; i < (len - 1); i++)
    {
        uint8_t current = datagram[i];

        for (uint32_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (current & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            current = current >> 1;
        }
    }

    return crc;
}

/* Sends a write datagram containing register address and data to the TMC2209 */
static tmc_error_t tmc_write_datagram(const tmc_write_datagram_t *tx)
{
    return tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_write_datagram_t));
}

/* Sends a read datagram and receives the response from TMC2209 */
static tmc_error_t tmc_read_datagram(const tmc_read_datagram_t *tx, tmc_write_datagram_t *rx)
{
    tmc_error_t error = tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_read_datagram_t));
    if (error != TMC_OK)
    {
        return error;
    }

    /* Discard the echo of the read datagram */
    tmc_uart_rx((uint8_t *)rx, (uint32_t)sizeof(tmc_read_datagram_t), TMC_UART_TIMEOUT_MS);

    /* Receive the actual response datagram */
    return tmc_uart_rx((uint8_t *)rx, (uint32_t)sizeof(tmc_write_datagram_t), TMC_UART_TIMEOUT_MS);
}

/* Internal speed ramping engine - handles non-blocking speed ramping process */
static void tmc_ramp_engine(void)
{
    /* Check if ramping is active */
    if (tmc_ramp.ramping_active == false)
    {
        return;
    }

    /* Rate limit: run every 10ms */
    const uint32_t tick = tmc_get_tick();
    if (tick < (tmc_ramp.ramp_tick + 10U))
    {
        return;
    }
    tmc_ramp.ramp_tick = tick;

    /* Check if ramping is complete */
    if (tick > tmc_ramp.ramp_end_tick)
    {
        /* Set final target speed and disable ramping */
        tmc_go(tmc_ramp.target_speed_rpm);
        tmc_ramp.ramping_active = false;
        return;
    }

    /* Calculate ramping progress (0.0 to 1.0) */
    const double completeness = ((double)tick - (double)tmc_ramp.ramp_start_tick) /
                                ((double)tmc_ramp.ramp_end_tick - (double)tmc_ramp.ramp_start_tick);

    /* Interpolate between start and target speed */
    const double new_speed = stats_linear_interp(
        (double)tmc_ramp.start_speed_rpm, (double)tmc_ramp.target_speed_rpm, completeness);

    /* Apply the interpolated speed */
    tmc_go((const float)new_speed);
}

/* Returns VACTUAL clamped to the valid signed 24-bit range.
 * speed_rpm: target speed in RPM (can be negative for reverse)
 * full_steps_per_rev: motor full steps per mechanical revolution (e.g., 400 for 0.9°)
 * fclk_hz: TMC2209 clock in Hz (12,000,000 by default)
 */
/* Compute step frequency from RPM */
static float tmc2209_compute_step_frequency_rpm(float speed_rpm, uint16_t full_steps_per_rev)
{
    /* Calculate step frequency: (RPM * full_steps_per_rev * microsteps_per_step) / 60 */
    return (float)fabs((speed_rpm * (float)full_steps_per_rev * (float)256.0f) / 1200.0f);
}

/* ========================================================================== */
/* REGISTER READ/WRITE FUNCTIONS                                              */
/* ========================================================================== */

/* Writes a 32-bit value to the specified TMC2209 register */
tmc_error_t tmc_write_reg(const uint8_t node, const uint8_t addr, const uint32_t value)
{
    /* Construct write datagram */
    tmc_write_datagram_t tx = {
        .sync       = TMC_SYNC_WORD,
        .slave      = node,
        .addr.idx   = addr,
        .addr.write = TMC_WRITE_BIT,
        .payload    = value,
    };

    /* Convert payload to big-endian format */
    tx.payload = __builtin_bswap32(tx.payload);

    /* Calculate and set CRC checksum */
    tx.crc = tmc_uart_calc_crc((const uint8_t *)&tx, (uint32_t)sizeof(tmc_write_datagram_t));

    /* Transmit datagram via UART */
    return tmc_write_datagram((const tmc_write_datagram_t *)&tx);
}

/* Reads a 32-bit value from the specified TMC2209 register */
tmc_error_t tmc_read_reg(const uint8_t node, const uint8_t addr, uint32_t *value)
{
    /* Construct read datagram */
    tmc_read_datagram_t tx = {
        .sync       = TMC_SYNC_WORD,
        .slave      = node,
        .addr.idx   = addr,
        .addr.write = TMC_READ_BIT,
    };

    /* Calculate and set CRC checksum */
    tx.crc = tmc_uart_calc_crc((const uint8_t *)&tx, (uint32_t)sizeof(tmc_read_datagram_t));

    /* Transmit read request and receive response */
    tmc_write_datagram_t rx    = {0};
    tmc_error_t          error = tmc_read_datagram((const tmc_read_datagram_t *)&tx, &rx);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Verify CRC checksum */
    uint8_t crc = tmc_uart_calc_crc((const uint8_t *)&rx, (uint32_t)sizeof(tmc_write_datagram_t));
    if (crc != rx.crc)
    {
        return TMC_ERROR_CRC;
    }

    /* Validate response address */
    if ((rx.slave != 0xFF) || (rx.addr.idx != tx.addr.idx))
    {
        return TMC_ERROR_INVALID_RX;
    }

    /* Extract and convert payload from big-endian format */
    *value = __builtin_bswap32(rx.payload);

    return TMC_OK;
}

void tmc_step_timer_callback(void)
{
    if ((tmc_step_count > 0u) && (tmc_is_stepping == true))
    {
        tmc_step_count--;
    }
}

/* ========================================================================== */
/* INITIALIZATION AND CONFIGURATION FUNCTIONS                                */
/* ========================================================================== */

/* Initialize TMC2209 driver with default settings and verify communication */
tmc_error_t tmc_init(const uint8_t         serial_address,
                     const tmc_microstep_t microstepping,
                     const uint32_t        steps_per_rev)
{
    /* Initialize hardware interface */
    tmc_error_t error = tmc_hw_init();
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set TMC2209 node address */
    error = tmc_set_address(serial_address);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Store node address */
    tmc_node_address = serial_address;

    /* Store steps per revolution */
    tmc_steps_per_rev = steps_per_rev;

    /* Initialize ramping state */
    memset(&tmc_ramp, 0, sizeof(tmc_ramp_t));
    tmc_ramp.ramping_active = false;

    /* Verify communication */
    if (tmc_is_communicating() == false)
    {
        return TMC_ERROR_INIT;
    }

    /* Get version needs a delay for some reason */
    tmc_delay_ms(1U);

    /* Configure external sense resistors */
    error = tmc_use_external_sense_resistors();
    if (error != TMC_OK)
    {
        return TMC_ERROR_INIT;
    }

    /* Set hold current to 10% */
    error = tmc_set_hold_current(10U);
    if (error != TMC_OK)
    {
        return TMC_ERROR_INIT;
    }

    /* Set run current to 100% */
    error = tmc_set_run_current(100U);
    if (error != TMC_OK)
    {
        return TMC_ERROR_INIT;
    }

    /* Set microstep resolution from parameter */
    tmc_microstepping = microstepping;
    error             = tmc_set_microsteps_per_step(microstepping);
    if (error != TMC_OK)
    {
        return TMC_ERROR_INIT;
    }

    return TMC_OK;
}

/* Set TMC2209 node address by configuring MS1 and MS2 pins */
tmc_error_t tmc_set_address(uint8_t address)
{
    /* Validate address range (0-3) */
    if (address > 3)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Set MS1 and MS2 pins based on binary address value */
    bool ms1 = (address & 0x01) != 0; /* LSB */
    bool ms2 = (address & 0x02) != 0; /* MSB */

    /* Set the MS1 and MS2 pins to configure the node address */
    tmc_gpio_ms1_write(ms1);
    tmc_gpio_ms2_write(ms2);

    return TMC_OK;
}

/* ========================================================================== */
/* MICROSTEP CONFIGURATION FUNCTIONS                                          */
/* ========================================================================== */

tmc_error_t tmc_set_microsteps_per_step(tmc_microstep_t microsteps)
{
    /* Validate microsteps per step (must be power of 2, 1-256) */
    if (microsteps == 0 || microsteps > 256)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Check if it's a power of 2 */
    if ((microsteps & (microsteps - 1)) != 0)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Calculate the microstep register value */
    uint8_t  mres = 0;
    uint16_t temp = (uint16_t)microsteps;
    while (temp > 1)
    {
        temp >>= 1;
        mres++;
    }

    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t result   = tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Update MRES field (bits 27-24) */
    chopconf &= ~(0x0F << 24); /* Clear MRES field */
    chopconf |= (mres << 24);  /* Set MRES field */

    /* Update global microstepping variable */
    tmc_microstepping = microsteps;

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_CHOPCONF, chopconf);
}

tmc_error_t tmc_set_microsteps_per_step_power_of_two(uint8_t exponent)
{
    if (exponent > 8)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint16_t microsteps = 1 << exponent;
    return tmc_set_microsteps_per_step(microsteps);
}

/* ========================================================================== */
/* CURRENT CONTROL FUNCTIONS                                                   */
/* ========================================================================== */

tmc_error_t tmc_set_run_current(uint8_t percent)
{
    if (percent > 100)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Convert percentage to register value (0-31) */
    uint8_t irun = (percent * 31) / 100;
    if (irun == 0 && percent > 0)
    {
        irun = 1; /* Minimum value */
    }

    /* Read current IHOLD_IRUN register */
    uint32_t    ihold_irun = 0;
    tmc_error_t result     = tmc_read_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, &ihold_irun);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Update IRUN field (bits 12-8) */
    ihold_irun &= ~(0x1F << 8); /* Clear IRUN field */
    ihold_irun |= (irun << 8);  /* Set IRUN field */

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
}

tmc_error_t tmc_set_hold_current(uint8_t percent)
{
    if (percent > 100)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Convert percentage to register value (0-31) */
    uint8_t ihold = (percent * 31) / 100;
    if (ihold == 0 && percent > 0)
    {
        ihold = 1; /* Minimum value */
    }

    /* Read current IHOLD_IRUN register */
    uint32_t    ihold_irun = 0;
    tmc_error_t result     = tmc_read_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, &ihold_irun);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Update IHOLD field (bits 4-0) */
    ihold_irun &= ~0x1F; /* Clear IHOLD field */
    ihold_irun |= ihold; /* Set IHOLD field */

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
}

tmc_error_t tmc_set_hold_delay(uint8_t percent)
{
    if (percent > 100)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Convert percentage to register value (0-15) */
    uint8_t iholddelay = (percent * 15) / 100;

    /* Read current IHOLD_IRUN register */
    uint32_t    ihold_irun = 0;
    tmc_error_t result     = tmc_read_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, &ihold_irun);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Update IHOLDDELAY field (bits 16-20) */
    ihold_irun &= ~(0x0F << 16);      /* Clear IHOLDDELAY field */
    ihold_irun |= (iholddelay << 16); /* Set IHOLDDELAY field */

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
}

tmc_error_t tmc_set_all_current_values(uint8_t run_current_percent,
                                       uint8_t hold_current_percent,
                                       uint8_t hold_delay_percent)
{
    tmc_error_t result;

    /* Set run current */
    result = tmc_set_run_current(run_current_percent);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Set hold current */
    result = tmc_set_hold_current(hold_current_percent);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Set hold delay */
    result = tmc_set_hold_delay(hold_delay_percent);
    if (result != TMC_OK)
    {
        return result;
    }

    return TMC_OK;
}

tmc_error_t tmc_set_rms_current(uint16_t mA, float r_sense, float hold_multiplier)
{
    if (r_sense <= 0.0f || hold_multiplier <= 0.0f || hold_multiplier > 1.0f)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Calculate run current setting */
    float   irun_float = (mA * r_sense * 32.0f) / (325.0f * 1.414f);
    uint8_t irun       = (uint8_t)irun_float;
    if (irun > 31)
        irun = 31;
    if (irun == 0 && mA > 0)
        irun = 1;

    /* Calculate hold current setting */
    float   ihold_float = irun_float * hold_multiplier;
    uint8_t ihold       = (uint8_t)ihold_float;
    if (ihold > 31)
        ihold = 31;
    if (ihold == 0 && mA > 0)
        ihold = 1;

    /* Read current IHOLD_IRUN register */
    uint32_t    ihold_irun = 0;
    tmc_error_t result     = tmc_read_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, &ihold_irun);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Update both IRUN and IHOLD fields */
    ihold_irun &= ~0x1F; /* Clear IRUN field */
    ihold_irun |= irun;  /* Set IRUN field */

    ihold_irun &= ~(0x1F << 8); /* Clear IHOLD field */
    ihold_irun |= (ihold << 8); /* Set IHOLD field */

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
}

/* ========================================================================== */
/* CHOPPER CONFIGURATION FUNCTIONS                                            */
/* ========================================================================== */

tmc_error_t tmc_enable_double_edge(void)
{
    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t result   = tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Set DEDGE bit (bit 2) */
    chopconf |= (1 << 2);

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_CHOPCONF, chopconf);
}

tmc_error_t tmc_disable_double_edge(void)
{
    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t result   = tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Clear DEDGE bit (bit 2) */
    chopconf &= ~(1 << 2);

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_CHOPCONF, chopconf);
}

tmc_error_t tmc_enable_vsense(void)
{
    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t result   = tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Set VSENSE bit (bit 1) */
    chopconf |= (1 << 1);

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_CHOPCONF, chopconf);
}

tmc_error_t tmc_disable_vsense(void)
{
    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t result   = tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Clear VSENSE bit (bit 1) */
    chopconf &= ~(1 << 1);

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_CHOPCONF, chopconf);
}

/* ========================================================================== */
/* MOTOR DIRECTION CONTROL FUNCTIONS                                          */
/* ========================================================================== */

tmc_error_t tmc_enable_inverse_motor_direction(void)
{
    /* Read current GCONF register */
    uint32_t    gconf  = 0;
    tmc_error_t result = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Set SHIFT bit (bit 4) */
    gconf |= (1 << 4);

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

tmc_error_t tmc_disable_inverse_motor_direction(void)
{
    /* Read current GCONF register */
    uint32_t    gconf  = 0;
    tmc_error_t result = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (result != TMC_OK)
    {
        return result;
    }

    /* Clear SHIFT bit (bit 4) */
    gconf &= ~(1 << 4);

    /* Write back to register */
    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* ========================================================================== */
/* MOTOR CONTROL FUNCTIONS                                                    */
/* ========================================================================== */

/* Enable or disable TMC2209 driver */
tmc_error_t tmc_enable_driver(bool enable)
{
    /* Control the enable pin (active low) */
    tmc_gpio_nen_write(!enable);
    return TMC_OK;
}

/* Set motor direction */
void tmc_set_direction(bool direction)
{
    /* Set the direction pin */
    tmc_gpio_dir_write(!direction);
}

void tmc_set_speed(const float speed_rpm)
{
    tmc_ramp.current_speed = speed_rpm;

    /* Set timer frequency for step generation */
    tmc_tim_step_freq(tmc2209_compute_step_frequency_rpm(speed_rpm, tmc_steps_per_rev));
}

void tmc_set_velocity(const float speed_rpm)
{
    /* Set direction based on speed sign */
    tmc_set_direction(speed_rpm >= 0.0f);

    tmc_set_speed(speed_rpm);
}

void tmc_go(const float speed_rpm)
{
    tmc_set_velocity(speed_rpm);

    /* Set the driver enable state if the speed is high enough */
    tmc_enable_driver((speed_rpm > 0.01f) || (speed_rpm < -0.01f));
}

/* ========================================================================== */
/* SPEED RAMPING FUNCTIONS                                                    */
/* ========================================================================== */

/* Start speed ramping to target */
tmc_error_t tmc_start_ramp(const float target, const float ramp_rate_rpm_per_sec)
{
    /* Validate ramp rate parameter */
    if (ramp_rate_rpm_per_sec <= 0.0f)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Initialize ramping state variables */
    tmc_ramp.start_speed_rpm       = tmc_ramp.current_speed;
    tmc_ramp.target_speed_rpm      = target;
    tmc_ramp.ramp_start_tick       = tmc_get_tick();
    tmc_ramp.ramp_rate_rpm_per_sec = ramp_rate_rpm_per_sec;

    /* Calculate ramping duration and end time */
    const double ramp_period_s =
        fabs((tmc_ramp.target_speed_rpm - tmc_ramp.start_speed_rpm) / ramp_rate_rpm_per_sec);
    tmc_ramp.ramp_end_tick  = (uint32_t)(ramp_period_s * 1000.f) + tmc_ramp.ramp_start_tick;
    tmc_ramp.ramping_active = true;
    tmc_ramp.ramp_tick      = tmc_ramp.ramp_start_tick;

    return TMC_OK;
}

tmc_error_t tmc_stop_ramp(void)
{
    tmc_ramp.ramping_active = false;

    tmc_go(0.0f);

    return TMC_OK;
}

tmc_error_t tmc_start_rotate_angle(const float turns, const float turns_per_second)
{
    /* Make sure we're not doing something else */
    tmc_stop_ramp();

    /* Stop so that we don't have any residual steps */
    tmc_go(0.0f);

    /* Calculate the number of steps to take */
    tmc_step_count = (uint32_t)(turns * (float)tmc_microstepping * tmc_steps_per_rev);

    /* Set the speed to the desired speed */
    tmc_go(turns_per_second * 60.0f);

    return TMC_OK;
}

/* Poll function for non-blocking speed ramping */
void tmc_poll(void)
{
    /* Process speed ramping engine */
    tmc_ramp_engine();
}

/* ========================================================================== */
/* REGISTER-BASED CONFIGURATION FUNCTIONS                                     */
/* ========================================================================== */

/* Set standstill mode */
tmc_error_t tmc_set_standstill_mode(tmc_standstill_mode_t mode)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set freewheel bits */
    pwmconf &= ~(0x03 << 8);       /* Clear FREEWHEEL[1:0] */
    pwmconf |= (mode & 0x03) << 8; /* Set FREEWHEEL[1:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Enable automatic current scaling */
tmc_error_t tmc_enable_automatic_current_scaling(void)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set PWM_AUTOSCALE bit */
    pwmconf |= (1 << 18);

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Disable automatic current scaling */
tmc_error_t tmc_disable_automatic_current_scaling(void)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear PWM_AUTOSCALE bit */
    pwmconf &= ~(1 << 18);

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Enable automatic gradient adaptation */
tmc_error_t tmc_enable_automatic_gradient_adaptation(void)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set PWM_AUTOGRAD bit */
    pwmconf |= (1 << 19);

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Disable automatic gradient adaptation */
tmc_error_t tmc_disable_automatic_gradient_adaptation(void)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear PWM_AUTOGRAD bit */
    pwmconf &= ~(1 << 19);

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Set PWM offset */
tmc_error_t tmc_set_pwm_offset(uint8_t pwm_amplitude)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set PWM_OFFSET bits */
    pwmconf &= ~(0xFF << 0);                /* Clear PWM_OFFSET[7:0] */
    pwmconf |= (pwm_amplitude & 0xFF) << 0; /* Set PWM_OFFSET[7:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Set PWM gradient */
tmc_error_t tmc_set_pwm_gradient(uint8_t pwm_amplitude)
{
    /* Read current PWMCONF register */
    uint32_t    pwmconf = 0;
    tmc_error_t error   = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set PWM_GRAD bits */
    pwmconf &= ~(0xFF << 8);                /* Clear PWM_GRAD[7:0] */
    pwmconf |= (pwm_amplitude & 0xFF) << 8; /* Set PWM_GRAD[7:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_PWMCONF, pwmconf);
}

/* Set power down delay */
tmc_error_t tmc_set_power_down_delay(uint8_t power_down_delay)
{
    /* Validate parameter */
    if (power_down_delay < 2)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_write_reg(tmc_node_address, TMC_REG_TPOWERDOWN, power_down_delay);
}

/* Set reply delay */
tmc_error_t tmc_set_reply_delay(uint8_t delay)
{
    /* Validate parameter */
    if (delay > 15)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Read current REPLYDELAY register */
    uint32_t    replydelay = 0;
    tmc_error_t error      = tmc_read_reg(tmc_node_address, TMC_REG_REPLYDELAY, &replydelay);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set replydelay bits */
    replydelay &= ~(0x0F << 8);        /* Clear replydelay[3:0] */
    replydelay |= (delay & 0x0F) << 8; /* Set replydelay[3:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_REPLYDELAY, replydelay);
}

/* Move at velocity */
tmc_error_t tmc_move_at_velocity(int32_t microsteps_per_period)
{
    return tmc_write_reg(tmc_node_address, TMC_REG_VACTUAL, (uint32_t)microsteps_per_period);
}

/* Move using step/dir interface */
tmc_error_t tmc_move_using_step_dir_interface(void)
{
    return tmc_write_reg(tmc_node_address, TMC_REG_VACTUAL, 0);
}

/* Enable StealthChop */
tmc_error_t tmc_enable_stealth_chop(void)
{
    /* Read current GCONF register */
    uint32_t    gconf = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear enable_spread_cycle bit */
    gconf &= ~(1 << 2);

    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* Disable StealthChop */
tmc_error_t tmc_disable_stealth_chop(void)
{
    /* Read current GCONF register */
    uint32_t    gconf = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set enable_spread_cycle bit */
    gconf |= (1 << 2);

    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* Set StealthChop duration threshold */
tmc_error_t tmc_set_stealth_chop_duration_threshold(uint32_t duration_threshold)
{
    return tmc_write_reg(tmc_node_address, TMC_REG_TPWMTHRS, duration_threshold);
}

/* Set StallGuard threshold */
tmc_error_t tmc_set_stall_guard_threshold(uint8_t stall_guard_threshold)
{
    return tmc_write_reg(tmc_node_address, TMC_REG_SGTHRS, stall_guard_threshold);
}

/* Enable CoolStep */
tmc_error_t tmc_enable_cool_step(uint8_t lower_threshold, uint8_t upper_threshold)
{
    /* Validate parameters */
    if (lower_threshold < 1 || lower_threshold > 15)
    {
        return TMC_ERROR_INVALID_PARAM;
    }
    if (upper_threshold > 15)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Read current COOLCONF register */
    uint32_t    coolconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_node_address, TMC_REG_COOLCONF, &coolconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set SEMIN and SEMAX bits */
    coolconf &= ~(0x0F << 0);                  /* Clear SEMIN[3:0] */
    coolconf |= (lower_threshold & 0x0F) << 0; /* Set SEMIN[3:0] */

    coolconf &= ~(0x0F << 8);                  /* Clear SEMAX[3:0] */
    coolconf |= (upper_threshold & 0x0F) << 8; /* Set SEMAX[3:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_COOLCONF, coolconf);
}

/* Disable CoolStep */
tmc_error_t tmc_disable_cool_step(void)
{
    /* Read current COOLCONF register */
    uint32_t    coolconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_node_address, TMC_REG_COOLCONF, &coolconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear SEMIN bits to disable CoolStep */
    coolconf &= ~(0x0F << 0); /* Clear SEMIN[3:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_COOLCONF, coolconf);
}

/* Set CoolStep current increment */
tmc_error_t tmc_set_cool_step_current_increment(tmc_current_increment_t current_increment)
{
    /* Read current COOLCONF register */
    uint32_t    coolconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_node_address, TMC_REG_COOLCONF, &coolconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set SEUP bits */
    coolconf &= ~(0x03 << 5);                    /* Clear SEUP[1:0] */
    coolconf |= (current_increment & 0x03) << 5; /* Set SEUP[1:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_COOLCONF, coolconf);
}

/* Set CoolStep measurement count */
tmc_error_t tmc_set_cool_step_measurement_count(tmc_measurement_count_t measurement_count)
{
    /* Read current COOLCONF register */
    uint32_t    coolconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_node_address, TMC_REG_COOLCONF, &coolconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set SEDN bits */
    coolconf &= ~(0x03 << 12);                    /* Clear SEDN[1:0] */
    coolconf |= (measurement_count & 0x03) << 12; /* Set SEDN[1:0] */

    return tmc_write_reg(tmc_node_address, TMC_REG_COOLCONF, coolconf);
}

/* Set CoolStep duration threshold */
tmc_error_t tmc_set_cool_step_duration_threshold(uint32_t duration_threshold)
{
    return tmc_write_reg(tmc_node_address, TMC_REG_TCOOLTHRS, duration_threshold);
}

/* Enable analog current scaling */
tmc_error_t tmc_enable_analog_current_scaling(void)
{
    /* Read current GCONF register */
    uint32_t    gconf = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set i_scale_analog bit */
    gconf |= (1 << 0);

    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* Disable analog current scaling */
tmc_error_t tmc_disable_analog_current_scaling(void)
{
    /* Read current GCONF register */
    uint32_t    gconf = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear i_scale_analog bit */
    gconf &= ~(1 << 0);

    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* Use external sense resistors */
tmc_error_t tmc_use_external_sense_resistors(void)
{
    /* Read current GCONF register */
    uint32_t    gconf = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear internal_rsense bit */
    gconf &= ~(1 << 1);

    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* Use internal sense resistors */
tmc_error_t tmc_use_internal_sense_resistors(void)
{
    /* Read current GCONF register */
    uint32_t    gconf = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set internal_rsense bit */
    gconf |= (1 << 1);

    return tmc_write_reg(tmc_node_address, TMC_REG_GCONF, gconf);
}

/* ========================================================================== */
/* STATUS AND MONITORING FUNCTIONS                                            */
/* ========================================================================== */

/* Get TMC2209 version */
uint8_t tmc_get_version(void)
{
    uint32_t ioin = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_IOIN, &ioin);
    return (uint8_t)((ioin >> 24) & 0xFF);
}

/* Check if driver is communicating */
bool tmc_is_communicating(void) { return (tmc_get_version() == 0x21); }

/* Check if driver is setup and communicating */
bool tmc_is_setup_and_communicating(void)
{
    if (tmc_is_communicating() == false)
    {
        return false;
    }

    uint32_t gconf = 0;
    if (tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf) != TMC_OK)
    {
        return false;
    }

    /* Check if pdn_disable bit is set (serial operation mode) */
    return (gconf & (1 << 7)) != 0;
}

/* Check if driver is communicating but not setup */
bool tmc_is_communicating_but_not_setup(void)
{
    return tmc_is_communicating() && !tmc_is_setup_and_communicating();
}

/* Check if driver is hardware disabled */
bool tmc_hardware_disabled(void)
{
    uint32_t ioin = 0;
    if (tmc_read_reg(tmc_node_address, TMC_REG_IOIN, &ioin) != TMC_OK)
    {
        return true;
    }

    /* Check ENN bit (bit 0) */
    return (ioin & (1 << 0)) != 0;
}

/* Get microsteps per step */
uint16_t tmc_get_microsteps_per_step(void)
{
    uint32_t chopconf = 0;
    if (tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf) != TMC_OK)
    {
        return 0;
    }

    uint8_t mres = (chopconf >> 24) & 0x0F;
    return 1 << (8 - mres);
}

/* Get current settings */
tmc_error_t tmc_get_settings(tmc_settings_t *settings)
{
    if (settings == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Initialize settings */
    memset(settings, 0, sizeof(tmc_settings_t));

    settings->is_communicating = tmc_is_communicating();
    if (!settings->is_communicating)
    {
        return TMC_OK;
    }

    /* Read registers */
    uint32_t    gconf = 0, chopconf = 0, pwmconf = 0, ihold_irun = 0;
    tmc_error_t error;

    error = tmc_read_reg(tmc_node_address, TMC_REG_GCONF, &gconf);
    if (error != TMC_OK)
        return error;

    error = tmc_read_reg(tmc_node_address, TMC_REG_CHOPCONF, &chopconf);
    if (error != TMC_OK)
        return error;

    error = tmc_read_reg(tmc_node_address, TMC_REG_PWMCONF, &pwmconf);
    if (error != TMC_OK)
        return error;

    error = tmc_read_reg(tmc_node_address, TMC_REG_IHOLD_IRUN, &ihold_irun);
    if (error != TMC_OK)
        return error;

    /* Parse settings */
    settings->is_setup                              = (gconf & (1 << 7)) != 0;
    settings->software_enabled                      = ((chopconf >> 0) & 0x0F) > 0;
    settings->microsteps_per_step                   = tmc_get_microsteps_per_step();
    settings->inverse_motor_direction_enabled       = (gconf & (1 << 3)) != 0;
    settings->stealth_chop_enabled                  = (gconf & (1 << 2)) == 0;
    settings->standstill_mode                       = (pwmconf >> 8) & 0x03;
    settings->irun_register_value                   = (ihold_irun >> 8) & 0x1F;
    settings->ihold_register_value                  = (ihold_irun >> 0) & 0x1F;
    settings->iholddelay_register_value             = (ihold_irun >> 16) & 0x0F;
    settings->automatic_current_scaling_enabled     = (pwmconf & (1 << 18)) != 0;
    settings->automatic_gradient_adaptation_enabled = (pwmconf & (1 << 19)) != 0;
    settings->pwm_offset                            = (pwmconf >> 0) & 0xFF;
    settings->pwm_gradient                          = (pwmconf >> 8) & 0xFF;
    settings->analog_current_scaling_enabled        = (gconf & (1 << 0)) != 0;
    settings->internal_sense_resistors_enabled      = (gconf & (1 << 1)) != 0;

    return TMC_OK;
}

/* Get driver status */
tmc_error_t tmc_get_status(tmc_status_t *status)
{
    if (status == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Parse status bits */
    status->over_temperature_warning  = (drv_status >> 0) & 0x01;
    status->over_temperature_shutdown = (drv_status >> 1) & 0x01;
    status->short_to_ground_a         = (drv_status >> 2) & 0x01;
    status->short_to_ground_b         = (drv_status >> 3) & 0x01;
    status->low_side_short_a          = (drv_status >> 4) & 0x01;
    status->low_side_short_b          = (drv_status >> 5) & 0x01;
    status->open_load_a               = (drv_status >> 6) & 0x01;
    status->open_load_b               = (drv_status >> 7) & 0x01;
    status->over_temperature_120c     = (drv_status >> 8) & 0x01;
    status->over_temperature_143c     = (drv_status >> 9) & 0x01;
    status->over_temperature_150c     = (drv_status >> 10) & 0x01;
    status->over_temperature_157c     = (drv_status >> 11) & 0x01;
    status->reserved0                 = (drv_status >> 12) & 0x0F;
    status->current_scaling           = (drv_status >> 16) & 0x1F;
    status->reserved1                 = (drv_status >> 21) & 0x1FF;
    status->stealth_chop_mode         = (drv_status >> 30) & 0x01;
    status->standstill                = (drv_status >> 31) & 0x01;

    return TMC_OK;
}

/* Get global status */
tmc_error_t tmc_get_global_status(tmc_global_status_t *global_status)
{
    if (global_status == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    gstat = 0;
    tmc_error_t error = tmc_read_reg(tmc_node_address, TMC_REG_GSTAT, &gstat);
    if (error != TMC_OK)
    {
        return error;
    }

    global_status->reset    = (gstat >> 0) & 0x01;
    global_status->drv_err  = (gstat >> 1) & 0x01;
    global_status->uv_cp    = (gstat >> 2) & 0x01;
    global_status->reserved = (gstat >> 3) & 0x1FFFFFFF;

    return TMC_OK;
}

/* Clear reset flag */
tmc_error_t tmc_clear_reset(void) { return tmc_write_reg(tmc_node_address, TMC_REG_GSTAT, 0x01); }

/* Clear drive error flag */
tmc_error_t tmc_clear_drive_error(void)
{
    return tmc_write_reg(tmc_node_address, TMC_REG_GSTAT, 0x02);
}

/* Get interface transmission counter */
uint8_t tmc_get_interface_transmission_counter(void)
{
    uint32_t ifcnt = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_IFCNT, &ifcnt);
    return (uint8_t)(ifcnt & 0xFF);
}

/* Get interstep duration */
uint32_t tmc_get_interstep_duration(void)
{
    uint32_t tstep = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_TSTEP, &tstep);
    return tstep;
}

/* Get StallGuard result */
uint16_t tmc_get_stall_guard_result(void)
{
    uint32_t sg_result = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_SG_RESULT, &sg_result);
    return (uint16_t)(sg_result & 0x3FF);
}

/* Get PWM scale sum */
uint8_t tmc_get_pwm_scale_sum(void)
{
    uint32_t pwm_scale = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_PWM_SCALE, &pwm_scale);
    return (uint8_t)(pwm_scale & 0xFF);
}

/* Get PWM scale auto */
int16_t tmc_get_pwm_scale_auto(void)
{
    uint32_t pwm_scale = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_PWM_SCALE, &pwm_scale);
    return (int16_t)((pwm_scale >> 16) & 0x1FF);
}

/* Get PWM offset auto */
uint8_t tmc_get_pwm_offset_auto(void)
{
    uint32_t pwm_auto = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_PWM_AUTO, &pwm_auto);
    return (uint8_t)(pwm_auto & 0xFF);
}

/* Get PWM gradient auto */
uint8_t tmc_get_pwm_gradient_auto(void)
{
    uint32_t pwm_auto = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_PWM_AUTO, &pwm_auto);
    return (uint8_t)((pwm_auto >> 16) & 0xFF);
}

/* Get microstep counter */
uint16_t tmc_get_microstep_counter(void)
{
    uint32_t mscnt = 0;
    tmc_read_reg(tmc_node_address, TMC_REG_MSCNT, &mscnt);
    return (uint16_t)(mscnt & 0x3FF);
}