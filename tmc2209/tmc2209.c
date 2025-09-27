/**
 * @file    tmc2209.c
 * @brief   TMC2209 stepper motor driver implementation
 * @version 1.0.0
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

/* ========================================================================== */
/* GLOBAL VARIABLES                                                           */
/* ========================================================================== */

/**
 * @brief Global TMC2209 configuration structure
 *
 * Contains all driver settings including motor parameters, control modes,
 * and ramping state variables.
 */
static tmc_config_t tmc_config = {0};

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Calculate CRC8 checksum for TMC2209 datagrams
 * @param datagram Pointer to datagram data
 * @param len Length of datagram in bytes
 * @return Calculated CRC8 checksum
 *
 * Implements the CRC8 polynomial 0x07 as specified in the TMC2209 datasheet.
 * Used for error detection in UART communication.
 */
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

/**
 * @brief Transmit write datagram to TMC2209
 * @param tx Pointer to write datagram structure
 * @return TMC_OK on success, error code otherwise
 *
 * Sends a write datagram containing register address and data to the TMC2209.
 * The datagram includes sync word, slave address, register address, payload,
 * and CRC checksum.
 */
static tmc_error_t tmc_write_datagram(const tmc_write_datagram_t *tx)
{
    return tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_write_datagram_t));
}

/**
 * @brief Transmit read datagram and receive response
 * @param tx Pointer to read datagram structure
 * @param rx Pointer to receive buffer for response
 * @return TMC_OK on success, error code otherwise
 *
 * Sends a read datagram and receives the response from TMC2209.
 * Handles the two-stage UART communication required for register reads.
 */
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

/**
 * @brief Internal speed ramping engine
 *
 * Handles the non-blocking speed ramping process. Called periodically by
 * tmc_poll() to update motor speed during ramping operations.
 *
 * Uses linear interpolation to smoothly transition between start and target speeds.
 * Runs every 10ms to provide smooth ramping without blocking the main loop.
 */
static void tmc_ramp_engine(void)
{
    /* Check if ramping is active */
    if (tmc_config.ramping_active == false)
    {
        return;
    }

    /* Rate limit: run every 10ms */
    const uint32_t tick = tmc_get_tick();
    if (tick < (tmc_config.ramp_tick + 10U))
    {
        return;
    }
    tmc_config.ramp_tick = tick;

    /* Check if ramping is complete */
    if (tick > tmc_config.ramp_end_tick)
    {
        /* Set final target speed and disable ramping */
        tmc_start(tmc_config.target_speed_rpm);
        tmc_config.ramping_active = false;
        return;
    }

    /* Calculate ramping progress (0.0 to 1.0) */
    const double completeness =
        ((double)tick - (double)tmc_config.ramp_start_tick) /
        ((double)tmc_config.ramp_end_tick - (double)tmc_config.ramp_start_tick);

    /* Interpolate between start and target speed */
    const double new_speed = stats_linear_interp(
        (double)tmc_config.start_speed_rpm, (double)tmc_config.target_speed_rpm, completeness);

    /* Apply the interpolated speed */
    tmc_start((const float)new_speed);
}

/* ========================================================================== */
/* REGISTER READ/WRITE FUNCTIONS                                              */
/* ========================================================================== */

/**
 * @brief Write a register to TMC2209
 * @param node TMC2209 node address (0-3)
 * @param addr Register address
 * @param value 32-bit value to write
 * @return TMC_OK on success, error code otherwise
 *
 * Writes a 32-bit value to the specified TMC2209 register.
 * Handles datagram construction, CRC calculation, and UART transmission.
 */
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

/**
 * @brief Read a register from TMC2209
 * @param node TMC2209 node address (0-3)
 * @param addr Register address
 * @param value Pointer to store the read value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads a 32-bit value from the specified TMC2209 register.
 * Handles datagram construction, CRC verification, and response validation.
 */
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

/* ========================================================================== */
/* INITIALIZATION AND CONFIGURATION FUNCTIONS                                */
/* ========================================================================== */

/**
 * @brief Initialize TMC2209 driver
 * @param stepping Microstepping mode
 * @param steps_per_rev Steps per revolution of the motor
 * @param address TMC2209 node address (0-3)
 * @return TMC_OK on success, error code otherwise
 *
 * Initializes the TMC2209 driver with default settings and verifies
 * communication. Sets up motor parameters and driver configuration.
 */
tmc_error_t
tmc_init(const tmc_microstepping_t stepping, const uint32_t steps_per_rev, const uint8_t address)
{
    /* Initialize hardware interface */
    tmc_error_t error = tmc_hw_init();
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set TMC2209 node address */
    error = tmc_set_address(address);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Initialize configuration structure with defaults */
    memset(&tmc_config, 0, sizeof(tmc_config));
    tmc_config.node_address          = 0x00;
    tmc_config.steps_per_rev         = steps_per_rev;
    tmc_config.microstepping         = stepping;
    tmc_config.mode                  = TMC_STEALTHCHOP;
    tmc_config.current_hold          = 500; /* 500mA hold current */
    tmc_config.current_run           = 800; /* 800mA run current */
    tmc_config.current_hold_delay    = 10;
    tmc_config.stealthchop_threshold = 0;
    tmc_config.coolstep_threshold    = 0;
    tmc_config.stall_threshold       = 0;
    tmc_config.enabled               = false;
    tmc_config.direction             = true;
    tmc_config.step_frequency        = 0;

    /* Initialize ramping state */
    tmc_config.current_speed_rpm = 0.0f;
    tmc_config.ramping_active    = false;

    /* Verify communication by reading GSTAT register */
    uint32_t gstat = 0;
    error          = tmc_read_reg(tmc_config.node_address, TMC_REG_GSTAT, &gstat);
    if (error != TMC_OK)
    {
        return TMC_ERROR_INIT;
    }

    return TMC_OK;
}

/**
 * @brief Set TMC2209 node address
 * @param address Node address (0-3)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the TMC2209 node address by configuring MS1 and MS2 pins.
 * Address is encoded as: MS2(MSB) MS1(LSB)
 */
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

    tmc_gpio_ms1_write(ms1);
    tmc_gpio_ms2_write(ms2);

    return TMC_OK;
}

/**
 * @brief Configure TMC2209 with custom settings
 * @param config Pointer to configuration structure
 * @return TMC_OK on success, error code otherwise
 *
 * Applies a complete configuration to the TMC2209 driver.
 * Configures chopper, current, thresholds, and PWM settings.
 */
tmc_error_t tmc_configure(const tmc_config_t *config)
{
    if (config == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Copy configuration to internal structure */
    memcpy(&tmc_config, config, sizeof(tmc_config_t));

    /* Configure CHOPCONF register for chopper settings */
    uint32_t chopconf = 0;
    chopconf |= (tmc_config.microstepping & 0x07) << 0; /* MRES[2:0] - Microstep resolution */
    chopconf |= (1 << 7);                               /* TBL[1:0] = 1 - Blank time */
    chopconf |= (1 << 15);                              /* TOFF[3:0] = 1 - Off time */
    chopconf |= (1 << 19);                              /* HSTRT[2:0] = 1 - Hysteresis start */
    chopconf |= (1 << 22);                              /* HEND[2:0] = 1 - Hysteresis end */
    chopconf |= (1 << 25);                              /* FD[2:0] = 1 - Fast decay */
    chopconf |= (1 << 28);                              /* DISFDCC = 1 - Disable fast decay */
    chopconf |= (1 << 30);                              /* RNDTF = 1 - Random TOFF */
    chopconf |= (1 << 31);                              /* CHM = 1 (SpreadCycle mode) */

    tmc_error_t error = tmc_write_reg(tmc_config.node_address, TMC_REG_CHOPCONF, chopconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure IHOLD_IRUN register for current settings */
    uint32_t ihold_irun = 0;
    ihold_irun |= (tmc_config.current_hold & 0x1F) << 0;        /* IHOLD[4:0] - Hold current */
    ihold_irun |= (tmc_config.current_run & 0x1F) << 8;         /* IRUN[4:0] - Run current */
    ihold_irun |= (tmc_config.current_hold_delay & 0x0F) << 16; /* IHOLDDELAY[3:0] - Hold delay */

    error = tmc_write_reg(tmc_config.node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure TPOWERDOWN register (power down time) */
    error = tmc_write_reg(tmc_config.node_address, TMC_REG_TPOWERDOWN, 0x00000000);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure TPWMTHRS register (StealthChop threshold) */
    uint32_t tpwmthrs = (uint32_t)tmc_config.stealthchop_threshold << 0;
    error             = tmc_write_reg(tmc_config.node_address, TMC_REG_TPWMTHRS, tpwmthrs);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure TCOOLTHRS register (CoolStep threshold) */
    uint32_t tcoolthrs = (uint32_t)tmc_config.coolstep_threshold << 0;
    error              = tmc_write_reg(tmc_config.node_address, TMC_REG_TCOOLTHRS, tcoolthrs);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure THIGH register (stall detection threshold) */
    uint32_t thigh = (uint32_t)tmc_config.stall_threshold << 0;
    error          = tmc_write_reg(tmc_config.node_address, TMC_REG_THIGH, thigh);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure PWMCONF register for StealthChop PWM settings */
    uint32_t pwmconf = 0;
    pwmconf |= (1 << 0);  /* PWM_AMPL[7:0] = 1 - PWM amplitude */
    pwmconf |= (1 << 8);  /* PWM_GRAD[7:0] = 1 - PWM gradient */
    pwmconf |= (1 << 16); /* PWM_FREQ[1:0] = 1 - PWM frequency */
    pwmconf |= (1 << 18); /* PWM_AUTOSCALE = 1 - Auto scale */
    pwmconf |= (1 << 19); /* PWM_AUTOGRAD = 1 - Auto gradient */
    pwmconf |= (1 << 20); /* FREEWHEEL[1:0] = 1 - Freewheeling */

    error = tmc_write_reg(tmc_config.node_address, TMC_REG_PWMCONF, pwmconf);
    if (error != TMC_OK)
    {
        return error;
    }

    return TMC_OK;
}

/**
 * @brief Set motor current settings
 * @param hold_current Hold current in mA
 * @param run_current Run current in mA
 * @param hold_delay Hold delay in steps
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the motor current settings for hold and run modes.
 * Current values are automatically scaled by the TMC2209.
 */
tmc_error_t tmc_set_current(uint16_t hold_current, uint16_t run_current, uint16_t hold_delay)
{
    /* Update internal configuration */
    tmc_config.current_hold       = hold_current;
    tmc_config.current_run        = run_current;
    tmc_config.current_hold_delay = hold_delay;

    /* Configure IHOLD_IRUN register */
    uint32_t ihold_irun = 0;
    ihold_irun |= (hold_current & 0x1F) << 0; /* IHOLD[4:0] - Hold current */
    ihold_irun |= (run_current & 0x1F) << 8;  /* IRUN[4:0] - Run current */
    ihold_irun |= (hold_delay & 0x0F) << 16;  /* IHOLDDELAY[3:0] - Hold delay */

    return tmc_write_reg(tmc_config.node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
}

/**
 * @brief Set TMC2209 operating mode
 * @param mode Operating mode (StealthChop or SpreadCycle)
 * @return TMC_OK on success, error code otherwise
 *
 * Switches between StealthChop (silent) and SpreadCycle (high torque) modes.
 * StealthChop provides silent operation, SpreadCycle provides maximum torque.
 */
tmc_error_t tmc_set_mode(tmc_mode_t mode)
{
    tmc_config.mode = mode;

    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_config.node_address, TMC_REG_CHOPCONF, &chopconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Configure CHM bit for mode selection */
    if (mode == TMC_STEALTHCHOP)
    {
        chopconf &= ~(1 << 31); /* Clear CHM bit for StealthChop */
    }
    else
    {
        chopconf |= (1 << 31); /* Set CHM bit for SpreadCycle */
    }

    return tmc_write_reg(tmc_config.node_address, TMC_REG_CHOPCONF, chopconf);
}

/**
 * @brief Set microstepping resolution
 * @param stepping Microstepping mode
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the microstepping resolution from 1/2 to 1/256 steps.
 * Higher microstepping provides smoother motion but requires higher step rates.
 */
tmc_error_t tmc_set_microstepping(const tmc_microstepping_t stepping)
{
    tmc_config.microstepping = stepping;

    /* Read current CHOPCONF register */
    uint32_t    chopconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_config.node_address, TMC_REG_CHOPCONF, &chopconf);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Clear and set MRES bits for microstepping */
    chopconf &= ~(0x07 << 0);
    chopconf |= (stepping & 0x07) << 0;

    return tmc_write_reg(tmc_config.node_address, TMC_REG_CHOPCONF, chopconf);
}

/**
 * @brief Set TMC2209 threshold values
 * @param stealthchop_threshold StealthChop threshold
 * @param coolstep_threshold CoolStep threshold
 * @param stall_threshold Stall detection threshold
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the threshold values for StealthChop, CoolStep, and stall detection.
 * These thresholds determine when the driver switches between different modes.
 */
tmc_error_t tmc_set_thresholds(uint8_t stealthchop_threshold,
                               uint8_t coolstep_threshold,
                               uint8_t stall_threshold)
{
    /* Update internal configuration */
    tmc_config.stealthchop_threshold = stealthchop_threshold;
    tmc_config.coolstep_threshold    = coolstep_threshold;
    tmc_config.stall_threshold       = stall_threshold;

    tmc_error_t error;

    /* Set StealthChop threshold (TPWMTHRS register) */
    uint32_t tpwmthrs = (uint32_t)stealthchop_threshold << 0;
    error             = tmc_write_reg(tmc_config.node_address, TMC_REG_TPWMTHRS, tpwmthrs);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set CoolStep threshold (TCOOLTHRS register) */
    uint32_t tcoolthrs = (uint32_t)coolstep_threshold << 0;
    error              = tmc_write_reg(tmc_config.node_address, TMC_REG_TCOOLTHRS, tcoolthrs);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set stall detection threshold (THIGH register) */
    uint32_t thigh = (uint32_t)stall_threshold << 0;
    return tmc_write_reg(tmc_config.node_address, TMC_REG_THIGH, thigh);
}

/* ========================================================================== */
/* MOTOR CONTROL FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Enable or disable TMC2209 driver
 * @param enable true to enable, false to disable
 * @return TMC_OK on success, error code otherwise
 *
 * Controls the enable pin of the TMC2209 driver.
 * When disabled, the motor is in high-impedance state.
 */
tmc_error_t tmc_enable(bool enable)
{
    tmc_config.enabled = enable;

    /* Control enable pin (NEN is active low) */
    tmc_gpio_nen_write(!enable);

    return TMC_OK;
}

/**
 * @brief Set motor direction
 * @param direction true for forward, false for reverse
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the direction pin to control motor rotation direction.
 * Direction is relative to the motor's wiring and microstepping configuration.
 */
tmc_error_t tmc_set_direction(bool direction)
{
    tmc_config.direction = direction;

    /* Control direction pin */
    tmc_gpio_dir_write(direction);

    return TMC_OK;
}

/**
 * @brief Set motor speed in RPM
 * @param speed_rpm Speed in revolutions per minute (can be negative)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the motor speed in RPM. Negative values indicate reverse direction.
 * Automatically calculates step frequency and sets direction.
 */
tmc_error_t tmc_set_speed(float speed_rpm)
{
    /* Update current speed for ramping calculations */
    tmc_config.current_speed_rpm = speed_rpm;

    /* Handle stop condition */
    if ((speed_rpm < 0.01f) && (speed_rpm > -0.01f))
    {
        return tmc_stop();
    }

    /* Determine direction from speed sign */
    tmc_config.direction = !(speed_rpm > 0);

    /* Set motor direction */
    tmc_set_direction(tmc_config.direction);

    /* Calculate step frequency from RPM */
    float steps_per_second =
        fabs(speed_rpm) * tmc_config.steps_per_rev * (1 << tmc_config.microstepping) / 60.0f;
    uint32_t frequency = (uint32_t)steps_per_second;

    return tmc_set_frequency(frequency);
}

/**
 * @brief Set step frequency directly
 * @param frequency_hz Step frequency in Hz
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the step frequency directly in Hz. Used internally by tmc_set_speed()
 * and for direct frequency control.
 */
tmc_error_t tmc_set_frequency(uint32_t frequency_hz)
{
    tmc_config.step_frequency = frequency_hz;

    /* Configure timer for step generation */
    if (frequency_hz > 0)
    {
        tmc_tim_step_freq((float)frequency_hz);
    }

    return TMC_OK;
}

/**
 * @brief Start motor with specified speed
 * @param speed_rpm Speed in RPM
 * @return TMC_OK on success, error code otherwise
 *
 * Enables the driver and sets the motor speed. Combines tmc_enable()
 * and tmc_set_speed() for convenience.
 */
tmc_error_t tmc_start(const float speed_rpm)
{
    /* Enable the driver */
    tmc_error_t error = tmc_enable(true);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Set the speed */
    return tmc_set_speed(speed_rpm);
}

/**
 * @brief Stop motor immediately
 * @return TMC_OK on success, error code otherwise
 *
 * Stops the motor by setting frequency to zero and disabling the driver.
 * Provides immediate stop without ramping.
 */
tmc_error_t tmc_stop(void)
{
    tmc_config.step_frequency = 0;

    /* Disable the driver */
    return tmc_enable(false);
}

/* ========================================================================== */
/* STATUS AND MONITORING FUNCTIONS                                            */
/* ========================================================================== */

/**
 * @brief Get TMC2209 driver status
 * @param status Pointer to store status register value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the DRV_STATUS register containing motor status information
 * including stall detection, temperature, and current measurements.
 */
tmc_error_t tmc_get_status(uint32_t *status)
{
    if (status == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, status);
}

/**
 * @brief Get motor position
 * @param position Pointer to store position value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the XDIRECT register containing the current motor position.
 * Position is relative to the last reset or initialization.
 */
tmc_error_t tmc_get_position(uint32_t *position)
{
    if (position == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_read_reg(tmc_config.node_address, TMC_REG_XDIRECT, position);
}

/**
 * @brief Get motor speed
 * @param speed Pointer to store speed value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the TSTEP register containing the current step timing.
 * Can be used to calculate actual motor speed.
 */
tmc_error_t tmc_get_speed(uint32_t *speed)
{
    if (speed == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_read_reg(tmc_config.node_address, TMC_REG_TSTEP, speed);
}

/**
 * @brief Get motor current
 * @param current Pointer to store current value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the actual motor current from the DRV_STATUS register.
 * Current is returned as a 5-bit value representing the current scale.
 */
tmc_error_t tmc_get_current(uint16_t *current)
{
    if (current == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Extract current from DRV_STATUS register (CS_ACTUAL field) */
    *current = (uint16_t)((drv_status >> 0) & 0x1F);
    return TMC_OK;
}

/**
 * @brief Get motor temperature
 * @param temperature Pointer to store temperature value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the motor temperature from the DRV_STATUS register.
 * Temperature is returned as an 8-bit value representing the temperature scale.
 */
tmc_error_t tmc_get_temperature(uint16_t *temperature)
{
    if (temperature == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Extract temperature from DRV_STATUS register */
    *temperature = (uint16_t)((drv_status >> 16) & 0xFF);
    return TMC_OK;
}

/**
 * @brief Get stall detection status
 * @param stalled Pointer to store stall status
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the stall detection status from the DRV_STATUS register.
 * Returns true if a stall condition is detected.
 */
tmc_error_t tmc_get_stall_status(bool *stalled)
{
    if (stalled == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_OK)
    {
        return error;
    }

    /* Check stall detection bit (STALLGUARD field) */
    *stalled = (bool)((drv_status >> 24) & 0x01);
    return TMC_OK;
}

/**
 * @brief Get human-readable status string
 * @param status Status register value
 * @return Pointer to formatted status string
 *
 * Converts the DRV_STATUS register value into a human-readable string
 * containing all status information including warnings and errors.
 */
char *tmc_get_status_string(uint32_t status)
{
    static char status_str[512];
    int         offset = 0;

    /* Clear the buffer */
    memset(status_str, 0, sizeof(status_str));

    /* Extract status bits from DRV_STATUS register */
    uint16_t sg_result  = (status >> 0) & 0x3FF; /* SG_RESULT[9:0] - StallGuard2 result */
    uint8_t  fsactive   = (status >> 15) & 0x01; /* FSACTIVE - Full step active (bit 15) */
    uint8_t  cs_actual  = (status >> 16) & 0x1F; /* CS_ACTUAL[4:0] - Actual motor current */
    uint8_t  stallguard = (status >> 24) & 0x01; /* STALLGUARD - Stall detected */
    uint8_t  ot         = (status >> 25) & 0x01; /* OT - Overtemperature prewarning */
    uint8_t  otpw       = (status >> 26) & 0x01; /* OTPW - Overtemperature warning */
    uint8_t  s2ga       = (status >> 27) & 0x01; /* S2GA - Short to ground phase A */
    uint8_t  s2gb       = (status >> 28) & 0x01; /* S2GB - Short to ground phase B */
    uint8_t  ola        = (status >> 29) & 0x01; /* OLA - Open load phase A */
    uint8_t  olb        = (status >> 30) & 0x01; /* OLB - Open load phase B */
    uint8_t  stst       = (status >> 31) & 0x01; /* STST - Standstill detected */

    /* Build status string with register value */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "DRV_STATUS: 0x%08lX\n",
                       (unsigned long)status);

    /* StallGuard2 result */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  SG_RESULT: %u (StallGuard2 result)\n",
                       sg_result);

    /* Full step active status */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  FSACTIVE: %s\n",
                       fsactive ? "YES" : "NO");

    /* Actual motor current */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  CS_ACTUAL: %u (Current scale)\n",
                       cs_actual);

    /* Stall detection status */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  STALLGUARD: %s\n",
                       stallguard ? "STALL DETECTED" : "OK");

    /* Temperature warning status */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  OT: %s\n",
                       ot ? "OVERTEMP PREWARNING" : "OK");
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  OTPW: %s\n",
                       otpw ? "OVERTEMP WARNING" : "OK");

    /* Short circuit detection status */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  S2GA: %s\n",
                       s2ga ? "SHORT TO GND PHASE A" : "OK");
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  S2GB: %s\n",
                       s2gb ? "SHORT TO GND PHASE B" : "OK");

    /* Open load detection status */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  OLA: %s\n",
                       ola ? "OPEN LOAD PHASE A" : "OK");
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  OLB: %s\n",
                       olb ? "OPEN LOAD PHASE B" : "OK");

    /* Standstill detection status */
    offset += snprintf(status_str + offset,
                       sizeof(status_str) - offset,
                       "  STST: %s\n",
                       stst ? "STANDSTILL" : "MOVING");

    /* Summary of critical issues */
    if (stallguard || ot || otpw || s2ga || s2gb || ola || olb)
    {
        offset +=
            snprintf(status_str + offset, sizeof(status_str) - offset, "\nCRITICAL ISSUES:\n");
        if (stallguard)
            offset +=
                snprintf(status_str + offset, sizeof(status_str) - offset, "  - STALL DETECTED\n");
        if (ot)
            offset += snprintf(
                status_str + offset, sizeof(status_str) - offset, "  - OVERTEMP PREWARNING\n");
        if (otpw)
            offset += snprintf(
                status_str + offset, sizeof(status_str) - offset, "  - OVERTEMP WARNING\n");
        if (s2ga)
            offset += snprintf(
                status_str + offset, sizeof(status_str) - offset, "  - SHORT TO GND PHASE A\n");
        if (s2gb)
            offset += snprintf(
                status_str + offset, sizeof(status_str) - offset, "  - SHORT TO GND PHASE B\n");
        if (ola)
            offset += snprintf(
                status_str + offset, sizeof(status_str) - offset, "  - OPEN LOAD PHASE A\n");
        if (olb)
            offset += snprintf(
                status_str + offset, sizeof(status_str) - offset, "  - OPEN LOAD PHASE B\n");
    }
    else
    {
        offset += snprintf(
            status_str + offset, sizeof(status_str) - offset, "\nSTATUS: All systems OK\n");
    }

    return status_str;
}

/* ========================================================================== */
/* SPEED RAMPING FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Start speed ramping to target
 * @param target Target speed in RPM
 * @param ramp_rate_rpm_per_sec Ramp rate in RPM per second
 * @return TMC_OK on success, error code otherwise
 *
 * Initiates smooth speed ramping from current speed to target speed.
 * Uses linear interpolation to provide smooth acceleration/deceleration.
 * Handles direction changes automatically.
 */
tmc_error_t tmc_start_ramp(const float target, const float ramp_rate_rpm_per_sec)
{
    /* Validate ramp rate parameter */
    if (ramp_rate_rpm_per_sec <= 0.0f)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    /* Initialize ramping state variables */
    tmc_config.start_speed_rpm  = tmc_config.current_speed_rpm;
    tmc_config.target_speed_rpm = target;
    tmc_config.ramp_start_tick  = tmc_get_tick();

    /* Calculate ramping duration and end time */
    const float ramp_period_s =
        fabs((tmc_config.target_speed_rpm - tmc_config.start_speed_rpm) / ramp_rate_rpm_per_sec);
    tmc_config.ramp_end_tick  = (uint32_t)(ramp_period_s * 1000.f) + tmc_config.ramp_start_tick;
    tmc_config.ramping_active = true;
    tmc_config.ramp_tick      = tmc_config.ramp_start_tick;

    return TMC_OK;
}

/**
 * @brief Poll function for non-blocking speed ramping
 *
 * This function should be called regularly in the main loop to handle
 * speed ramping operations. It processes the ramping engine without
 * blocking the main execution thread.
 */
void tmc_poll(void)
{
    /* Process speed ramping engine */
    tmc_ramp_engine();
}