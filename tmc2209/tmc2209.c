#include "tmc2209.h"

#include <math.h>
#include <string.h>

// Global configuration
static tmc_config_t tmc_config = {0};

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

static tmc_error_t tmc_write_datagram(const tmc_write_datagram_t *tx)
{
    return tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_write_datagram_t));
}

static tmc_error_t tmc_read_datagram(const tmc_read_datagram_t *tx, tmc_write_datagram_t *rx)
{
    tmc_error_t error = tmc_uart_tx((const uint8_t *)tx, (uint32_t)sizeof(tmc_read_datagram_t));
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Get rid of the bytes we read from the TX
    tmc_uart_rx((uint8_t *)rx, (uint32_t)sizeof(tmc_read_datagram_t), TMC_UART_TIMEOUT_MS);

    // Now, get the real RXd ones
    return tmc_uart_rx((uint8_t *)rx, (uint32_t)sizeof(tmc_write_datagram_t), TMC_UART_TIMEOUT_MS);
}

tmc_error_t tmc_write_reg(const uint8_t node, const uint8_t addr, const uint32_t value)
{
    // Init the datagram
    tmc_write_datagram_t tx = {
        .sync       = TMC_SYNC_WORD,
        .slave      = node,
        .addr.idx   = addr,
        .addr.write = TMC_WRITE_BIT,
        .payload    = value,
    };

    // Swap the endianness of the payload for the controller
    tx.payload = __builtin_bswap32(tx.payload);

    // Calc the CRC
    tx.crc = tmc_uart_calc_crc((const uint8_t *)&tx, (uint32_t)sizeof(tmc_write_datagram_t));

    // Send on the UART
    return tmc_write_datagram((const tmc_write_datagram_t *)&tx);
}

tmc_error_t tmc_read_reg(const uint8_t node, const uint8_t addr, uint32_t *value)
{
    // Init the datagram
    tmc_read_datagram_t tx = {
        .sync       = TMC_SYNC_WORD,
        .slave      = node,
        .addr.idx   = addr,
        .addr.write = TMC_READ_BIT,
    };

    // Calc the CRC
    tx.crc = tmc_uart_calc_crc((const uint8_t *)&tx, (uint32_t)sizeof(tmc_read_datagram_t));

    // Send on the UART
    tmc_write_datagram_t rx    = {0};
    tmc_error_t          error = tmc_read_datagram((const tmc_read_datagram_t *)&tx, &rx);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // is the CRC ok?
    uint8_t crc = tmc_uart_calc_crc((const uint8_t *)&rx, (uint32_t)sizeof(tmc_write_datagram_t));
    if (crc != rx.crc)
    {
        return TMC_ERROR_CRC;
    }

    // Is what we got for us?
    if ((rx.slave != 0xFF) || (rx.addr.idx != tx.addr.idx))
    {
        return TMC_ERROR_INVALID_RX;
    }

    // Let's get the data...
    *value = __builtin_bswap32(rx.payload);

    return TMC_ERROR_OK;
}

tmc_error_t tmc_init(const tmc_microstepping_t stepping, const uint32_t steps_per_rev)
{
    // Initialize configuration
    memset(&tmc_config, 0, sizeof(tmc_config));
    tmc_config.node_address          = 0x00;
    tmc_config.steps_per_rev         = steps_per_rev;
    tmc_config.microstepping         = stepping;
    tmc_config.mode                  = TMC_STEALTHCHOP;
    tmc_config.current_hold          = 500; // 500mA
    tmc_config.current_run           = 800; // 800mA
    tmc_config.current_hold_delay    = 10;
    tmc_config.stealthchop_threshold = 0;
    tmc_config.coolstep_threshold    = 0;
    tmc_config.stall_threshold       = 0;
    tmc_config.enabled               = false;
    tmc_config.direction             = true;
    tmc_config.step_frequency        = 0;

    // Read GSTAT to check communication
    uint32_t    gstat = 0;
    tmc_error_t error = tmc_read_reg(tmc_config.node_address, TMC_REG_GSTAT, &gstat);
    if (error != TMC_ERROR_OK)
    {
        return TMC_ERROR_INIT;
    }

    // Configure the driver
    return tmc_configure(&tmc_config);
}

tmc_error_t tmc_configure(const tmc_config_t *config)
{
    if (config == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    // Copy configuration
    memcpy(&tmc_config, config, sizeof(tmc_config_t));

    // Configure CHOPCONF register
    uint32_t chopconf = 0;
    chopconf |= (tmc_config.microstepping & 0x07) << 0; // MRES[2:0]
    chopconf |= (1 << 7);                               // TBL[1:0] = 1
    chopconf |= (1 << 15);                              // TOFF[3:0] = 1
    chopconf |= (1 << 19);                              // HSTRT[2:0] = 1
    chopconf |= (1 << 22);                              // HEND[2:0] = 1
    chopconf |= (1 << 25);                              // FD[2:0] = 1
    chopconf |= (1 << 28);                              // DISFDCC = 1
    chopconf |= (1 << 30);                              // RNDTF = 1
    chopconf |= (1 << 31);                              // CHM = 1 (spreadCycle)

    tmc_error_t error = tmc_write_reg(tmc_config.node_address, TMC_REG_CHOPCONF, chopconf);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Configure IHOLD_IRUN register
    uint32_t ihold_irun = 0;
    ihold_irun |= (tmc_config.current_hold & 0x1F) << 0;        // IHOLD[4:0]
    ihold_irun |= (tmc_config.current_run & 0x1F) << 8;         // IRUN[4:0]
    ihold_irun |= (tmc_config.current_hold_delay & 0x0F) << 16; // IHOLDDELAY[3:0]

    error = tmc_write_reg(tmc_config.node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Configure TPOWERDOWN register
    error = tmc_write_reg(tmc_config.node_address, TMC_REG_TPOWERDOWN, 0x00000000);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Configure TPWMTHRS register (StealthChop threshold)
    uint32_t tpwmthrs = (uint32_t)tmc_config.stealthchop_threshold << 0;
    error             = tmc_write_reg(tmc_config.node_address, TMC_REG_TPWMTHRS, tpwmthrs);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Configure TCOOLTHRS register (CoolStep threshold)
    uint32_t tcoolthrs = (uint32_t)tmc_config.coolstep_threshold << 0;
    error              = tmc_write_reg(tmc_config.node_address, TMC_REG_TCOOLTHRS, tcoolthrs);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Configure THIGH register (stall detection threshold)
    uint32_t thigh = (uint32_t)tmc_config.stall_threshold << 0;
    error          = tmc_write_reg(tmc_config.node_address, TMC_REG_THIGH, thigh);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Configure PWMCONF register for StealthChop
    uint32_t pwmconf = 0;
    pwmconf |= (1 << 0);  // PWM_AMPL[7:0] = 1
    pwmconf |= (1 << 8);  // PWM_GRAD[7:0] = 1
    pwmconf |= (1 << 16); // PWM_FREQ[1:0] = 1
    pwmconf |= (1 << 18); // PWM_AUTOSCALE = 1
    pwmconf |= (1 << 19); // PWM_AUTOGRAD = 1
    pwmconf |= (1 << 20); // FREEWHEEL[1:0] = 1

    error = tmc_write_reg(tmc_config.node_address, TMC_REG_PWMCONF, pwmconf);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    return TMC_ERROR_OK;
}

tmc_error_t tmc_set_current(uint16_t hold_current, uint16_t run_current, uint16_t hold_delay)
{
    tmc_config.current_hold       = hold_current;
    tmc_config.current_run        = run_current;
    tmc_config.current_hold_delay = hold_delay;

    uint32_t ihold_irun = 0;
    ihold_irun |= (hold_current & 0x1F) << 0; // IHOLD[4:0]
    ihold_irun |= (run_current & 0x1F) << 8;  // IRUN[4:0]
    ihold_irun |= (hold_delay & 0x0F) << 16;  // IHOLDDELAY[3:0]

    return tmc_write_reg(tmc_config.node_address, TMC_REG_IHOLD_IRUN, ihold_irun);
}

tmc_error_t tmc_set_mode(tmc_mode_t mode)
{
    tmc_config.mode = mode;

    // Configure CHOPCONF register for mode
    uint32_t    chopconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_config.node_address, TMC_REG_CHOPCONF, &chopconf);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    if (mode == TMC_STEALTHCHOP)
    {
        chopconf &= ~(1 << 31); // Clear CHM bit for StealthChop
    }
    else
    {
        chopconf |= (1 << 31); // Set CHM bit for SpreadCycle
    }

    return tmc_write_reg(tmc_config.node_address, TMC_REG_CHOPCONF, chopconf);
}

tmc_error_t tmc_set_microstepping(const tmc_microstepping_t stepping)
{
    tmc_config.microstepping = stepping;

    uint32_t    chopconf = 0;
    tmc_error_t error    = tmc_read_reg(tmc_config.node_address, TMC_REG_CHOPCONF, &chopconf);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Clear and set MRES bits
    chopconf &= ~(0x07 << 0);
    chopconf |= (stepping & 0x07) << 0;

    return tmc_write_reg(tmc_config.node_address, TMC_REG_CHOPCONF, chopconf);
}

tmc_error_t tmc_set_thresholds(uint8_t stealthchop_threshold,
                               uint8_t coolstep_threshold,
                               uint8_t stall_threshold)
{
    tmc_config.stealthchop_threshold = stealthchop_threshold;
    tmc_config.coolstep_threshold    = coolstep_threshold;
    tmc_config.stall_threshold       = stall_threshold;

    tmc_error_t error;

    // Set StealthChop threshold
    uint32_t tpwmthrs = (uint32_t)stealthchop_threshold << 0;
    error             = tmc_write_reg(tmc_config.node_address, TMC_REG_TPWMTHRS, tpwmthrs);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Set CoolStep threshold
    uint32_t tcoolthrs = (uint32_t)coolstep_threshold << 0;
    error              = tmc_write_reg(tmc_config.node_address, TMC_REG_TCOOLTHRS, tcoolthrs);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Set stall detection threshold
    uint32_t thigh = (uint32_t)stall_threshold << 0;
    return tmc_write_reg(tmc_config.node_address, TMC_REG_THIGH, thigh);
}

tmc_error_t tmc_enable(bool enable)
{
    tmc_config.enabled = enable;

    if (enable)
    {
        // Enable by setting IHOLD to non-zero
        return tmc_set_current(
            tmc_config.current_hold, tmc_config.current_run, tmc_config.current_hold_delay);
    }
    else
    {
        // Disable by setting IHOLD to zero
        return tmc_set_current(0, tmc_config.current_run, tmc_config.current_hold_delay);
    }
}

tmc_error_t tmc_set_direction(bool direction)
{
    tmc_config.direction = direction;
    // Direction is typically controlled by external GPIO or step/dir interface
    // This function is provided for completeness but may need external implementation
    return TMC_ERROR_OK;
}

tmc_error_t tmc_set_speed(float speed_rpm)
{
    if (speed_rpm == 0.0f)
    {
        return tmc_stop();
    }

    // Calculate step frequency
    float steps_per_second =
        fabs(speed_rpm) * tmc_config.steps_per_rev * (1 << tmc_config.microstepping) / 60.0f;
    uint32_t frequency = (uint32_t)steps_per_second;

    return tmc_set_frequency(frequency);
}

tmc_error_t tmc_set_frequency(uint32_t frequency_hz)
{
    tmc_config.step_frequency = frequency_hz;
    tmc_config.direction      = (frequency_hz > 0);

    // Set direction
    tmc_set_direction(tmc_config.direction);

    // The actual step frequency is typically controlled by external timer/PWM
    // This function sets up the configuration for external control
    return TMC_ERROR_OK;
}

tmc_error_t tmc_start(const float speed_rpm)
{
    // Enable the driver
    tmc_error_t error = tmc_enable(true);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Set the speed
    return tmc_set_speed(speed_rpm);
}

tmc_error_t tmc_stop(void)
{
    tmc_config.step_frequency = 0;

    // Disable the driver
    return tmc_enable(false);
}

tmc_error_t tmc_get_status(uint32_t *status)
{
    if (status == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, status);
}

tmc_error_t tmc_get_position(uint32_t *position)
{
    if (position == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_read_reg(tmc_config.node_address, TMC_REG_XDIRECT, position);
}

tmc_error_t tmc_get_speed(uint32_t *speed)
{
    if (speed == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    return tmc_read_reg(tmc_config.node_address, TMC_REG_TSTEP, speed);
}

tmc_error_t tmc_get_current(uint16_t *current)
{
    if (current == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Extract current from DRV_STATUS register
    *current = (uint16_t)((drv_status >> 0) & 0x1F);
    return TMC_ERROR_OK;
}

tmc_error_t tmc_get_temperature(uint16_t *temperature)
{
    if (temperature == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Extract temperature from DRV_STATUS register
    *temperature = (uint16_t)((drv_status >> 16) & 0xFF);
    return TMC_ERROR_OK;
}

tmc_error_t tmc_get_stall_status(bool *stalled)
{
    if (stalled == NULL)
    {
        return TMC_ERROR_INVALID_PARAM;
    }

    uint32_t    drv_status = 0;
    tmc_error_t error      = tmc_read_reg(tmc_config.node_address, TMC_REG_DRV_STATUS, &drv_status);
    if (error != TMC_ERROR_OK)
    {
        return error;
    }

    // Check stall detection bit
    *stalled = (bool)((drv_status >> 24) & 0x01);
    return TMC_ERROR_OK;
}
