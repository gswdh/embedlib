#include "ar0822.h"

#include <assert.h>

void __attribute__((weak)) ar_set_nrst(const bool en)
{
    assert(false && "AR driver interface has not been implemented.");
}

void __attribute__((weak)) ar_set_xshutdown(const bool en)
{
    assert(false && "AR driver interface has not been implemented.");
}

void __attribute__((weak)) ar_enable_clock(const bool en)
{
    assert(false && "AR driver interface has not been implemented.");
}

uint8_t __attribute__((weak)) ar_get_gpio(void)
{
    assert(false && "AR driver interface has not been implemented.");
    return 0U;
}

ar_error_t __attribute__((weak)) ar_i2c_write(const uint16_t reg, const uint8_t *data, const uint32_t len)
{
    assert(false && "AR driver interface has not been implemented.");
    return AR_ERROR_I2C_FAIL;
}

ar_error_t __attribute__((weak)) ar_i2c_read(const uint16_t reg, const uint8_t *data, const uint32_t len)
{
    assert(false && "AR driver interface has not been implemented.");
    return AR_ERROR_I2C_FAIL;
}

void __attribute__((weak)) ar_delay_ms(const uint32_t time_ms)
{
    assert(false && "AR driver interface has not been implemented.");
}

uint32_t __attribute__((weak)) ar_tick_ms(void)
{
    assert(false && "AR driver interface has not been implemented.");
    return 0U;
}

static uint16_t reverse_endian(uint16_t value)
{
    return (value << 8) | (value >> 8);
}

static ar_error_t ar_read_reg(const uint16_t reg, uint16_t *const value)
{
    ar_error_t error = ar_i2c_read(reg, (uint8_t *)value, sizeof(value));
    if (error != AR_OK)
    {
        return error;
    }

    // Reverse the edianess
    *value = reverse_endian(*value);

    return AR_OK;
}

static ar_error_t ar_write_reg(const uint16_t reg, uint16_t value)
{
    // Reverse the edianess
    value = reverse_endian(value);

    // Write back to the device
    ar_error_t error = ar_i2c_write(reg, (uint8_t *)&value, sizeof(value));
    if (error != AR_OK)
    {
        return error;
    }

    return AR_OK;
}

static ar_error_t ar_reg_init(const ar_reg_write_t *config, uint32_t len)
{
    // Go through all the register settings
    for (uint32_t i = 0; i < len; i++)
    {
        ar_error_t error = AR_OK;
        uint16_t data = 0;

        // Get the value in the current state
        error = ar_read_reg(config[i].addr, &data);
        if (error != AR_OK)
        {
            return error;
        }

        // Clear the relevant bits
        data &= ~config[i].mask;

        // Set the new value
        data |= config[i].data & config[i].mask;

        // Write back to the device
        error = ar_write_reg(config[i].addr, data);
        if (error != AR_OK)
        {
            return error;
        }
    }

    // Wait for the status to be in the stream mode
    uint16_t status = 0;
    uint32_t t_start = ar_tick_ms();
    while ((status & AR_REG_FRAME_STATUS_STREAM_BIT) != AR_REG_FRAME_STATUS_STREAM_BIT)
    {
        ar_error_t error = ar_frame_status(&status);
        if (error != AR_OK)
        {
            return error;
        }

        // Timeout condition
        if (ar_tick_ms() > (t_start + AR_INIT_SYNC_TO_MS))
        {
            return AR_ERROR_SYNC_TO;
        }

        // A little delay to save the CPU
        ar_delay_ms(10);
    }

    // Went well
    return AR_OK;
}

ar_error_t ar_init(const ar_reg_write_t *config, uint32_t len)
{
    // Start off with the sensor in reset and no clock
    ar_set_xshutdown(false);
    ar_set_nrst(false);
    ar_enable_clock(false);

    // Wait to settle (at least 100us)
    ar_delay_ms(1);

    // Shutdown bring online
    ar_set_xshutdown(true);

    // Wait to settle (at least 100us)
    ar_delay_ms(1);

    // Enable the clock
    ar_enable_clock(true);

    // Wait to settle (at least 100us)
    ar_delay_ms(1);

    // Bring out of reset
    ar_set_nrst(true);

    // Wait to settle (at least 1600000 clock cycles)
    ar_delay_ms(1000);

    //  Set the register values
    ar_error_t error = ar_reg_init(config, len);

    if (error != AR_OK)
    {
        return error;
    }

    return AR_OK;
}

ar_error_t ar_frame_status(uint16_t *const status)
{
    ar_error_t error = ar_read_reg(AR_REG_FRAME_STATUS, status);
    if (error != AR_OK)
    {
        return error;
    }

    return AR_OK;
}

ar_error_t ar_gpio_config(uint16_t *const config)
{
    ar_error_t error = ar_read_reg(AR_REG_FRAME_STATUS, config);
    if (error != AR_OK)
    {
        return error;
    }

    return AR_OK;
}

ar_error_t ar_trigger_frame(void)
{
    return AR_OK;
}

ar_error_t ar_set_gain(const float gain)
{
    return AR_OK;
}

ar_error_t ar_set_shutter_time_s(const float time_s)
{
    return AR_OK;
}

ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y)
{
    return AR_OK;
}
