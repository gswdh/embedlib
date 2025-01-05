#include "ar0822.h"

#include <assert.h>
#include <math.h>

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

static ar_error_t ar_i2c_modify_reg(const uint16_t addr, const uint16_t data, const uint16_t mask)
{
    // Get the value in the current state
    uint16_t current = 0;
    ar_error_t error = ar_read_reg(addr, &current);
    if (error != AR_OK)
    {
        return error;
    }

    // Clear the relevant bits
    current &= ~mask;

    // Set the new value
    current |= data & mask;

    // Write back to the device
    error = ar_write_reg(addr, current);
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
        ar_error_t error = ar_i2c_modify_reg(config[i].addr, config[i].data, config[i].mask);
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

static uint16_t ar_gain_float_to_register(const float value)
{
    // Scale the float value to match the fixed-point format (xxxx.yyyyyyy)
    uint16_t fixed_point = (uint16_t)roundf(value * 128.0f);

    // Ensure it fits within 11 bits
    if (fixed_point > 0x7FF)
    {
        fixed_point = 0x7FF; // Clamp to max value
    }

    return fixed_point;
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

    // Enable colour gains
    ar_i2c_modify_reg(AR_REG_HDR_CONTROL, 0x0010, 0x0010);

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

ar_error_t ar_get_gain(float *gain_db)
{
    uint16_t reg_value = 0;
    if (ar_read_reg(AR_REG_GLOBAL_GAIN, &reg_value) != AR_OK)
    {
        return AR_ERROR_I2C_FAIL;
    }

    *gain_db = ((float)(reg_value & 0xFF)) * AR_REG_GLOBAL_GAIN_STEP;

    return AR_OK;
}

ar_error_t ar_set_gain(const float gain_db)
{
    const uint16_t reg_value = (uint16_t)(gain_db / AR_REG_GLOBAL_GAIN_STEP) & 0x00FF;

    if (reg_value > AR_REG_GLOBAL_GAIN_MAX)
    {
        return AR_ERROR_INVALID_PARAM;
    }

    if (ar_write_reg(AR_REG_GLOBAL_GAIN, reg_value) != AR_OK)
    {
        return AR_ERROR_I2C_FAIL;
    }

    return AR_OK;
}

ar_error_t ar_set_colour_gain(const float gain, const ar_colour_t colour)
{
    const uint16_t reg_value = ar_gain_float_to_register(gain);

    switch (colour)
    {
    case AR_COLOUR_R:
        if (ar_write_reg(AR_REG_GAIN_R, reg_value) != AR_OK)
        {
            return AR_ERROR_I2C_FAIL;
        }
        break;
    case AR_COLOUR_G:
        if (ar_write_reg(AR_REG_GAIN_G1, reg_value) != AR_OK)
        {
            return AR_ERROR_I2C_FAIL;
        }
        if (ar_write_reg(AR_REG_GAIN_G2, reg_value) != AR_OK)
        {
            return AR_ERROR_I2C_FAIL;
        }
        break;
    case AR_COLOUR_B:
        if (ar_write_reg(AR_REG_GAIN_B, reg_value) != AR_OK)
        {
            return AR_ERROR_I2C_FAIL;
        }
        break;
    default:
        return AR_ERROR_INVALID_PARAM;
        break;
    }

    return AR_OK;
}

ar_error_t ar_get_shutter_time_s(float *time_s)
{
    uint16_t reg_value = 0;
    if (ar_read_reg(AR_REG_COARSE_INT, &reg_value) != AR_OK)
    {
        return AR_ERROR_I2C_FAIL;
    }

    *time_s = ((float)(reg_value)) * AR_ROW_TIME_S;

    return AR_OK;
}

ar_error_t ar_set_shutter_time_s(const float time_s)
{
    const uint16_t reg_value = (uint16_t)(time_s / AR_ROW_TIME_S);

    if (ar_write_reg(AR_REG_COARSE_INT, reg_value) != AR_OK)
    {
        return AR_ERROR_I2C_FAIL;
    }

    return AR_OK;
}

ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y)
{
    return AR_OK;
}

char *ar_debug_gpio_state(const uint8_t gpio_state)
{
    switch (gpio_state & 0x07)
    {
    case 0:
        return "M3ROM upload in progress";
        break;
    case 1:
        return "critical OTPM upload in progress";
        break;
    case 2:
        return "non-critical OTPM upload in progress";
        break;
    case 3:
        return "register scan in progress";
        break;
    case 4:
        return "startup MBIST in progress";
        break;
    case 5:
        return "test frame in progress";
        break;
    case 6:
        return "sensor in standby";
        break;
    case 7:
        return "sensor streaming";
        break;
    default:
        return "";
        break;
    }
}