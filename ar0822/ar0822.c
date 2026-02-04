#include "ar0822.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

/* Cached row time in nanoseconds - calculated once during initialization */
static uint32_t g_cached_row_time_ns = 0U;
static bool     g_row_time_cached    = false;

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

ar_error_t __attribute__((weak))
ar_i2c_write(const uint16_t reg, const uint8_t *data, const uint32_t len)
{
    assert(false && "AR driver interface has not been implemented.");
    return AR_ERROR_I2C_FAIL;
}

ar_error_t __attribute__((weak)) ar_i2c_read(const uint16_t reg, uint8_t *data, const uint32_t len)
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

static uint16_t reverse_endian(uint16_t value) { return (value << 8) | (value >> 8); }

/* Get cached row time or calculate it if not cached */
static ar_error_t ar_get_cached_row_time_ns(uint32_t *time_ns)
{
    if (time_ns == NULL)
    {
        return AR_ERROR_INVALID_PARAM;
    }

    if (g_row_time_cached)
    {
        *time_ns = g_cached_row_time_ns;
        return AR_OK;
    }
    else
    {
        /* Fallback to calculation if not cached and cache the result */
        ar_error_t error = ar_get_row_time_ns(time_ns);
        if (error == AR_OK)
        {
            g_cached_row_time_ns = *time_ns;
            g_row_time_cached    = true;
        }
        return error;
    }
}

static ar_error_t ar_read_reg(const uint16_t reg, uint16_t *const value)
{
    ar_error_t error = ar_i2c_read(reg, (uint8_t *)value, sizeof(uint16_t));
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
    ar_error_t error = ar_i2c_write(reg, (uint8_t *)&value, sizeof(uint16_t));
    if (error != AR_OK)
    {
        return error;
    }

    return AR_OK;
}

static ar_error_t ar_i2c_modify_reg(const uint16_t addr, const uint16_t data, const uint16_t mask)
{
    // Get the value in the current state
    uint16_t   current = 0;
    ar_error_t error   = ar_read_reg(addr, &current);
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
    uint16_t status  = 0;
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
    ar_delay_ms(10U);

    //  Set the register values
    ar_error_t error = ar_reg_init(config, len);
    if (error != AR_OK)
    {
        return error;
    }

    // Cache the row time after successful initialization
    error = ar_get_cached_row_time_ns(&g_cached_row_time_ns);
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

ar_error_t ar_trigger_frame(void) { return AR_OK; }

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
    uint16_t reg_value   = 0;
    uint32_t row_time_ns = 0;

    if (ar_read_reg(AR_REG_COARSE_INT, &reg_value) != AR_OK)
    {
        return AR_ERROR_I2C_FAIL;
    }

    /* Get the cached row time */
    ar_error_t error = ar_get_cached_row_time_ns(&row_time_ns);
    if (error != AR_OK)
    {
        return error;
    }

    /* Convert nanoseconds to seconds and multiply by number of rows */
    *time_s = ((float)(reg_value)) * ((float)row_time_ns) / 1000000000.0f;

    return AR_OK;
}

ar_error_t ar_set_shutter_time_s(const float time_s)
{
    uint32_t row_time_ns = 0;

    /* Get the cached row time */
    ar_error_t error = ar_get_cached_row_time_ns(&row_time_ns);
    if (error != AR_OK)
    {
        return error;
    }

    /* Convert row time from nanoseconds to seconds */
    const float row_time_s = (float)row_time_ns / 1000000000.0f;

    /* Calculate number of rows needed for the desired shutter time */
    const uint16_t reg_value = (uint16_t)(time_s / row_time_s);

    if (ar_write_reg(AR_REG_COARSE_INT, reg_value) != AR_OK)
    {
        return AR_ERROR_I2C_FAIL;
    }

    return AR_OK;
}

ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y) { return AR_OK; }

ar_error_t ar_set_pin_function(const ar_pin_t pin, const ar_function_t function)
{
    ar_error_t error;
    uint16_t   gpio_control1 = 0U;
    uint16_t   gpio_control2 = 0U;
    uint16_t   gpio_select   = 0U;
    uint16_t   pin_mask;
    uint16_t   pin_shift;
    uint16_t   function_value;
    bool       is_input_function  = false;
    bool       is_output_function = false;

    /* Validate input parameters */
    if ((pin >= AR_PIN_MAX) || (function >= AR_FUNC_MAX))
    {
        return AR_ERROR_INVALID_PARAM;
    }

    /* Determine if this is an input or output function */
    if (function <= AR_FUNC_STANDBY)
    {
        is_input_function = true;
    }
    else
    {
        is_output_function = true;
    }

    /* Calculate pin-specific bit positions and masks */
    pin_shift = (uint16_t)(pin * 4U);
    pin_mask  = (uint16_t)(0x0FU << pin_shift);

    /* Read current register values */
    error = ar_read_reg(AR_REG_GPIO_CONTROL1, &gpio_control1);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_read_reg(AR_REG_GPIO_CONTROL2, &gpio_control2);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_read_reg(AR_REG_GPIO_SELECT, &gpio_select);
    if (error != AR_OK)
    {
        return error;
    }

    /* Clear previous configuration for this pin */
    gpio_control1 &= (uint16_t)~pin_mask;               /* Clear output enable */
    gpio_control1 &= (uint16_t) ~(pin_mask << 4U);      /* Clear input disable */
    gpio_control2 &= (uint16_t) ~(0x03U << (pin * 2U)); /* Clear input select */
    gpio_select &= (uint16_t) ~(0x0FU << pin_shift);    /* Clear output select */

    if (is_input_function)
    {
        /* Configure as input function */
        gpio_control1 |= (uint16_t)(1U << (pin + 4U)); /* Disable input buffer */
        gpio_control1 &= (uint16_t) ~(1U << pin);      /* Disable output driver */

        /* Map function to input select value */
        switch (function)
        {
        case AR_FUNC_NO_INPUT:
            function_value = 0U;
            break;
        case AR_FUNC_OUTPUT_ENABLE_N:
            function_value = 1U;
            break;
        case AR_FUNC_TRIGGER:
            function_value = 2U;
            break;
        case AR_FUNC_STANDBY:
            function_value = 3U;
            break;
        default:
            return AR_ERROR_INVALID_PARAM;
        }

        /* Enable input buffer and set input select */
        gpio_control1 &= (uint16_t) ~(1U << (pin + 4U)); /* Enable input buffer */
        gpio_control2 |= (uint16_t)(function_value << (pin * 2U));
    }
    else if (is_output_function)
    {
        /* Configure as output function */
        gpio_control1 |= (uint16_t)(1U << (pin + 4U)); /* Disable input buffer */
        gpio_control1 |= (uint16_t)(1U << pin);        /* Enable output driver */

        /* Map function to output select value */
        switch (function)
        {
        case AR_FUNC_BOOT_STATUS_0:
            function_value = 0U;
            break;
        case AR_FUNC_BOOT_STATUS_1:
            function_value = 1U;
            break;
        case AR_FUNC_BOOT_STATUS_2:
            function_value = 2U;
            break;
        case AR_FUNC_SHUTTER_READOUT:
            function_value = 3U;
            break;
        case AR_FUNC_FLASH:
            function_value = 4U;
            break;
        case AR_FUNC_SHUTTER:
            function_value = 5U;
            break;
        case AR_FUNC_LINE_VALID:
            function_value = 6U;
            break;
        case AR_FUNC_FRAME_VALID:
            function_value = 7U;
            break;
        case AR_FUNC_PIXCLK:
            function_value = 8U;
            break;
        case AR_FUNC_MD_MOTION:
            function_value = 9U;
            break;
        case AR_FUNC_MD_STOP:
            function_value = 10U;
            break;
        case AR_FUNC_NEW_ROW_PULSE:
            function_value = 11U;
            break;
        case AR_FUNC_NEW_FRAME_PULSE:
            function_value = 12U;
            break;
        default:
            return AR_ERROR_INVALID_PARAM;
        }

        gpio_select |= (uint16_t)(function_value << pin_shift);
    }

    /* Write updated register values */
    error = ar_write_reg(AR_REG_GPIO_CONTROL1, gpio_control1);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_write_reg(AR_REG_GPIO_CONTROL2, gpio_control2);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_write_reg(AR_REG_GPIO_SELECT, gpio_select);
    if (error != AR_OK)
    {
        return error;
    }

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

ar_error_t ar_get_row_time_ns(uint32_t *time_ns)
{
    /* Input validation */
    if (time_ns == NULL)
    {
        return AR_ERROR_INVALID_PARAM;
    }

    /* Constants for calculation */
    const uint64_t EXTCLK_HZ              = 19200000ULL; /* 19.2 MHz external clock */
    const uint64_t NANOSECONDS_PER_SECOND = 1000000000ULL;

    /* Register values */
    uint16_t line_length_pck = 0U;
    uint16_t pre_pll_clk_div = 0U;
    uint16_t pll_multiplier  = 0U;
    uint16_t vt_sys_clk_div  = 0U;
    uint16_t vt_pix_clk_div  = 0U;

    /* Read all required registers */
    ar_error_t error = ar_read_reg(AR_REG_LINE_LENGTH_PCK, &line_length_pck);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_read_reg(AR_REG_PRE_PLL_CLK_DIV, &pre_pll_clk_div);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_read_reg(AR_REG_PLL_MULTIPLIER, &pll_multiplier);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_read_reg(AR_REG_VT_SYS_CLK_DIV, &vt_sys_clk_div);
    if (error != AR_OK)
    {
        return error;
    }

    error = ar_read_reg(AR_REG_VT_PIX_CLK_DIV, &vt_pix_clk_div);
    if (error != AR_OK)
    {
        return error;
    }

    /* Check for divide-by-zero conditions */
    if ((pre_pll_clk_div == 0U) || (vt_sys_clk_div == 0U) || (vt_pix_clk_div == 0U))
    {
        return AR_ERROR_INVALID_PARAM;
    }

    /* Calculate CLK_PIX using 64-bit arithmetic to avoid overflow */
    /* CLK_PIX = (EXTCLK * pll_multiplier) / (pre_pll_clk_div * vt_sys_clk_div * vt_pix_clk_div) */
    const uint64_t numerator = EXTCLK_HZ * (uint64_t)pll_multiplier;
    const uint64_t denominator =
        (uint64_t)pre_pll_clk_div * (uint64_t)vt_sys_clk_div * (uint64_t)vt_pix_clk_div;

    if (denominator == 0ULL)
    {
        return AR_ERROR_INVALID_PARAM;
    }

    const uint64_t clk_pix_hz = numerator / denominator;

    /* Calculate row time in seconds: T_ROW = line_length_pck / CLK_PIX */
    if (clk_pix_hz == 0ULL)
    {
        return AR_ERROR_INVALID_PARAM;
    }

    const uint64_t row_time_s_numerator = (uint64_t)line_length_pck * NANOSECONDS_PER_SECOND;
    const uint64_t row_time_ns          = row_time_s_numerator / clk_pix_hz;

    /* Check for overflow in the final result */
    if (row_time_ns > UINT32_MAX)
    {
        return AR_ERROR_INVALID_PARAM;
    }

    *time_ns = (uint32_t)row_time_ns;

    return AR_OK;
}