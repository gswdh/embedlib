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
    return 0;
}

ar_error_t __attribute__((weak)) ar_i2c_write(const uint16_t reg, const uint8_t *data, const uint32_t len)
{
    assert(false && "AR driver interface has not been implemented.");
    return AR_I2C_FAIL;
}

ar_error_t __attribute__((weak)) ar_i2c_read(const uint16_t reg, const uint8_t *data, const uint32_t len)
{
    assert(false && "AR driver interface has not been implemented.");
    return AR_I2C_FAIL;
}
void __attribute__((weak)) ar_delay_ms(const uint32_t time_ms)
{
    assert(false && "AR driver interface has not been implemented.");
}

static ar_error_t _ar_reg_init(const ar_reg_write_t *config, uint32_t len)
{
    // Go through all the register settings
    for (uint32_t i = 0; i < len; i++)
    {
        ar_error_t error = AR_OK;
        uint16_t data = 0;

        // Get the value in the current state
        error = ar_i2c_read(config[i].addr, (uint8_t *)&data, sizeof(data));
        if (error != AR_OK)
        {
            return error;
        }

        // Clear the relevant bits
        data &= ~config[i].mask;

        // Set the new value
        data |= config[i].data & config[i].mask;

        // Write back to the device
        error = ar_i2c_write(config[i].addr, (uint8_t *)&data, sizeof(data));
        if (error != AR_OK)
        {
            return error;
        }
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
    ar_enable_clock(false);

    // Wait to settle (at least 100us)
    ar_delay_ms(1);

    // Bring out of reset
    ar_set_nrst(true);

    // Wait to settle (at least 1600000 clock cycles)
    ar_delay_ms(1000);

    //  Set the register values
    ar_error_t error = _ar_reg_init(config, len);

    if (error != AR_OK)
    {
        return error;
    }

    return AR_INIT_FAIL;
}
