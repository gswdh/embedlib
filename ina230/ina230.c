#include "ina230.h"

/* Register pointers. */
#define INA230_REG_CONFIG      (0x00U)
#define INA230_REG_BUS_VOLTAGE (0x02U)
#define INA230_REG_MFR_ID      (0xFEU)

/* Manufacturer ID reported by the INA230 (ASCII "TI"). */
#define INA230_MFR_ID (0x5449U)

/* Power-on default configuration: continuous shunt+bus conversion, 1-sample
 * average, 1.1 ms conversion times.  Bus voltage needs no calibration, so the
 * default is sufficient; it is written explicitly to be deterministic. */
#define INA230_CONFIG_DEFAULT (0x4127U)

/* Bus-voltage register LSB is 1.25 mV. */
#define INA230_BUS_VOLTAGE_LSB_V (0.00125F)

static ina230_error_t ina230_read_reg(const uint8_t reg, uint16_t *const value)
{
    uint8_t              rx[2] = {0};
    const ina230_error_t err =
        ina230_i2c_read(INA230_I2C_ADDR_DEFAULT, reg, rx, (uint16_t)sizeof(rx));
    if (err != INA230_OK)
    {
        return err;
    }
    *value = (uint16_t)(((uint16_t)rx[0] << 8U) | (uint16_t)rx[1]);
    return INA230_OK;
}

static ina230_error_t ina230_write_reg(const uint8_t reg, const uint16_t value)
{
    const uint8_t tx[2] = {(uint8_t)(value >> 8U), (uint8_t)(value & 0xFFU)};
    return ina230_i2c_write(INA230_I2C_ADDR_DEFAULT, reg, tx, (uint16_t)sizeof(tx));
}

ina230_error_t ina230_init(void)
{
    uint16_t             mfr_id = 0U;
    const ina230_error_t err    = ina230_read_reg(INA230_REG_MFR_ID, &mfr_id);
    if (err != INA230_OK)
    {
        return err;
    }
    if (mfr_id != INA230_MFR_ID)
    {
        return INA230_ERR_ID;
    }

    return ina230_write_reg(INA230_REG_CONFIG, INA230_CONFIG_DEFAULT);
}

ina230_error_t ina230_read_voltage(float *const volts)
{
    uint16_t             raw = 0U;
    const ina230_error_t err = ina230_read_reg(INA230_REG_BUS_VOLTAGE, &raw);
    if (err != INA230_OK)
    {
        return err;
    }
    *volts = (float)raw * INA230_BUS_VOLTAGE_LSB_V;
    return INA230_OK;
}
