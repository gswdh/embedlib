#ifndef INA230_H
#define INA230_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * INA230 / INA226 bus-voltage monitor (I2C).
 *
 * Portable driver: the register logic lives here, while the I2C transfers are
 * delegated to two hooks the application implements for its platform.
 * ------------------------------------------------------------------------- */

typedef enum
{
    INA230_OK = 0,
    INA230_ERR_I2C,
    INA230_ERR_ID,
} ina230_error_t;

/* 7-bit I2C address with A0 and A1 tied to GND. */
#define INA230_I2C_ADDR_DEFAULT (0x40U)

/* -------------------------------------------------------------------------
 * Platform hooks — implemented by the application.
 * ------------------------------------------------------------------------- */

/**
 * @brief Read @p len bytes starting at register pointer @p reg from the device
 *        at 7-bit address @p dev_addr.
 * @return INA230_OK on success, INA230_ERR_I2C on any bus error.
 */
ina230_error_t
ina230_i2c_read(const uint8_t dev_addr, const uint8_t reg, uint8_t *rx, const uint16_t len);

/**
 * @brief Write @p len bytes to register pointer @p reg of the device at 7-bit
 *        address @p dev_addr.
 * @return INA230_OK on success, INA230_ERR_I2C on any bus error.
 */
ina230_error_t
ina230_i2c_write(const uint8_t dev_addr, const uint8_t reg, const uint8_t *tx, const uint16_t len);

/* -------------------------------------------------------------------------
 * Device API.
 * ------------------------------------------------------------------------- */

/**
 * @brief Confirm the INA230 is present (manufacturer ID) and configure it for
 *        continuous bus-voltage conversion.  The I2C bus must already be up.
 * @return INA230_OK when present and configured.
 */
ina230_error_t ina230_init(void);

/**
 * @brief Read the bus voltage, in volts.
 */
ina230_error_t ina230_read_voltage(float *volts);

#endif /* INA230_H */
