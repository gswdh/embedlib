#include "adxl355.h"

/* SPI frame buffer ceiling — covers address byte + largest non-FIFO burst */
#define SPI_BUF_LEN (16U)

/* Acceleration data is 20-bit, left-justified in three registers.
 * XDATA3 holds bits[19:12], XDATA2 holds bits[11:4], XDATA1 holds bits[3:0]
 * in the upper nibble.  Shift/mask constants for reconstruction: */
#define ACCEL_SHIFT_D3   (12U)
#define ACCEL_SHIFT_D2   (4U)
#define ACCEL_SHIFT_D1   (4U)
#define ACCEL_20BIT_SIGN (0x00080000UL)
#define ACCEL_20BIT_EXT  (0xFFF00000UL)

/* Temperature is 12-bit unsigned, TEMP2 holds bits[11:8], TEMP1 holds bits[7:0] */
#define TEMP_SHIFT_H     (8U)

/* FIFO: each location is 24 bits (3 bytes).  The bottom 4 bits carry
 * 2 virtual zero bits, an empty-indicator, and the x-axis marker.
 * The 20 acceleration bits occupy [23:4]. */
#define FIFO_DATA_SHIFT  (4U)

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static adxl_error_t spi_read(const uint8_t addr, uint8_t *data,
                              const uint8_t len)
{
    uint8_t      tx[SPI_BUF_LEN];
    uint8_t      rx[SPI_BUF_LEN];
    adxl_error_t err;
    uint8_t      i;

    /* 1 address byte + data bytes must fit in the internal buffer */
    if (len >= SPI_BUF_LEN)
    {
        return ADXL_ERR_BURST_READ_LEN;
    }

    tx[0] = (uint8_t)(((uint32_t)addr << 1U) | 0x01U);
    for (i = 1U; i <= len; i++)
    {
        tx[i] = 0x00U;
    }

    err = adxl_spi_transfer(tx, rx, (uint8_t)(len + 1U));
    if (err != ADXL_OK)
    {
        return ADXL_ERR_BURST_READ_COMM;
    }

    for (i = 0U; i < len; i++)
    {
        data[i] = rx[i + 1U];
    }

    return ADXL_OK;
}

static adxl_error_t spi_write(const uint8_t addr, const uint8_t *data,
                               const uint8_t len)
{
    uint8_t      tx[SPI_BUF_LEN];
    uint8_t      rx[SPI_BUF_LEN];
    adxl_error_t err;
    uint8_t      i;

    if (len >= SPI_BUF_LEN)
    {
        return ADXL_ERR_REG_WRITE_COMM;
    }

    tx[0] = (uint8_t)(((uint32_t)addr << 1U) & 0xFEU);
    for (i = 0U; i < len; i++)
    {
        tx[i + 1U] = data[i];
    }

    err = adxl_spi_transfer(tx, rx, (uint8_t)(len + 1U));
    if (err != ADXL_OK)
    {
        return ADXL_ERR_REG_WRITE_COMM;
    }

    return ADXL_OK;
}

/* Reconstruct a 20-bit twos-complement signed value from three raw bytes.
 * d3 = bits[19:12], d2 = bits[11:4], d1 = bits[7:4] holds bits[3:0]. */
static int32_t accel_20bit_to_signed(const uint8_t d3, const uint8_t d2,
                                     const uint8_t d1)
{
    const uint32_t raw = ((uint32_t)d3 << ACCEL_SHIFT_D3)
                       | ((uint32_t)d2 << ACCEL_SHIFT_D2)
                       | ((uint32_t)d1 >> ACCEL_SHIFT_D1);

    const int32_t val = ((raw & ACCEL_20BIT_SIGN) != 0UL)
                      ? (int32_t)(raw | ACCEL_20BIT_EXT)
                      : (int32_t)raw;

    return val;
}

/* Reconstruct a 20-bit signed value from three FIFO bytes.
 * Byte layout: b0=bits[23:16], b1=bits[15:8], b2=bits[7:0].
 * Bits [3:0] of b2 carry virtual(00), empty-flag, x-marker — discard. */
static int32_t fifo_bytes_to_signed(const uint8_t b0, const uint8_t b1,
                                    const uint8_t b2)
{
    const uint32_t raw = (((uint32_t)b0 << 16U) | ((uint32_t)b1 << 8U) |
                           (uint32_t)b2) >> FIFO_DATA_SHIFT;

    const int32_t val = ((raw & ACCEL_20BIT_SIGN) != 0UL)
                      ? (int32_t)(raw | ACCEL_20BIT_EXT)
                      : (int32_t)raw;

    return val;
}

/* -------------------------------------------------------------------------
 * Low-level register access
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_reg_read(const uint8_t addr, uint8_t *val)
{
    adxl_error_t err;

    if (val == NULL)
    {
        return ADXL_ERR_REG_READ_NULL;
    }

    err = spi_read(addr, val, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_REG_READ_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_reg_write(const uint8_t addr, const uint8_t val)
{
    const adxl_error_t err = spi_write(addr, &val, 1U);

    if (err != ADXL_OK)
    {
        return ADXL_ERR_REG_WRITE_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_reg_read_burst(const uint8_t addr, uint8_t *data,
                                   const uint8_t len)
{
    adxl_error_t err;

    if (data == NULL)
    {
        return ADXL_ERR_BURST_READ_NULL;
    }

    if (len == 0U)
    {
        return ADXL_ERR_BURST_READ_LEN;
    }

    err = spi_read(addr, data, len);
    if (err == ADXL_ERR_BURST_READ_LEN)
    {
        return ADXL_ERR_BURST_READ_LEN;
    }

    if (err != ADXL_OK)
    {
        return ADXL_ERR_BURST_READ_COMM;
    }

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Device control
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_init(const adxl_range_t range, const adxl_odr_lpf_t odr)
{
    uint8_t      id;
    uint8_t      reg;
    adxl_error_t err;

    if ((range != ADXL_RANGE_2G) && (range != ADXL_RANGE_4G) &&
        (range != ADXL_RANGE_8G))
    {
        return ADXL_ERR_INIT_RANGE;
    }

    if ((uint8_t)odr > (uint8_t)ADXL_ODR_3HZ)
    {
        return ADXL_ERR_INIT_FILTER;
    }

    err = spi_read(ADXL_REG_DEVID_AD, &id, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_INIT_COMM;
    }
    if (id != ADXL_DEVID_AD_VALUE)
    {
        return ADXL_ERR_INIT_DEVID_AD;
    }

    err = spi_read(ADXL_REG_DEVID_MST, &id, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_INIT_COMM;
    }
    if (id != ADXL_DEVID_MST_VALUE)
    {
        return ADXL_ERR_INIT_DEVID_MST;
    }

    err = spi_read(ADXL_REG_PARTID, &id, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_INIT_COMM;
    }
    if (id != ADXL_PARTID_VALUE)
    {
        return ADXL_ERR_INIT_PARTID;
    }

    /* Device must be in standby before configuring registers */
    reg = ADXL_PWR_STANDBY_BIT;
    err = spi_write(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_INIT_STANDBY;
    }

    /* Configure range (preserve I2C_HS reset default of 1) */
    reg = (uint8_t)(ADXL_I2C_HS_BIT | (uint8_t)range);
    err = spi_write(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_INIT_RANGE;
    }

    /* Configure ODR/LPF, HPF disabled */
    reg = (uint8_t)odr;
    err = spi_write(ADXL_REG_FILTER, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_INIT_FILTER;
    }

    return ADXL_OK;
}

adxl_error_t adxl_standby(void)
{
    uint8_t      reg;
    adxl_error_t err;

    err = spi_read(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_STANDBY_COMM;
    }

    reg = (uint8_t)(reg | ADXL_PWR_STANDBY_BIT);
    err = spi_write(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_STANDBY_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_measure(void)
{
    uint8_t      reg;
    adxl_error_t err;

    err = spi_read(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_MEASURE_COMM;
    }

    reg = (uint8_t)(reg & (uint8_t)(~ADXL_PWR_STANDBY_BIT));
    err = spi_write(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_MEASURE_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_reset(void)
{
    const uint8_t      code = ADXL_RESET_CODE;
    const adxl_error_t err  = spi_write(ADXL_REG_RESET, &code, 1U);

    if (err != ADXL_OK)
    {
        return ADXL_ERR_RESET_COMM;
    }

    /* Datasheet specifies <10 ms startup from standby; wait for NVM load */
    adxl_delay_ms(10U);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Status and identity
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_get_status(uint8_t *status)
{
    adxl_error_t err;

    if (status == NULL)
    {
        return ADXL_ERR_GET_STATUS_NULL;
    }

    err = spi_read(ADXL_REG_STATUS, status, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_GET_STATUS_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_data_ready(uint8_t *ready)
{
    uint8_t status;

    if (ready == NULL)
    {
        return ADXL_ERR_DATA_RDY_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_STATUS, &status, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_DATA_RDY_COMM;
        }
    }

    *ready = (uint8_t)((status & ADXL_STATUS_DATA_RDY) != 0U ? 1U : 0U);

    return ADXL_OK;
}

adxl_error_t adxl_get_device_id(uint8_t *devid_ad, uint8_t *devid_mst,
                                  uint8_t *partid,   uint8_t *revid)
{
    uint8_t buf[4];

    if ((devid_ad == NULL) || (devid_mst == NULL) ||
        (partid == NULL) || (revid == NULL))
    {
        return ADXL_ERR_GET_DEVID_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_DEVID_AD, buf, 4U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_DEVID_COMM;
        }
    }

    *devid_ad  = buf[0];
    *devid_mst = buf[1];
    *partid    = buf[2];
    *revid     = buf[3];

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Data readout
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_get_accel_raw(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t buf[9];

    if ((x == NULL) || (y == NULL) || (z == NULL))
    {
        return ADXL_ERR_GET_ACCEL_NULL;
    }

    /* Burst-read all three axes: XDATA3(0x08) through ZDATA1(0x10) */
    {
        const adxl_error_t err = spi_read(ADXL_REG_XDATA3, buf, 9U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_ACCEL_COMM;
        }
    }

    *x = accel_20bit_to_signed(buf[0], buf[1], buf[2]);
    *y = accel_20bit_to_signed(buf[3], buf[4], buf[5]);
    *z = accel_20bit_to_signed(buf[6], buf[7], buf[8]);

    return ADXL_OK;
}

adxl_error_t adxl_get_temp_raw(uint16_t *temp)
{
    uint8_t buf[2];

    if (temp == NULL)
    {
        return ADXL_ERR_GET_TEMP_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_TEMP2, buf, 2U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_TEMP_COMM;
        }
    }

    /* TEMP2[3:0] = bits[11:8], TEMP1[7:0] = bits[7:0] */
    *temp = (uint16_t)(((uint16_t)(buf[0] & 0x0FU) << TEMP_SHIFT_H) |
                        (uint16_t)buf[1]);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * FIFO
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_get_fifo_entries(uint8_t *count)
{
    uint8_t reg;

    if (count == NULL)
    {
        return ADXL_ERR_FIFO_ENTRIES_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_FIFO_ENTRIES, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_FIFO_ENTRIES_COMM;
        }
    }

    *count = (uint8_t)(reg & 0x7FU);

    return ADXL_OK;
}

adxl_error_t adxl_set_fifo_samples(const uint8_t samples)
{
    if ((samples == 0U) || (samples > (uint8_t)ADXL_FIFO_MAX_SAMPLES))
    {
        return ADXL_ERR_SET_FIFO_SMPL_RANGE;
    }

    {
        const adxl_error_t err = spi_write(ADXL_REG_FIFO_SAMPLES, &samples, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_SET_FIFO_SMPL_COMM;
        }
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_fifo_samples(uint8_t *samples)
{
    uint8_t reg;

    if (samples == NULL)
    {
        return ADXL_ERR_GET_FIFO_SMPL_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_FIFO_SAMPLES, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_FIFO_SMPL_COMM;
        }
    }

    *samples = (uint8_t)(reg & 0x7FU);

    return ADXL_OK;
}

adxl_error_t adxl_fifo_read(adxl_accel_raw_t *samples, const uint8_t count,
                              uint8_t *read_count)
{
    /* FIFO_DATA read address byte is a compile-time constant */
    const uint8_t tx[ADXL_FIFO_BYTES_PER + 1U] = {
        (uint8_t)(((uint32_t)ADXL_REG_FIFO_DATA << 1U) | 0x01U),
        0x00U, 0x00U, 0x00U
    };
    uint8_t rx[ADXL_FIFO_BYTES_PER + 1U];
    uint8_t i;

    if ((samples == NULL) || (read_count == NULL))
    {
        return ADXL_ERR_FIFO_READ_NULL;
    }

    if ((count == 0U) || (count > (uint8_t)ADXL_FIFO_MAX_SAMPLES))
    {
        return ADXL_ERR_FIFO_READ_COUNT;
    }

    *read_count = 0U;

    /* The FIFO writes complete XYZ triplets atomically, so FIFO_ENTRIES is
     * always a multiple of three and reads always start on an X entry.
     * Read entries in fixed X-Y-Z order; no x-marker sync needed. */
    for (i = 0U; i < count; i++)
    {
        adxl_error_t err;

        err = adxl_spi_transfer(tx, rx, (uint8_t)(ADXL_FIFO_BYTES_PER + 1U));
        if (err != ADXL_OK)
        {
            return ADXL_ERR_FIFO_READ_COMM;
        }
        samples[i].x = fifo_bytes_to_signed(rx[1], rx[2], rx[3]);

        err = adxl_spi_transfer(tx, rx, (uint8_t)(ADXL_FIFO_BYTES_PER + 1U));
        if (err != ADXL_OK)
        {
            return ADXL_ERR_FIFO_READ_COMM;
        }
        samples[i].y = fifo_bytes_to_signed(rx[1], rx[2], rx[3]);

        err = adxl_spi_transfer(tx, rx, (uint8_t)(ADXL_FIFO_BYTES_PER + 1U));
        if (err != ADXL_OK)
        {
            return ADXL_ERR_FIFO_READ_COMM;
        }
        samples[i].z = fifo_bytes_to_signed(rx[1], rx[2], rx[3]);

        *read_count = (uint8_t)(*read_count + 1U);
    }

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Filter configuration
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_filter(const adxl_hpf_corner_t hpf,
                              const adxl_odr_lpf_t odr_lpf)
{
    const uint8_t reg = (uint8_t)(((uint8_t)hpf << ADXL_FILTER_HPF_SHIFT) |
                                   (uint8_t)odr_lpf);

    if ((uint8_t)hpf > (uint8_t)ADXL_HPF_238E6)
    {
        return ADXL_ERR_SET_FILTER_HPF;
    }

    if ((uint8_t)odr_lpf > (uint8_t)ADXL_ODR_3HZ)
    {
        return ADXL_ERR_SET_FILTER_ODR;
    }

    {
        const adxl_error_t err = spi_write(ADXL_REG_FILTER, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_SET_FILTER_COMM;
        }
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_filter(adxl_hpf_corner_t *hpf, adxl_odr_lpf_t *odr_lpf)
{
    uint8_t reg;

    if ((hpf == NULL) || (odr_lpf == NULL))
    {
        return ADXL_ERR_GET_FILTER_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_FILTER, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_FILTER_COMM;
        }
    }

    *hpf     = (adxl_hpf_corner_t)((reg & ADXL_FILTER_HPF_MASK) >>
                                     ADXL_FILTER_HPF_SHIFT);
    *odr_lpf = (adxl_odr_lpf_t)(reg & ADXL_FILTER_ODR_MASK);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Range configuration
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_range(const adxl_range_t range)
{
    uint8_t      reg;
    adxl_error_t err;

    if ((range != ADXL_RANGE_2G) && (range != ADXL_RANGE_4G) &&
        (range != ADXL_RANGE_8G))
    {
        return ADXL_ERR_SET_RANGE_INVALID;
    }

    err = spi_read(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_RANGE_COMM;
    }

    reg = (uint8_t)((reg & (uint8_t)(~ADXL_RANGE_MASK)) | (uint8_t)range);
    err = spi_write(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_RANGE_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_range(adxl_range_t *range)
{
    uint8_t reg;

    if (range == NULL)
    {
        return ADXL_ERR_GET_RANGE_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_RANGE, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_RANGE_COMM;
        }
    }

    *range = (adxl_range_t)(reg & ADXL_RANGE_MASK);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Interrupt configuration
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_int_map(const uint8_t map)
{
    const adxl_error_t err = spi_write(ADXL_REG_INT_MAP, &map, 1U);

    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_INT_MAP_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_int_map(uint8_t *map)
{
    if (map == NULL)
    {
        return ADXL_ERR_GET_INT_MAP_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_INT_MAP, map, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_INT_MAP_COMM;
        }
    }

    return ADXL_OK;
}

adxl_error_t adxl_set_int_pol(const uint8_t active_high)
{
    uint8_t      reg;
    adxl_error_t err;

    err = spi_read(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_INT_POL_COMM;
    }

    if (active_high != 0U)
    {
        reg = (uint8_t)(reg | ADXL_INT_POL_BIT);
    }
    else
    {
        reg = (uint8_t)(reg & (uint8_t)(~ADXL_INT_POL_BIT));
    }

    err = spi_write(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_INT_POL_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_int_pol(uint8_t *active_high)
{
    uint8_t reg;

    if (active_high == NULL)
    {
        return ADXL_ERR_GET_INT_POL_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_RANGE, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_INT_POL_COMM;
        }
    }

    *active_high = (uint8_t)((reg & ADXL_INT_POL_BIT) != 0U ? 1U : 0U);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Synchronisation
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_sync(const uint8_t sync_cfg)
{
    uint8_t      reg;
    adxl_error_t err;

    /* Only bits [2:0] are writable; preserve reserved bits */
    err = spi_read(ADXL_REG_SYNC, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_SYNC_COMM;
    }

    reg = (uint8_t)((reg & 0xF8U) | (sync_cfg & 0x07U));
    err = spi_write(ADXL_REG_SYNC, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_SYNC_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_sync(uint8_t *sync_cfg)
{
    if (sync_cfg == NULL)
    {
        return ADXL_ERR_GET_SYNC_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_SYNC, sync_cfg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_SYNC_COMM;
        }
    }

    *sync_cfg = (uint8_t)(*sync_cfg & 0x07U);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * I2C speed
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_i2c_hs(const uint8_t hs)
{
    uint8_t      reg;
    adxl_error_t err;

    err = spi_read(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_I2C_HS_COMM;
    }

    if (hs != 0U)
    {
        reg = (uint8_t)(reg | ADXL_I2C_HS_BIT);
    }
    else
    {
        reg = (uint8_t)(reg & (uint8_t)(~ADXL_I2C_HS_BIT));
    }

    err = spi_write(ADXL_REG_RANGE, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_I2C_HS_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_i2c_hs(uint8_t *hs)
{
    uint8_t reg;

    if (hs == NULL)
    {
        return ADXL_ERR_GET_I2C_HS_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_RANGE, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_I2C_HS_COMM;
        }
    }

    *hs = (uint8_t)((reg & ADXL_I2C_HS_BIT) != 0U ? 1U : 0U);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Offset trim
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_offset(const int16_t x, const int16_t y, const int16_t z)
{
    uint8_t      buf[2];
    adxl_error_t err;

    buf[0] = (uint8_t)((uint16_t)x >> 8U);
    buf[1] = (uint8_t)((uint16_t)x & 0xFFU);
    err    = spi_write(ADXL_REG_OFFSET_X_H, buf, 2U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_OFFSET_COMM;
    }

    buf[0] = (uint8_t)((uint16_t)y >> 8U);
    buf[1] = (uint8_t)((uint16_t)y & 0xFFU);
    err    = spi_write(ADXL_REG_OFFSET_Y_H, buf, 2U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_OFFSET_COMM;
    }

    buf[0] = (uint8_t)((uint16_t)z >> 8U);
    buf[1] = (uint8_t)((uint16_t)z & 0xFFU);
    err    = spi_write(ADXL_REG_OFFSET_Z_H, buf, 2U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_OFFSET_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_offset(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if ((x == NULL) || (y == NULL) || (z == NULL))
    {
        return ADXL_ERR_GET_OFFSET_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_OFFSET_X_H, buf, 6U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_OFFSET_COMM;
        }
    }

    *x = (int16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
    *y = (int16_t)(((uint16_t)buf[2] << 8U) | (uint16_t)buf[3]);
    *z = (int16_t)(((uint16_t)buf[4] << 8U) | (uint16_t)buf[5]);

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Activity detection
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_activity(const uint8_t axes_en, const uint16_t thresh,
                                const uint8_t count)
{
    const uint8_t en = (uint8_t)(axes_en & 0x07U);
    uint8_t       buf[2];
    adxl_error_t  err;

    if ((axes_en & (uint8_t)(~(ADXL_ACT_EN_X | ADXL_ACT_EN_Y | ADXL_ACT_EN_Z)))
        != 0U)
    {
        return ADXL_ERR_SET_ACT_AXES;
    }

    err = spi_write(ADXL_REG_ACT_EN, &en, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_ACT_COMM;
    }

    buf[0] = (uint8_t)(thresh >> 8U);
    buf[1] = (uint8_t)(thresh & 0xFFU);
    err    = spi_write(ADXL_REG_ACT_THRESH_H, buf, 2U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_ACT_COMM;
    }

    err = spi_write(ADXL_REG_ACT_COUNT, &count, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_ACT_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_activity_count(uint8_t *count)
{
    if (count == NULL)
    {
        return ADXL_ERR_GET_ACT_CNT_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_ACT_COUNT, count, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_ACT_CNT_COMM;
        }
    }

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Power control
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_set_power_ctl(const uint8_t drdy_off, const uint8_t temp_off)
{
    uint8_t      reg;
    adxl_error_t err;

    err = spi_read(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_PWR_CTL_COMM;
    }

    if (drdy_off != 0U)
    {
        reg = (uint8_t)(reg | ADXL_PWR_DRDY_OFF_BIT);
    }
    else
    {
        reg = (uint8_t)(reg & (uint8_t)(~ADXL_PWR_DRDY_OFF_BIT));
    }

    if (temp_off != 0U)
    {
        reg = (uint8_t)(reg | ADXL_PWR_TEMP_OFF_BIT);
    }
    else
    {
        reg = (uint8_t)(reg & (uint8_t)(~ADXL_PWR_TEMP_OFF_BIT));
    }

    err = spi_write(ADXL_REG_POWER_CTL, &reg, 1U);
    if (err != ADXL_OK)
    {
        return ADXL_ERR_SET_PWR_CTL_COMM;
    }

    return ADXL_OK;
}

adxl_error_t adxl_get_power_ctl(uint8_t *ctl)
{
    if (ctl == NULL)
    {
        return ADXL_ERR_GET_PWR_CTL_NULL;
    }

    {
        const adxl_error_t err = spi_read(ADXL_REG_POWER_CTL, ctl, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_GET_PWR_CTL_COMM;
        }
    }

    return ADXL_OK;
}

/* -------------------------------------------------------------------------
 * Self-test
 * ------------------------------------------------------------------------- */

adxl_error_t adxl_self_test(const uint8_t st1, const uint8_t st2)
{
    uint8_t reg = 0U;

    if (st1 != 0U)
    {
        reg = (uint8_t)(reg | ADXL_ST1_BIT);
    }

    if (st2 != 0U)
    {
        reg = (uint8_t)(reg | ADXL_ST2_BIT);
    }

    {
        const adxl_error_t err = spi_write(ADXL_REG_SELF_TEST, &reg, 1U);
        if (err != ADXL_OK)
        {
            return ADXL_ERR_SELF_TEST_COMM;
        }
    }

    return ADXL_OK;
}
