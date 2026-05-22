#ifndef ADXL355_H
#define ADXL355_H

#include <stdint.h>
#include <stddef.h>

/* -------------------------------------------------------------------------
 * Register addresses
 * ------------------------------------------------------------------------- */
#define ADXL_REG_DEVID_AD      (0x00U)
#define ADXL_REG_DEVID_MST     (0x01U)
#define ADXL_REG_PARTID        (0x02U)
#define ADXL_REG_REVID         (0x03U)
#define ADXL_REG_STATUS        (0x04U)
#define ADXL_REG_FIFO_ENTRIES  (0x05U)
#define ADXL_REG_TEMP2         (0x06U)
#define ADXL_REG_TEMP1         (0x07U)
#define ADXL_REG_XDATA3        (0x08U)
#define ADXL_REG_XDATA2        (0x09U)
#define ADXL_REG_XDATA1        (0x0AU)
#define ADXL_REG_YDATA3        (0x0BU)
#define ADXL_REG_YDATA2        (0x0CU)
#define ADXL_REG_YDATA1        (0x0DU)
#define ADXL_REG_ZDATA3        (0x0EU)
#define ADXL_REG_ZDATA2        (0x0FU)
#define ADXL_REG_ZDATA1        (0x10U)
#define ADXL_REG_FIFO_DATA     (0x11U)
#define ADXL_REG_OFFSET_X_H    (0x1EU)
#define ADXL_REG_OFFSET_X_L    (0x1FU)
#define ADXL_REG_OFFSET_Y_H    (0x20U)
#define ADXL_REG_OFFSET_Y_L    (0x21U)
#define ADXL_REG_OFFSET_Z_H    (0x22U)
#define ADXL_REG_OFFSET_Z_L    (0x23U)
#define ADXL_REG_ACT_EN        (0x24U)
#define ADXL_REG_ACT_THRESH_H  (0x25U)
#define ADXL_REG_ACT_THRESH_L  (0x26U)
#define ADXL_REG_ACT_COUNT     (0x27U)
#define ADXL_REG_FILTER        (0x28U)
#define ADXL_REG_FIFO_SAMPLES  (0x29U)
#define ADXL_REG_INT_MAP       (0x2AU)
#define ADXL_REG_SYNC          (0x2BU)
#define ADXL_REG_RANGE         (0x2CU)
#define ADXL_REG_POWER_CTL    (0x2DU)
#define ADXL_REG_SELF_TEST     (0x2EU)
#define ADXL_REG_RESET         (0x2FU)

/* -------------------------------------------------------------------------
 * Fixed device identity values
 * ------------------------------------------------------------------------- */
#define ADXL_DEVID_AD_VALUE    (0xADU)
#define ADXL_DEVID_MST_VALUE   (0x1DU)
#define ADXL_PARTID_VALUE      (0xEDU)
#define ADXL_RESET_CODE        (0x52U)

/* -------------------------------------------------------------------------
 * STATUS register bits (0x04)
 * ------------------------------------------------------------------------- */
#define ADXL_STATUS_DATA_RDY   (0x01U)
#define ADXL_STATUS_FIFO_FULL  (0x02U)
#define ADXL_STATUS_FIFO_OVR   (0x04U)
#define ADXL_STATUS_ACTIVITY   (0x08U)
#define ADXL_STATUS_NVM_BUSY   (0x10U)

/* -------------------------------------------------------------------------
 * ACT_EN register bits (0x24)
 * ------------------------------------------------------------------------- */
#define ADXL_ACT_EN_X          (0x01U)
#define ADXL_ACT_EN_Y          (0x02U)
#define ADXL_ACT_EN_Z          (0x04U)

/* -------------------------------------------------------------------------
 * INT_MAP register bits (0x2A)
 * ------------------------------------------------------------------------- */
#define ADXL_INT_RDY_EN1       (0x01U)
#define ADXL_INT_FULL_EN1      (0x02U)
#define ADXL_INT_OVR_EN1       (0x04U)
#define ADXL_INT_ACT_EN1       (0x08U)
#define ADXL_INT_RDY_EN2       (0x10U)
#define ADXL_INT_FULL_EN2      (0x20U)
#define ADXL_INT_OVR_EN2       (0x40U)
#define ADXL_INT_ACT_EN2       (0x80U)

/* -------------------------------------------------------------------------
 * SYNC register bits (0x2B)
 * ------------------------------------------------------------------------- */
#define ADXL_SYNC_EXT_SYNC_MSK (0x03U)
#define ADXL_SYNC_EXT_CLK_BIT  (0x04U)

/* -------------------------------------------------------------------------
 * RANGE register bits (0x2C)
 * ------------------------------------------------------------------------- */
#define ADXL_RANGE_MASK        (0x03U)
#define ADXL_INT_POL_BIT       (0x40U)
#define ADXL_I2C_HS_BIT        (0x80U)

/* -------------------------------------------------------------------------
 * POWER_CTL register bits (0x2D)
 * ------------------------------------------------------------------------- */
#define ADXL_PWR_STANDBY_BIT   (0x01U)
#define ADXL_PWR_TEMP_OFF_BIT  (0x02U)
#define ADXL_PWR_DRDY_OFF_BIT  (0x04U)

/* -------------------------------------------------------------------------
 * SELF_TEST register bits (0x2E)
 * ------------------------------------------------------------------------- */
#define ADXL_ST1_BIT           (0x01U)
#define ADXL_ST2_BIT           (0x02U)

/* -------------------------------------------------------------------------
 * FILTER register bit masks (0x28)
 * ------------------------------------------------------------------------- */
#define ADXL_FILTER_ODR_MASK   (0x0FU)
#define ADXL_FILTER_HPF_MASK   (0x70U)
#define ADXL_FILTER_HPF_SHIFT  (4U)

/* -------------------------------------------------------------------------
 * FIFO limits
 * ------------------------------------------------------------------------- */
#define ADXL_FIFO_MAX_SAMPLES  (96U)
#define ADXL_FIFO_BYTES_PER    (3U)

/* -------------------------------------------------------------------------
 * Measurement range selection
 * ------------------------------------------------------------------------- */
typedef enum
{
    ADXL_RANGE_2G  = 0x01U,
    ADXL_RANGE_4G  = 0x02U,
    ADXL_RANGE_8G  = 0x03U
} adxl_range_t;

/* -------------------------------------------------------------------------
 * Acceleration scale factors (g per LSB of the 20-bit output)
 * ------------------------------------------------------------------------- */
#define ADXL_SCALE_2G (3.9e-6)
#define ADXL_SCALE_4G (7.8e-6)
#define ADXL_SCALE_8G (15.6e-6)

/**
 * @brief Resolve the g/LSB scale factor for a given range at compile time.
 *
 * Expands to a double constant. When @p range is a literal enum value the
 * entire expression is folded by the compiler. Defaults to ADXL_SCALE_2G for
 * any unrecognised range value.
 *
 * Example: double g = raw * ADXL_G_SCALE_FACTOR(ADXL_RANGE_4G); // 7.8e-6
 */
#define ADXL_G_SCALE_FACTOR(range) \
    (((range) == ADXL_RANGE_4G) ? ADXL_SCALE_4G : \
     (((range) == ADXL_RANGE_8G) ? ADXL_SCALE_8G : ADXL_SCALE_2G))

/* -------------------------------------------------------------------------
 * ODR and low-pass filter corner (FILTER register bits [3:0])
 * ------------------------------------------------------------------------- */
typedef enum
{
    ADXL_ODR_4000HZ  = 0x00U,
    ADXL_ODR_2000HZ  = 0x01U,
    ADXL_ODR_1000HZ  = 0x02U,
    ADXL_ODR_500HZ   = 0x03U,
    ADXL_ODR_250HZ   = 0x04U,
    ADXL_ODR_125HZ   = 0x05U,
    ADXL_ODR_62HZ    = 0x06U,
    ADXL_ODR_31HZ    = 0x07U,
    ADXL_ODR_15HZ    = 0x08U,
    ADXL_ODR_7HZ     = 0x09U,
    ADXL_ODR_3HZ     = 0x0AU
} adxl_odr_lpf_t;

/* -------------------------------------------------------------------------
 * High-pass filter corner (FILTER register bits [6:4])
 * ------------------------------------------------------------------------- */
typedef enum
{
    ADXL_HPF_DISABLE = 0x00U,
    ADXL_HPF_247E4   = 0x01U,
    ADXL_HPF_62E4    = 0x02U,
    ADXL_HPF_15E4    = 0x03U,
    ADXL_HPF_3862E6  = 0x04U,
    ADXL_HPF_954E6   = 0x05U,
    ADXL_HPF_238E6   = 0x06U
} adxl_hpf_corner_t;

/* -------------------------------------------------------------------------
 * Raw 3-axis accelerometer sample (20-bit signed, stored in int32_t)
 * ------------------------------------------------------------------------- */
typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} adxl_accel_raw_t;

/* -------------------------------------------------------------------------
 * Error codes — each function has its own unique codes.
 * ADXL_OK (0) is the only shared success value.
 * ------------------------------------------------------------------------- */
typedef enum
{
    ADXL_OK                        = 0,

    /* adxl_spi_transfer (interface, user-implemented) */
    ADXL_ERR_SPI_XFER_FAIL         = 1,

    /* adxl_reg_read */
    ADXL_ERR_REG_READ_NULL         = 2,
    ADXL_ERR_REG_READ_COMM         = 3,

    /* adxl_reg_write */
    ADXL_ERR_REG_WRITE_COMM        = 4,

    /* adxl_reg_read_burst */
    ADXL_ERR_BURST_READ_NULL       = 5,
    ADXL_ERR_BURST_READ_LEN        = 6,
    ADXL_ERR_BURST_READ_COMM       = 7,

    /* adxl_init */
    ADXL_ERR_INIT_COMM             = 8,
    ADXL_ERR_INIT_DEVID_AD         = 9,
    ADXL_ERR_INIT_DEVID_MST        = 10,
    ADXL_ERR_INIT_PARTID           = 11,
    ADXL_ERR_INIT_RANGE            = 12,
    ADXL_ERR_INIT_FILTER           = 13,
    ADXL_ERR_INIT_STANDBY          = 14,

    /* adxl_standby */
    ADXL_ERR_STANDBY_COMM          = 15,

    /* adxl_measure */
    ADXL_ERR_MEASURE_COMM          = 16,

    /* adxl_reset */
    ADXL_ERR_RESET_COMM            = 17,

    /* adxl_data_ready */
    ADXL_ERR_DATA_RDY_NULL         = 18,
    ADXL_ERR_DATA_RDY_COMM         = 19,

    /* adxl_get_status */
    ADXL_ERR_GET_STATUS_NULL       = 20,
    ADXL_ERR_GET_STATUS_COMM       = 21,

    /* adxl_get_device_id */
    ADXL_ERR_GET_DEVID_NULL        = 22,
    ADXL_ERR_GET_DEVID_COMM        = 23,

    /* adxl_get_accel_raw */
    ADXL_ERR_GET_ACCEL_NULL        = 24,
    ADXL_ERR_GET_ACCEL_COMM        = 25,

    /* adxl_get_temp_raw */
    ADXL_ERR_GET_TEMP_NULL         = 26,
    ADXL_ERR_GET_TEMP_COMM         = 27,

    /* adxl_get_fifo_entries */
    ADXL_ERR_FIFO_ENTRIES_NULL     = 28,
    ADXL_ERR_FIFO_ENTRIES_COMM     = 29,

    /* adxl_fifo_read */
    ADXL_ERR_FIFO_READ_NULL        = 30,
    ADXL_ERR_FIFO_READ_COUNT       = 31,
    ADXL_ERR_FIFO_READ_COMM        = 32,

    /* adxl_set_range */
    ADXL_ERR_SET_RANGE_INVALID     = 33,
    ADXL_ERR_SET_RANGE_COMM        = 34,

    /* adxl_get_range */
    ADXL_ERR_GET_RANGE_NULL        = 35,
    ADXL_ERR_GET_RANGE_COMM        = 36,

    /* adxl_set_filter */
    ADXL_ERR_SET_FILTER_HPF        = 37,
    ADXL_ERR_SET_FILTER_ODR        = 38,
    ADXL_ERR_SET_FILTER_COMM       = 39,

    /* adxl_get_filter */
    ADXL_ERR_GET_FILTER_NULL       = 40,
    ADXL_ERR_GET_FILTER_COMM       = 41,

    /* adxl_set_fifo_samples */
    ADXL_ERR_SET_FIFO_SMPL_RANGE   = 42,
    ADXL_ERR_SET_FIFO_SMPL_COMM    = 43,

    /* adxl_get_fifo_samples */
    ADXL_ERR_GET_FIFO_SMPL_NULL    = 44,
    ADXL_ERR_GET_FIFO_SMPL_COMM    = 45,

    /* adxl_set_int_map */
    ADXL_ERR_SET_INT_MAP_COMM      = 46,

    /* adxl_get_int_map */
    ADXL_ERR_GET_INT_MAP_NULL      = 47,
    ADXL_ERR_GET_INT_MAP_COMM      = 48,

    /* adxl_set_sync */
    ADXL_ERR_SET_SYNC_COMM         = 49,

    /* adxl_get_sync */
    ADXL_ERR_GET_SYNC_NULL         = 50,
    ADXL_ERR_GET_SYNC_COMM         = 51,

    /* adxl_set_int_pol */
    ADXL_ERR_SET_INT_POL_COMM      = 52,

    /* adxl_get_int_pol */
    ADXL_ERR_GET_INT_POL_NULL      = 53,
    ADXL_ERR_GET_INT_POL_COMM      = 54,

    /* adxl_set_i2c_hs */
    ADXL_ERR_SET_I2C_HS_COMM       = 55,

    /* adxl_get_i2c_hs */
    ADXL_ERR_GET_I2C_HS_NULL       = 56,
    ADXL_ERR_GET_I2C_HS_COMM       = 57,

    /* adxl_set_offset */
    ADXL_ERR_SET_OFFSET_COMM       = 58,

    /* adxl_get_offset */
    ADXL_ERR_GET_OFFSET_NULL       = 59,
    ADXL_ERR_GET_OFFSET_COMM       = 60,

    /* adxl_set_activity */
    ADXL_ERR_SET_ACT_AXES          = 61,
    ADXL_ERR_SET_ACT_COMM          = 62,

    /* adxl_get_activity_count */
    ADXL_ERR_GET_ACT_CNT_NULL      = 63,
    ADXL_ERR_GET_ACT_CNT_COMM      = 64,

    /* adxl_set_power_ctl */
    ADXL_ERR_SET_PWR_CTL_COMM      = 65,

    /* adxl_get_power_ctl */
    ADXL_ERR_GET_PWR_CTL_NULL      = 66,
    ADXL_ERR_GET_PWR_CTL_COMM      = 67,

    /* adxl_self_test */
    ADXL_ERR_SELF_TEST_COMM        = 68

} adxl_error_t;

/* -------------------------------------------------------------------------
 * Hardware interface — declared here, MUST be implemented in adxl_inf.c
 * ------------------------------------------------------------------------- */

/**
 * @brief Perform a full-duplex SPI exchange of @p len bytes.
 *
 * Assert CS, simultaneously clock @p len bytes out of @p tx and into @p rx,
 * then deassert CS. Both buffers must be at least @p len bytes long.
 *
 * @param[in]  tx  Bytes to transmit.
 * @param[out] rx  Buffer for received bytes (same length as @p tx).
 * @param[in]  len Number of bytes to exchange.
 * @return ADXL_OK on success, ADXL_ERR_SPI_XFER_FAIL on any bus error.
 */
adxl_error_t adxl_spi_transfer(const uint8_t *tx, uint8_t *rx, const uint8_t len);

/**
 * @brief Block for at least @p ms milliseconds.
 *
 * @param[in] ms Minimum delay in milliseconds.
 */
void adxl_delay_ms(const uint32_t ms);

/* -------------------------------------------------------------------------
 * Low-level register access
 * ------------------------------------------------------------------------- */

/**
 * @brief Read a single register byte.
 *
 * @param[in]  addr Register address (use ADXL_REG_* macros).
 * @param[out] val  Received byte.
 * @return ADXL_OK, ADXL_ERR_REG_READ_NULL, or ADXL_ERR_REG_READ_COMM.
 */
adxl_error_t adxl_reg_read(const uint8_t addr, uint8_t *val);

/**
 * @brief Write a single register byte.
 *
 * @param[in] addr Register address (use ADXL_REG_* macros).
 * @param[in] val  Byte to write.
 * @return ADXL_OK or ADXL_ERR_REG_WRITE_COMM.
 */
adxl_error_t adxl_reg_write(const uint8_t addr, const uint8_t val);

/**
 * @brief Burst-read @p len consecutive registers starting at @p addr.
 *
 * The device auto-increments the address for each byte. Maximum @p len is
 * limited by the internal SPI buffer; values > 15 return
 * ADXL_ERR_BURST_READ_LEN.
 *
 * @param[in]  addr Starting register address.
 * @param[out] data Output buffer of at least @p len bytes.
 * @param[in]  len  Number of registers to read (1–15).
 * @return ADXL_OK, ADXL_ERR_BURST_READ_NULL, ADXL_ERR_BURST_READ_LEN,
 *         or ADXL_ERR_BURST_READ_COMM.
 */
adxl_error_t adxl_reg_read_burst(const uint8_t addr, uint8_t *data,
                                  const uint8_t len);

/* -------------------------------------------------------------------------
 * Device control
 * ------------------------------------------------------------------------- */

/**
 * @brief Verify device identity and apply initial range and ODR/LPF settings.
 *
 * Reads DEVID_AD, DEVID_MST, and PARTID and returns an error if any value
 * does not match the expected constant. Then places the device in standby,
 * writes the range to the RANGE register (preserving the I2C_HS default),
 * and writes the ODR/LPF to the FILTER register with HPF disabled. The device
 * remains in standby after this call; invoke adxl_measure() to begin sampling.
 *
 * @param[in] range Measurement range (ADXL_RANGE_2G / 4G / 8G).
 * @param[in] odr   Output data rate and low-pass filter corner.
 * @return ADXL_OK, ADXL_ERR_INIT_COMM, ADXL_ERR_INIT_DEVID_AD,
 *         ADXL_ERR_INIT_DEVID_MST, ADXL_ERR_INIT_PARTID,
 *         ADXL_ERR_INIT_STANDBY, ADXL_ERR_INIT_RANGE, or
 *         ADXL_ERR_INIT_FILTER.
 */
adxl_error_t adxl_init(const adxl_range_t range, const adxl_odr_lpf_t odr);

/**
 * @brief Enter standby mode; data conversion halts.
 *
 * Sets the STANDBY bit in POWER_CTL via read-modify-write, preserving all
 * other bits. Must be in standby to safely modify configuration registers.
 *
 * @return ADXL_OK or ADXL_ERR_STANDBY_COMM.
 */
adxl_error_t adxl_standby(void);

/**
 * @brief Enter measurement mode; data conversion begins.
 *
 * Clears the STANDBY bit in POWER_CTL via read-modify-write, preserving all
 * other bits.
 *
 * @return ADXL_OK or ADXL_ERR_MEASURE_COMM.
 */
adxl_error_t adxl_measure(void);

/**
 * @brief Issue a soft reset and wait for NVM reload.
 *
 * Writes the reset code (0x52) to the RESET register, then blocks for 10 ms
 * to allow NVM load to complete. All registers return to power-on defaults;
 * the device enters standby.
 *
 * @return ADXL_OK or ADXL_ERR_RESET_COMM.
 */
adxl_error_t adxl_reset(void);

/* -------------------------------------------------------------------------
 * Status and identity
 * ------------------------------------------------------------------------- */

/**
 * @brief Read the STATUS register.
 *
 * Test the result against ADXL_STATUS_DATA_RDY, ADXL_STATUS_FIFO_FULL,
 * ADXL_STATUS_FIFO_OVR, ADXL_STATUS_ACTIVITY, and ADXL_STATUS_NVM_BUSY.
 *
 * @param[out] status Raw STATUS register byte.
 * @return ADXL_OK, ADXL_ERR_GET_STATUS_NULL, or ADXL_ERR_GET_STATUS_COMM.
 */
adxl_error_t adxl_get_status(uint8_t *status);

/**
 * @brief Check whether a new acceleration sample is ready.
 *
 * Convenience wrapper around adxl_get_status(). Sets *ready to 1 if the
 * DATA_RDY bit is set, 0 otherwise.
 *
 * @param[out] ready 1 if data ready, 0 if not.
 * @return ADXL_OK, ADXL_ERR_DATA_RDY_NULL, or ADXL_ERR_DATA_RDY_COMM.
 */
adxl_error_t adxl_data_ready(uint8_t *ready);

/**
 * @brief Read all four device identity registers in a single burst.
 *
 * Expected values: devid_ad = 0xAD, devid_mst = 0x1D, partid = 0xED.
 * revid identifies the silicon revision.
 *
 * @param[out] devid_ad  Analog Devices device ID (expect 0xAD).
 * @param[out] devid_mst MEMS device ID (expect 0x1D).
 * @param[out] partid    Part ID (expect 0xED).
 * @param[out] revid     Silicon revision ID.
 * @return ADXL_OK, ADXL_ERR_GET_DEVID_NULL, or ADXL_ERR_GET_DEVID_COMM.
 */
adxl_error_t adxl_get_device_id(uint8_t *devid_ad, uint8_t *devid_mst,
                                 uint8_t *partid,   uint8_t *revid);

/* -------------------------------------------------------------------------
 * Data readout
 * ------------------------------------------------------------------------- */

/**
 * @brief Read one acceleration sample from the output registers.
 *
 * Burst-reads XDATA3 through ZDATA1 (9 bytes) and reconstructs each axis as
 * a 20-bit twos-complement value sign-extended into int32_t. The LSB
 * represents the scale factor for the configured range (3.9 µg/LSB at ±2 g).
 *
 * @param[out] x X-axis raw acceleration (20-bit signed).
 * @param[out] y Y-axis raw acceleration (20-bit signed).
 * @param[out] z Z-axis raw acceleration (20-bit signed).
 * @return ADXL_OK, ADXL_ERR_GET_ACCEL_NULL, or ADXL_ERR_GET_ACCEL_COMM.
 */
adxl_error_t adxl_get_accel_raw(int32_t *x, int32_t *y, int32_t *z);

/**
 * @brief Read the raw temperature ADC code.
 *
 * Returns the 12-bit unsigned value from TEMP2:TEMP1. To convert to °C apply
 * the datasheet formula: T(°C) = ((*temp - 1852) / -9.05) + 25.
 *
 * @param[out] temp 12-bit raw temperature ADC code.
 * @return ADXL_OK, ADXL_ERR_GET_TEMP_NULL, or ADXL_ERR_GET_TEMP_COMM.
 */
adxl_error_t adxl_get_temp_raw(uint16_t *temp);

/* -------------------------------------------------------------------------
 * FIFO
 * ------------------------------------------------------------------------- */

/**
 * @brief Read the number of complete samples currently stored in the FIFO.
 *
 * Returns the value of FIFO_ENTRIES[6:0]. A value of 96 indicates the FIFO
 * is full. Each entry is one axis measurement; complete XYZ triplets arrive
 * in groups of three.
 *
 * @param[out] count Number of FIFO entries (0–96).
 * @return ADXL_OK, ADXL_ERR_FIFO_ENTRIES_NULL, or ADXL_ERR_FIFO_ENTRIES_COMM.
 */
adxl_error_t adxl_get_fifo_entries(uint8_t *count);

/**
 * @brief Set the FIFO watermark level.
 *
 * The FIFO_FULL interrupt fires when the entry count reaches this value.
 * Valid range is 1–96.
 *
 * @param[in] samples Watermark threshold in samples (1–96).
 * @return ADXL_OK, ADXL_ERR_SET_FIFO_SMPL_RANGE, or ADXL_ERR_SET_FIFO_SMPL_COMM.
 */
adxl_error_t adxl_set_fifo_samples(const uint8_t samples);

/**
 * @brief Read the configured FIFO watermark level.
 *
 * @param[out] samples Current watermark threshold (1–96).
 * @return ADXL_OK, ADXL_ERR_GET_FIFO_SMPL_NULL, or ADXL_ERR_GET_FIFO_SMPL_COMM.
 */
adxl_error_t adxl_get_fifo_samples(uint8_t *samples);

/**
 * @brief Pop @p count complete XYZ triplets from the FIFO.
 *
 * Each SPI transaction reads one 3-byte FIFO entry. The x-axis marker
 * (bit 0 of the third byte) is used to synchronise triplet boundaries, so
 * reads stay aligned even if the FIFO is entered mid-triplet. The actual
 * number of complete triplets written to @p samples is reported in
 * @p read_count.
 *
 * @param[out]    samples    Output array of at least @p count entries.
 * @param[in]     count      Number of complete XYZ triplets to read (1–96).
 * @param[out]    read_count Number of triplets actually written to @p samples.
 * @return ADXL_OK, ADXL_ERR_FIFO_READ_NULL, ADXL_ERR_FIFO_READ_COUNT,
 *         or ADXL_ERR_FIFO_READ_COMM.
 */
adxl_error_t adxl_fifo_read(adxl_accel_raw_t *samples, const uint8_t count,
                             uint8_t *read_count);

/* -------------------------------------------------------------------------
 * Filter configuration
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the high-pass and low-pass filter corners atomically.
 *
 * Builds and writes the FILTER register in a single transaction. Both values
 * must be valid enum members; passing ADXL_HPF_DISABLE removes the HPF.
 *
 * @param[in] hpf     High-pass filter corner frequency.
 * @param[in] odr_lpf Output data rate and low-pass filter corner.
 * @return ADXL_OK, ADXL_ERR_SET_FILTER_HPF, ADXL_ERR_SET_FILTER_ODR,
 *         or ADXL_ERR_SET_FILTER_COMM.
 */
adxl_error_t adxl_set_filter(const adxl_hpf_corner_t hpf,
                              const adxl_odr_lpf_t odr_lpf);

/**
 * @brief Read and unpack the current filter settings from the FILTER register.
 *
 * @param[out] hpf     Current high-pass filter corner.
 * @param[out] odr_lpf Current ODR and low-pass filter corner.
 * @return ADXL_OK, ADXL_ERR_GET_FILTER_NULL, or ADXL_ERR_GET_FILTER_COMM.
 */
adxl_error_t adxl_get_filter(adxl_hpf_corner_t *hpf, adxl_odr_lpf_t *odr_lpf);

/* -------------------------------------------------------------------------
 * Range configuration
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the measurement range.
 *
 * Performs a read-modify-write on the RANGE register, preserving the INT_POL
 * and I2C_HS bits. The device should be in standby before changing range.
 *
 * @param[in] range Desired measurement range.
 * @return ADXL_OK, ADXL_ERR_SET_RANGE_INVALID, or ADXL_ERR_SET_RANGE_COMM.
 */
adxl_error_t adxl_set_range(const adxl_range_t range);

/**
 * @brief Read the currently configured measurement range.
 *
 * @param[out] range Current measurement range.
 * @return ADXL_OK, ADXL_ERR_GET_RANGE_NULL, or ADXL_ERR_GET_RANGE_COMM.
 */
adxl_error_t adxl_get_range(adxl_range_t *range);

/* -------------------------------------------------------------------------
 * Interrupt configuration
 * ------------------------------------------------------------------------- */

/**
 * @brief Write the INT_MAP register to route events to INT1 or INT2 pins.
 *
 * Build @p map from ADXL_INT_RDY_EN1/2, ADXL_INT_FULL_EN1/2,
 * ADXL_INT_OVR_EN1/2, and ADXL_INT_ACT_EN1/2. The full byte is overwritten.
 *
 * @param[in] map Interrupt mapping bitmask.
 * @return ADXL_OK or ADXL_ERR_SET_INT_MAP_COMM.
 */
adxl_error_t adxl_set_int_map(const uint8_t map);

/**
 * @brief Read the INT_MAP register.
 *
 * @param[out] map Current interrupt mapping bitmask.
 * @return ADXL_OK, ADXL_ERR_GET_INT_MAP_NULL, or ADXL_ERR_GET_INT_MAP_COMM.
 */
adxl_error_t adxl_get_int_map(uint8_t *map);

/**
 * @brief Set the interrupt output polarity.
 *
 * Performs a read-modify-write on bit 6 of the RANGE register. When
 * @p active_high is non-zero the interrupt pins assert high; otherwise they
 * assert low (default).
 *
 * @param[in] active_high Non-zero for active-high, 0 for active-low.
 * @return ADXL_OK or ADXL_ERR_SET_INT_POL_COMM.
 */
adxl_error_t adxl_set_int_pol(const uint8_t active_high);

/**
 * @brief Read the interrupt output polarity setting.
 *
 * @param[out] active_high 1 if active-high, 0 if active-low.
 * @return ADXL_OK, ADXL_ERR_GET_INT_POL_NULL, or ADXL_ERR_GET_INT_POL_COMM.
 */
adxl_error_t adxl_get_int_pol(uint8_t *active_high);

/* -------------------------------------------------------------------------
 * Synchronisation
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure external sync and clock options.
 *
 * Writes bits [2:0] of the SYNC register via read-modify-write, preserving
 * reserved bits. Bit 2 selects the external clock; bits [1:0] select the
 * sync mode (see ADXL_SYNC_EXT_SYNC_MSK and ADXL_SYNC_EXT_CLK_BIT).
 *
 * @param[in] sync_cfg Bits [2:0] of the desired SYNC register value.
 * @return ADXL_OK or ADXL_ERR_SET_SYNC_COMM.
 */
adxl_error_t adxl_set_sync(const uint8_t sync_cfg);

/**
 * @brief Read the sync configuration (bits [2:0] of the SYNC register).
 *
 * @param[out] sync_cfg Current SYNC bits [2:0].
 * @return ADXL_OK, ADXL_ERR_GET_SYNC_NULL, or ADXL_ERR_GET_SYNC_COMM.
 */
adxl_error_t adxl_get_sync(uint8_t *sync_cfg);

/* -------------------------------------------------------------------------
 * I2C speed
 * ------------------------------------------------------------------------- */

/**
 * @brief Enable or disable I2C high-speed mode.
 *
 * Performs a read-modify-write on bit 7 of the RANGE register. High-speed
 * mode is enabled by default after reset; disable only if operating at
 * standard or fast I2C speeds.
 *
 * @param[in] hs Non-zero to enable I2C HS mode, 0 to disable.
 * @return ADXL_OK or ADXL_ERR_SET_I2C_HS_COMM.
 */
adxl_error_t adxl_set_i2c_hs(const uint8_t hs);

/**
 * @brief Read the I2C high-speed mode setting.
 *
 * @param[out] hs 1 if I2C HS enabled, 0 if disabled.
 * @return ADXL_OK, ADXL_ERR_GET_I2C_HS_NULL, or ADXL_ERR_GET_I2C_HS_COMM.
 */
adxl_error_t adxl_get_i2c_hs(uint8_t *hs);

/* -------------------------------------------------------------------------
 * Offset trim (16-bit signed, significance matches XDATA bits [19:4])
 * ------------------------------------------------------------------------- */

/**
 * @brief Write per-axis offset trim registers.
 *
 * Each 16-bit signed value is written to a pair of H/L registers. The
 * significance of bit 0 corresponds to XDATA bit 4 (i.e. one step is 16 LSB
 * of the 20-bit output). The device should be in standby before calling this.
 *
 * @param[in] x X-axis offset trim (16-bit signed).
 * @param[in] y Y-axis offset trim (16-bit signed).
 * @param[in] z Z-axis offset trim (16-bit signed).
 * @return ADXL_OK or ADXL_ERR_SET_OFFSET_COMM.
 */
adxl_error_t adxl_set_offset(const int16_t x, const int16_t y, const int16_t z);

/**
 * @brief Read per-axis offset trim registers.
 *
 * @param[out] x X-axis offset trim (16-bit signed).
 * @param[out] y Y-axis offset trim (16-bit signed).
 * @param[out] z Z-axis offset trim (16-bit signed).
 * @return ADXL_OK, ADXL_ERR_GET_OFFSET_NULL, or ADXL_ERR_GET_OFFSET_COMM.
 */
adxl_error_t adxl_get_offset(int16_t *x, int16_t *y, int16_t *z);

/* -------------------------------------------------------------------------
 * Activity detection
 * axes_en: bitmask of ADXL_ACT_EN_X / _Y / _Z
 * thresh:  16-bit unsigned threshold (significance matches XDATA bits [18:3])
 * count:   number of consecutive events required
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure activity detection threshold and axes.
 *
 * Sets ACT_EN, ACT_THRESH_H/L, and ACT_COUNT registers. @p axes_en must be a
 * combination of ADXL_ACT_EN_X/Y/Z; any other bits return an error. @p thresh
 * has the same significance as XDATA bits [18:3]: bit 0 equals 32 LSB of the
 * 20-bit output. @p count sets the number of consecutive ODR periods the
 * threshold must be exceeded before the ACTIVITY flag is set.
 *
 * @param[in] axes_en Axis enable bitmask (ADXL_ACT_EN_X | _Y | _Z).
 * @param[in] thresh  Detection threshold (16-bit unsigned).
 * @param[in] count   Consecutive threshold crossings required (1–255).
 * @return ADXL_OK, ADXL_ERR_SET_ACT_AXES, or ADXL_ERR_SET_ACT_COMM.
 */
adxl_error_t adxl_set_activity(const uint8_t axes_en, const uint16_t thresh,
                                const uint8_t count);

/**
 * @brief Read the activity count register.
 *
 * Returns the number of consecutive ODR periods the activity threshold has
 * been exceeded since the last read.
 *
 * @param[out] count Current ACT_COUNT value.
 * @return ADXL_OK, ADXL_ERR_GET_ACT_CNT_NULL, or ADXL_ERR_GET_ACT_CNT_COMM.
 */
adxl_error_t adxl_get_activity_count(uint8_t *count);

/* -------------------------------------------------------------------------
 * Power control (granular bit access)
 * ------------------------------------------------------------------------- */

/**
 * @brief Control the DRDY output disable and temperature sensor power.
 *
 * Performs a read-modify-write on POWER_CTL, leaving the STANDBY bit and any
 * reserved bits unchanged.
 *
 * @param[in] drdy_off Non-zero to disable the DRDY output pin, 0 to enable.
 * @param[in] temp_off Non-zero to power down the temperature sensor, 0 to enable.
 * @return ADXL_OK or ADXL_ERR_SET_PWR_CTL_COMM.
 */
adxl_error_t adxl_set_power_ctl(const uint8_t drdy_off, const uint8_t temp_off);

/**
 * @brief Read the raw POWER_CTL register byte.
 *
 * Test the result against ADXL_PWR_STANDBY_BIT, ADXL_PWR_TEMP_OFF_BIT, and
 * ADXL_PWR_DRDY_OFF_BIT.
 *
 * @param[out] ctl Raw POWER_CTL register value.
 * @return ADXL_OK, ADXL_ERR_GET_PWR_CTL_NULL, or ADXL_ERR_GET_PWR_CTL_COMM.
 */
adxl_error_t adxl_get_power_ctl(uint8_t *ctl);

/* -------------------------------------------------------------------------
 * Self-test
 * st1: enable self-test mode (0 or 1)
 * st2: enable self-test force (0 or 1)
 * ------------------------------------------------------------------------- */

/**
 * @brief Write the SELF_TEST register to enable or disable self-test.
 *
 * ST1 (bit 0) enables self-test mode; ST2 (bit 1) applies an electrostatic
 * self-test force. To verify operation, set ST1=1, ST2=1, wait one ODR
 * period, read acceleration, then set ST1=0, ST2=0 and read again. The
 * difference should fall within the datasheet limits.
 *
 * @param[in] st1 Non-zero to enable self-test mode.
 * @param[in] st2 Non-zero to enable self-test force.
 * @return ADXL_OK or ADXL_ERR_SELF_TEST_COMM.
 */
adxl_error_t adxl_self_test(const uint8_t st1, const uint8_t st2);

#endif /* ADXL355_H */
