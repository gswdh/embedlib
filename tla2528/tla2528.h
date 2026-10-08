/**
 * @file tla2528.h
 * @brief TI TLA2528: 8-channel 12-bit SAR ADC with I2C and per-channel GPIO
 *
 * Datasheet SBAS961A. Each of the eight AINx/GPIOx pins is an analog input,
 * a digital input or a digital output (open-drain or push-pull). Conversions
 * are read in manual mode (one channel, host selected) or as an
 * auto-sequence over a channel mask; an averaging filter (OSR) returns 16-bit
 * results. The reference is AVDD, so codes are a fraction of the supply.
 *
 * The application implements the I2C and delay hooks at the top of the file.
 * Register traffic uses the device's opcodes (read 0x10, write 0x08, set bits
 * 0x18, clear bits 0x20, block read 0x30); a conversion is a plain 2- or
 * 3-byte read after the channel is selected, the device stretching SCL while
 * it converts.
 */

#ifndef TLA2528_H
#define TLA2528_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TLA_OK = 0,
    TLA_ERROR_I2C,           /* a hook failed */
    TLA_ERROR_NOT_FOUND,     /* no TLA2528 answered sanely at the address */
    TLA_ERROR_INVALID_PARAM, /* channel or mode out of range */
    TLA_ERROR_TIMEOUT,       /* calibration or reset did not complete */
    TLA_ERROR_NOT_ANALOG,    /* conversion asked of a pin configured as GPIO */
} tla_error_t;

/* I2C addresses by the ADDR pin resistors (datasheet Table 2): R1 to AVDD
 * 0 / 11k / 33k / 100k -> 0x17 / 0x16 / 0x15 / 0x14, none -> 0x10, R2 to GND
 * 11k / 33k / 100k -> 0x11 / 0x12 / 0x13. */
#define TLA_ADDR_R1_0R   (0x17U)
#define TLA_ADDR_R1_11K  (0x16U)
#define TLA_ADDR_R1_33K  (0x15U)
#define TLA_ADDR_R1_100K (0x14U)
#define TLA_ADDR_NONE    (0x10U)
#define TLA_ADDR_R2_11K  (0x11U)
#define TLA_ADDR_R2_33K  (0x12U)
#define TLA_ADDR_R2_100K (0x13U)

#define TLA_CHANNELS (8U)

/* Oversampling: samples averaged per result (OSR_CFG) */
typedef enum
{
    TLA_OSR_1 = 0, /* no averaging, 12-bit result */
    TLA_OSR_2,
    TLA_OSR_4,
    TLA_OSR_8,
    TLA_OSR_16,
    TLA_OSR_32,
    TLA_OSR_64,
    TLA_OSR_128,
} tla_osr_t;

/* What a channel pin does (PIN_CFG, GPIO_CFG, GPO_DRIVE_CFG) */
typedef enum
{
    TLA_PIN_ANALOG_IN = 0, /* default */
    TLA_PIN_DIGITAL_IN,
    TLA_PIN_OUTPUT_OPEN_DRAIN,
    TLA_PIN_OUTPUT_PUSH_PULL,
} tla_pin_mode_t;

/* Registers */
#define TLA_REG_SYSTEM_STATUS   (0x00U)
#define TLA_REG_GENERAL_CFG     (0x01U)
#define TLA_REG_DATA_CFG        (0x02U)
#define TLA_REG_OSR_CFG         (0x03U)
#define TLA_REG_OPMODE_CFG      (0x04U)
#define TLA_REG_PIN_CFG         (0x05U)
#define TLA_REG_GPIO_CFG        (0x07U)
#define TLA_REG_GPO_DRIVE_CFG   (0x09U)
#define TLA_REG_GPO_VALUE       (0x0BU)
#define TLA_REG_GPI_VALUE       (0x0DU)
#define TLA_REG_SEQUENCE_CFG    (0x10U)
#define TLA_REG_CHANNEL_SEL     (0x11U)
#define TLA_REG_AUTO_SEQ_CH_SEL (0x12U)

/* SYSTEM_STATUS */
#define TLA_STATUS_RSVD_ONE     (0x80U) /* always reads 1: the device is there */
#define TLA_STATUS_SEQ_STATUS   (0x40U)
#define TLA_STATUS_OSR_DONE     (0x08U)
#define TLA_STATUS_CRC_ERR_FUSE (0x04U)
#define TLA_STATUS_BOR          (0x01U)
/* GENERAL_CFG */
#define TLA_CFG_CNVST  (0x08U)
#define TLA_CFG_CH_RST (0x04U)
#define TLA_CFG_CAL    (0x02U)
#define TLA_CFG_RST    (0x01U)
/* DATA_CFG */
#define TLA_DATA_FIX_PAT       (0x80U)
#define TLA_DATA_APPEND_CHID   (0x10U)
#define TLA_DATA_FIXED_PATTERN (0xA5AU) /* with FIX_PAT, every result */
/* SEQUENCE_CFG */
#define TLA_SEQ_START     (0x10U)
#define TLA_SEQ_MODE_AUTO (0x01U)

/* ------------------------------------------------------------------------ */
/* Hardware interface: implemented by the application                        */
/* ------------------------------------------------------------------------ */

/** @brief Write `len` bytes to the 7-bit address (START, address+W, data, STOP) */
tla_error_t tla_i2c_write(const uint8_t addr7, const uint8_t *data, const uint32_t len);

/** @brief Read `len` bytes from the 7-bit address (START, address+R, data, STOP).
 *         The device may stretch SCL for the conversion time (up to ~1 ms at OSR 128). */
tla_error_t tla_i2c_read(const uint8_t addr7, uint8_t *data, const uint32_t len);

/** @brief Write `tx_len` bytes then, after a repeated START, read `rx_len` bytes */
tla_error_t tla_i2c_write_read(const uint8_t  addr7,
                               const uint8_t *tx,
                               const uint32_t tx_len,
                               uint8_t       *rx,
                               const uint32_t rx_len);

void tla_delay_ms(const uint32_t ms);

/* ------------------------------------------------------------------------ */
/* Driver                                                                    */
/* ------------------------------------------------------------------------ */

/**
 * @brief Reset the device, check it answers, clear the brown-out flag and run
 *        the offset calibration. Every pin is an analog input afterwards.
 * @param addr7 I2C address (TLA_ADDR_*)
 */
tla_error_t tla_init(const uint8_t addr7);

/** @brief The address tla_init() was given */
uint8_t tla_address(void);

/** @brief Software reset (all registers to defaults) */
tla_error_t tla_reset(void);

/** @brief Offset calibration against the current temperature and AVDD; ~1 ms */
tla_error_t tla_calibrate(void);

/** @brief Register access */
tla_error_t tla_read_register(const uint8_t reg, uint8_t *const value);
tla_error_t tla_write_register(const uint8_t reg, const uint8_t value);
tla_error_t tla_set_bits(const uint8_t reg, const uint8_t bits);
tla_error_t tla_clear_bits(const uint8_t reg, const uint8_t bits);

/** @brief Averaging for every conversion from now on */
tla_error_t tla_set_oversampling(const tla_osr_t osr);

/** @brief What a channel pin does; outputs start low */
tla_error_t tla_configure_pin(const uint8_t channel, const tla_pin_mode_t mode);

/** @brief Level on a digital output */
tla_error_t tla_gpo_write(const uint8_t channel, const bool high);

/** @brief Level on a digital input (or any pin: the input stage reads the pad) */
tla_error_t tla_gpi_read(const uint8_t channel, bool *const high);

/** @brief Levels of all eight pins at once, bit n = channel n */
tla_error_t tla_gpo_write_all(const uint8_t levels);
tla_error_t tla_gpi_read_all(uint8_t *const levels);

/**
 * @brief One conversion of an analog channel, manual mode.
 * @param code Left-aligned 16-bit result: the 12-bit code in bits 15..4
 *        without averaging, 16 bits with. tla_code_to_12bit() and
 *        tla_code_to_millivolts() scale it.
 */
tla_error_t tla_read(const uint8_t channel, uint16_t *const code);

/**
 * @brief One conversion of each channel in `mask` (bit n = channel n), in
 *        ascending channel order, by the device's auto-sequencer. Pins
 *        configured as GPIO must not be in the mask.
 * @param codes TLA_CHANNELS entries; only the masked ones are written
 */
tla_error_t tla_read_sequence(const uint8_t mask, uint16_t *const codes);

/** @brief Debug: every conversion returns TLA_DATA_FIXED_PATTERN */
tla_error_t tla_set_fixed_pattern(const bool enable);

/** @brief 0..4095 from a result code */
static inline uint16_t tla_code_to_12bit(const uint16_t code) { return (uint16_t)(code >> 4); }

/** @brief Millivolts from a result code, for an AVDD of avdd_mv */
static inline uint32_t tla_code_to_millivolts(const uint16_t code, const uint32_t avdd_mv)
{
    return ((uint32_t)code * avdd_mv) / 65536U;
}

#endif /* TLA2528_H */
