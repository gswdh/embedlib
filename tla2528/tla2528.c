/**
 * @file tla2528.c
 * @brief TI TLA2528 driver (see tla2528.h)
 */

#include "tla2528.h"

/* Opcodes (datasheet Table 7) */
#define TLA_OP_READ        (0x10U)
#define TLA_OP_WRITE       (0x08U)
#define TLA_OP_SET_BITS    (0x18U)
#define TLA_OP_CLEAR_BITS  (0x20U)
#define TLA_OP_BLOCK_READ  (0x30U)
#define TLA_OP_BLOCK_WRITE (0x28U)

/* Calibration and reset complete well inside this */
#define TLA_SETTLE_POLLS   (10U)
#define TLA_SETTLE_STEP_MS (1U)

static uint8_t tla_addr7 = TLA_ADDR_NONE;
static uint8_t tla_pin_cfg = 0U; /* PIN_CFG as written: 1 = GPIO */
static uint8_t tla_gpo     = 0U; /* GPO_VALUE as written */
static bool    tla_osr_on  = false;

/* ------------------------------------------------------------------------ */
/* Hooks: weak so a build without the application's I2C still links         */
/* ------------------------------------------------------------------------ */

tla_error_t __attribute__((weak))
tla_i2c_write(const uint8_t addr7, const uint8_t *data, const uint32_t len)
{
    (void)addr7;
    (void)data;
    (void)len;
    return TLA_ERROR_I2C;
}

tla_error_t __attribute__((weak)) tla_i2c_read(const uint8_t addr7, uint8_t *data, const uint32_t len)
{
    (void)addr7;
    (void)data;
    (void)len;
    return TLA_ERROR_I2C;
}

tla_error_t __attribute__((weak)) tla_i2c_write_read(
    const uint8_t addr7, const uint8_t *tx, const uint32_t tx_len, uint8_t *rx, const uint32_t rx_len)
{
    (void)addr7;
    (void)tx;
    (void)tx_len;
    (void)rx;
    (void)rx_len;
    return TLA_ERROR_I2C;
}

void __attribute__((weak)) tla_delay_ms(const uint32_t ms) { (void)ms; }

/* ------------------------------------------------------------------------ */
/* Registers                                                                 */
/* ------------------------------------------------------------------------ */

tla_error_t tla_read_register(const uint8_t reg, uint8_t *const value)
{
    const uint8_t tx[2] = {TLA_OP_READ, reg};
    return tla_i2c_write_read(tla_addr7, tx, sizeof(tx), value, 1U);
}

static tla_error_t tla_command(const uint8_t opcode, const uint8_t reg, const uint8_t value)
{
    const uint8_t tx[3] = {opcode, reg, value};
    return tla_i2c_write(tla_addr7, tx, sizeof(tx));
}

tla_error_t tla_write_register(const uint8_t reg, const uint8_t value)
{
    return tla_command(TLA_OP_WRITE, reg, value);
}

tla_error_t tla_set_bits(const uint8_t reg, const uint8_t bits)
{
    return tla_command(TLA_OP_SET_BITS, reg, bits);
}

tla_error_t tla_clear_bits(const uint8_t reg, const uint8_t bits)
{
    return tla_command(TLA_OP_CLEAR_BITS, reg, bits);
}

/* Wait for a self-clearing GENERAL_CFG bit (RST, CAL) to drop */
static tla_error_t tla_wait_clear(const uint8_t bit)
{
    for (uint32_t i = 0U; i < TLA_SETTLE_POLLS; i++)
    {
        uint8_t cfg = 0U;
        tla_delay_ms(TLA_SETTLE_STEP_MS);
        if (tla_read_register(TLA_REG_GENERAL_CFG, &cfg) != TLA_OK)
        {
            return TLA_ERROR_I2C;
        }
        if ((cfg & bit) == 0U)
        {
            return TLA_OK;
        }
    }
    return TLA_ERROR_TIMEOUT;
}

/* ------------------------------------------------------------------------ */
/* Setup                                                                     */
/* ------------------------------------------------------------------------ */

tla_error_t tla_reset(void)
{
    const tla_error_t error = tla_write_register(TLA_REG_GENERAL_CFG, TLA_CFG_RST);
    if (error != TLA_OK)
    {
        return error;
    }
    tla_pin_cfg = 0U;
    tla_gpo     = 0U;
    tla_osr_on  = false;
    return tla_wait_clear(TLA_CFG_RST);
}

tla_error_t tla_calibrate(void)
{
    const tla_error_t error = tla_set_bits(TLA_REG_GENERAL_CFG, TLA_CFG_CAL);
    if (error != TLA_OK)
    {
        return error;
    }
    return tla_wait_clear(TLA_CFG_CAL);
}

tla_error_t tla_init(const uint8_t addr7)
{
    tla_addr7 = addr7;

    /* Alive? Bit 7 of SYSTEM_STATUS always reads 1 */
    uint8_t status = 0U;
    if (tla_read_register(TLA_REG_SYSTEM_STATUS, &status) != TLA_OK)
    {
        return TLA_ERROR_NOT_FOUND;
    }
    if (((status & TLA_STATUS_RSVD_ONE) == 0U) || ((status & TLA_STATUS_CRC_ERR_FUSE) != 0U))
    {
        return TLA_ERROR_NOT_FOUND;
    }

    tla_error_t error = tla_reset();
    if (error != TLA_OK)
    {
        return error;
    }

    /* Brown-out flag: ours now */
    error = tla_write_register(TLA_REG_SYSTEM_STATUS, TLA_STATUS_BOR);
    if (error != TLA_OK)
    {
        return error;
    }

    return tla_calibrate();
}

uint8_t tla_address(void) { return tla_addr7; }

tla_error_t tla_set_oversampling(const tla_osr_t osr)
{
    if ((uint8_t)osr > (uint8_t)TLA_OSR_128)
    {
        return TLA_ERROR_INVALID_PARAM;
    }
    const tla_error_t error = tla_write_register(TLA_REG_OSR_CFG, (uint8_t)osr);
    if (error == TLA_OK)
    {
        tla_osr_on = (osr != TLA_OSR_1);
    }
    return error;
}

tla_error_t tla_set_fixed_pattern(const bool enable)
{
    return enable ? tla_set_bits(TLA_REG_DATA_CFG, TLA_DATA_FIX_PAT)
                  : tla_clear_bits(TLA_REG_DATA_CFG, TLA_DATA_FIX_PAT);
}

/* ------------------------------------------------------------------------ */
/* GPIO                                                                      */
/* ------------------------------------------------------------------------ */

tla_error_t tla_configure_pin(const uint8_t channel, const tla_pin_mode_t mode)
{
    if (channel >= TLA_CHANNELS)
    {
        return TLA_ERROR_INVALID_PARAM;
    }
    const uint8_t bit = (uint8_t)(1U << channel);
    tla_error_t   error;

    switch (mode)
    {
    case TLA_PIN_ANALOG_IN:
        error = tla_clear_bits(TLA_REG_PIN_CFG, bit);
        if (error == TLA_OK)
        {
            tla_pin_cfg &= (uint8_t)~bit;
        }
        return error;

    case TLA_PIN_DIGITAL_IN:
        error = tla_clear_bits(TLA_REG_GPIO_CFG, bit);
        break;

    case TLA_PIN_OUTPUT_OPEN_DRAIN:
    case TLA_PIN_OUTPUT_PUSH_PULL:
        /* Level and drive before the pin becomes an output: it starts low */
        tla_gpo &= (uint8_t)~bit;
        error = tla_write_register(TLA_REG_GPO_VALUE, tla_gpo);
        if (error == TLA_OK)
        {
            error = (mode == TLA_PIN_OUTPUT_PUSH_PULL) ? tla_set_bits(TLA_REG_GPO_DRIVE_CFG, bit)
                                                       : tla_clear_bits(TLA_REG_GPO_DRIVE_CFG, bit);
        }
        if (error == TLA_OK)
        {
            error = tla_set_bits(TLA_REG_GPIO_CFG, bit);
        }
        break;

    default:
        return TLA_ERROR_INVALID_PARAM;
    }
    if (error != TLA_OK)
    {
        return error;
    }

    /* Direction set: now a GPIO rather than an analog input */
    error = tla_set_bits(TLA_REG_PIN_CFG, bit);
    if (error == TLA_OK)
    {
        tla_pin_cfg |= bit;
    }
    return error;
}

tla_error_t tla_gpo_write_all(const uint8_t levels)
{
    const tla_error_t error = tla_write_register(TLA_REG_GPO_VALUE, levels);
    if (error == TLA_OK)
    {
        tla_gpo = levels;
    }
    return error;
}

tla_error_t tla_gpo_write(const uint8_t channel, const bool high)
{
    if (channel >= TLA_CHANNELS)
    {
        return TLA_ERROR_INVALID_PARAM;
    }
    const uint8_t bit = (uint8_t)(1U << channel);
    return tla_gpo_write_all(high ? (uint8_t)(tla_gpo | bit) : (uint8_t)(tla_gpo & (uint8_t)~bit));
}

tla_error_t tla_gpi_read_all(uint8_t *const levels)
{
    return tla_read_register(TLA_REG_GPI_VALUE, levels);
}

tla_error_t tla_gpi_read(const uint8_t channel, bool *const high)
{
    if (channel >= TLA_CHANNELS)
    {
        return TLA_ERROR_INVALID_PARAM;
    }
    uint8_t           levels = 0U;
    const tla_error_t error  = tla_gpi_read_all(&levels);
    if (error == TLA_OK)
    {
        *high = ((levels >> channel) & 1U) != 0U;
    }
    return error;
}

/* ------------------------------------------------------------------------ */
/* Conversions                                                               */
/* ------------------------------------------------------------------------ */

/* One result frame: 2 bytes, MSB first (frame A / B of datasheet Figure 25) */
static tla_error_t tla_read_result(uint16_t *const code)
{
    uint8_t           rx[2] = {0U, 0U};
    const tla_error_t error = tla_i2c_read(tla_addr7, rx, sizeof(rx));
    if (error == TLA_OK)
    {
        *code = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    }
    return error;
}

tla_error_t tla_read(const uint8_t channel, uint16_t *const code)
{
    if (channel >= TLA_CHANNELS)
    {
        return TLA_ERROR_INVALID_PARAM;
    }
    if ((tla_pin_cfg & (1U << channel)) != 0U)
    {
        return TLA_ERROR_NOT_ANALOG;
    }

    tla_error_t error = tla_write_register(TLA_REG_CHANNEL_SEL, channel);
    if (error != TLA_OK)
    {
        return error;
    }

    /* The conversion a read frame returns was started at the end of the
     * previous frame, so the first frame after a channel change can still
     * hold the old channel: read twice and keep the second. */
    uint16_t discard = 0U;
    error            = tla_read_result(&discard);
    if (error != TLA_OK)
    {
        return error;
    }
    return tla_read_result(code);
}

tla_error_t tla_read_sequence(const uint8_t mask, uint16_t *const codes)
{
    if ((mask == 0U) || ((mask & tla_pin_cfg) != 0U))
    {
        return (mask == 0U) ? TLA_ERROR_INVALID_PARAM : TLA_ERROR_NOT_ANALOG;
    }

    /* Channel ID appended so the order can be checked, auto-sequence over the mask */
    tla_error_t error = tla_set_bits(TLA_REG_DATA_CFG, TLA_DATA_APPEND_CHID);
    if (error == TLA_OK)
    {
        error = tla_write_register(TLA_REG_AUTO_SEQ_CH_SEL, mask);
    }
    if (error == TLA_OK)
    {
        error = tla_write_register(TLA_REG_SEQUENCE_CFG, TLA_SEQ_MODE_AUTO | TLA_SEQ_START);
    }

    /* Frame C (2 bytes) or D (3 bytes, averaged), one per enabled channel,
     * plus one frame of latency at the start */
    const uint32_t frame = tla_osr_on ? 3U : 2U;
    uint32_t       count = 0U;
    for (uint8_t ch = 0U; ch < TLA_CHANNELS; ch++)
    {
        count += ((mask >> ch) & 1U);
    }
    for (uint32_t n = 0U; (error == TLA_OK) && (n <= count); n++)
    {
        uint8_t rx[3] = {0U, 0U, 0U};
        error         = tla_i2c_read(tla_addr7, rx, frame);
        if ((error != TLA_OK) || (n == 0U))
        {
            continue;
        }
        if (frame == 2U)
        {
            /* D11..D4, then D3..D0 and the channel ID */
            codes[rx[1] & 0x0FU] = (uint16_t)(((uint16_t)rx[0] << 8) | (rx[1] & 0xF0U));
        }
        else
        {
            codes[rx[2] >> 4] = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
        }
    }

    /* Sequencer off and the ID append with it, whatever happened */
    const tla_error_t stop = tla_write_register(TLA_REG_SEQUENCE_CFG, 0U);
    const tla_error_t tidy = tla_clear_bits(TLA_REG_DATA_CFG, TLA_DATA_APPEND_CHID);
    if (error == TLA_OK)
    {
        error = (stop != TLA_OK) ? stop : tidy;
    }
    return error;
}
