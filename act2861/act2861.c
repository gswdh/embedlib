#include "act2861.h"

/* -------------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------------- */

static act2861_config_t s_config      = {0.0F, 0.0F, 0.0F, 0.0F, 0U};
static uint8_t          s_initialised = 0U;

/* Number of write/read-back attempts made by the verified write helper. */
#define ACT2861_VERIFY_ATTEMPTS (3U)
/* Settling delay between a verified write and its read-back, milliseconds. */
#define ACT2861_VERIFY_DELAY_MS (1U)
/* Poll interval used while waiting on ADC_DATA_READY or REGISTER_RESET. */
#define ACT2861_POLL_INTERVAL_MS (1U)
/* Default budget for a single ADC conversion, milliseconds. */
#define ACT2861_ADC_TIMEOUT_MS (50U)
/* Budget for REGISTER_RESET to self-clear, milliseconds. */
#define ACT2861_RESET_TIMEOUT_MS (20U)

/* -------------------------------------------------------------------------
 * Weak platform hooks.  The application overrides these; the defaults report
 * a distinct error so a missing port cannot masquerade as a dead bus.
 * ------------------------------------------------------------------------- */

act2861_error_t __attribute__((weak))
act2861_i2c_read(const uint8_t dev_addr, const uint8_t reg, uint8_t *rx, const uint16_t len)
{
    (void)dev_addr;
    (void)reg;
    (void)rx;
    (void)len;
    return ACT2861_ERR_NO_PLATFORM_READ;
}

act2861_error_t __attribute__((weak)) act2861_i2c_write(const uint8_t  dev_addr,
                                                        const uint8_t  reg,
                                                        const uint8_t *tx,
                                                        const uint16_t len)
{
    (void)dev_addr;
    (void)reg;
    (void)tx;
    (void)len;
    return ACT2861_ERR_NO_PLATFORM_WRITE;
}

void __attribute__((weak)) act2861_delay_ms(const uint32_t ms) { (void)ms; }

/* -------------------------------------------------------------------------
 * Raw register access
 * ------------------------------------------------------------------------- */

static uint8_t act2861_address(void)
{
    return (s_config.i2c_address != 0U) ? s_config.i2c_address : (uint8_t)ACT2861_I2C_ADDR_DEFAULT;
}

act2861_error_t act2861_read_register(const uint8_t reg, uint8_t *value)
{
    if (value == (uint8_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }
    if (reg > (uint8_t)ACT2861_REG_MAX)
    {
        return ACT2861_ERR_BAD_REGISTER;
    }
    return act2861_i2c_read(act2861_address(), reg, value, 1U);
}

act2861_error_t act2861_write_register(const uint8_t reg, const uint8_t value)
{
    if (reg > (uint8_t)ACT2861_REG_MAX)
    {
        return ACT2861_ERR_BAD_REGISTER;
    }
    return act2861_i2c_write(act2861_address(), reg, &value, 1U);
}

act2861_error_t act2861_write_verify_register(const uint8_t reg, const uint8_t value)
{
    uint8_t attempt = 0U;

    for (attempt = 0U; attempt < (uint8_t)ACT2861_VERIFY_ATTEMPTS; attempt++)
    {
        uint8_t         read_back = 0U;
        act2861_error_t err       = act2861_write_register(reg, value);
        if (err != ACT2861_OK)
        {
            return err;
        }

        act2861_delay_ms(ACT2861_VERIFY_DELAY_MS);

        err = act2861_read_register(reg, &read_back);
        if (err != ACT2861_OK)
        {
            return err;
        }
        if (read_back == value)
        {
            return ACT2861_OK;
        }
    }
    return ACT2861_ERR_WRITE_VERIFY;
}

act2861_error_t act2861_read_registers(const uint8_t reg, uint8_t *values, const uint8_t count)
{
    if (values == (uint8_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }
    if (reg > (uint8_t)ACT2861_REG_MAX)
    {
        return ACT2861_ERR_BAD_REGISTER;
    }
    if (count == 0U)
    {
        return ACT2861_ERR_BAD_LENGTH;
    }
    if (((uint16_t)reg + (uint16_t)count - 1U) > (uint16_t)ACT2861_REG_MAX)
    {
        return ACT2861_ERR_BAD_LENGTH;
    }
    return act2861_i2c_read(act2861_address(), reg, values, (uint16_t)count);
}

act2861_error_t
act2861_update_register(const uint8_t reg, const uint8_t mask, const uint8_t value)
{
    uint8_t               current = 0U;
    const act2861_error_t err     = act2861_read_register(reg, &current);
    if (err != ACT2861_OK)
    {
        return err;
    }

    const uint8_t updated = (uint8_t)((current & (uint8_t)(~mask)) | (value & mask));
    if (updated == current)
    {
        return ACT2861_OK;
    }
    return act2861_write_register(reg, updated);
}

/* Set or clear a single bit in a register. */
static act2861_error_t
act2861_set_bit(const uint8_t reg, const uint8_t bit, const uint8_t enabled)
{
    return act2861_update_register(reg, bit, (enabled != 0U) ? bit : 0U);
}

/* Write an enumerated field, validating the value against its width first. */
static act2861_error_t act2861_set_field(const uint8_t         reg,
                                         const uint8_t         mask,
                                         const uint8_t         shift,
                                         const uint32_t        value,
                                         const uint32_t        max_value,
                                         const act2861_error_t bad_value_error)
{
    if (value > max_value)
    {
        return bad_value_error;
    }
    return act2861_update_register(reg, mask, (uint8_t)((uint8_t)value << shift));
}

/* -------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_init(const act2861_config_t *cfg)
{
    uint8_t probe = 0U;

    if (cfg == (const act2861_config_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }
    if ((cfg->rcs_in_ohms <= 0.0F) || (cfg->rilim_ohms <= 0.0F) || (cfg->rcs_out_ohms <= 0.0F) ||
        (cfg->rolim_ohms <= 0.0F))
    {
        return ACT2861_ERR_BAD_SENSE_RESISTOR;
    }

    s_config      = *cfg;
    s_initialised = 0U;

    /* Prove the device acknowledges before declaring the driver usable. */
    const act2861_error_t err = act2861_read_register(ACT2861_REG_MAIN_CONTROL_2, &probe);
    if (err != ACT2861_OK)
    {
        return err;
    }

    s_initialised = 1U;
    return ACT2861_OK;
}

act2861_error_t act2861_reset(void)
{
    uint32_t waited = 0U;

    act2861_error_t err =
        act2861_write_register(ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_REGISTER_RESET);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /* REGISTER_RESET clears itself once the defaults have been restored. */
    for (;;)
    {
        uint8_t value = 0U;

        err = act2861_read_register(ACT2861_REG_MAIN_CONTROL_1, &value);
        if (err != ACT2861_OK)
        {
            return err;
        }
        if ((value & ACT2861_MC1_REGISTER_RESET) == 0U)
        {
            return ACT2861_OK;
        }
        if (waited >= (uint32_t)ACT2861_RESET_TIMEOUT_MS)
        {
            return ACT2861_ERR_RESET_TIMEOUT;
        }

        act2861_delay_ms(ACT2861_POLL_INTERVAL_MS);
        waited += ACT2861_POLL_INTERVAL_MS;
    }
}

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_get_general_status(act2861_general_status_t *status)
{
    uint8_t raw = 0U;

    if (status == (act2861_general_status_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_register(ACT2861_REG_GENERAL_STATUS, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    status->raw              = raw;
    status->battery_low      = ((raw & ACT2861_GS_NVBAT_GOOD) != 0U) ? 1U : 0U;
    status->irq_asserted     = ((raw & ACT2861_GS_NIRQ_PIN_STATUS) != 0U) ? 1U : 0U;
    status->notg_pin_high    = ((raw & ACT2861_GS_NOTG_PIN_STATUS) != 0U) ? 1U : 0U;
    status->input_below_uvlo = ((raw & ACT2861_GS_INPUT_UVLO_CHG) != 0U) ? 1U : 0U;
    status->input_above_ov   = ((raw & ACT2861_GS_INPUT_OV_CHG) != 0U) ? 1U : 0U;
    status->gpio_in          = ((raw & ACT2861_GS_GPIO_IN) != 0U) ? 1U : 0U;
    status->mode             = (act2861_mode_t)(raw & ACT2861_GS_OPERATION_MASK);
    return ACT2861_OK;
}

act2861_error_t act2861_get_mode(act2861_mode_t *mode)
{
    uint8_t raw = 0U;

    if (mode == (act2861_mode_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_register(ACT2861_REG_GENERAL_STATUS, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    *mode = (act2861_mode_t)(raw & ACT2861_GS_OPERATION_MASK);
    return ACT2861_OK;
}

act2861_error_t act2861_get_charger_status(act2861_charger_status_t *status)
{
    uint8_t raw = 0U;

    if (status == (act2861_charger_status_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_register(ACT2861_REG_CHARGER_STATUS, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    status->raw                    = raw;
    status->en_chg_pin_high        = ((raw & ACT2861_CS_EN_CHG_PIN_STATUS) != 0U) ? 1U : 0U;
    status->thermal_regulation     = ((raw & ACT2861_CS_THERMAL_ACTIVE) != 0U) ? 1U : 0U;
    status->in_input_current_limit = ((raw & ACT2861_CS_INPUT_IINLIM) != 0U) ? 1U : 0U;
    status->in_input_voltage_limit = ((raw & ACT2861_CS_INPUT_VINLIM) != 0U) ? 1U : 0U;
    status->state = (act2861_charge_state_t)(raw & ACT2861_CS_CHG_STATUS_MASK);
    return ACT2861_OK;
}

act2861_error_t act2861_get_temperature_status(act2861_temp_status_t *status)
{
    uint8_t raw = 0U;

    if (status == (act2861_temp_status_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_register(ACT2861_REG_TEMP_STATUS, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    status->raw               = raw;
    status->power_ok          = ((raw & ACT2861_TS_POK_VOUT) != 0U) ? 1U : 0U;
    status->battery_detected  = ((raw & ACT2861_TS_TH_BAT_DETECT) != 0U) ? 1U : 0U;
    status->otg_cold_disabled = ((raw & ACT2861_TS_OTG_COLD_DIS) != 0U) ? 1U : 0U;
    status->otg_hot_disabled  = ((raw & ACT2861_TS_OTG_HOT_DIS) != 0U) ? 1U : 0U;
    status->charge_cold       = ((raw & ACT2861_TS_CHRG_COLD) != 0U) ? 1U : 0U;
    status->charge_cool       = ((raw & ACT2861_TS_CHRG_COOL) != 0U) ? 1U : 0U;
    status->charge_warm       = ((raw & ACT2861_TS_CHRG_WARM) != 0U) ? 1U : 0U;
    status->charge_hot        = ((raw & ACT2861_TS_CHRG_HOT) != 0U) ? 1U : 0U;
    return ACT2861_OK;
}

act2861_error_t act2861_get_faults(act2861_faults_t *faults)
{
    uint8_t raw[2] = {0U, 0U};

    if (faults == (act2861_faults_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    /* One burst so both latching registers are cleared in a single pass. */
    const act2861_error_t err = act2861_read_registers(ACT2861_REG_FAULTS_1, raw, 2U);
    if (err != ACT2861_OK)
    {
        return err;
    }

    faults->raw1 = raw[0];
    faults->raw2 = raw[1];

    faults->charge_timer_expired = ((raw[0] & ACT2861_F1_CHG_TIMER_EXPIRED) != 0U) ? 1U : 0U;
    faults->charge_vbat_ov       = ((raw[0] & ACT2861_F1_CHG_VBAT_OV) != 0U) ? 1U : 0U;
    faults->vreg_oc_uvlo         = ((raw[0] & ACT2861_F1_VREG_OC_UVLO) != 0U) ? 1U : 0U;
    faults->thermal_shutdown     = ((raw[0] & ACT2861_F1_TSD) != 0U) ? 1U : 0U;
    faults->fet_overcurrent      = ((raw[0] & ACT2861_F1_FET_OC) != 0U) ? 1U : 0U;
    faults->input_overvoltage    = ((raw[0] & ACT2861_F1_CHG_INPUT_OV) != 0U) ? 1U : 0U;
    faults->input_undervoltage   = ((raw[0] & ACT2861_F1_CHG_INPUT_UV) != 0U) ? 1U : 0U;

    faults->watchdog_fault  = ((raw[1] & ACT2861_F2_WATCHDOG_FAULT) != 0U) ? 1U : 0U;
    faults->otg_vout_hiccup = ((raw[1] & ACT2861_F2_OTG_VOUT_FAULT) != 0U) ? 1U : 0U;
    faults->otg_vbat_cutoff = ((raw[1] & ACT2861_F2_OTG_VBAT_CUTOFF) != 0U) ? 1U : 0U;
    faults->otg_vout_ov     = ((raw[1] & ACT2861_F2_OTG_VOUT_OV) != 0U) ? 1U : 0U;
    faults->otg_light_load  = ((raw[1] & ACT2861_F2_OTG_LIGHT_LOAD) != 0U) ? 1U : 0U;
    faults->otg_vbat_ov     = ((raw[1] & ACT2861_F2_OTG_VBAT_OV) != 0U) ? 1U : 0U;
    faults->i2c_fault       = ((raw[1] & ACT2861_F2_I2C_FAULT) != 0U) ? 1U : 0U;
    faults->dead_battery    = ((raw[1] & ACT2861_F2_DEADBATTERY) != 0U) ? 1U : 0U;
    return ACT2861_OK;
}

act2861_error_t act2861_get_otg_status(act2861_otg_status_t *status)
{
    uint8_t raw = 0U;

    if (status == (act2861_otg_status_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_register(ACT2861_REG_OTG_STATUS, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    status->raw                      = raw;
    status->battery_constant_current = ((raw & ACT2861_OTGS_BATTERY_CC) != 0U) ? 1U : 0U;
    status->output_constant_current  = ((raw & ACT2861_OTGS_OUTPUT_CC) != 0U) ? 1U : 0U;
    status->vbat_below_cutoff        = ((raw & ACT2861_OTGS_VBAT_CUTOFF) != 0U) ? 1U : 0U;
    status->vbat_above_ov            = ((raw & ACT2861_OTGS_VBAT_OV) != 0U) ? 1U : 0U;
    status->state = (act2861_otg_state_t)(raw & ACT2861_OTGS_STATE_MASK);
    return ACT2861_OK;
}

act2861_error_t act2861_clear_irq(void)
{
    /*
     * nIRQ_Clear is write-1-to-clear and self-clearing, and the remaining
     * bits of Faults 1 are read-to-clear, so this is a plain write rather
     * than a read-modify-write.
     */
    return act2861_write_register(ACT2861_REG_FAULTS_1, ACT2861_F1_NIRQ_CLEAR);
}

/* -------------------------------------------------------------------------
 * Mode and power control
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_hiz(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_HIZ, enabled);
}

/* HIZ silently defeats both converter enables, so check before writing. */
static act2861_error_t act2861_check_not_hiz(void)
{
    uint8_t               value = 0U;
    const act2861_error_t err   = act2861_read_register(ACT2861_REG_MAIN_CONTROL_1, &value);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return ((value & ACT2861_MC1_HIZ) != 0U) ? ACT2861_ERR_HIZ_ACTIVE : ACT2861_OK;
}

act2861_error_t act2861_set_charging_enabled(const uint8_t enabled)
{
    if (enabled != 0U)
    {
        const act2861_error_t err = act2861_check_not_hiz();
        if (err != ACT2861_OK)
        {
            return err;
        }
    }
    return act2861_set_bit(
        ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_OVERRIDE_EN_CHG, enabled);
}

act2861_error_t act2861_set_otg_enabled(const uint8_t enabled)
{
    if (enabled != 0U)
    {
        const act2861_error_t err = act2861_check_not_hiz();
        if (err != ACT2861_OK)
        {
            return err;
        }
    }

    /*
     * OTG_EN is the master enable and EN_OVERRIDE hands control of it to
     * I2C; both move together so the nOTG pin is never left in charge of a
     * software-requested state.
     */
    const uint8_t mask  = (uint8_t)(ACT2861_OTG1_OTG_EN | ACT2861_OTG1_EN_OVERRIDE);
    const uint8_t value = (enabled != 0U) ? mask : 0U;

    return act2861_update_register(ACT2861_REG_OTG_CONTROL_1, mask, value);
}

act2861_error_t act2861_enter_ship_mode(void)
{
    return act2861_set_bit(ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_SHIPM_ENTER, 1U);
}

act2861_error_t act2861_cancel_ship_mode(void)
{
    return act2861_set_bit(ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_SHIPM_ENTER, 0U);
}

act2861_error_t act2861_set_vreg_enabled(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_MAIN_CONTROL_2, ACT2861_MC2_VREG_EN, enabled);
}

act2861_error_t act2861_set_nchg_enabled(const uint8_t enabled)
{
    /* DIS_nCHG_CHG is active high, so the sense is inverted here. */
    return act2861_set_bit(
        ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_DIS_NCHG_CHG, (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_audio_frequency_limit(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_AUDIO_FREQ_LIMIT, enabled);
}

act2861_error_t act2861_set_switching_frequency(const act2861_frequency_t frequency)
{
    return act2861_set_field(ACT2861_REG_TEMP_SETTING,
                             ACT2861_TEMP_FREQ_MASK,
                             ACT2861_TEMP_FREQ_SHIFT,
                             (uint32_t)frequency,
                             3U,
                             ACT2861_ERR_BAD_FREQUENCY);
}

/* -------------------------------------------------------------------------
 * Watchdog
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_watchdog(const act2861_watchdog_t timeout)
{
    return act2861_set_field(ACT2861_REG_MAIN_CONTROL_2,
                             ACT2861_MC2_WATCHDOG_MASK,
                             0U,
                             (uint32_t)timeout,
                             3U,
                             ACT2861_ERR_BAD_WATCHDOG);
}

act2861_error_t act2861_kick_watchdog(void)
{
    /* Auto-clearing on write, so no read-modify-write. */
    return act2861_write_register(ACT2861_REG_MAIN_CONTROL_1, ACT2861_MC1_WATCHDOG_RESET);
}

/* -------------------------------------------------------------------------
 * Voltage and percentage encoding helpers
 * ------------------------------------------------------------------------- */

/* Convert volts to a register code, rejecting anything outside the range. */
static act2861_error_t act2861_encode_voltage(const float  volts,
                                              const float  offset,
                                              const float  lsb,
                                              const float  max_volts,
                                              const uint16_t max_code,
                                              uint16_t      *code)
{
    if ((volts < offset) || (volts > max_volts))
    {
        return ACT2861_ERR_BAD_VOLTAGE;
    }

    const float    steps  = (volts - offset) / lsb;
    const uint32_t rounded = (uint32_t)(steps + 0.5F);

    *code = (rounded > (uint32_t)max_code) ? max_code : (uint16_t)rounded;
    return ACT2861_OK;
}

static act2861_error_t act2861_check_percent(const uint8_t percent,
                                             const uint8_t min_percent,
                                             const uint8_t max_percent)
{
    if ((percent < min_percent) || (percent > max_percent))
    {
        return ACT2861_ERR_BAD_PERCENT;
    }
    return ACT2861_OK;
}

/* -------------------------------------------------------------------------
 * Charge configuration
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_charge_voltage(const float volts)
{
    uint16_t code = 0U;

    act2861_error_t err = act2861_encode_voltage(volts,
                                                 ACT2861_VTERM_OFFSET_V,
                                                 ACT2861_VTERM_LSB_V,
                                                 ACT2861_VTERM_MAX_V,
                                                 (uint16_t)ACT2861_VTERM_MAX_CODE,
                                                 &code);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /* VTERM[10:8] shares register 0x11 with the VREG LDO setting. */
    err = act2861_update_register(ACT2861_REG_VBAT_REG_1,
                                  ACT2861_VBAT1_VTERM_HI_MASK,
                                  (uint8_t)((code >> 8U) & ACT2861_VBAT1_VTERM_HI_MASK));
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_write_register(ACT2861_REG_VBAT_REG_2, (uint8_t)(code & 0xFFU));
}

act2861_error_t act2861_get_charge_voltage(float *volts)
{
    uint8_t raw[2] = {0U, 0U};

    if (volts == (float *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_registers(ACT2861_REG_VBAT_REG_1, raw, 2U);
    if (err != ACT2861_OK)
    {
        return err;
    }

    const uint16_t code =
        (uint16_t)((((uint16_t)(raw[0] & ACT2861_VBAT1_VTERM_HI_MASK)) << 8U) | (uint16_t)raw[1]);

    *volts = ACT2861_VTERM_OFFSET_V + ((float)code * ACT2861_VTERM_LSB_V);
    if (*volts > ACT2861_VTERM_MAX_V)
    {
        *volts = ACT2861_VTERM_MAX_V;
    }
    return ACT2861_OK;
}

act2861_error_t act2861_set_vreg_voltage(const float volts)
{
    uint16_t code = 0U;

    const act2861_error_t err = act2861_encode_voltage(volts,
                                                       ACT2861_VREG_OFFSET_V,
                                                       ACT2861_VREG_LSB_V,
                                                       ACT2861_VREG_MAX_V,
                                                       31U,
                                                       &code);
    if (err != ACT2861_OK)
    {
        return err;
    }

    return act2861_update_register(ACT2861_REG_VBAT_REG_1,
                                   ACT2861_VBAT1_VREG_MASK,
                                   (uint8_t)((uint8_t)code << ACT2861_VBAT1_VREG_SHIFT));
}

act2861_error_t act2861_set_fast_charge_percent(const uint8_t percent)
{
    const act2861_error_t err =
        act2861_check_percent(percent, (uint8_t)ACT2861_PERCENT_MIN, (uint8_t)ACT2861_PERCENT_MAX);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_update_register(ACT2861_REG_FAST_CHG_CURRENT, ACT2861_IFCHG_MASK, percent);
}

act2861_error_t act2861_set_precharge_percent(const uint8_t percent)
{
    const act2861_error_t err = act2861_check_percent(
        percent, (uint8_t)ACT2861_PRE_TERM_MIN_PCT, (uint8_t)ACT2861_PRE_TERM_MAX_PCT);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /* The field is an offset from 5 percent. */
    const uint8_t code = (uint8_t)(percent - (uint8_t)ACT2861_PRE_TERM_MIN_PCT);
    return act2861_update_register(ACT2861_REG_PRE_TERM_CURRENT,
                                   ACT2861_IPRECHG_MASK,
                                   (uint8_t)(code << ACT2861_IPRECHG_SHIFT));
}

act2861_error_t act2861_set_termination_percent(const uint8_t percent)
{
    const act2861_error_t err = act2861_check_percent(
        percent, (uint8_t)ACT2861_PRE_TERM_MIN_PCT, (uint8_t)ACT2861_PRE_TERM_MAX_PCT);
    if (err != ACT2861_OK)
    {
        return err;
    }

    const uint8_t code = (uint8_t)(percent - (uint8_t)ACT2861_PRE_TERM_MIN_PCT);
    return act2861_update_register(ACT2861_REG_PRE_TERM_CURRENT, ACT2861_ITERM_MASK, code);
}

act2861_error_t act2861_set_termination_enabled(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_CHARGE_CONTROL_2, ACT2861_CC2_EN_TERM, enabled);
}

act2861_error_t act2861_set_input_current_percent(const uint8_t percent)
{
    const act2861_error_t err =
        act2861_check_percent(percent, (uint8_t)ACT2861_PERCENT_MIN, (uint8_t)ACT2861_PERCENT_MAX);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_update_register(ACT2861_REG_INPUT_CURR_LIMIT, ACT2861_IIN_LIMIT_MASK, percent);
}

act2861_error_t act2861_set_input_current_limit_enabled(const uint8_t enabled)
{
    return act2861_set_bit(
        ACT2861_REG_INPUT_CURR_LIMIT, ACT2861_IIN_DIS_LIMIT, (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_input_voltage_limit(const float volts)
{
    uint16_t code = 0U;

    const act2861_error_t err = act2861_encode_voltage(volts,
                                                       ACT2861_VINLIM_OFFSET_V,
                                                       ACT2861_VINLIM_LSB_V,
                                                       ACT2861_VINLIM_MAX_V,
                                                       127U,
                                                       &code);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_update_register(
        ACT2861_REG_INPUT_VOLT_LIMIT, ACT2861_VIN_LIMIT_MASK, (uint8_t)code);
}

act2861_error_t act2861_set_input_voltage_limit_enabled(const uint8_t enabled)
{
    return act2861_set_bit(
        ACT2861_REG_INPUT_VOLT_LIMIT, ACT2861_VIN_DIS_LIMIT, (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_battery_low_voltage(const float volts)
{
    uint16_t code = 0U;

    const act2861_error_t err = act2861_encode_voltage(volts,
                                                       ACT2861_VBAT_LOW_OFFSET_V,
                                                       ACT2861_VBAT_LOW_LSB_V,
                                                       ACT2861_VBAT_LOW_MAX_V,
                                                       127U,
                                                       &code);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_update_register(ACT2861_REG_VBAT_LOW, ACT2861_VBAT_LOW_MASK, (uint8_t)code);
}

act2861_error_t act2861_set_battery_good_threshold(const act2861_vbatgood_t threshold)
{
    return act2861_set_field(ACT2861_REG_CHARGE_CONTROL_3,
                             ACT2861_CC3_VBATGOOD_MASK,
                             0U,
                             (uint32_t)threshold,
                             3U,
                             ACT2861_ERR_BAD_VBATGOOD);
}

act2861_error_t act2861_set_battery_short(const act2861_vbat_short_t    threshold,
                                          const act2861_short_current_t current)
{
    if ((uint32_t)threshold > 7U)
    {
        return ACT2861_ERR_BAD_VBAT_SHORT;
    }
    if ((uint32_t)current > 3U)
    {
        return ACT2861_ERR_BAD_SHORT_CURRENT;
    }

    const uint8_t mask =
        (uint8_t)(ACT2861_CC1_VBAT_SHORT_MASK | ACT2861_CC1_SHORT_CURRENT_MASK);
    const uint8_t value =
        (uint8_t)(((uint8_t)threshold << ACT2861_CC1_VBAT_SHORT_SHIFT) |
                  ((uint8_t)current << ACT2861_CC1_SHORT_CURRENT_SHIFT));

    return act2861_update_register(ACT2861_REG_CHARGE_CONTROL_1, mask, value);
}

act2861_error_t act2861_set_recharge_threshold(const act2861_vrecharge_t threshold)
{
    return act2861_set_field(ACT2861_REG_CHARGE_CONTROL_3,
                             ACT2861_CC3_VRECHARGE_MASK,
                             ACT2861_CC3_VRECHARGE_SHIFT,
                             (uint32_t)threshold,
                             7U,
                             ACT2861_ERR_BAD_VRECHARGE);
}

act2861_error_t act2861_set_start_delay(const act2861_start_delay_t delay)
{
    return act2861_set_field(ACT2861_REG_CHARGE_CONTROL_3,
                             ACT2861_CC3_VIN_STRT_DLY_MASK,
                             ACT2861_CC3_VIN_STRT_DLY_SHIFT,
                             (uint32_t)delay,
                             3U,
                             ACT2861_ERR_BAD_START_DELAY);
}

act2861_error_t act2861_set_path_compensation(const act2861_path_comp_t resistance,
                                              const act2861_vclamp_t    clamp)
{
    if ((uint32_t)resistance > 7U)
    {
        return ACT2861_ERR_BAD_PATH_COMP;
    }
    if ((uint32_t)clamp > 7U)
    {
        return ACT2861_ERR_BAD_VCLAMP;
    }

    const uint8_t mask  = (uint8_t)(ACT2861_CC2_VCLAMP_MASK | ACT2861_CC2_PATH_COMP_MASK);
    const uint8_t value = (uint8_t)(((uint8_t)clamp << ACT2861_CC2_VCLAMP_SHIFT) |
                                    (uint8_t)resistance);

    return act2861_update_register(ACT2861_REG_CHARGE_CONTROL_2, mask, value);
}

act2861_error_t act2861_set_fet_current_limit(const act2861_fet_limit_t limit)
{
    if ((uint32_t)limit > 1U)
    {
        return ACT2861_ERR_BAD_FET_LIMIT;
    }
    return act2861_set_bit(
        ACT2861_REG_MAIN_CONTROL_2, ACT2861_MC2_FET_ILIMIT, (uint8_t)limit);
}

act2861_error_t act2861_set_low_current_range(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_CHARGE_CONTROL_2, ACT2861_CC2_ILIM_LOW, enabled);
}

/* -------------------------------------------------------------------------
 * Safety timer
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_safety_timer_hours(const float hours)
{
    if ((hours < ACT2861_SAFETY_TIMER_MIN_H) || (hours > ACT2861_SAFETY_TIMER_MAX_H))
    {
        return ACT2861_ERR_BAD_HOURS;
    }

    /* The field counts half hours from a 0.5 hour offset. */
    const float    steps   = (hours - ACT2861_SAFETY_TIMER_MIN_H) / ACT2861_SAFETY_TIMER_LSB_H;
    const uint32_t rounded = (uint32_t)(steps + 0.5F);
    const uint8_t  code    = (rounded > 31U) ? 31U : (uint8_t)rounded;

    return act2861_update_register(ACT2861_REG_SAFETY_TIMER, ACT2861_ST_FC_TIMER_MASK, code);
}

act2861_error_t act2861_set_safety_timer_enabled(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_SAFETY_TIMER,
                           ACT2861_ST_DIS_SAFETY_TIMER,
                           (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_safety_timer_suspended(const uint8_t suspended)
{
    return act2861_set_bit(
        ACT2861_REG_SAFETY_TIMER, ACT2861_ST_SUSPEND_SAFETY_TIMER, suspended);
}

/* -------------------------------------------------------------------------
 * Thermal and JEITA
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_thermistor_enabled(const uint8_t enabled)
{
    return act2861_set_bit(
        ACT2861_REG_MAIN_CONTROL_2, ACT2861_MC2_DIS_TH, (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_jeita_enabled(const uint8_t enabled)
{
    return act2861_set_bit(
        ACT2861_REG_JEITA, ACT2861_JEITA_DIS_JEITA, (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_jeita_profile(const act2861_jeita_vseth_t warm_voltage,
                                          const uint8_t               warm_full_current,
                                          const act2861_jeita_isetc_t cool_current)
{
    if ((uint32_t)warm_voltage > 7U)
    {
        return ACT2861_ERR_BAD_JEITA_VSETH;
    }
    if ((uint32_t)cool_current > 3U)
    {
        return ACT2861_ERR_BAD_JEITA_ISETC;
    }

    const uint8_t mask = (uint8_t)(ACT2861_JEITA_VSETH_MASK | ACT2861_JEITA_ISETH |
                                   ACT2861_JEITA_ISETC_MASK);
    uint8_t       value =
        (uint8_t)(((uint8_t)warm_voltage << ACT2861_JEITA_VSETH_SHIFT) | (uint8_t)cool_current);
    if (warm_full_current != 0U)
    {
        value |= ACT2861_JEITA_ISETH;
    }

    return act2861_update_register(ACT2861_REG_JEITA, mask, value);
}

act2861_error_t act2861_set_otg_temperature_limits(const act2861_otg_hot_t hot,
                                                   const uint8_t           cold_minus_10c)
{
    if ((uint32_t)hot > 3U)
    {
        return ACT2861_ERR_BAD_OTG_HOT;
    }

    const uint8_t mask  = (uint8_t)(ACT2861_TEMP_OTG_HOT_MASK | ACT2861_TEMP_OTG_COLD);
    uint8_t       value = (uint8_t)((uint8_t)hot << ACT2861_TEMP_OTG_HOT_SHIFT);
    if (cold_minus_10c != 0U)
    {
        value |= ACT2861_TEMP_OTG_COLD;
    }

    return act2861_update_register(ACT2861_REG_TEMP_SETTING, mask, value);
}

act2861_error_t act2861_set_thermal_regulation(const act2861_treg_t threshold)
{
    return act2861_set_field(ACT2861_REG_TEMP_SETTING,
                             ACT2861_TEMP_TREG_MASK,
                             0U,
                             (uint32_t)threshold,
                             3U,
                             ACT2861_ERR_BAD_TREG);
}

/* -------------------------------------------------------------------------
 * OTG configuration
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_otg_voltage(const float volts)
{
    uint16_t code = 0U;

    act2861_error_t err = act2861_encode_voltage(volts,
                                                 ACT2861_OTG_VOUT_OFFSET_V,
                                                 ACT2861_OTG_VOUT_LSB_V,
                                                 ACT2861_OTG_VOUT_MAX_V,
                                                 1023U,
                                                 &code);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /* OTG_VOUT[9:7] sits in 0x13; OTG_VOUT[6:0] occupies 0x14 bits 7:1. */
    err = act2861_update_register(ACT2861_REG_OTG_VOLTAGE_1,
                                  ACT2861_OTGV1_VOUT_HI_MASK,
                                  (uint8_t)((code >> 7U) & ACT2861_OTGV1_VOUT_HI_MASK));
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_update_register(
        ACT2861_REG_OTG_VOLTAGE_2,
        ACT2861_OTGV2_VOUT_LO_MASK,
        (uint8_t)((uint8_t)(code & 0x7FU) << ACT2861_OTGV2_VOUT_LO_SHIFT));
}

act2861_error_t act2861_get_otg_voltage(float *volts)
{
    uint8_t raw[2] = {0U, 0U};

    if (volts == (float *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_registers(ACT2861_REG_OTG_VOLTAGE_1, raw, 2U);
    if (err != ACT2861_OK)
    {
        return err;
    }

    const uint16_t code =
        (uint16_t)((((uint16_t)(raw[0] & ACT2861_OTGV1_VOUT_HI_MASK)) << 7U) |
                   (uint16_t)((raw[1] & ACT2861_OTGV2_VOUT_LO_MASK) >> ACT2861_OTGV2_VOUT_LO_SHIFT));

    *volts = ACT2861_OTG_VOUT_OFFSET_V + ((float)code * ACT2861_OTG_VOUT_LSB_V);
    return ACT2861_OK;
}

act2861_error_t act2861_set_otg_external_feedback(const uint8_t external)
{
    return act2861_set_bit(ACT2861_REG_OTG_VOLTAGE_1, ACT2861_OTGV1_VOUT_I2C, external);
}

act2861_error_t act2861_set_otg_current_percent(const uint8_t percent)
{
    const act2861_error_t err =
        act2861_check_percent(percent, (uint8_t)ACT2861_PERCENT_MIN, (uint8_t)ACT2861_PERCENT_MAX);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_update_register(ACT2861_REG_OTG_CURR_LIMIT, ACT2861_OTG_CC_MASK, percent);
}

act2861_error_t act2861_set_otg_current_limit_enabled(const uint8_t enabled)
{
    return act2861_set_bit(
        ACT2861_REG_OTG_CURR_LIMIT, ACT2861_OTG_DIS_CC, (uint8_t)((enabled != 0U) ? 0U : 1U));
}

act2861_error_t act2861_set_otg_battery_current_limit(const act2861_otg_bat_ilim_t scaling)
{
    return act2861_set_field(ACT2861_REG_OTG_CONTROL_3,
                             ACT2861_OTG3_BAT_ILIM_MASK,
                             ACT2861_OTG3_BAT_ILIM_SHIFT,
                             (uint32_t)scaling,
                             3U,
                             ACT2861_ERR_BAD_OTG_BAT_ILIM);
}

act2861_error_t act2861_set_otg_battery_cutoff(const act2861_otg_cutoff_t cutoff)
{
    return act2861_set_field(ACT2861_REG_OTG_CONTROL_2,
                             ACT2861_OTG2_VBAT_CUTOFF_MASK,
                             ACT2861_OTG2_VBAT_CUTOFF_SHIFT,
                             (uint32_t)cutoff,
                             7U,
                             ACT2861_ERR_BAD_OTG_CUTOFF);
}

act2861_error_t act2861_set_otg_soft_start_slow(const uint8_t slow)
{
    return act2861_set_bit(ACT2861_REG_OTG_CONTROL_1, ACT2861_OTG1_SOFT_START, slow);
}

act2861_error_t act2861_set_otg_enable_delay(const act2861_otg_en_delay_t delay)
{
    return act2861_set_field(ACT2861_REG_OTG_CONTROL_2,
                             ACT2861_OTG2_EN_DLY_MASK,
                             0U,
                             (uint32_t)delay,
                             3U,
                             ACT2861_ERR_BAD_OTG_EN_DELAY);
}

act2861_error_t act2861_set_otg_light_load_delay(const act2861_otg_off_delay_t delay)
{
    if ((uint32_t)delay > 3U)
    {
        return ACT2861_ERR_BAD_OTG_OFF_DELAY;
    }

    /*
     * A disabled delay also means the light-load shutdown itself is off, so
     * OTG_OFF_LOAD_EN follows the selection.
     */
    const uint8_t mask  = (uint8_t)(ACT2861_OTG1_OFF_DLY_MASK | ACT2861_OTG1_OFF_LOAD_EN);
    uint8_t       value = (uint8_t)((uint8_t)delay << ACT2861_OTG1_OFF_DLY_SHIFT);
    if (delay != ACT2861_OTG_OFF_DELAY_DISABLED)
    {
        value |= ACT2861_OTG1_OFF_LOAD_EN;
    }

    return act2861_update_register(ACT2861_REG_OTG_CONTROL_1, mask, value);
}

act2861_error_t act2861_set_otg_slew_rate(const act2861_otg_slew_t slew)
{
    return act2861_set_field(ACT2861_REG_OTG_CONTROL_3,
                             ACT2861_OTG3_SLEW_MASK,
                             ACT2861_OTG3_SLEW_SHIFT,
                             (uint32_t)slew,
                             3U,
                             ACT2861_ERR_BAD_OTG_SLEW);
}

act2861_error_t act2861_set_otg_cord_compensation(const act2861_cord_comp_t compensation)
{
    return act2861_set_field(ACT2861_REG_OTG_CONTROL_2,
                             ACT2861_OTG2_CORD_COMP_MASK,
                             ACT2861_OTG2_CORD_COMP_SHIFT,
                             (uint32_t)compensation,
                             3U,
                             ACT2861_ERR_BAD_CORD_COMP);
}

act2861_error_t act2861_set_otg_nchg_enabled(const uint8_t enabled)
{
    return act2861_set_bit(ACT2861_REG_OTG_CONTROL_2, ACT2861_OTG2_EN_OTG_NCHG, enabled);
}

/* -------------------------------------------------------------------------
 * Interrupts
 * ------------------------------------------------------------------------- */

act2861_error_t act2861_set_irq_mask_1(const uint8_t mask)
{
    return act2861_write_register(ACT2861_REG_IRQ_CONTROL_1, mask);
}

act2861_error_t act2861_set_irq_mask_2(const uint8_t mask)
{
    return act2861_write_register(ACT2861_REG_IRQ_CONTROL_2, mask);
}

act2861_error_t act2861_set_irq_mask_i2c(const uint8_t masked)
{
    return act2861_set_bit(ACT2861_REG_OTG_STATUS, ACT2861_OTGS_NIRQ_I2C_ERROR, masked);
}

/* -------------------------------------------------------------------------
 * ADC
 * ------------------------------------------------------------------------- */

static act2861_error_t act2861_check_channel(const act2861_adc_channel_t channel)
{
    return ((uint32_t)channel > 7U) ? ACT2861_ERR_BAD_CHANNEL : ACT2861_OK;
}

/* Assemble ADC_OUT[13:2] from the two output registers. */
static act2861_error_t act2861_adc_fetch(uint16_t *code)
{
    uint8_t raw[2] = {0U, 0U};

    const act2861_error_t err = act2861_read_registers(ACT2861_REG_ADC_OUT_1, raw, 2U);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /* ADC_OUT[13:6] in 0x07, ADC_OUT[5:0] in the low six bits of 0x08. */
    const uint16_t full = (uint16_t)(((uint16_t)raw[0] << 6U) | (uint16_t)(raw[1] & 0x3FU));

    /* Every conversion equation is written in terms of ADC_OUT[13:2]. */
    *code = (uint16_t)(full >> 2U);
    return ACT2861_OK;
}

act2861_error_t act2861_adc_data_ready(uint8_t *ready)
{
    uint8_t raw = 0U;

    if (ready == (uint8_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    const act2861_error_t err = act2861_read_register(ACT2861_REG_ADC_CONFIG_2, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    *ready = ((raw & ACT2861_ADC2_DATA_READY) != 0U) ? 1U : 0U;
    return ACT2861_OK;
}

/* Point both the conversion and the read multiplexer at one channel. */
static act2861_error_t act2861_adc_select(const act2861_adc_channel_t channel)
{
    const uint8_t mask =
        (uint8_t)(ACT2861_ADC2_CH_READ_MASK | ACT2861_ADC2_CH_CONV_MASK);
    const uint8_t value = (uint8_t)(((uint8_t)channel << ACT2861_ADC2_CH_READ_SHIFT) |
                                    (uint8_t)channel);

    return act2861_update_register(ACT2861_REG_ADC_CONFIG_2, mask, value);
}

act2861_error_t act2861_adc_convert(const act2861_adc_channel_t channel,
                                    const uint32_t              timeout_ms,
                                    uint16_t                   *code)
{
    uint32_t waited = 0U;

    if (code == (uint16_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    act2861_error_t err = act2861_check_channel(channel);
    if (err != ACT2861_OK)
    {
        return err;
    }

    err = act2861_adc_select(channel);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /*
     * Single-shot per the datasheet: ADC_ONE_SHOT set, ADC_CH_SCAN clear and
     * the input buffer left enabled.  Writing EN_ADC starts the conversion,
     * and the device clears it again when the result is ready.
     */
    err = act2861_update_register(
        ACT2861_REG_ADC_CONFIG_1,
        (uint8_t)(ACT2861_ADC1_EN_ADC | ACT2861_ADC1_ONE_SHOT | ACT2861_ADC1_CH_SCAN |
                  ACT2861_ADC1_DIS_ADC_BUFFER),
        (uint8_t)(ACT2861_ADC1_EN_ADC | ACT2861_ADC1_ONE_SHOT));
    if (err != ACT2861_OK)
    {
        return err;
    }

    for (;;)
    {
        uint8_t ready = 0U;

        err = act2861_adc_data_ready(&ready);
        if (err != ACT2861_OK)
        {
            return err;
        }
        if (ready != 0U)
        {
            break;
        }
        if (waited >= timeout_ms)
        {
            return ACT2861_ERR_ADC_TIMEOUT;
        }

        act2861_delay_ms(ACT2861_POLL_INTERVAL_MS);
        waited += ACT2861_POLL_INTERVAL_MS;
    }

    /* Reading the result also deasserts nIRQ. */
    return act2861_adc_fetch(code);
}

act2861_error_t act2861_adc_start_continuous(void)
{
    return act2861_update_register(
        ACT2861_REG_ADC_CONFIG_1,
        (uint8_t)(ACT2861_ADC1_EN_ADC | ACT2861_ADC1_ONE_SHOT | ACT2861_ADC1_CH_SCAN |
                  ACT2861_ADC1_DIS_ADC_BUFFER),
        (uint8_t)(ACT2861_ADC1_EN_ADC | ACT2861_ADC1_CH_SCAN));
}

act2861_error_t act2861_adc_stop(void)
{
    return act2861_set_bit(ACT2861_REG_ADC_CONFIG_1, ACT2861_ADC1_EN_ADC, 0U);
}

act2861_error_t act2861_adc_read_channel(const act2861_adc_channel_t channel, uint16_t *code)
{
    if (code == (uint16_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }

    act2861_error_t err = act2861_check_channel(channel);
    if (err != ACT2861_OK)
    {
        return err;
    }

    err = act2861_update_register(ACT2861_REG_ADC_CONFIG_2,
                                  ACT2861_ADC2_CH_READ_MASK,
                                  (uint8_t)((uint8_t)channel << ACT2861_ADC2_CH_READ_SHIFT));
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_adc_fetch(code);
}

/* Resolve the klim scaling term for the battery-current channel (Table 15). */
static act2861_error_t act2861_resolve_klim(float *klim)
{
    act2861_mode_t mode = ACT2861_MODE_HIZ;

    act2861_error_t err = act2861_get_mode(&mode);
    if (err != ACT2861_OK)
    {
        return err;
    }

    if (mode != ACT2861_MODE_OTG)
    {
        *klim = 1.0F;
        return ACT2861_OK;
    }

    uint8_t raw = 0U;
    err         = act2861_read_register(ACT2861_REG_OTG_CONTROL_3, &raw);
    if (err != ACT2861_OK)
    {
        return err;
    }

    const uint8_t setting =
        (uint8_t)((raw & ACT2861_OTG3_BAT_ILIM_MASK) >> ACT2861_OTG3_BAT_ILIM_SHIFT);

    switch (setting)
    {
        case (uint8_t)ACT2861_OTG_BAT_ILIM_150PCT:
        case (uint8_t)ACT2861_OTG_BAT_ILIM_150PCT_B:
            *klim = 1.5F;
            break;
        case (uint8_t)ACT2861_OTG_BAT_ILIM_200PCT:
            *klim = 2.0F;
            break;
        default:
            /* 00 disables the battery-side limit; IBAT has no defined scale. */
            return ACT2861_ERR_KLIM_DISABLED;
    }
    return ACT2861_OK;
}

act2861_error_t act2861_adc_scale(const act2861_adc_channel_t channel,
                                  const uint16_t              code,
                                  const float                 klim,
                                  float                      *value)
{
    if (value == (float *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }
    if (s_initialised == 0U)
    {
        return ACT2861_ERR_NOT_INITIALISED;
    }

    const act2861_error_t err = act2861_check_channel(channel);
    if (err != ACT2861_OK)
    {
        return err;
    }

    /* Every channel except die temperature is offset-binary about mid-scale. */
    const float centred = (float)((int32_t)code - (int32_t)ACT2861_ADC_MIDSCALE);

    switch (channel)
    {
        case ACT2861_ADC_CH_INPUT_CURRENT:
            *value = ACT2861_ADC_K_CURRENT * centred / s_config.rcs_in_ohms / s_config.rilim_ohms;
            break;

        case ACT2861_ADC_CH_INPUT_VOLTAGE:
            *value = ACT2861_ADC_K_VIN * centred;
            break;

        case ACT2861_ADC_CH_BATTERY_VOLTAGE:
            *value = ACT2861_ADC_K_VBAT * centred;
            break;

        case ACT2861_ADC_CH_BATTERY_CURRENT:
        {
            float scale = klim;
            if (scale <= 0.0F)
            {
                const act2861_error_t klim_err = act2861_resolve_klim(&scale);
                if (klim_err != ACT2861_OK)
                {
                    return klim_err;
                }
            }
            *value = scale * ACT2861_ADC_K_CURRENT * centred / s_config.rcs_out_ohms /
                     s_config.rolim_ohms;
            break;
        }

        case ACT2861_ADC_CH_THERMISTOR:
            *value = ACT2861_ADC_K_VTH * centred;
            break;

        case ACT2861_ADC_CH_DIE_TEMPERATURE:
            /* The only channel expressed against the raw code, not mid-scale. */
            *value = (ACT2861_ADC_K_TJ_GAIN * (float)code) - ACT2861_ADC_K_TJ_OFFSET;
            break;

        case ACT2861_ADC_CH_EXTERNAL_INPUT:
            *value = ACT2861_ADC_K_VADC * centred;
            break;

        case ACT2861_ADC_CH_AGND:
        default:
            *value = ACT2861_ADC_K_VADC * centred;
            break;
    }
    return ACT2861_OK;
}

/* Convert one channel and scale it in a single call. */
static act2861_error_t act2861_read_scaled(const act2861_adc_channel_t channel, float *value)
{
    uint16_t code = 0U;

    if (value == (float *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }
    if (s_initialised == 0U)
    {
        return ACT2861_ERR_NOT_INITIALISED;
    }

    const act2861_error_t err =
        act2861_adc_convert(channel, (uint32_t)ACT2861_ADC_TIMEOUT_MS, &code);
    if (err != ACT2861_OK)
    {
        return err;
    }
    return act2861_adc_scale(channel, code, 0.0F, value);
}

act2861_error_t act2861_read_input_current(float *amps)
{
    return act2861_read_scaled(ACT2861_ADC_CH_INPUT_CURRENT, amps);
}

act2861_error_t act2861_read_input_voltage(float *volts)
{
    return act2861_read_scaled(ACT2861_ADC_CH_INPUT_VOLTAGE, volts);
}

act2861_error_t act2861_read_battery_voltage(float *volts)
{
    return act2861_read_scaled(ACT2861_ADC_CH_BATTERY_VOLTAGE, volts);
}

act2861_error_t act2861_read_battery_current(float *amps)
{
    return act2861_read_scaled(ACT2861_ADC_CH_BATTERY_CURRENT, amps);
}

act2861_error_t act2861_read_thermistor_voltage(float *volts)
{
    return act2861_read_scaled(ACT2861_ADC_CH_THERMISTOR, volts);
}

act2861_error_t act2861_read_die_temperature(float *celsius)
{
    return act2861_read_scaled(ACT2861_ADC_CH_DIE_TEMPERATURE, celsius);
}

act2861_error_t act2861_read_external_voltage(float *volts)
{
    return act2861_read_scaled(ACT2861_ADC_CH_EXTERNAL_INPUT, volts);
}

act2861_error_t act2861_read_measurements(act2861_measurements_t *out)
{
    if (out == (act2861_measurements_t *)0)
    {
        return ACT2861_ERR_NULL_PARAM;
    }
    if (s_initialised == 0U)
    {
        return ACT2861_ERR_NOT_INITIALISED;
    }

    act2861_error_t err = act2861_read_input_current(&out->input_current_a);
    if (err == ACT2861_OK)
    {
        err = act2861_read_input_voltage(&out->input_voltage_v);
    }
    if (err == ACT2861_OK)
    {
        err = act2861_read_battery_voltage(&out->battery_voltage_v);
    }
    if (err == ACT2861_OK)
    {
        err = act2861_read_battery_current(&out->battery_current_a);
    }
    if (err == ACT2861_OK)
    {
        err = act2861_read_thermistor_voltage(&out->thermistor_voltage_v);
    }
    if (err == ACT2861_OK)
    {
        err = act2861_read_die_temperature(&out->die_temperature_c);
    }
    if (err == ACT2861_OK)
    {
        err = act2861_read_external_voltage(&out->external_voltage_v);
    }
    return err;
}
