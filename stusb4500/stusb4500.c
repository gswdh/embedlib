#include "stusb4500.h"

/* -------------------------------------------------------------------------
 * Internal state.  One device per build, like the other drivers here: the
 * part is a port controller and boards carry one.
 * ------------------------------------------------------------------------- */

/* 7-bit address; 0 until stusb4500_init() succeeds. */
static uint8_t s_i2c_address = 0U;

/* Largest burst any internal helper needs: seven 32-bit data objects. */
#define STUSB4500_MAX_BURST (STUSB4500_SOURCE_PDO_MAX * 4U)

/* -------------------------------------------------------------------------
 * Weak platform hooks.  The defaults report a distinct error so a missing
 * port cannot masquerade as a dead bus.
 * ------------------------------------------------------------------------- */

stusb4500_error_t __attribute__((weak)) stusb4500_i2c_read(const uint8_t  dev_addr,
                                                           const uint8_t  reg,
                                                           uint8_t       *rx,
                                                           const uint16_t len)
{
    (void)dev_addr;
    (void)reg;
    (void)rx;
    (void)len;
    return STUSB4500_ERR_NO_PLATFORM_READ;
}

stusb4500_error_t __attribute__((weak)) stusb4500_i2c_write(const uint8_t  dev_addr,
                                                            const uint8_t  reg,
                                                            const uint8_t *tx,
                                                            const uint16_t len)
{
    (void)dev_addr;
    (void)reg;
    (void)tx;
    (void)len;
    return STUSB4500_ERR_NO_PLATFORM_WRITE;
}

void __attribute__((weak)) stusb4500_delay_ms(const uint32_t ms) { (void)ms; }

stusb4500_error_t __attribute__((weak)) stusb4500_set_reset_pin(const uint8_t asserted)
{
    (void)asserted;
    return STUSB4500_ERR_NO_PLATFORM_RESET;
}

/* -------------------------------------------------------------------------
 * Raw register access
 * ------------------------------------------------------------------------- */

/* Every accessor funnels through here, so an uninitialised driver reports
 * one clear error rather than transacting with address zero. */
static stusb4500_error_t stusb4500_address(uint8_t *address)
{
    if (s_i2c_address == 0U)
    {
        return STUSB4500_ERR_NOT_INITIALISED;
    }
    *address = s_i2c_address;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_i2c_address(uint8_t *address)
{
    if (address == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    return stusb4500_address(address);
}

stusb4500_error_t stusb4500_read_registers(const uint8_t reg, uint8_t *rx, const uint16_t len)
{
    uint8_t address = 0U;

    if (rx == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    if (len == 0U)
    {
        return STUSB4500_ERR_BAD_LENGTH;
    }

    const stusb4500_error_t err = stusb4500_address(&address);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    return stusb4500_i2c_read(address, reg, rx, len);
}

stusb4500_error_t
stusb4500_write_registers(const uint8_t reg, const uint8_t *tx, const uint16_t len)
{
    uint8_t address = 0U;

    if (tx == (const uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    if (len == 0U)
    {
        return STUSB4500_ERR_BAD_LENGTH;
    }

    const stusb4500_error_t err = stusb4500_address(&address);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    return stusb4500_i2c_write(address, reg, tx, len);
}

stusb4500_error_t stusb4500_read_register(const uint8_t reg, uint8_t *value)
{
    return stusb4500_read_registers(reg, value, 1U);
}

stusb4500_error_t stusb4500_write_register(const uint8_t reg, const uint8_t value)
{
    return stusb4500_write_registers(reg, &value, 1U);
}

stusb4500_error_t
stusb4500_update_register(const uint8_t reg, const uint8_t mask, const uint8_t value)
{
    uint8_t                 current = 0U;
    const stusb4500_error_t err     = stusb4500_read_register(reg, &current);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    const uint8_t updated = (uint8_t)((current & (uint8_t)(~mask)) | (value & mask));
    if (updated == current)
    {
        return STUSB4500_OK;
    }
    return stusb4500_write_register(reg, updated);
}

stusb4500_error_t stusb4500_read_word(const uint8_t reg, uint16_t *value)
{
    uint8_t rx[2] = {0U, 0U};

    if (value == (uint16_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_read_registers(reg, rx, 2U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    /* Multi-byte quantities travel least-significant byte first. */
    *value = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8U));
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_write_word(const uint8_t reg, const uint16_t value)
{
    const uint8_t tx[2] = {(uint8_t)(value & 0xFFU), (uint8_t)(value >> 8U)};
    return stusb4500_write_registers(reg, tx, 2U);
}

stusb4500_error_t stusb4500_read_dword(const uint8_t reg, uint32_t *value)
{
    uint8_t rx[4] = {0U, 0U, 0U, 0U};

    if (value == (uint32_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_read_registers(reg, rx, 4U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    *value = (uint32_t)rx[0] | ((uint32_t)rx[1] << 8U) | ((uint32_t)rx[2] << 16U) |
             ((uint32_t)rx[3] << 24U);
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_write_dword(const uint8_t reg, const uint32_t value)
{
    const uint8_t tx[4] = {(uint8_t)(value & 0xFFU),
                           (uint8_t)((value >> 8U) & 0xFFU),
                           (uint8_t)((value >> 16U) & 0xFFU),
                           (uint8_t)((value >> 24U) & 0xFFU)};
    return stusb4500_write_registers(reg, tx, 4U);
}

/* -------------------------------------------------------------------------
 * Initialisation and identity
 * ------------------------------------------------------------------------- */

stusb4500_error_t stusb4500_get_device_id(uint8_t *device_id)
{
    if (device_id == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    return stusb4500_read_register(STUSB4500_REG_DEVICE_ID, device_id);
}

stusb4500_error_t stusb4500_init(const stusb4500_config_t *cfg)
{
    uint8_t address   = STUSB4500_I2C_ADDR_DEFAULT;
    uint8_t device_id = 0U;

    if (cfg != (const stusb4500_config_t *)0)
    {
        if (cfg->i2c_address != 0U)
        {
            if ((cfg->i2c_address < STUSB4500_I2C_ADDR_MIN) ||
                (cfg->i2c_address > STUSB4500_I2C_ADDR_MAX))
            {
                return STUSB4500_ERR_BAD_ADDRESS;
            }
            address = cfg->i2c_address;
        }
    }

    /* Published before the identity read, because that read goes through the
     * same accessors.  Cleared again if the device is not there, so a failed
     * init leaves nothing usable behind. */
    s_i2c_address = address;

    const stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_DEVICE_ID, &device_id);
    if (err != STUSB4500_OK)
    {
        s_i2c_address = 0U;
        return err;
    }
    if ((device_id != STUSB4500_DEVICE_ID_A) && (device_id != STUSB4500_DEVICE_ID_B))
    {
        s_i2c_address = 0U;
        return STUSB4500_ERR_DEVICE_ID;
    }

    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_revisions(uint16_t *typec_bcd, uint16_t *usbpd_bcd)
{
    uint8_t rx[4] = {0U, 0U, 0U, 0U};

    /* 06h..09h are contiguous: Type-C low/high then USB PD low/high. */
    const stusb4500_error_t err =
        stusb4500_read_registers(STUSB4500_REG_BCD_TYPEC_REV_LOW, rx, 4U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    if (typec_bcd != (uint16_t *)0)
    {
        *typec_bcd = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8U));
    }
    if (usbpd_bcd != (uint16_t *)0)
    {
        *usbpd_bcd = (uint16_t)((uint16_t)rx[2] | ((uint16_t)rx[3] << 8U));
    }
    return STUSB4500_OK;
}

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

/* Every decoded getter shares this shape: read one byte, then unpack. */
static stusb4500_error_t stusb4500_read_status(const uint8_t reg, const void *out, uint8_t *raw)
{
    if (out == (const void *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    return stusb4500_read_register(reg, raw);
}

stusb4500_error_t stusb4500_get_alerts(stusb4500_alert_t *alerts)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err =
        stusb4500_read_status(STUSB4500_REG_ALERT_STATUS_1, alerts, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    alerts->raw               = raw;
    alerts->phy_status        = ((raw & STUSB4500_ALERT_PHY_STATUS) != 0U) ? 1U : 0U;
    alerts->prt_status        = ((raw & STUSB4500_ALERT_PRT_STATUS) != 0U) ? 1U : 0U;
    alerts->pd_typec_status   = ((raw & STUSB4500_ALERT_PD_TYPEC_STATUS) != 0U) ? 1U : 0U;
    alerts->hw_fault_status   = ((raw & STUSB4500_ALERT_HW_FAULT_STATUS) != 0U) ? 1U : 0U;
    alerts->monitoring_status = ((raw & STUSB4500_ALERT_MONITORING_STATUS) != 0U) ? 1U : 0U;
    alerts->cc_detection      = ((raw & STUSB4500_ALERT_CC_DETECTION) != 0U) ? 1U : 0U;
    alerts->hard_reset        = ((raw & STUSB4500_ALERT_HARD_RESET) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_alert_mask(uint8_t *mask)
{
    if (mask == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    return stusb4500_read_register(STUSB4500_REG_ALERT_STATUS_1_MASK, mask);
}

stusb4500_error_t stusb4500_set_alert_mask(const uint8_t mask)
{
    return stusb4500_write_register(STUSB4500_REG_ALERT_STATUS_1_MASK, mask);
}

stusb4500_error_t stusb4500_get_port_status(stusb4500_port_status_t *status)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err = stusb4500_read_status(STUSB4500_REG_PORT_STATUS_1, status, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    status->raw                = raw;
    status->attached           = ((raw & STUSB4500_PORT_ATTACHED) != 0U) ? 1U : 0U;
    status->vconn_supplied     = ((raw & STUSB4500_PORT_VCONN_SUPPLIED) != 0U) ? 1U : 0U;
    status->data_role_dfp      = ((raw & STUSB4500_PORT_DATA_ROLE_DFP) != 0U) ? 1U : 0U;
    status->power_role_source  = ((raw & STUSB4500_PORT_POWER_ROLE_SOURCE) != 0U) ? 1U : 0U;
    status->startup_power_mode = ((raw & STUSB4500_PORT_STARTUP_POWER) != 0U) ? 1U : 0U;
    status->attach_mode =
        (uint8_t)((raw & STUSB4500_PORT_ATTACH_MODE_MASK) >> STUSB4500_PORT_ATTACH_MODE_SHIFT);
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_monitoring_status(stusb4500_monitoring_t *status)
{
    uint8_t rx[2] = {0U, 0U};

    if (status == (stusb4500_monitoring_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    /* 0Fh and 10h are adjacent, and 0Fh carries the two live comparator
     * outputs as well as the transition flags, so both are wanted. */
    const stusb4500_error_t err =
        stusb4500_read_registers(STUSB4500_REG_MONITORING_STATUS_0, rx, 2U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    status->raw_transitions = rx[0];
    status->raw             = rx[1];
    status->vbus_low        = ((rx[0] & STUSB4500_MONITOR_VBUS_LOW) != 0U) ? 1U : 0U;
    status->vbus_high       = ((rx[0] & STUSB4500_MONITOR_VBUS_HIGH) != 0U) ? 1U : 0U;
    status->vconn_valid     = ((rx[1] & STUSB4500_MONITOR_VCONN_VALID) != 0U) ? 1U : 0U;
    status->vbus_valid_snk  = ((rx[1] & STUSB4500_MONITOR_VBUS_VALID_SNK) != 0U) ? 1U : 0U;
    status->vbus_vsafe0v    = ((rx[1] & STUSB4500_MONITOR_VBUS_VSAFE0V) != 0U) ? 1U : 0U;
    status->vbus_ready      = ((rx[1] & STUSB4500_MONITOR_VBUS_READY) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_cc_status(stusb4500_cc_status_t *status)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err = stusb4500_read_status(STUSB4500_REG_CC_STATUS, status, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    status->raw       = raw;
    status->cc1_state = (uint8_t)((raw & STUSB4500_CC_CC1_STATE_MASK) >>
                                 STUSB4500_CC_CC1_STATE_SHIFT);
    status->cc2_state = (uint8_t)((raw & STUSB4500_CC_CC2_STATE_MASK) >>
                                 STUSB4500_CC_CC2_STATE_SHIFT);
    status->presenting_rd = ((raw & STUSB4500_CC_CONNECT_RESULT) != 0U) ? 1U : 0U;
    status->looking_for_connection =
        ((raw & STUSB4500_CC_LOOKING_4_CONNECTION) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_hw_faults(stusb4500_hw_fault_t *faults)
{
    uint8_t rx[2] = {0U, 0U};

    if (faults == (stusb4500_hw_fault_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    /* 12h holds transitions plus the live thermal flag; 13h holds the live
     * fault states. */
    const stusb4500_error_t err = stusb4500_read_registers(STUSB4500_REG_HW_FAULT_STATUS_0, rx, 2U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    faults->raw_transitions = rx[0];
    faults->raw             = rx[1];
    faults->thermal         = ((rx[0] & STUSB4500_FAULT_THERMAL) != 0U) ? 1U : 0U;
    faults->vconn_sw_ovp    = ((rx[1] & STUSB4500_FAULT_VCONN_SW_OVP) != 0U) ? 1U : 0U;
    faults->vconn_sw_ocp    = ((rx[1] & STUSB4500_FAULT_VCONN_SW_OCP) != 0U) ? 1U : 0U;
    faults->vconn_sw_rvp    = ((rx[1] & STUSB4500_FAULT_VCONN_SW_RVP) != 0U) ? 1U : 0U;
    faults->vsrc_discharge  = ((rx[1] & STUSB4500_FAULT_VSRC_DISCH) != 0U) ? 1U : 0U;
    faults->vbus_discharge  = ((rx[1] & STUSB4500_FAULT_VBUS_DISCH) != 0U) ? 1U : 0U;
    faults->vpu_presence    = ((rx[1] & STUSB4500_FAULT_VPU_PRESENCE) != 0U) ? 1U : 0U;
    faults->vpu_ovp         = ((rx[1] & STUSB4500_FAULT_VPU_OVP) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_prt_status(stusb4500_prt_status_t *status)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err = stusb4500_read_status(STUSB4500_REG_PRT_STATUS, status, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    status->raw                 = raw;
    status->hard_reset_received = ((raw & STUSB4500_PRT_HWRESET_RECEIVED) != 0U) ? 1U : 0U;
    status->hard_reset_done     = ((raw & STUSB4500_PRT_HWRESET_DONE) != 0U) ? 1U : 0U;
    status->message_received    = ((raw & STUSB4500_PRT_MSG_RECEIVED) != 0U) ? 1U : 0U;
    status->message_sent        = ((raw & STUSB4500_PRT_MSG_SENT) != 0U) ? 1U : 0U;
    status->bist_received       = ((raw & STUSB4500_PRT_BIST_RECEIVED) != 0U) ? 1U : 0U;
    status->bist_sent           = ((raw & STUSB4500_PRT_BIST_SENT) != 0U) ? 1U : 0U;
    status->tx_error            = ((raw & STUSB4500_PRT_TX_ERROR) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_phy_status(stusb4500_phy_status_t *status)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err = stusb4500_read_status(STUSB4500_REG_PHY_STATUS, status, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    status->raw                    = raw;
    status->tx_message_failed      = ((raw & STUSB4500_PHY_TX_MSG_FAIL) != 0U) ? 1U : 0U;
    status->tx_message_discarded   = ((raw & STUSB4500_PHY_TX_MSG_DISC) != 0U) ? 1U : 0U;
    status->tx_message_succeeded   = ((raw & STUSB4500_PHY_TX_MSG_SUCCESS) != 0U) ? 1U : 0U;
    status->idle                   = ((raw & STUSB4500_PHY_IDLE) != 0U) ? 1U : 0U;
    status->sop_rx_type =
        (uint8_t)((raw & STUSB4500_PHY_SOP_RX_TYPE_MASK) >> STUSB4500_PHY_SOP_RX_TYPE_SHIFT);
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_typec_status(stusb4500_typec_status_t *status)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err = stusb4500_read_status(STUSB4500_REG_TYPEC_STATUS, status, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    status->raw          = raw;
    status->fsm_state    = (uint8_t)(raw & STUSB4500_TYPEC_FSM_STATE_MASK);
    status->snk_tx_rp    = ((raw & STUSB4500_TYPEC_PD_SNK_TX_RP) != 0U) ? 1U : 0U;
    status->src_tx_rp    = ((raw & STUSB4500_TYPEC_PD_SRC_TX_RP) != 0U) ? 1U : 0U;
    status->cc2_attached = ((raw & STUSB4500_TYPEC_REVERSE) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_pe_state(uint8_t *state)
{
    if (state == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    return stusb4500_read_register(STUSB4500_REG_PE_FSM, state);
}

stusb4500_error_t stusb4500_is_attached(uint8_t *attached)
{
    uint8_t raw = 0U;

    if (attached == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_PORT_STATUS_1, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    *attached = ((raw & STUSB4500_PORT_ATTACHED) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_is_vbus_ready(uint8_t *ready)
{
    uint8_t raw = 0U;

    if (ready == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_MONITORING_STATUS_1, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    *ready = ((raw & STUSB4500_MONITOR_VBUS_READY) != 0U) ? 1U : 0U;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_wait_sink_ready(const uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    for (;;)
    {
        uint8_t                 state = 0U;
        const stusb4500_error_t err   = stusb4500_read_register(STUSB4500_REG_PE_FSM, &state);
        if (err != STUSB4500_OK)
        {
            return err;
        }
        if (state == STUSB4500_PE_SNK_READY)
        {
            return STUSB4500_OK;
        }
        if (waited >= timeout_ms)
        {
            return STUSB4500_ERR_NOT_READY;
        }

        stusb4500_delay_ms(STUSB4500_T_POLL_INTERVAL_MS);
        waited += STUSB4500_T_POLL_INTERVAL_MS;
    }
}

/* -------------------------------------------------------------------------
 * PDO / RDO encoding
 * ------------------------------------------------------------------------- */

void stusb4500_decode_pdo(const uint32_t raw, stusb4500_pdo_t *pdo)
{
    if (pdo == (stusb4500_pdo_t *)0)
    {
        return;
    }

    pdo->raw  = raw;
    pdo->type = (uint8_t)((raw & STUSB4500_PDO_TYPE_MASK) >> STUSB4500_PDO_TYPE_SHIFT);

    /* Only a fixed supply has a single voltage and current in these fields;
     * variable and battery objects reuse the bits for a range, so leaving
     * them at zero is the honest answer rather than a misread. */
    if (pdo->type == STUSB4500_PDO_TYPE_FIXED)
    {
        pdo->voltage_mv = (uint16_t)(((raw & STUSB4500_PDO_VOLTAGE_MASK) >>
                                      STUSB4500_PDO_VOLTAGE_SHIFT) *
                                     STUSB4500_PDO_VOLTAGE_LSB_MV);
        pdo->current_ma = (uint16_t)(((raw & STUSB4500_PDO_CURRENT_MASK) >>
                                      STUSB4500_PDO_CURRENT_SHIFT) *
                                     STUSB4500_PDO_CURRENT_LSB_MA);
    }
    else
    {
        pdo->voltage_mv = 0U;
        pdo->current_ma = 0U;
    }

    pdo->dual_role_power     = ((raw & STUSB4500_PDO_DUAL_ROLE_POWER) != 0U) ? 1U : 0U;
    pdo->higher_capability   = ((raw & STUSB4500_PDO_HIGHER_CAPABILITY) != 0U) ? 1U : 0U;
    pdo->unconstrained_power = ((raw & STUSB4500_PDO_UNCONSTRAINED_POWER) != 0U) ? 1U : 0U;
    pdo->usb_comm_capable    = ((raw & STUSB4500_PDO_USB_COMM_CAPABLE) != 0U) ? 1U : 0U;
    pdo->dual_role_data      = ((raw & STUSB4500_PDO_DUAL_ROLE_DATA) != 0U) ? 1U : 0U;
}

void stusb4500_decode_rdo(const uint32_t raw, stusb4500_rdo_t *rdo)
{
    if (rdo == (stusb4500_rdo_t *)0)
    {
        return;
    }

    rdo->raw = raw;
    rdo->object_position =
        (uint8_t)((raw & STUSB4500_RDO_OBJECT_POS_MASK) >> STUSB4500_RDO_OBJECT_POS_SHIFT);
    rdo->operating_current_ma =
        (uint16_t)(((raw & STUSB4500_RDO_OP_CURRENT_MASK) >> STUSB4500_RDO_OP_CURRENT_SHIFT) *
                   STUSB4500_PDO_CURRENT_LSB_MA);
    rdo->max_current_ma =
        (uint16_t)(((raw & STUSB4500_RDO_MAX_CURRENT_MASK) >> STUSB4500_RDO_MAX_CURRENT_SHIFT) *
                   STUSB4500_PDO_CURRENT_LSB_MA);
    rdo->capability_mismatch = ((raw & STUSB4500_RDO_CAPABILITY_MISMATCH) != 0U) ? 1U : 0U;
    rdo->give_back           = ((raw & STUSB4500_RDO_GIVE_BACK) != 0U) ? 1U : 0U;
    rdo->usb_comm_capable    = ((raw & STUSB4500_RDO_USB_COMM_CAPABLE) != 0U) ? 1U : 0U;
    rdo->no_usb_suspend      = ((raw & STUSB4500_RDO_NO_USB_SUSPEND) != 0U) ? 1U : 0U;
    rdo->unchunked_supported = ((raw & STUSB4500_RDO_UNCHUNKED_SUPPORT) != 0U) ? 1U : 0U;
}

stusb4500_error_t stusb4500_encode_fixed_pdo(const uint16_t voltage_mv,
                                             const uint16_t current_ma,
                                             uint32_t      *raw)
{
    if (raw == (uint32_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    /* Both fields are 10 bits with a fixed step.  A request that is not a
     * whole number of steps is refused rather than silently rounded: a sink
     * that quietly advertises a different voltage than it asked for is worse
     * than one that reports the mistake. */
    if (((voltage_mv % STUSB4500_PDO_VOLTAGE_LSB_MV) != 0U) ||
        ((current_ma % STUSB4500_PDO_CURRENT_LSB_MA) != 0U))
    {
        return STUSB4500_ERR_BAD_PARAM;
    }

    const uint32_t voltage_steps = (uint32_t)voltage_mv / STUSB4500_PDO_VOLTAGE_LSB_MV;
    const uint32_t current_steps = (uint32_t)current_ma / STUSB4500_PDO_CURRENT_LSB_MA;

    if ((voltage_steps > 0x3FFUL) || (current_steps > 0x3FFUL))
    {
        return STUSB4500_ERR_BAD_PARAM;
    }

    *raw = ((voltage_steps << STUSB4500_PDO_VOLTAGE_SHIFT) & STUSB4500_PDO_VOLTAGE_MASK) |
           ((current_steps << STUSB4500_PDO_CURRENT_SHIFT) & STUSB4500_PDO_CURRENT_MASK);
    return STUSB4500_OK;
}

/* -------------------------------------------------------------------------
 * Sink power data objects
 * ------------------------------------------------------------------------- */

static stusb4500_error_t stusb4500_pdo_register(const uint8_t index, uint8_t *reg)
{
    if ((index < 1U) || (index > STUSB4500_SINK_PDO_COUNT))
    {
        return STUSB4500_ERR_BAD_PDO_INDEX;
    }
    *reg = (uint8_t)(STUSB4500_REG_DPM_SNK_PDO1 + (4U * (uint8_t)(index - 1U)));
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_get_pdo_count(uint8_t *count)
{
    uint8_t raw = 0U;

    if (count == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_DPM_PDO_NUMB, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    *count = (uint8_t)(raw & 0x07U);
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_set_pdo_count(const uint8_t count)
{
    if ((count < 1U) || (count > STUSB4500_SINK_PDO_COUNT))
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    return stusb4500_update_register(STUSB4500_REG_DPM_PDO_NUMB, 0x07U, count);
}

stusb4500_error_t stusb4500_get_sink_pdo(const uint8_t index, stusb4500_pdo_t *pdo)
{
    uint8_t  reg = 0U;
    uint32_t raw = 0U;

    if (pdo == (stusb4500_pdo_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    stusb4500_error_t err = stusb4500_pdo_register(index, &reg);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_read_dword(reg, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    stusb4500_decode_pdo(raw, pdo);
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_set_sink_pdo_raw(const uint8_t index, const uint32_t pdo)
{
    uint8_t                 reg = 0U;
    const stusb4500_error_t err = stusb4500_pdo_register(index, &reg);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    return stusb4500_write_dword(reg, pdo);
}

stusb4500_error_t stusb4500_set_sink_pdo(const uint8_t  index,
                                         const uint16_t voltage_mv,
                                         const uint16_t current_ma)
{
    uint8_t  reg      = 0U;
    uint32_t current  = 0U;
    uint32_t encoded  = 0U;

    stusb4500_error_t err = stusb4500_pdo_register(index, &reg);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    /* PDO1 is the mandatory vSafe5V object; the spec does not allow it to
     * advertise anything else, and a source will reject a contract built on
     * a moved PDO1. */
    if ((index == 1U) && (voltage_mv != STUSB4500_PDO1_VOLTAGE_MV))
    {
        return STUSB4500_ERR_BAD_PARAM;
    }

    err = stusb4500_encode_fixed_pdo(voltage_mv, current_ma, &encoded);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_read_dword(reg, &current);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    /* Keep every bit outside the voltage and current fields: the flags say
     * whether the sink does USB data and whether it is externally powered,
     * and they came from the NVM for a reason. */
    const uint32_t preserved =
        current & ~(STUSB4500_PDO_VOLTAGE_MASK | STUSB4500_PDO_CURRENT_MASK);

    return stusb4500_write_dword(reg, preserved | encoded);
}

stusb4500_error_t stusb4500_get_rdo(stusb4500_rdo_t *rdo)
{
    uint32_t raw = 0U;

    if (rdo == (stusb4500_rdo_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_read_dword(STUSB4500_REG_RDO_REG_STATUS, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    stusb4500_decode_rdo(raw, rdo);
    return STUSB4500_OK;
}

/* -------------------------------------------------------------------------
 * PD messaging and reset
 * ------------------------------------------------------------------------- */

stusb4500_error_t stusb4500_send_pd_message(const uint8_t message_type)
{
    const uint16_t header =
        (uint16_t)(((uint16_t)message_type << STUSB4500_HEADER_MSG_TYPE_SHIFT) &
                   STUSB4500_HEADER_MSG_TYPE_MASK);

    /* The device supplies the message ID, roles and revision; only the type
     * has to be staged before the send command. */
    const stusb4500_error_t err = stusb4500_write_word(STUSB4500_REG_TX_HEADER, header);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    return stusb4500_write_register(STUSB4500_REG_PD_COMMAND_CTRL,
                                    STUSB4500_PD_CMD_SEND_MESSAGE);
}

stusb4500_error_t stusb4500_send_pd_soft_reset(void)
{
    return stusb4500_send_pd_message(STUSB4500_CTRL_MSG_SOFT_RESET);
}

/* Poll DEVICE_ID until the part answers with a value we recognise. */
static stusb4500_error_t stusb4500_wait_alive(const uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    for (;;)
    {
        uint8_t device_id = 0U;

        if (stusb4500_read_register(STUSB4500_REG_DEVICE_ID, &device_id) == STUSB4500_OK)
        {
            if ((device_id == STUSB4500_DEVICE_ID_A) || (device_id == STUSB4500_DEVICE_ID_B))
            {
                return STUSB4500_OK;
            }
        }
        if (waited >= timeout_ms)
        {
            return STUSB4500_ERR_RESET_TIMEOUT;
        }

        stusb4500_delay_ms(STUSB4500_T_POLL_INTERVAL_MS);
        waited += STUSB4500_T_POLL_INTERVAL_MS;
    }
}

stusb4500_error_t stusb4500_software_reset(void)
{
    /* SW_RESET_EN holds the device in reset for as long as it is set, so it
     * has to be released again explicitly. */
    stusb4500_error_t err =
        stusb4500_write_register(STUSB4500_REG_RESET_CTRL, STUSB4500_RESET_SW_EN);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    stusb4500_delay_ms(STUSB4500_T_RESET_MS);

    err = stusb4500_write_register(STUSB4500_REG_RESET_CTRL, 0x00U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_wait_alive(STUSB4500_T_STARTUP_TIMEOUT_MS);
}

stusb4500_error_t stusb4500_hardware_reset(void)
{
    stusb4500_error_t err = stusb4500_set_reset_pin(1U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    stusb4500_delay_ms(STUSB4500_T_RESET_MS);

    err = stusb4500_set_reset_pin(0U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_wait_alive(STUSB4500_T_STARTUP_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------
 * Source capabilities and negotiation
 * ------------------------------------------------------------------------- */

stusb4500_error_t stusb4500_read_source_capabilities(stusb4500_pdo_t *pdos,
                                                     const uint8_t    max_pdos,
                                                     uint8_t         *count,
                                                     const uint32_t   timeout_ms)
{
    uint8_t  buffer[STUSB4500_MAX_BURST] = {0U};
    uint8_t  alerts                      = 0U;
    uint16_t header                      = 0U;
    uint8_t  objects                     = 0U;
    uint8_t  byte_count                  = 0U;
    uint32_t waited                      = 0U;
    uint8_t  index                       = 0U;

    if ((pdos == (stusb4500_pdo_t *)0) || (count == (uint8_t *)0))
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    if ((max_pdos == 0U) || (max_pdos > STUSB4500_SOURCE_PDO_MAX))
    {
        return STUSB4500_ERR_BAD_LENGTH;
    }

    *count = 0U;

    /* ALERT_STATUS_1 is read-clear, so draining it here means the wait below
     * observes a message that arrived after this call started rather than one
     * left over from the previous attach. */
    (void)stusb4500_read_register(STUSB4500_REG_ALERT_STATUS_1, &alerts);

    for (;;)
    {
        uint8_t prt = 0U;

        stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_PRT_STATUS, &prt);
        if (err != STUSB4500_OK)
        {
            return err;
        }

        if ((prt & STUSB4500_PRT_MSG_RECEIVED) != 0U)
        {
            err = stusb4500_read_word(STUSB4500_REG_RX_HEADER, &header);
            if (err != STUSB4500_OK)
            {
                return err;
            }

            objects = (uint8_t)((header & STUSB4500_HEADER_NUM_DATA_OBJ_MASK) >>
                                STUSB4500_HEADER_NUM_DATA_OBJ_SHIFT);

            /* A data message with type 1 is Source_Capabilities; a control
             * message with type 1 is GoodCRC, which is why the object count
             * has to be part of the test. */
            if ((objects != 0U) && ((header & STUSB4500_HEADER_MSG_TYPE_MASK) ==
                                    STUSB4500_DATA_MSG_SOURCE_CAP))
            {
                break;
            }
        }

        if (waited >= timeout_ms)
        {
            return STUSB4500_ERR_NO_MESSAGE;
        }
        stusb4500_delay_ms(STUSB4500_T_POLL_INTERVAL_MS);
        waited += STUSB4500_T_POLL_INTERVAL_MS;
    }

    if (objects > STUSB4500_SOURCE_PDO_MAX)
    {
        return STUSB4500_ERR_MESSAGE_TRUNCATED;
    }

    stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_RX_BYTE_CNT, &byte_count);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    if (byte_count != (uint8_t)(objects * 4U))
    {
        return STUSB4500_ERR_MESSAGE_TRUNCATED;
    }

    /* One burst, deliberately: the source's next message overwrites part of
     * the receive buffer, so the objects have to come out before it lands. */
    err = stusb4500_read_registers(STUSB4500_REG_RX_DATA_OBJ, buffer, (uint16_t)(objects * 4U));
    if (err != STUSB4500_OK)
    {
        return err;
    }

    if (objects > max_pdos)
    {
        objects = max_pdos;
    }

    for (index = 0U; index < objects; index++)
    {
        const uint8_t *p   = &buffer[index * 4U];
        const uint32_t raw = (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) |
                             ((uint32_t)p[3] << 24U);
        stusb4500_decode_pdo(raw, &pdos[index]);
    }

    *count = objects;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_negotiate(const stusb4500_request_t *request,
                                      stusb4500_pdo_t           *selected)
{
    stusb4500_pdo_t offers[STUSB4500_SOURCE_PDO_MAX];
    stusb4500_pdo_t best     = {0};
    uint32_t        best_mw  = 0U;
    uint8_t         count    = 0U;
    uint8_t         index    = 0U;
    uint8_t         attached = 0U;
    uint8_t         found    = 0U;

    if (request == (const stusb4500_request_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    if (request->min_voltage_mv > request->max_voltage_mv)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }

    stusb4500_error_t err = stusb4500_is_attached(&attached);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    if (attached == 0U)
    {
        return STUSB4500_ERR_NOT_ATTACHED;
    }

    /* Nothing may be staged while the policy engine is mid-transaction, and
     * the soft reset is what makes the source re-advertise. */
    err = stusb4500_wait_sink_ready(STUSB4500_T_NEGOTIATION_TIMEOUT_MS);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    err = stusb4500_send_pd_soft_reset();
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_read_source_capabilities(
        offers, STUSB4500_SOURCE_PDO_MAX, &count, STUSB4500_T_NEGOTIATION_TIMEOUT_MS);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    for (index = 0U; index < count; index++)
    {
        const stusb4500_pdo_t *pdo = &offers[index];

        if (pdo->type != STUSB4500_PDO_TYPE_FIXED)
        {
            continue;
        }
        if ((pdo->voltage_mv < request->min_voltage_mv) ||
            (pdo->voltage_mv > request->max_voltage_mv) ||
            (pdo->current_ma < request->min_current_ma))
        {
            continue;
        }

        /* Milliwatts fits comfortably in 32 bits at 51 V and 10 A. */
        const uint32_t mw =
            ((uint32_t)pdo->voltage_mv * (uint32_t)pdo->current_ma) / 1000UL;
        if ((found == 0U) || (mw > best_mw))
        {
            best    = *pdo;
            best_mw = mw;
            found   = 1U;
        }
    }

    if (found == 0U)
    {
        return STUSB4500_ERR_NO_SUITABLE_PDO;
    }

    err = stusb4500_wait_sink_ready(STUSB4500_T_NEGOTIATION_TIMEOUT_MS);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    /* PDO3 is the highest-priority object the device offers, so the chosen
     * contract goes there and PDO1's 5 V fallback stays intact. */
    err = stusb4500_set_sink_pdo(3U, best.voltage_mv, best.current_ma);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    err = stusb4500_set_pdo_count(STUSB4500_SINK_PDO_COUNT);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    if (selected != (stusb4500_pdo_t *)0)
    {
        *selected = best;
    }

    return stusb4500_send_pd_soft_reset();
}

/* -------------------------------------------------------------------------
 * VBUS monitoring, discharge and GPIO3
 * ------------------------------------------------------------------------- */

stusb4500_error_t stusb4500_set_voltage_window(const uint8_t under_pct, const uint8_t over_pct)
{
    if ((under_pct > 0x0FU) || (over_pct > 0x0FU))
    {
        return STUSB4500_ERR_BAD_PARAM;
    }

    const uint8_t value =
        (uint8_t)(((uint8_t)(under_pct << STUSB4500_MONITORING_VSHIFT_LOW_SHIFT) &
                   STUSB4500_MONITORING_VSHIFT_LOW_MASK) |
                  ((uint8_t)(over_pct << STUSB4500_MONITORING_VSHIFT_HIGH_SHIFT) &
                   STUSB4500_MONITORING_VSHIFT_HIGH_MASK));
    return stusb4500_write_register(STUSB4500_REG_MONITORING_CTRL_2, value);
}

stusb4500_error_t stusb4500_get_voltage_window(uint8_t *under_pct, uint8_t *over_pct)
{
    uint8_t raw = 0U;

    const stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_MONITORING_CTRL_2, &raw);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    if (under_pct != (uint8_t *)0)
    {
        *under_pct = (uint8_t)((raw & STUSB4500_MONITORING_VSHIFT_LOW_MASK) >>
                               STUSB4500_MONITORING_VSHIFT_LOW_SHIFT);
    }
    if (over_pct != (uint8_t *)0)
    {
        *over_pct = (uint8_t)((raw & STUSB4500_MONITORING_VSHIFT_HIGH_MASK) >>
                              STUSB4500_MONITORING_VSHIFT_HIGH_SHIFT);
    }
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_set_vbus_discharge(const uint8_t enabled)
{
    return stusb4500_update_register(STUSB4500_REG_VBUS_DISCHARGE_CTRL,
                                     STUSB4500_VBUS_DISCHARGE_EN,
                                     (enabled != 0U) ? STUSB4500_VBUS_DISCHARGE_EN : 0U);
}

stusb4500_error_t stusb4500_set_vbus_sink_enabled(const uint8_t enabled)
{
    return stusb4500_update_register(STUSB4500_REG_VBUS_CTRL,
                                     STUSB4500_VBUS_SINK_EN,
                                     (enabled != 0U) ? STUSB4500_VBUS_SINK_EN : 0U);
}

stusb4500_error_t stusb4500_set_gpio3(const uint8_t low)
{
    return stusb4500_update_register(STUSB4500_REG_GPIO3_SW,
                                     STUSB4500_GPIO3_SW_EN,
                                     (low != 0U) ? STUSB4500_GPIO3_SW_EN : 0U);
}

/* -------------------------------------------------------------------------
 * NVM: the FTP controller
 *
 * Sequences follow ST's customer NVM API.  Every step that hands work to the
 * FTP state machine is followed by a bounded poll of FTP_CUST_REQ, which is
 * the one place ST's reference spins forever.
 * ------------------------------------------------------------------------- */

static stusb4500_error_t stusb4500_ftp_wait(void)
{
    uint32_t waited = 0U;

    for (;;)
    {
        uint8_t                 ctrl = 0U;
        const stusb4500_error_t err = stusb4500_read_register(STUSB4500_REG_FTP_CTRL_0, &ctrl);
        if (err != STUSB4500_OK)
        {
            return err;
        }
        if ((ctrl & STUSB4500_FTP_CUST_REQ) == 0U)
        {
            return STUSB4500_OK;
        }
        if (waited >= STUSB4500_T_FTP_TIMEOUT_MS)
        {
            return STUSB4500_ERR_FTP_TIMEOUT;
        }

        stusb4500_delay_ms(STUSB4500_T_POLL_INTERVAL_MS);
        waited += STUSB4500_T_POLL_INTERVAL_MS;
    }
}

/* Stage an opcode in FTP_CTRL_1, then request it with the sector selected in
 * FTP_CTRL_0, and wait for the state machine to finish. */
static stusb4500_error_t stusb4500_ftp_execute(const uint8_t opcode,
                                               const uint8_t ser_mask,
                                               const uint8_t sector)
{
    const uint8_t ctrl_1 =
        (uint8_t)(((uint8_t)(ser_mask << STUSB4500_FTP_CUST_SER_SHIFT) &
                   STUSB4500_FTP_CUST_SER_MASK) |
                  (opcode & STUSB4500_FTP_CUST_OPCODE));

    stusb4500_error_t err = stusb4500_write_register(STUSB4500_REG_FTP_CTRL_1, ctrl_1);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0,
                                  (uint8_t)((sector & STUSB4500_FTP_CUST_SECT) |
                                            STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N |
                                            STUSB4500_FTP_CUST_REQ));
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_ftp_wait();
}

/* Unlock the FTP controller and bring it out of reset. */
static stusb4500_error_t stusb4500_ftp_power_up(void)
{
    stusb4500_error_t err =
        stusb4500_write_register(STUSB4500_REG_FTP_CUST_PASSWORD, STUSB4500_FTP_PASSWORD);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0, 0x00U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0,
                                   (uint8_t)(STUSB4500_FTP_CUST_PWR |
                                             STUSB4500_FTP_CUST_RST_N));
}

/* Clear the control registers and the password.  Always attempted, even
 * after a failure, so a half-finished operation cannot leave the FTP
 * controller unlocked. */
static stusb4500_error_t stusb4500_ftp_power_down(void)
{
    stusb4500_error_t err =
        stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0, STUSB4500_FTP_CUST_RST_N);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_write_register(STUSB4500_REG_FTP_CTRL_1, 0x00U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_write_register(STUSB4500_REG_FTP_CUST_PASSWORD, 0x00U);
}

/*
 * ST's enter-write-mode order, kept exactly: the password, then RW_BUFFER
 * cleared, then the power sequence.  RW_BUFFER has to be zero before the
 * erase opcode runs for the partial-erase path to behave, and ST clears it
 * ahead of powering the controller rather than after.
 */
static stusb4500_error_t stusb4500_ftp_enter_write_mode(void)
{
    stusb4500_error_t err =
        stusb4500_write_register(STUSB4500_REG_FTP_CUST_PASSWORD, STUSB4500_FTP_PASSWORD);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_write_register(STUSB4500_REG_RW_BUFFER, 0x00U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0, 0x00U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0,
                                   (uint8_t)(STUSB4500_FTP_CUST_PWR |
                                             STUSB4500_FTP_CUST_RST_N));
}

static stusb4500_error_t stusb4500_ftp_erase_all(void)
{
    /* The erase path is three staged opcodes: publish the sector mask into
     * the sector-erase register, soft-program those sectors, then erase. */
    stusb4500_error_t err = stusb4500_ftp_execute(
        STUSB4500_FTP_OP_WRITE_SER, STUSB4500_FTP_SECTOR_ALL, 0U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_ftp_execute(STUSB4500_FTP_OP_SOFT_PROG, 0U, 0U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_ftp_execute(STUSB4500_FTP_OP_ERASE, 0U, 0U);
}

static stusb4500_error_t stusb4500_ftp_read_sector(const uint8_t sector, uint8_t *data)
{
    stusb4500_error_t err = stusb4500_write_register(
        STUSB4500_REG_FTP_CTRL_0,
        (uint8_t)(STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N));
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_ftp_execute(STUSB4500_FTP_OP_READ_SECTOR, 0U, sector);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_read_registers(STUSB4500_REG_RW_BUFFER, data, STUSB4500_NVM_SECTOR_SIZE);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0, 0x00U);
}

static stusb4500_error_t stusb4500_ftp_write_sector(const uint8_t sector, const uint8_t *data)
{
    /* The eight bytes go into the program-load register first, then the
     * program opcode commits them to the selected sector. */
    stusb4500_error_t err =
        stusb4500_write_registers(STUSB4500_REG_RW_BUFFER, data, STUSB4500_NVM_SECTOR_SIZE);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_write_register(STUSB4500_REG_FTP_CTRL_0,
                                  (uint8_t)(STUSB4500_FTP_CUST_PWR | STUSB4500_FTP_CUST_RST_N));
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_ftp_execute(STUSB4500_FTP_OP_WRITE_PL, 0U, 0U);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    return stusb4500_ftp_execute(STUSB4500_FTP_OP_PROGRAM, 0U, sector);
}

stusb4500_error_t stusb4500_nvm_read_sector(const uint8_t sector, uint8_t *data)
{
    if (data == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    if (sector >= STUSB4500_NVM_SECTOR_COUNT)
    {
        return STUSB4500_ERR_BAD_SECTOR;
    }

    stusb4500_error_t err = stusb4500_ftp_power_up();
    if (err == STUSB4500_OK)
    {
        err = stusb4500_ftp_read_sector(sector, data);
    }

    const stusb4500_error_t down_err = stusb4500_ftp_power_down();
    return (err != STUSB4500_OK) ? err : down_err;
}

stusb4500_error_t stusb4500_nvm_read(uint8_t *nvm)
{
    uint8_t sector = 0U;

    if (nvm == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    stusb4500_error_t err = stusb4500_ftp_power_up();
    if (err == STUSB4500_OK)
    {
        for (sector = 0U; sector < STUSB4500_NVM_SECTOR_COUNT; sector++)
        {
            err = stusb4500_ftp_read_sector(sector, &nvm[sector * STUSB4500_NVM_SECTOR_SIZE]);
            if (err != STUSB4500_OK)
            {
                break;
            }
        }
    }

    const stusb4500_error_t down_err = stusb4500_ftp_power_down();
    return (err != STUSB4500_OK) ? err : down_err;
}

/* Byte-wise compare; no string.h dependency in this driver. */
static uint8_t stusb4500_nvm_same(const uint8_t *a, const uint8_t *b)
{
    uint8_t i = 0U;

    for (i = 0U; i < STUSB4500_NVM_SIZE; i++)
    {
        if (a[i] != b[i])
        {
            return 0U;
        }
    }
    return 1U;
}

stusb4500_error_t stusb4500_nvm_write(const uint8_t *nvm)
{
    uint8_t current[STUSB4500_NVM_SIZE] = {0U};
    uint8_t sector                      = 0U;

    if (nvm == (const uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    /* The program-cycle budget is finite, so a write that would change
     * nothing is not a write. */
    stusb4500_error_t err = stusb4500_nvm_read(current);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    if (stusb4500_nvm_same(current, nvm) != 0U)
    {
        return STUSB4500_OK;
    }

    err = stusb4500_ftp_enter_write_mode();
    if (err == STUSB4500_OK)
    {
        err = stusb4500_ftp_erase_all();
    }
    if (err == STUSB4500_OK)
    {
        for (sector = 0U; sector < STUSB4500_NVM_SECTOR_COUNT; sector++)
        {
            err = stusb4500_ftp_write_sector(sector, &nvm[sector * STUSB4500_NVM_SECTOR_SIZE]);
            if (err != STUSB4500_OK)
            {
                break;
            }
        }
    }

    const stusb4500_error_t down_err = stusb4500_ftp_power_down();
    if (err != STUSB4500_OK)
    {
        return err;
    }
    if (down_err != STUSB4500_OK)
    {
        return down_err;
    }

    err = stusb4500_nvm_read(current);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    if (stusb4500_nvm_same(current, nvm) == 0U)
    {
        return STUSB4500_ERR_NVM_VERIFY;
    }

    /* Stored contents only govern behaviour from the next startup. */
    return stusb4500_software_reset();
}

/* -------------------------------------------------------------------------
 * NVM field layout
 *
 * Each sector is a little-endian bit stream: bit b of a sector lives in byte
 * b/8 at bit b%8.  The field descriptors below are (sector, first bit, width)
 * triples, expanded straight into the accessor argument lists, and they come
 * from ST's NVM technical note.  Every one of them has been checked against
 * the factory-default image in stusb4500_nvm_default_image.
 * ------------------------------------------------------------------------- */

/* Sector 0: identity. */
#define STUSB4500_NVM_VENDOR_ID     0U, 0U, 16U
#define STUSB4500_NVM_PRODUCT_ID    0U, 16U, 16U
#define STUSB4500_NVM_BCD_DEVICE_ID 0U, 32U, 16U

/* Sector 1: pin behaviour and VBUS discharge timing. */
#define STUSB4500_NVM_GPIO_CFG            1U, 4U, 2U
#define STUSB4500_NVM_VBUS_DCHG_MASK      1U, 13U, 1U
#define STUSB4500_NVM_DISCHARGE_TO_PDO    1U, 16U, 4U
#define STUSB4500_NVM_DISCHARGE_TO_0V     1U, 20U, 4U

/* Sector 3: sink policy and the per-PDO current indices and windows. */
#define STUSB4500_NVM_USB_COMM_CAPABLE 3U, 16U, 1U
#define STUSB4500_NVM_PDO_NUMB         3U, 17U, 2U
#define STUSB4500_NVM_UNCONS_POWER     3U, 19U, 1U
#define STUSB4500_NVM_PDO1_I           3U, 20U, 4U
#define STUSB4500_NVM_SNK_LL1          3U, 24U, 4U
#define STUSB4500_NVM_SNK_HL1          3U, 28U, 4U
#define STUSB4500_NVM_PDO2_I           3U, 32U, 4U
#define STUSB4500_NVM_SNK_LL2          3U, 36U, 4U
#define STUSB4500_NVM_SNK_HL2          3U, 40U, 4U
#define STUSB4500_NVM_PDO3_I           3U, 44U, 4U
#define STUSB4500_NVM_SNK_LL3          3U, 48U, 4U
#define STUSB4500_NVM_SNK_HL3          3U, 52U, 4U

/* Sector 4: PDO2/PDO3 voltages (ST names them the flex voltages), the flex
 * current, and the remaining sink options. */
#define STUSB4500_NVM_PDO2_V             4U, 6U, 10U
#define STUSB4500_NVM_PDO3_V             4U, 16U, 10U
#define STUSB4500_NVM_FLEX_I             4U, 26U, 10U
#define STUSB4500_NVM_POWER_OK_CFG       4U, 37U, 2U
#define STUSB4500_NVM_POWER_ONLY_ABOVE_5V 4U, 51U, 1U
#define STUSB4500_NVM_REQ_SRC_CURRENT    4U, 52U, 1U
#define STUSB4500_NVM_ALERT_MASK         4U, 56U, 8U

/* Resolutions of the sector-1 discharge timers. */
#define STUSB4500_NVM_DISCHARGE_TO_PDO_LSB_MS (20U)
#define STUSB4500_NVM_DISCHARGE_TO_0V_LSB_MS  (84U)

/*
 * ST's factory-default image.  Reproduced here so a board can be put back to
 * a known state, and so the field descriptors above have something to be
 * checked against: with this image, PDO1 is 5 V at 1.5 A, PDO2 15 V at 1.5 A,
 * PDO3 20 V at 1 A, three PDOs advertised, flex current 2 A, GPIO3 in error-
 * recovery mode and POWER_OK configuration 2.
 */
static const uint8_t stusb4500_nvm_default_image[STUSB4500_NVM_SIZE] = {
    0x00, 0x00, 0xB0, 0xAA, 0x00, 0x45, 0x00, 0x00, /* sector 0 */
    0x10, 0x40, 0x9C, 0x1C, 0xFF, 0x01, 0x3C, 0xDF, /* sector 1 */
    0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1, /* sector 2 */
    0x00, 0x19, 0x56, 0xAF, 0xF5, 0x35, 0x5F, 0x00, /* sector 3 */
    0x00, 0x4B, 0x90, 0x21, 0x43, 0x00, 0x40, 0xFB, /* sector 4 */
};

/*
 * The sink operating current is stored as a 4-bit index into this table, not
 * as a number of milliamps, so only these values can be advertised.  Index 0
 * means the current is taken from the flex field instead; it reads back as
 * zero rather than as a current the part is not actually asking for.
 */
static const uint16_t stusb4500_nvm_current_table[16] = {
    0U,    /* 0: use flex_current_ma */
    500U,  750U,  1000U, 1250U, 1500U, 1750U, 2000U, 2250U,
    2500U, 2750U, 3000U, 3500U, 4000U, 4500U, 5000U};

static uint32_t
stusb4500_nvm_get(const uint8_t *nvm, const uint8_t sector, const uint8_t first, const uint8_t width)
{
    uint32_t value = 0U;
    uint8_t  i     = 0U;

    for (i = 0U; i < width; i++)
    {
        const uint16_t bit  = (uint16_t)(first + i);
        const uint8_t  byte = nvm[(sector * STUSB4500_NVM_SECTOR_SIZE) + (bit / 8U)];

        if ((byte & (uint8_t)(1U << (bit % 8U))) != 0U)
        {
            value |= (uint32_t)1UL << i;
        }
    }
    return value;
}

static void stusb4500_nvm_set(uint8_t       *nvm,
                              const uint8_t  sector,
                              const uint8_t  first,
                              const uint8_t  width,
                              const uint32_t value)
{
    uint8_t i = 0U;

    for (i = 0U; i < width; i++)
    {
        const uint16_t bit   = (uint16_t)(first + i);
        const uint16_t index = (uint16_t)((sector * STUSB4500_NVM_SECTOR_SIZE) + (bit / 8U));
        const uint8_t  mask  = (uint8_t)(1U << (bit % 8U));

        if (((value >> i) & 1UL) != 0UL)
        {
            nvm[index] |= mask;
        }
        else
        {
            nvm[index] &= (uint8_t)~mask;
        }
    }
}

uint16_t stusb4500_nvm_index_to_current(const uint8_t index)
{
    return stusb4500_nvm_current_table[index & 0x0FU];
}

stusb4500_error_t stusb4500_nvm_current_to_index(const uint16_t current_ma, uint8_t *index)
{
    uint8_t i = 0U;

    if (index == (uint8_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }
    if (current_ma == 0U)
    {
        *index = 0U;
        return STUSB4500_OK;
    }

    /* Round up: a sink that advertises less current than it needs invites a
     * contract it cannot live with. */
    for (i = 1U; i < 16U; i++)
    {
        if (stusb4500_nvm_current_table[i] >= current_ma)
        {
            *index = i;
            return STUSB4500_OK;
        }
    }
    return STUSB4500_ERR_BAD_PARAM;
}

void stusb4500_nvm_factory_default(uint8_t *nvm)
{
    uint8_t i = 0U;

    if (nvm == (uint8_t *)0)
    {
        return;
    }
    for (i = 0U; i < STUSB4500_NVM_SIZE; i++)
    {
        nvm[i] = stusb4500_nvm_default_image[i];
    }
}

stusb4500_error_t stusb4500_nvm_decode(const uint8_t *nvm, stusb4500_nvm_config_t *cfg)
{
    if ((nvm == (const uint8_t *)0) || (cfg == (stusb4500_nvm_config_t *)0))
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    cfg->vendor_id     = (uint16_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_VENDOR_ID);
    cfg->product_id    = (uint16_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_PRODUCT_ID);
    cfg->bcd_device_id = (uint16_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_BCD_DEVICE_ID);

    cfg->gpio_cfg = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_GPIO_CFG);
    cfg->vbus_discharge_masked = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_VBUS_DCHG_MASK);
    cfg->discharge_to_pdo_ms =
        (uint16_t)(stusb4500_nvm_get(nvm, STUSB4500_NVM_DISCHARGE_TO_PDO) *
                   STUSB4500_NVM_DISCHARGE_TO_PDO_LSB_MS);
    cfg->discharge_to_0v_ms =
        (uint16_t)(stusb4500_nvm_get(nvm, STUSB4500_NVM_DISCHARGE_TO_0V) *
                   STUSB4500_NVM_DISCHARGE_TO_0V_LSB_MS);

    cfg->pdo_count           = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_PDO_NUMB);
    cfg->usb_comm_capable    = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_USB_COMM_CAPABLE);
    cfg->unconstrained_power = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_UNCONS_POWER);
    cfg->power_ok_cfg        = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_POWER_OK_CFG);
    cfg->power_only_above_5v =
        (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_POWER_ONLY_ABOVE_5V);
    cfg->req_src_current = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_REQ_SRC_CURRENT);
    cfg->alert_mask      = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_ALERT_MASK);

    /* PDO1's voltage is fixed by the specification and has no NVM field. */
    cfg->pdo_voltage_mv[0] = STUSB4500_PDO1_VOLTAGE_MV;
    cfg->pdo_voltage_mv[1] = (uint16_t)(stusb4500_nvm_get(nvm, STUSB4500_NVM_PDO2_V) *
                                        STUSB4500_PDO_VOLTAGE_LSB_MV);
    cfg->pdo_voltage_mv[2] = (uint16_t)(stusb4500_nvm_get(nvm, STUSB4500_NVM_PDO3_V) *
                                        STUSB4500_PDO_VOLTAGE_LSB_MV);

    cfg->pdo_current_ma[0] =
        stusb4500_nvm_index_to_current((uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_PDO1_I));
    cfg->pdo_current_ma[1] =
        stusb4500_nvm_index_to_current((uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_PDO2_I));
    cfg->pdo_current_ma[2] =
        stusb4500_nvm_index_to_current((uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_PDO3_I));

    cfg->flex_current_ma = (uint16_t)(stusb4500_nvm_get(nvm, STUSB4500_NVM_FLEX_I) *
                                      STUSB4500_PDO_CURRENT_LSB_MA);

    cfg->under_voltage_pct[0] = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_SNK_LL1);
    cfg->under_voltage_pct[1] = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_SNK_LL2);
    cfg->under_voltage_pct[2] = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_SNK_LL3);
    cfg->over_voltage_pct[0]  = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_SNK_HL1);
    cfg->over_voltage_pct[1]  = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_SNK_HL2);
    cfg->over_voltage_pct[2]  = (uint8_t)stusb4500_nvm_get(nvm, STUSB4500_NVM_SNK_HL3);

    return STUSB4500_OK;
}

/* Validate and scale one of the sector-1 discharge timers. */
static stusb4500_error_t
stusb4500_nvm_encode_time(const uint16_t ms, const uint16_t lsb_ms, uint32_t *code)
{
    if ((ms % lsb_ms) != 0U)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    const uint16_t steps = (uint16_t)(ms / lsb_ms);
    if (steps > 0x0FU)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    *code = steps;
    return STUSB4500_OK;
}

/* Validate and scale one of the PDO2/PDO3 voltages. */
static stusb4500_error_t stusb4500_nvm_encode_voltage(const uint16_t mv, uint32_t *code)
{
    if ((mv % STUSB4500_PDO_VOLTAGE_LSB_MV) != 0U)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    const uint16_t steps = (uint16_t)(mv / STUSB4500_PDO_VOLTAGE_LSB_MV);
    if (steps > 0x3FFU)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    *code = steps;
    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_nvm_encode(uint8_t *nvm, const stusb4500_nvm_config_t *cfg)
{
    uint8_t  indices[STUSB4500_SINK_PDO_COUNT] = {0U, 0U, 0U};
    uint32_t volts[STUSB4500_SINK_PDO_COUNT]   = {0U, 0U, 0U};
    uint32_t to_pdo  = 0U;
    uint32_t to_0v   = 0U;
    uint32_t flex    = 0U;
    uint8_t  i       = 0U;

    if ((nvm == (uint8_t *)0) || (cfg == (const stusb4500_nvm_config_t *)0))
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    /* Everything is validated before anything is written, so a rejected
     * config leaves the caller's image exactly as it was. */
    if ((cfg->gpio_cfg > 3U) || (cfg->power_ok_cfg > 3U) || (cfg->pdo_count < 1U) ||
        (cfg->pdo_count > STUSB4500_SINK_PDO_COUNT))
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    for (i = 0U; i < STUSB4500_SINK_PDO_COUNT; i++)
    {
        if ((cfg->under_voltage_pct[i] > 0x0FU) || (cfg->over_voltage_pct[i] > 0x0FU))
        {
            return STUSB4500_ERR_BAD_PARAM;
        }
    }

    stusb4500_error_t err = stusb4500_nvm_encode_time(
        cfg->discharge_to_pdo_ms, STUSB4500_NVM_DISCHARGE_TO_PDO_LSB_MS, &to_pdo);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    err = stusb4500_nvm_encode_time(
        cfg->discharge_to_0v_ms, STUSB4500_NVM_DISCHARGE_TO_0V_LSB_MS, &to_0v);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    /* PDO1's voltage is not stored, so only PDO2 and PDO3 are encoded. */
    for (i = 1U; i < STUSB4500_SINK_PDO_COUNT; i++)
    {
        err = stusb4500_nvm_encode_voltage(cfg->pdo_voltage_mv[i], &volts[i]);
        if (err != STUSB4500_OK)
        {
            return err;
        }
    }
    for (i = 0U; i < STUSB4500_SINK_PDO_COUNT; i++)
    {
        err = stusb4500_nvm_current_to_index(cfg->pdo_current_ma[i], &indices[i]);
        if (err != STUSB4500_OK)
        {
            return err;
        }
    }

    if ((cfg->flex_current_ma % STUSB4500_PDO_CURRENT_LSB_MA) != 0U)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }
    flex = (uint32_t)cfg->flex_current_ma / STUSB4500_PDO_CURRENT_LSB_MA;
    if (flex > 0x3FFUL)
    {
        return STUSB4500_ERR_BAD_PARAM;
    }

    /* Sector 0 is identity and is left alone: rewriting the vendor and
     * product IDs is not something a configuration change should do. */

    stusb4500_nvm_set(nvm, STUSB4500_NVM_GPIO_CFG, cfg->gpio_cfg);
    stusb4500_nvm_set(
        nvm, STUSB4500_NVM_VBUS_DCHG_MASK, (cfg->vbus_discharge_masked != 0U) ? 1UL : 0UL);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_DISCHARGE_TO_PDO, to_pdo);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_DISCHARGE_TO_0V, to_0v);

    stusb4500_nvm_set(nvm, STUSB4500_NVM_PDO_NUMB, cfg->pdo_count);
    stusb4500_nvm_set(
        nvm, STUSB4500_NVM_USB_COMM_CAPABLE, (cfg->usb_comm_capable != 0U) ? 1UL : 0UL);
    stusb4500_nvm_set(
        nvm, STUSB4500_NVM_UNCONS_POWER, (cfg->unconstrained_power != 0U) ? 1UL : 0UL);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_POWER_OK_CFG, cfg->power_ok_cfg);
    stusb4500_nvm_set(
        nvm, STUSB4500_NVM_POWER_ONLY_ABOVE_5V, (cfg->power_only_above_5v != 0U) ? 1UL : 0UL);
    stusb4500_nvm_set(
        nvm, STUSB4500_NVM_REQ_SRC_CURRENT, (cfg->req_src_current != 0U) ? 1UL : 0UL);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_ALERT_MASK, cfg->alert_mask);

    stusb4500_nvm_set(nvm, STUSB4500_NVM_PDO1_I, indices[0]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_PDO2_I, indices[1]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_PDO3_I, indices[2]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_PDO2_V, volts[1]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_PDO3_V, volts[2]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_FLEX_I, flex);

    stusb4500_nvm_set(nvm, STUSB4500_NVM_SNK_LL1, cfg->under_voltage_pct[0]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_SNK_LL2, cfg->under_voltage_pct[1]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_SNK_LL3, cfg->under_voltage_pct[2]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_SNK_HL1, cfg->over_voltage_pct[0]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_SNK_HL2, cfg->over_voltage_pct[1]);
    stusb4500_nvm_set(nvm, STUSB4500_NVM_SNK_HL3, cfg->over_voltage_pct[2]);

    return STUSB4500_OK;
}

stusb4500_error_t stusb4500_nvm_get_config(stusb4500_nvm_config_t *cfg)
{
    uint8_t nvm[STUSB4500_NVM_SIZE] = {0U};

    if (cfg == (stusb4500_nvm_config_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    const stusb4500_error_t err = stusb4500_nvm_read(nvm);
    if (err != STUSB4500_OK)
    {
        return err;
    }
    return stusb4500_nvm_decode(nvm, cfg);
}

stusb4500_error_t stusb4500_nvm_set_config(const stusb4500_nvm_config_t *cfg)
{
    uint8_t nvm[STUSB4500_NVM_SIZE] = {0U};

    if (cfg == (const stusb4500_nvm_config_t *)0)
    {
        return STUSB4500_ERR_NULL_PARAM;
    }

    /* Read-modify-write against the device's own contents, so the reserved
     * fields and anything this driver cannot name survive untouched. */
    stusb4500_error_t err = stusb4500_nvm_read(nvm);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    err = stusb4500_nvm_encode(nvm, cfg);
    if (err != STUSB4500_OK)
    {
        return err;
    }

    /* stusb4500_nvm_write() is a no-op if nothing actually changed. */
    return stusb4500_nvm_write(nvm);
}
