#include "max17320.h"

#ifdef DEBUG
#include "logging.h"
#define LOG_TAG "BMS_DRV"
#endif

// List of errors, that if occur, the device will not be revived
const uint16_t bms_revive_blacklist[] = {
    BMS_BIT_OVP, BMS_BIT_IMBALANCE, BMS_BIT_PREQF, BMS_BIT_PMFAIL};

bms_error_t __attribute__((weak)) bms_reg_write(uint16_t reg, uint8_t *data, uint16_t len)
{
    return BMS_I2C_MEM_WRITE_ERROR;
}

bms_error_t __attribute__((weak)) bms_reg_read(uint16_t reg, uint8_t *data, uint16_t len)
{
    return BMS_I2C_MEM_READ_ERROR;
}

static bms_error_t bms_full_reset() { return BMS_RESET_ERROR; }

bms_error_t bms_bat_okay()
{
    uint16_t data = 0;

    // 1. Check for permanent fault. If so leave it and return the result.

    // Read the battery voltage data
    uint8_t status = bms_reg_read(BMS_REG_NBATTSTATUS, (uint8_t *)&data, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Error reading nBattStatus register.\n");
#endif
        return status;
    }

    // See if the perm fail bit is set
    if (data & BMS_BIT_PERMFAIL)
    {
#ifdef DEBUG
        log_fatal(LOG_TAG, "Permanent fail detected, this device is irrepairable.\n");
#endif
        return BMS_PERM_FAIL_ERROR;
    }

    // 2. Read the status register to see if there has been any protection events
    status = bms_reg_read(BMS_REG_STATUS, (uint8_t *)&data, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Error reading status register.\n");
#endif
        return status;
    }

    // See if the PA (protection alert) bit is set
    if (data & BMS_BIT_PROTALRT)
    {
#ifdef DEBUG
        log_info(LOG_TAG, "Protection alert bit set, looking to see if it can be corrected.\n");
#endif

        // 2.1 Get the historic protection alert data
        status = bms_reg_read(BMS_REG_PROTALRT, (uint8_t *)&data, 2);

        if (status != BMS_OK)
        {
#ifdef DEBUG
            log_error(LOG_TAG, "Error reading protection alert register register.\n");
#endif
            return status;
        }

        // Go through the blacklist in case we should not revive
        for (uint32_t i = 0; i < (sizeof(bms_revive_blacklist) / sizeof(bms_revive_blacklist[0]));
             i++)
        {
            if (data & bms_revive_blacklist[i])
            {
                // If there is an irrepairable fault
#ifdef DEBUG
                log_fatal(LOG_TAG,
                          "Cannot recover from %u fault in PROTALRT reg.\n",
                          bms_revive_blacklist[i]);
#endif
                return BMS_CANNOT_REC_ERROR;
            }
        }

#ifdef DEBUG
        log_info(LOG_TAG,
                 "Found no issues that cannot be repaired. Fully resetting the BMS to repair.\n");
#endif

        // 2.2 Full reset
        status = bms_full_reset();

        if (status != BMS_OK)
        {
#ifdef DEBUG
            log_fatal(LOG_TAG, "Could not reset the BMS.\n");
#endif
            return status;
        }
    }

#ifdef DEBUG
    log_info(LOG_TAG, "BMS is okay.\n");
#endif

    return BMS_OK;
}

bms_error_t bms_get_voltage(float *voltage)
{
    uint16_t data = 0;

    // Read the battery voltage data
    uint8_t status = bms_reg_read(BMS_REG_VBAT, (uint8_t *)&data, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(
            LOG_TAG, "Error reading raw voltage data from the BMS, error code = %u.\n", status);
#endif
        return status;
    }

    // Convert
    *voltage = (float)data * 0.3125e-3;

    return BMS_OK;
}

bms_error_t bms_get_current(float *current)
{
    int16_t data = 0;

    // Read the resistor voltage data
    uint8_t status = bms_reg_read(BMS_REG_AVG_CBAT, (uint8_t *)&data, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Error reading current data from the BMS, error code = %u.\n", status);
#endif
        return status;
    }

    // Convert
    *current = (float)data * 156.25e-6;

    return BMS_OK;
}

bms_error_t bms_get_soc(float *soc)
{
    uint16_t data = 0;

    // Read the resistor voltage data
    uint8_t status = bms_reg_read(BMS_REG_REPCAP, (uint8_t *)&data, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Error reading SoC data from the BMS, error code = %u.\n", status);
#endif
        return status;
    }

    // Convert
    *soc = (float)data * 10e-3;

    return BMS_OK;
}

bms_error_t bms_get_full_capacity(float *full_cap)
{
    uint16_t data = 0;

    // Read the resistor voltage data
    uint8_t status = bms_reg_read(BMS_REG_FULLCAP, (uint8_t *)&data, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(
            LOG_TAG, "Error reading full capacity data from the BMS, error code = %u.\n", status);
#endif
        return status;
    }

    // Convert
    *full_cap = (float)data * 500e-6;

    return BMS_OK;
}

bms_error_t bms_get_general_status(bms_gen_status_t *value)
{
    // Read the resistor voltage data
    uint8_t status = bms_reg_read(BMS_REG_STATUS, (uint8_t *)value, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Error reading status from the BMS, error code = %u.\n", status);
#endif
        return status;
    }

    return BMS_OK;
}

bms_error_t bms_get_protection_status(bms_fault_status_t *value)
{
    // Read the resistor voltage data
    uint8_t status = bms_reg_read(BMS_REG_PROTSTATUS, (uint8_t *)value, 2);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(
            LOG_TAG, "Error reading protection status from the BMS, error code = %u.\n", status);
#endif
        return status;
    }

    return BMS_OK;
}

bms_error_t bms_get_stats(bms_stats_t *stats)
{
    uint8_t status = BMS_OK;

    // Get voltage
    status = bms_get_voltage(&stats->volts);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Reading voltage not okay, error code = %u.\n", status);
#endif
        return status;
    }

    // Get current
    status = bms_get_current(&stats->amps);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Reading current not okay, error code = %u.\n", status);
#endif
        return status;
    }

    // Get SoC
    status = bms_get_soc(&stats->soc);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Reading SoC not okay, error code = %u.\n", status);
#endif
        return status;
    }

    // Get Capacity
    status = bms_get_full_capacity(&stats->cap);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Reading capacity not okay, error code = %u.\n", status);
#endif
        return status;
    }

    // Get general status
    status = bms_get_general_status(&stats->status);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Reading status not okay, error code = %u.\n", status);
#endif
        return status;
    }

    // Get Capacity
    status = bms_get_protection_status(&stats->fault);

    if (status != BMS_OK)
    {
#ifdef DEBUG
        log_error(LOG_TAG, "Reading protection status not okay, error code = %u.\n", status);
#endif
        return status;
    }

    return status;
}
