#include "max17320.h"

/* -------------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------------- */

/* Sense resistance in ohms; 0.0F until max17320_init() succeeds. */
static float   s_r_sense_ohms = 0.0F;
/* Series cell count, 2..4; 0 until max17320_init() succeeds. */
static uint8_t s_cell_count   = 0U;

/* Number of write/read-back attempts made by the verified write helper. */
#define MAX17320_VERIFY_ATTEMPTS (3U)
/* Settling delay between a verified write and its read-back, milliseconds. */
#define MAX17320_VERIFY_DELAY_MS (1U)
/* Poll interval used while waiting on CommStat.NVBusy or Config2.POR_CMD. */
#define MAX17320_POLL_INTERVAL_MS (5U)

/* CommStat bits that make up the write-protection state. */
#define MAX17320_COMMSTAT_WP_MASK                                                     \
    (MAX17320_COMMSTAT_WPGLOBAL | MAX17320_COMMSTAT_WP1 | MAX17320_COMMSTAT_WP2 |     \
     MAX17320_COMMSTAT_WP3 | MAX17320_COMMSTAT_WP4 | MAX17320_COMMSTAT_WP5)

/* Power-up values of the min/max logging registers. */
#define MAX17320_MAXMINVOLT_RESET (0x00FFU)
#define MAX17320_MAXMINCURR_RESET (0x807FU)
#define MAX17320_MAXMINTEMP_RESET (0x807FU)

/* nHibCfg.EnHib */
#define MAX17320_NHIBCFG_ENHIB (0x8000U)

/* -------------------------------------------------------------------------
 * Weak platform hooks.  The application overrides these; the defaults report
 * a distinct error so a missing port cannot masquerade as a dead bus.
 * ------------------------------------------------------------------------- */

max17320_error_t __attribute__((weak)) max17320_i2c_read(const uint8_t  dev_addr,
                                                         const uint8_t  mem_addr,
                                                         uint8_t       *rx,
                                                         const uint16_t len)
{
    (void)dev_addr;
    (void)mem_addr;
    (void)rx;
    (void)len;
    return MAX17320_ERR_NO_PLATFORM_READ;
}

max17320_error_t __attribute__((weak)) max17320_i2c_write(const uint8_t  dev_addr,
                                                          const uint8_t  mem_addr,
                                                          const uint8_t *tx,
                                                          const uint16_t len)
{
    (void)dev_addr;
    (void)mem_addr;
    (void)tx;
    (void)len;
    return MAX17320_ERR_NO_PLATFORM_WRITE;
}

void __attribute__((weak)) max17320_delay_ms(const uint32_t ms) { (void)ms; }

/* -------------------------------------------------------------------------
 * Address decoding
 * ------------------------------------------------------------------------- */

static uint8_t max17320_slave_of(const uint16_t addr)
{
    return (addr <= MAX17320_ADDR_SPACE_SPLIT) ? MAX17320_I2C_ADDR_LOW_7B
                                               : MAX17320_I2C_ADDR_HIGH_7B;
}

/* Last address the device will auto-increment to before ignoring the burst. */
static uint16_t max17320_space_end_of(const uint16_t addr)
{
    return (addr <= MAX17320_ADDR_SPACE_SPLIT) ? MAX17320_ADDR_SPACE_SPLIT : MAX17320_ADDR_MAX;
}

/* Addresses 100h-17Fh belong to the SBS block and must be accessed singly. */
static uint8_t max17320_is_single_word(const uint16_t addr)
{
    return ((addr >= 0x100U) && (addr <= 0x17FU)) ? 1U : 0U;
}

/* -------------------------------------------------------------------------
 * Raw register access
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_read_register(const uint16_t addr, uint16_t *value)
{
    uint8_t rx[2] = {0U, 0U};

    if (value == (uint16_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    if (addr > MAX17320_ADDR_MAX)
    {
        return MAX17320_ERR_BAD_ADDRESS;
    }

    const max17320_error_t err =
        max17320_i2c_read(max17320_slave_of(addr), (uint8_t)(addr & 0xFFU), rx, 2U);
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* Register words travel least-significant byte first. */
    *value = (uint16_t)((uint16_t)rx[0] | ((uint16_t)rx[1] << 8U));
    return MAX17320_OK;
}

max17320_error_t max17320_write_register(const uint16_t addr, const uint16_t value)
{
    if (addr > MAX17320_ADDR_MAX)
    {
        return MAX17320_ERR_BAD_ADDRESS;
    }

    const uint8_t tx[2] = {(uint8_t)(value & 0xFFU), (uint8_t)(value >> 8U)};
    return max17320_i2c_write(max17320_slave_of(addr), (uint8_t)(addr & 0xFFU), tx, 2U);
}

max17320_error_t max17320_write_verify_register(const uint16_t addr, const uint16_t value)
{
    uint16_t attempt = 0U;

    for (attempt = 0U; attempt < MAX17320_VERIFY_ATTEMPTS; attempt++)
    {
        uint16_t         read_back = 0U;
        max17320_error_t err       = max17320_write_register(addr, value);
        if (err != MAX17320_OK)
        {
            return err;
        }

        max17320_delay_ms(MAX17320_VERIFY_DELAY_MS);

        err = max17320_read_register(addr, &read_back);
        if (err != MAX17320_OK)
        {
            return err;
        }
        if (read_back == value)
        {
            return MAX17320_OK;
        }
    }

    return MAX17320_ERR_WRITE_VERIFY;
}

max17320_error_t
max17320_read_registers(const uint16_t addr, uint16_t *values, const uint16_t count)
{
    uint16_t index = 0U;

    if (values == (uint16_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    if (addr > MAX17320_ADDR_MAX)
    {
        return MAX17320_ERR_BAD_ADDRESS;
    }
    if (count == 0U)
    {
        return MAX17320_ERR_BAD_LENGTH;
    }
    /* A burst may not run past the end of the address space it started in. */
    if ((uint32_t)addr + (uint32_t)count - 1U > (uint32_t)max17320_space_end_of(addr))
    {
        return MAX17320_ERR_BAD_LENGTH;
    }

    while (index < count)
    {
        const uint16_t here      = (uint16_t)(addr + index);
        const uint16_t remaining = (uint16_t)(count - index);
        uint16_t       chunk     = remaining;
        uint8_t        rx[32]    = {0U};
        uint16_t       word      = 0U;

        if (max17320_is_single_word(here) != 0U)
        {
            chunk = 1U;
        }
        else if (chunk > (uint16_t)(sizeof(rx) / 2U))
        {
            chunk = (uint16_t)(sizeof(rx) / 2U);
        }
        else
        {
            /* chunk already fits */
        }

        const max17320_error_t err = max17320_i2c_read(
            max17320_slave_of(here), (uint8_t)(here & 0xFFU), rx, (uint16_t)(chunk * 2U));
        if (err != MAX17320_OK)
        {
            return err;
        }

        for (word = 0U; word < chunk; word++)
        {
            values[index + word] =
                (uint16_t)((uint16_t)rx[word * 2U] | ((uint16_t)rx[(word * 2U) + 1U] << 8U));
        }
        index = (uint16_t)(index + chunk);
    }

    return MAX17320_OK;
}

max17320_error_t
max17320_update_register(const uint16_t addr, const uint16_t mask, const uint16_t value)
{
    uint16_t               current = 0U;
    const max17320_error_t err     = max17320_read_register(addr, &current);
    if (err != MAX17320_OK)
    {
        return err;
    }

    const uint16_t updated = (uint16_t)((current & (uint16_t)(~mask)) | (value & mask));
    if (updated == current)
    {
        return MAX17320_OK;
    }
    return max17320_write_register(addr, updated);
}

/* -------------------------------------------------------------------------
 * Write protection
 * ------------------------------------------------------------------------- */

/*
 * CommStat must be written twice in a row without any intervening register
 * access, otherwise the device ignores the change to the protection bits.
 */
static max17320_error_t max17320_write_comm_stat_twice(const uint16_t value)
{
    max17320_error_t err = max17320_write_register(MAX17320_REG_COMMSTAT, value);
    if (err != MAX17320_OK)
    {
        return err;
    }
    err = max17320_write_register(MAX17320_REG_COMMSTAT, value);
    return err;
}

max17320_error_t max17320_unlock_write_protection(void)
{
    uint16_t         comm_stat = 0U;
    max17320_error_t err       = max17320_write_comm_stat_twice(MAX17320_COMMSTAT_UNLOCKED);
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_read_register(MAX17320_REG_COMMSTAT, &comm_stat);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((comm_stat & MAX17320_COMMSTAT_WP_MASK) != 0U)
    {
        return MAX17320_ERR_UNLOCK_FAILED;
    }
    return MAX17320_OK;
}

max17320_error_t max17320_lock_write_protection(void)
{
    uint16_t         comm_stat = 0U;
    max17320_error_t err       = max17320_write_comm_stat_twice(MAX17320_COMMSTAT_LOCKED);
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_read_register(MAX17320_REG_COMMSTAT, &comm_stat);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((comm_stat & MAX17320_COMMSTAT_WP_MASK) != MAX17320_COMMSTAT_WP_MASK)
    {
        return MAX17320_ERR_LOCK_FAILED;
    }
    return MAX17320_OK;
}

/*
 * Unlock, apply one register write, then re-lock.  The lock result only
 * overrides a successful write so the caller sees the first real failure.
 *
 * @p verify selects the read-back check.  It must be suppressed for registers
 * the device itself updates (Status, the MaxMin logs), where a read-back
 * mismatch would be normal behaviour rather than a dropped write.
 */
static max17320_error_t
max17320_write_protected_ex(const uint16_t addr, const uint16_t value, const uint8_t verify)
{
    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    const max17320_error_t write_err = (verify != 0U)
                                           ? max17320_write_verify_register(addr, value)
                                           : max17320_write_register(addr, value);
    const max17320_error_t lock_err  = max17320_lock_write_protection();

    return (write_err != MAX17320_OK) ? write_err : lock_err;
}

static max17320_error_t max17320_write_protected(const uint16_t addr, const uint16_t value)
{
    return max17320_write_protected_ex(addr, value, 1U);
}

static max17320_error_t max17320_write_protected_volatile(const uint16_t addr,
                                                          const uint16_t value)
{
    return max17320_write_protected_ex(addr, value, 0U);
}

/* Read-modify-write of a single field, bracketed by unlock/lock. */
static max17320_error_t
max17320_update_protected(const uint16_t addr, const uint16_t mask, const uint16_t value)
{
    uint16_t               current = 0U;
    const max17320_error_t err     = max17320_read_register(addr, &current);
    if (err != MAX17320_OK)
    {
        return err;
    }

    const uint16_t updated = (uint16_t)((current & (uint16_t)(~mask)) | (value & mask));
    return max17320_write_protected(addr, updated);
}

max17320_error_t max17320_get_comm_status(max17320_comm_stat_t *status)
{
    uint16_t raw = 0U;

    if (status == (max17320_comm_stat_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_COMMSTAT, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    status->raw                  = raw;
    status->write_protect_global = ((raw & MAX17320_COMMSTAT_WPGLOBAL) != 0U) ? 1U : 0U;
    status->write_protect[0]     = ((raw & MAX17320_COMMSTAT_WP1) != 0U) ? 1U : 0U;
    status->write_protect[1]     = ((raw & MAX17320_COMMSTAT_WP2) != 0U) ? 1U : 0U;
    status->write_protect[2]     = ((raw & MAX17320_COMMSTAT_WP3) != 0U) ? 1U : 0U;
    status->write_protect[3]     = ((raw & MAX17320_COMMSTAT_WP4) != 0U) ? 1U : 0U;
    status->write_protect[4]     = ((raw & MAX17320_COMMSTAT_WP5) != 0U) ? 1U : 0U;
    status->nv_busy              = ((raw & MAX17320_COMMSTAT_NVBUSY) != 0U) ? 1U : 0U;
    status->nv_error             = ((raw & MAX17320_COMMSTAT_NVERROR) != 0U) ? 1U : 0U;
    status->chg_fet_off          = ((raw & MAX17320_COMMSTAT_CHGOFF) != 0U) ? 1U : 0U;
    status->dis_fet_off          = ((raw & MAX17320_COMMSTAT_DISOFF) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_get_lock_state(uint16_t *locks)
{
    if (locks == (uint16_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    return max17320_read_register(MAX17320_REG_LOCK, locks);
}

/* -------------------------------------------------------------------------
 * Commands, reset and nonvolatile memory
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_send_command(const uint16_t command)
{
    return max17320_write_register(MAX17320_REG_COMMAND, command);
}

/* Poll CommStat.NVBusy until it clears or the budget runs out. */
static max17320_error_t max17320_wait_nv_idle(const uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    for (;;)
    {
        uint16_t               comm_stat = 0U;
        const max17320_error_t err = max17320_read_register(MAX17320_REG_COMMSTAT, &comm_stat);
        if (err != MAX17320_OK)
        {
            return err;
        }
        if ((comm_stat & MAX17320_COMMSTAT_NVBUSY) == 0U)
        {
            return MAX17320_OK;
        }
        if (waited >= timeout_ms)
        {
            return MAX17320_ERR_NVM_BUSY_TIMEOUT;
        }

        max17320_delay_ms(MAX17320_POLL_INTERVAL_MS);
        waited += MAX17320_POLL_INTERVAL_MS;
    }
}

static max17320_error_t max17320_check_nv_error(void)
{
    uint16_t               comm_stat = 0U;
    const max17320_error_t err = max17320_read_register(MAX17320_REG_COMMSTAT, &comm_stat);
    if (err != MAX17320_OK)
    {
        return err;
    }
    return ((comm_stat & MAX17320_COMMSTAT_NVERROR) != 0U) ? MAX17320_ERR_NVM_ERROR : MAX17320_OK;
}

/* Poll Config2.POR_CMD until the firmware restart completes. */
static max17320_error_t max17320_wait_por_complete(const uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    for (;;)
    {
        uint16_t               config2 = 0U;
        const max17320_error_t err = max17320_read_register(MAX17320_REG_CONFIG2, &config2);
        if (err != MAX17320_OK)
        {
            return err;
        }
        if ((config2 & MAX17320_CONFIG2_POR_CMD) == 0U)
        {
            return MAX17320_OK;
        }
        if (waited >= timeout_ms)
        {
            return MAX17320_ERR_POR_TIMEOUT;
        }

        max17320_delay_ms(MAX17320_POLL_INTERVAL_MS);
        waited += MAX17320_POLL_INTERVAL_MS;
    }
}

/* Steps shared by the FULL RESET tail and the block-copy tail. */
static max17320_error_t max17320_restart_firmware(void)
{
    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_write_register(MAX17320_REG_CONFIG2, MAX17320_CMD_FUEL_GAUGE_RESET);
    if (err != MAX17320_OK)
    {
        (void)max17320_lock_write_protection();
        return err;
    }

    const max17320_error_t por_err  = max17320_wait_por_complete(MAX17320_T_BLOCK_MAX_MS);
    const max17320_error_t lock_err = max17320_lock_write_protection();

    return (por_err != MAX17320_OK) ? por_err : lock_err;
}

max17320_error_t max17320_full_reset(void)
{
    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_send_command(MAX17320_CMD_HARDWARE_RESET);
    if (err != MAX17320_OK)
    {
        (void)max17320_lock_write_protection();
        return err;
    }

    /* Write protection re-arms itself across the hardware reset. */
    max17320_delay_ms(MAX17320_T_POR_MS);

    return max17320_restart_firmware();
}

max17320_error_t max17320_fuel_gauge_reset(void) { return max17320_restart_firmware(); }

max17320_error_t max17320_nv_recall(void)
{
    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* A third unlock write clears any stale NVError before the command. */
    err = max17320_write_register(MAX17320_REG_COMMSTAT, MAX17320_COMMSTAT_UNLOCKED);
    if (err == MAX17320_OK)
    {
        err = max17320_send_command(MAX17320_CMD_NV_RECALL);
    }
    if (err != MAX17320_OK)
    {
        (void)max17320_lock_write_protection();
        return err;
    }

    max17320_delay_ms(MAX17320_T_RECALL_MS);

    max17320_error_t result = max17320_wait_nv_idle(MAX17320_T_UPDATE_MAX_MS);
    if (result == MAX17320_OK)
    {
        result = max17320_check_nv_error();
        if (result == MAX17320_ERR_NVM_ERROR)
        {
            result = MAX17320_ERR_NVM_RECALL_FAILED;
        }
    }

    const max17320_error_t lock_err = max17320_lock_write_protection();
    return (result != MAX17320_OK) ? result : lock_err;
}

max17320_error_t max17320_get_remaining_nvm_updates(uint8_t *remaining)
{
    uint16_t flags = 0U;
    uint8_t  used  = 0U;
    uint8_t  bit   = 0U;

    if (remaining == (uint8_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_send_command(MAX17320_CMD_HISTORY_CFG_UPDATES);
    if (err == MAX17320_OK)
    {
        max17320_delay_ms(MAX17320_T_RECALL_MS);
        err = max17320_wait_nv_idle(MAX17320_T_UPDATE_MAX_MS);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_read_register(MAX17320_REG_HISTORY_UPDATES, &flags);
    }

    const max17320_error_t lock_err = max17320_lock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }
    if (lock_err != MAX17320_OK)
    {
        return lock_err;
    }

    /* Each write sets one redundant flag in both bytes; OR then count them. */
    const uint8_t merged = (uint8_t)((flags >> 8U) | (flags & 0x00FFU));
    for (bit = 0U; bit < 8U; bit++)
    {
        if ((merged & (uint8_t)(1U << bit)) != 0U)
        {
            used++;
        }
    }

    *remaining = (used >= MAX17320_NVM_MAX_UPDATES)
                     ? 0U
                     : (uint8_t)(MAX17320_NVM_MAX_UPDATES - used);
    return MAX17320_OK;
}

max17320_error_t max17320_nv_block_copy(const uint8_t check_budget)
{
    if (check_budget != 0U)
    {
        uint8_t                remaining = 0U;
        const max17320_error_t err       = max17320_get_remaining_nvm_updates(&remaining);
        if (err != MAX17320_OK)
        {
            return err;
        }
        if (remaining == 0U)
        {
            return MAX17320_ERR_NVM_NO_UPDATES;
        }
    }

    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* Third unlock write clears CommStat.NVError before the copy. */
    err = max17320_write_register(MAX17320_REG_COMMSTAT, MAX17320_COMMSTAT_UNLOCKED);
    if (err == MAX17320_OK)
    {
        err = max17320_send_command(MAX17320_CMD_COPY_NV_BLOCK);
    }
    if (err != MAX17320_OK)
    {
        (void)max17320_lock_write_protection();
        return err;
    }

    max17320_error_t result = max17320_wait_nv_idle(MAX17320_T_BLOCK_MAX_MS);
    if (result == MAX17320_OK)
    {
        result = max17320_check_nv_error();
    }
    if (result != MAX17320_OK)
    {
        (void)max17320_lock_write_protection();
        return result;
    }

    /* The new nonvolatile contents only take effect after a full reset. */
    return max17320_full_reset();
}

max17320_error_t max17320_lock_memory_blocks(const uint16_t lock_mask)
{
    const uint16_t selected = (uint16_t)(lock_mask & 0x001FU);

    if (selected == 0U)
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_write_register(MAX17320_REG_COMMSTAT, MAX17320_COMMSTAT_UNLOCKED);
    if (err == MAX17320_OK)
    {
        err = max17320_send_command((uint16_t)(MAX17320_CMD_NV_LOCK_BASE | selected));
    }
    if (err != MAX17320_OK)
    {
        (void)max17320_lock_write_protection();
        return err;
    }

    max17320_delay_ms(MAX17320_T_UPDATE_MAX_MS);

    max17320_error_t result = max17320_wait_nv_idle(MAX17320_T_UPDATE_MAX_MS);
    if (result == MAX17320_OK)
    {
        result = max17320_check_nv_error();
    }

    const max17320_error_t lock_err = max17320_lock_write_protection();
    return (result != MAX17320_OK) ? result : lock_err;
}

/* -------------------------------------------------------------------------
 * Unit conversion helpers
 * ------------------------------------------------------------------------- */

static max17320_error_t max17320_sense(float *ohms)
{
    if (s_r_sense_ohms <= 0.0F)
    {
        return MAX17320_ERR_NOT_INITIALISED;
    }
    *ohms = s_r_sense_ohms;
    return MAX17320_OK;
}

static int16_t max17320_as_signed(const uint16_t raw) { return (int16_t)raw; }

static int8_t max17320_as_signed8(const uint8_t raw) { return (int8_t)raw; }

/* Read an unsigned register and scale it by a fixed LSB weight. */
static max17320_error_t
max17320_read_scaled(const uint16_t addr, const float lsb, float *out)
{
    uint16_t raw = 0U;

    if (out == (float *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(addr, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *out = (float)raw * lsb;
    return MAX17320_OK;
}

/* Read a two's-complement register and scale it by a fixed LSB weight. */
static max17320_error_t
max17320_read_scaled_signed(const uint16_t addr, const float lsb, float *out)
{
    uint16_t raw = 0U;

    if (out == (float *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(addr, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *out = (float)max17320_as_signed(raw) * lsb;
    return MAX17320_OK;
}

/* Read a register whose LSB weight is divided by the sense resistance. */
static max17320_error_t
max17320_read_sensed(const uint16_t addr, const float lsb_vr, const uint8_t is_signed, float *out)
{
    float    r_sense = 0.0F;
    uint16_t raw     = 0U;

    if (out == (float *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    max17320_error_t err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_read_register(addr, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    const float counts = (is_signed != 0U) ? (float)max17320_as_signed(raw) : (float)raw;
    *out               = counts * lsb_vr / r_sense;
    return MAX17320_OK;
}

/* Clamp a float to an inclusive integer range and round to nearest. */
static int32_t max17320_clamp_round(const float value, const int32_t lo, const int32_t hi)
{
    const float rounded = (value >= 0.0F) ? (value + 0.5F) : (value - 0.5F);
    int32_t     result  = (int32_t)rounded;

    if (result < lo)
    {
        result = lo;
    }
    if (result > hi)
    {
        result = hi;
    }
    return result;
}

/* -------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_set_sense_resistor(const float ohms)
{
    if (ohms <= 0.0F)
    {
        return MAX17320_ERR_BAD_SENSE_RESISTOR;
    }
    s_r_sense_ohms = ohms;
    return MAX17320_OK;
}

max17320_error_t max17320_get_sense_resistor(float *ohms)
{
    if (ohms == (float *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    *ohms = s_r_sense_ohms;
    return MAX17320_OK;
}

max17320_error_t max17320_get_device_name(uint16_t *dev_name)
{
    if (dev_name == (uint16_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    return max17320_read_register(MAX17320_REG_DEVNAME, dev_name);
}

max17320_error_t max17320_get_cell_count(uint8_t *cells)
{
    uint16_t pack_cfg = 0U;

    if (cells == (uint8_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_NPACKCFG, &pack_cfg);
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* NCELLS is stored as cellcount - 2. */
    *cells = (uint8_t)((pack_cfg & MAX17320_NPACKCFG_NCELLS_MASK) + 2U);
    return MAX17320_OK;
}

max17320_error_t max17320_get_thermistor_count(uint8_t *thermistors)
{
    uint16_t pack_cfg = 0U;

    if (thermistors == (uint8_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_NPACKCFG, &pack_cfg);
    if (err != MAX17320_OK)
    {
        return err;
    }

    uint8_t count =
        (uint8_t)((pack_cfg & MAX17320_NPACKCFG_NTHRMS_MASK) >> MAX17320_NPACKCFG_NTHRMS_SHIFT);
    if (count > 4U)
    {
        count = 4U;
    }
    *thermistors = count;
    return MAX17320_OK;
}

max17320_error_t max17320_init(const max17320_config_t *cfg)
{
    uint16_t dev_name = 0U;
    uint8_t  cells    = 0U;

    if (cfg == (const max17320_config_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    if (cfg->r_sense_ohms <= 0.0F)
    {
        return MAX17320_ERR_BAD_SENSE_RESISTOR;
    }
    if ((cfg->cell_count != 0U) && ((cfg->cell_count < 2U) || (cfg->cell_count > 4U)))
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    max17320_error_t err = max17320_read_register(MAX17320_REG_DEVNAME, &dev_name);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((dev_name & MAX17320_DEVNAME_REVISION_MASK) != MAX17320_DEVNAME_REVISION)
    {
        return MAX17320_ERR_DEVICE_NAME;
    }

    if (cfg->cell_count != 0U)
    {
        cells = cfg->cell_count;
    }
    else
    {
        err = max17320_get_cell_count(&cells);
        if (err != MAX17320_OK)
        {
            return err;
        }
        if ((cells < 2U) || (cells > 4U))
        {
            return MAX17320_ERR_BAD_PARAM;
        }
    }

    s_r_sense_ohms = cfg->r_sense_ohms;
    s_cell_count   = cells;
    return MAX17320_OK;
}

max17320_error_t max17320_get_rom_id(uint64_t *rom_id)
{
    uint16_t words[4] = {0U, 0U, 0U, 0U};

    if (rom_id == (uint64_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_registers(MAX17320_REG_NROMID0, words, 4U);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *rom_id = ((uint64_t)words[3] << 48U) | ((uint64_t)words[2] << 32U) |
              ((uint64_t)words[1] << 16U) | (uint64_t)words[0];
    return MAX17320_OK;
}

/* -------------------------------------------------------------------------
 * Status
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_status(max17320_status_t *status)
{
    uint16_t raw = 0U;

    if (status == (max17320_status_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_STATUS, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    status->raw              = raw;
    status->power_on_reset   = ((raw & MAX17320_STATUS_POR) != 0U) ? 1U : 0U;
    status->current_min      = ((raw & MAX17320_STATUS_IMN) != 0U) ? 1U : 0U;
    status->current_max      = ((raw & MAX17320_STATUS_IMX) != 0U) ? 1U : 0U;
    status->soc_change       = ((raw & MAX17320_STATUS_DSOCI) != 0U) ? 1U : 0U;
    status->voltage_min      = ((raw & MAX17320_STATUS_VMN) != 0U) ? 1U : 0U;
    status->temp_min         = ((raw & MAX17320_STATUS_TMN) != 0U) ? 1U : 0U;
    status->soc_min          = ((raw & MAX17320_STATUS_SMN) != 0U) ? 1U : 0U;
    status->voltage_max      = ((raw & MAX17320_STATUS_VMX) != 0U) ? 1U : 0U;
    status->temp_max         = ((raw & MAX17320_STATUS_TMX) != 0U) ? 1U : 0U;
    status->soc_max          = ((raw & MAX17320_STATUS_SMX) != 0U) ? 1U : 0U;
    status->protection_alert = ((raw & MAX17320_STATUS_PA) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_get_protection_status(max17320_prot_status_t *status)
{
    uint16_t raw = 0U;

    if (status == (max17320_prot_status_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_PROTSTATUS, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    status->raw                   = raw;
    status->ship                  = ((raw & MAX17320_PROT_SHIP) != 0U) ? 1U : 0U;
    status->resd_fault            = ((raw & MAX17320_PROT_RESDFAULT) != 0U) ? 1U : 0U;
    status->overdischarge_current = ((raw & MAX17320_PROT_ODCP) != 0U) ? 1U : 0U;
    status->undervoltage          = ((raw & MAX17320_PROT_UVP) != 0U) ? 1U : 0U;
    status->too_hot_discharge     = ((raw & MAX17320_PROT_TOOHOTD) != 0U) ? 1U : 0U;
    status->die_hot               = ((raw & MAX17320_PROT_DIEHOT) != 0U) ? 1U : 0U;
    status->permanent_fail        = ((raw & MAX17320_PROT_PERMFAIL) != 0U) ? 1U : 0U;
    status->imbalance             = ((raw & MAX17320_PROT_IMBALANCE) != 0U) ? 1U : 0U;
    status->prequal_timeout       = ((raw & MAX17320_PROT_PREQF) != 0U) ? 1U : 0U;
    status->capacity_overflow     = ((raw & MAX17320_PROT_QOVFLW) != 0U) ? 1U : 0U;
    status->overcharge_current    = ((raw & MAX17320_PROT_OCCP) != 0U) ? 1U : 0U;
    status->overvoltage           = ((raw & MAX17320_PROT_OVP) != 0U) ? 1U : 0U;
    status->too_cold_charge       = ((raw & MAX17320_PROT_TOOCOLDC) != 0U) ? 1U : 0U;
    status->full                  = ((raw & MAX17320_PROT_FULL) != 0U) ? 1U : 0U;
    status->too_hot_charge        = ((raw & MAX17320_PROT_TOOHOTC) != 0U) ? 1U : 0U;
    status->charge_watchdog       = ((raw & MAX17320_PROT_CHGWDT) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_get_protection_alert(max17320_prot_alert_t *alert)
{
    uint16_t raw = 0U;

    if (alert == (max17320_prot_alert_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_PROTALRT, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    alert->raw                   = raw;
    alert->leak_detect           = ((raw & MAX17320_PROT_LDET) != 0U) ? 1U : 0U;
    alert->resd_fault            = ((raw & MAX17320_PROT_RESDFAULT) != 0U) ? 1U : 0U;
    alert->overdischarge_current = ((raw & MAX17320_PROT_ODCP) != 0U) ? 1U : 0U;
    alert->undervoltage          = ((raw & MAX17320_PROT_UVP) != 0U) ? 1U : 0U;
    alert->too_hot_discharge     = ((raw & MAX17320_PROT_TOOHOTD) != 0U) ? 1U : 0U;
    alert->die_hot               = ((raw & MAX17320_PROT_DIEHOT) != 0U) ? 1U : 0U;
    alert->permanent_fail        = ((raw & MAX17320_PROT_PERMFAIL) != 0U) ? 1U : 0U;
    alert->imbalance             = ((raw & MAX17320_PROT_IMBALANCE) != 0U) ? 1U : 0U;
    alert->prequal_timeout       = ((raw & MAX17320_PROT_PREQF) != 0U) ? 1U : 0U;
    alert->capacity_overflow     = ((raw & MAX17320_PROT_QOVFLW) != 0U) ? 1U : 0U;
    alert->overcharge_current    = ((raw & MAX17320_PROT_OCCP) != 0U) ? 1U : 0U;
    alert->overvoltage           = ((raw & MAX17320_PROT_OVP) != 0U) ? 1U : 0U;
    alert->too_cold_charge       = ((raw & MAX17320_PROT_TOOCOLDC) != 0U) ? 1U : 0U;
    alert->full                  = ((raw & MAX17320_PROT_FULL) != 0U) ? 1U : 0U;
    alert->too_hot_charge        = ((raw & MAX17320_PROT_TOOHOTC) != 0U) ? 1U : 0U;
    alert->charge_watchdog       = ((raw & MAX17320_PROT_CHGWDT) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_get_battery_status(max17320_batt_status_t *status)
{
    uint16_t raw     = 0U;
    float    r_sense = 0.0F;

    if (status == (max17320_batt_status_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    max17320_error_t err = max17320_read_register(MAX17320_REG_NBATTSTATUS, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    status->raw                  = raw;
    status->permanent_fail       = ((raw & MAX17320_BATTSTAT_PERMFAIL) != 0U) ? 1U : 0U;
    status->overvoltage_fail     = ((raw & MAX17320_BATTSTAT_OVPF) != 0U) ? 1U : 0U;
    status->overtemperature_fail = ((raw & MAX17320_BATTSTAT_OTPF) != 0U) ? 1U : 0U;
    status->chg_fet_short        = ((raw & MAX17320_BATTSTAT_CFETFS) != 0U) ? 1U : 0U;
    status->dis_fet_short        = ((raw & MAX17320_BATTSTAT_DFETFS) != 0U) ? 1U : 0U;
    status->fet_open             = ((raw & MAX17320_BATTSTAT_FETFO) != 0U) ? 1U : 0U;
    status->leak_detect          = ((raw & MAX17320_BATTSTAT_LDET) != 0U) ? 1U : 0U;
    status->checksum_or_uvpf     = ((raw & MAX17320_BATTSTAT_CHKSUMF_UVPF) != 0U) ? 1U : 0U;

    err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        status->leak_current_a = 0.0F;
        return err;
    }
    status->leak_current_a =
        (float)(raw & MAX17320_BATTSTAT_LEAKCURR_MASK) * MAX17320_LSB_LEAKCURR_STAT_VR / r_sense;
    return MAX17320_OK;
}

max17320_error_t max17320_get_fuel_gauge_status(max17320_fstat_t *status)
{
    uint16_t raw = 0U;

    if (status == (max17320_fstat_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_FSTAT, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    status->raw            = raw;
    status->data_not_ready = ((raw & MAX17320_FSTAT_DNR) != 0U) ? 1U : 0U;
    status->empty_detect   = ((raw & MAX17320_FSTAT_EDET) != 0U) ? 1U : 0U;
    status->relaxed        = ((raw & MAX17320_FSTAT_RELDT) != 0U) ? 1U : 0U;
    status->long_relaxed   = ((raw & MAX17320_FSTAT_RELDT2) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_get_fault_log(uint16_t *faults)
{
    if (faults == (uint16_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    return max17320_read_register(MAX17320_REG_NFAULTLOG, faults);
}

max17320_error_t max17320_clear_por(void)
{
    uint16_t               raw = 0U;
    const max17320_error_t err = max17320_read_register(MAX17320_REG_STATUS, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((raw & MAX17320_STATUS_POR) == 0U)
    {
        return MAX17320_OK;
    }
    return max17320_write_protected_volatile(MAX17320_REG_STATUS,
                                             (uint16_t)(raw & (uint16_t)(~MAX17320_STATUS_POR)));
}

max17320_error_t max17320_clear_protection_alert(void)
{
    uint16_t status = 0U;

    max17320_error_t err = max17320_unlock_write_protection();
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* ProtAlrt must reach 0000h before Status.PA may be cleared. */
    err = max17320_write_verify_register(MAX17320_REG_PROTALRT, 0x0000U);
    if (err == MAX17320_OK)
    {
        err = max17320_read_register(MAX17320_REG_STATUS, &status);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_write_register(MAX17320_REG_STATUS,
                                      (uint16_t)(status & (uint16_t)(~MAX17320_STATUS_PA)));
    }

    const max17320_error_t lock_err = max17320_lock_write_protection();
    return (err != MAX17320_OK) ? err : lock_err;
}

max17320_error_t max17320_check_health(void)
{
    uint16_t batt_status = 0U;
    uint16_t status      = 0U;

    max17320_error_t err = max17320_read_register(MAX17320_REG_NBATTSTATUS, &batt_status);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((batt_status & MAX17320_BATTSTAT_PERMFAIL) != 0U)
    {
        return MAX17320_ERR_PERMANENT_FAIL;
    }

    err = max17320_read_register(MAX17320_REG_STATUS, &status);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((status & MAX17320_STATUS_PA) != 0U)
    {
        return MAX17320_ERR_PROTECTION_ALERT;
    }
    return MAX17320_OK;
}

max17320_error_t max17320_is_hibernating(uint8_t *hibernating)
{
    uint16_t raw = 0U;

    if (hibernating == (uint8_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_STATUS2, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *hibernating = ((raw & MAX17320_STATUS2_HIB) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_wait_data_ready(const uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    for (;;)
    {
        uint16_t               raw = 0U;
        const max17320_error_t err = max17320_read_register(MAX17320_REG_FSTAT, &raw);
        if (err != MAX17320_OK)
        {
            return err;
        }
        if ((raw & MAX17320_FSTAT_DNR) == 0U)
        {
            return MAX17320_OK;
        }
        if (waited >= timeout_ms)
        {
            return MAX17320_ERR_FG_NOT_READY;
        }

        max17320_delay_ms(MAX17320_POLL_INTERVAL_MS);
        waited += MAX17320_POLL_INTERVAL_MS;
    }
}

/* -------------------------------------------------------------------------
 * Voltage
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_cell_voltage(const uint8_t cell, float *volts)
{
    if ((cell < 1U) || (cell > 4U))
    {
        return MAX17320_ERR_BAD_CELL_INDEX;
    }
    /* Cell1 is the highest address of the descending Cell4..Cell1 block. */
    return max17320_read_scaled(
        (uint16_t)(MAX17320_REG_CELL1 + 1U - cell), MAX17320_LSB_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_avg_cell_voltage(const uint8_t cell, float *volts)
{
    if ((cell < 1U) || (cell > 4U))
    {
        return MAX17320_ERR_BAD_CELL_INDEX;
    }
    return max17320_read_scaled(
        (uint16_t)(MAX17320_REG_AVGCELL1 + 1U - cell), MAX17320_LSB_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_vcell(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_VCELL, MAX17320_LSB_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_avg_vcell(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_AVGVCELL, MAX17320_LSB_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_pack_voltage(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_BATT, MAX17320_LSB_PACK_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_pckp_voltage(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_PCKP, MAX17320_LSB_PACK_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_open_circuit_voltage(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_VFOCV, MAX17320_LSB_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_ripple_voltage(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_VRIPPLE, MAX17320_LSB_VRIPPLE_V, volts);
}

max17320_error_t max17320_get_max_min_voltage(float *max_volts, float *min_volts)
{
    uint16_t raw = 0U;

    if ((max_volts == (float *)0) || (min_volts == (float *)0))
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_MAXMINVOLT, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *max_volts = (float)(uint8_t)(raw >> 8U) * MAX17320_LSB_MAXMIN_VOLT_V;
    *min_volts = (float)(uint8_t)(raw & 0x00FFU) * MAX17320_LSB_MAXMIN_VOLT_V;
    return MAX17320_OK;
}

max17320_error_t max17320_reset_max_min_voltage(void)
{
    return max17320_write_protected_volatile(MAX17320_REG_MAXMINVOLT, MAX17320_MAXMINVOLT_RESET);
}

/* -------------------------------------------------------------------------
 * Current
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_current(float *amps)
{
    return max17320_read_sensed(MAX17320_REG_CURRENT, MAX17320_LSB_CURRENT_VR, 1U, amps);
}

max17320_error_t max17320_get_avg_current(float *amps)
{
    return max17320_read_sensed(MAX17320_REG_AVGCURRENT, MAX17320_LSB_CURRENT_VR, 1U, amps);
}

max17320_error_t max17320_get_max_min_current(float *max_amps, float *min_amps)
{
    uint16_t raw     = 0U;
    float    r_sense = 0.0F;

    if ((max_amps == (float *)0) || (min_amps == (float *)0))
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    max17320_error_t err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_read_register(MAX17320_REG_MAXMINCURR, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *max_amps = (float)max17320_as_signed8((uint8_t)(raw >> 8U)) * MAX17320_LSB_MAXMIN_CURR_VR /
                r_sense;
    *min_amps = (float)max17320_as_signed8((uint8_t)(raw & 0x00FFU)) *
                MAX17320_LSB_MAXMIN_CURR_VR / r_sense;
    return MAX17320_OK;
}

max17320_error_t max17320_reset_max_min_current(void)
{
    return max17320_write_protected_volatile(MAX17320_REG_MAXMINCURR, MAX17320_MAXMINCURR_RESET);
}

/* -------------------------------------------------------------------------
 * Temperature
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_temperature(float *celsius)
{
    return max17320_read_scaled_signed(MAX17320_REG_TEMP, MAX17320_LSB_TEMPERATURE_C, celsius);
}

max17320_error_t max17320_get_avg_temperature(float *celsius)
{
    return max17320_read_scaled_signed(MAX17320_REG_AVGTA, MAX17320_LSB_TEMPERATURE_C, celsius);
}

max17320_error_t max17320_get_die_temperature(float *celsius)
{
    return max17320_read_scaled_signed(MAX17320_REG_DIETEMP, MAX17320_LSB_TEMPERATURE_C, celsius);
}

max17320_error_t max17320_get_avg_die_temperature(float *celsius)
{
    return max17320_read_scaled_signed(
        MAX17320_REG_AVGDIETEMP, MAX17320_LSB_TEMPERATURE_C, celsius);
}

max17320_error_t max17320_get_thermistor_temperature(const uint8_t channel, float *celsius)
{
    if ((channel < 1U) || (channel > 4U))
    {
        return MAX17320_ERR_BAD_THERM_INDEX;
    }
    /* Temp1..Temp4 occupy 13Ah down to 137h. */
    return max17320_read_scaled_signed(
        (uint16_t)(MAX17320_REG_TEMP1 + 1U - channel), MAX17320_LSB_TEMPERATURE_C, celsius);
}

max17320_error_t max17320_get_avg_thermistor_temperature(const uint8_t channel, float *celsius)
{
    if ((channel < 1U) || (channel > 4U))
    {
        return MAX17320_ERR_BAD_THERM_INDEX;
    }
    return max17320_read_scaled_signed(
        (uint16_t)(MAX17320_REG_AVGTEMP1 + 1U - channel), MAX17320_LSB_TEMPERATURE_C, celsius);
}

max17320_error_t max17320_get_max_min_temperature(float *max_celsius, float *min_celsius)
{
    uint16_t raw = 0U;

    if ((max_celsius == (float *)0) || (min_celsius == (float *)0))
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_MAXMINTEMP, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    *max_celsius = (float)max17320_as_signed8((uint8_t)(raw >> 8U));
    *min_celsius = (float)max17320_as_signed8((uint8_t)(raw & 0x00FFU));
    return MAX17320_OK;
}

max17320_error_t max17320_reset_max_min_temperature(void)
{
    return max17320_write_protected_volatile(MAX17320_REG_MAXMINTEMP, MAX17320_MAXMINTEMP_RESET);
}

/* -------------------------------------------------------------------------
 * Power
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_power(float *watts)
{
    return max17320_read_sensed(MAX17320_REG_POWER, MAX17320_LSB_POWER_VVR, 1U, watts);
}

max17320_error_t max17320_get_avg_power(float *watts)
{
    return max17320_read_sensed(MAX17320_REG_AVGPOWER, MAX17320_LSB_POWER_VVR, 1U, watts);
}

/* -------------------------------------------------------------------------
 * Capacity, state of charge and timing
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_reported_capacity(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_REPCAP, MAX17320_LSB_CAPACITY_VHR, 0U, amp_hours);
}

max17320_error_t max17320_get_full_capacity(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_FULLCAPREP, MAX17320_LSB_CAPACITY_VHR, 0U, amp_hours);
}

max17320_error_t max17320_get_full_capacity_nominal(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_FULLCAPNOM, MAX17320_LSB_CAPACITY_VHR, 0U, amp_hours);
}

max17320_error_t max17320_get_design_capacity(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_DESIGNCAP, MAX17320_LSB_CAPACITY_VHR, 0U, amp_hours);
}

max17320_error_t max17320_get_available_capacity(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_AVCAP, MAX17320_LSB_CAPACITY_VHR, 0U, amp_hours);
}

max17320_error_t max17320_get_mix_capacity(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_MIXCAP, MAX17320_LSB_CAPACITY_VHR, 0U, amp_hours);
}

max17320_error_t max17320_get_residual_capacity(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_QRESIDUAL, MAX17320_LSB_CAPACITY_VHR, 1U, amp_hours);
}

max17320_error_t max17320_get_coulomb_count(float *amp_hours)
{
    return max17320_read_sensed(MAX17320_REG_QH, MAX17320_LSB_CAPACITY_VHR, 1U, amp_hours);
}

max17320_error_t max17320_get_reported_soc(float *percent)
{
    return max17320_read_scaled(MAX17320_REG_REPSOC, MAX17320_LSB_PERCENT, percent);
}

max17320_error_t max17320_get_available_soc(float *percent)
{
    return max17320_read_scaled(MAX17320_REG_AVSOC, MAX17320_LSB_PERCENT, percent);
}

max17320_error_t max17320_get_mix_soc(float *percent)
{
    return max17320_read_scaled(MAX17320_REG_MIXSOC, MAX17320_LSB_PERCENT, percent);
}

max17320_error_t max17320_get_vf_soc(float *percent)
{
    return max17320_read_scaled(MAX17320_REG_VFSOC, MAX17320_LSB_PERCENT, percent);
}

max17320_error_t max17320_get_age(float *percent)
{
    return max17320_read_scaled(MAX17320_REG_AGE, MAX17320_LSB_PERCENT, percent);
}

max17320_error_t max17320_get_time_to_empty(float *seconds)
{
    return max17320_read_scaled(MAX17320_REG_TTE, MAX17320_LSB_TIME_S, seconds);
}

max17320_error_t max17320_get_time_to_full(float *seconds)
{
    return max17320_read_scaled(MAX17320_REG_TTF, MAX17320_LSB_TIME_S, seconds);
}

max17320_error_t max17320_get_cycles(float *cycles)
{
    return max17320_read_scaled(MAX17320_REG_CYCLES, MAX17320_LSB_CYCLES, cycles);
}

max17320_error_t max17320_get_age_timer(float *seconds)
{
    return max17320_read_scaled(MAX17320_REG_TIMERH, MAX17320_LSB_TIMERH_S, seconds);
}

max17320_error_t max17320_get_cell_resistance(float *ohms)
{
    return max17320_read_scaled(MAX17320_REG_RCELL, MAX17320_LSB_RESISTANCE_OHM, ohms);
}

max17320_error_t max17320_get_leakage_current(float *amps)
{
    float    r_sense = 0.0F;
    uint16_t raw     = 0U;

    if (amps == (float *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    max17320_error_t err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        return err;
    }

    err = max17320_read_register(MAX17320_REG_LEAKCURRREP, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* Bit 15 is always zero; the reported value is a 15-bit unsigned count. */
    *amps = (float)(raw & 0x7FFFU) * MAX17320_LSB_LEAKCURR_REP_VR / r_sense;
    return MAX17320_OK;
}

/* -------------------------------------------------------------------------
 * Charging prescription
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_charging_voltage(float *volts)
{
    return max17320_read_scaled(MAX17320_REG_CHARGINGVOLTAGE, MAX17320_LSB_VOLTAGE_V, volts);
}

max17320_error_t max17320_get_charging_current(float *amps)
{
    return max17320_read_sensed(MAX17320_REG_CHARGINGCURRENT, MAX17320_LSB_CURRENT_VR, 0U, amps);
}

/* -------------------------------------------------------------------------
 * At-rate estimation
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_estimate_at_rate(const float load_amps,
                                           float      *time_to_empty_s,
                                           float      *soc_percent,
                                           float      *capacity_ah)
{
    float r_sense = 0.0F;

    max17320_error_t err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        return err;
    }

    const float   counts = (load_amps * r_sense) / MAX17320_LSB_CURRENT_VR;
    const int32_t raw    = max17320_clamp_round(counts, -32768, 32767);

    err = max17320_write_protected(MAX17320_REG_ATRATE, (uint16_t)(int16_t)raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    /* The at-rate outputs need two full task periods to settle. */
    max17320_delay_ms((uint32_t)MAX17320_T_TASK_PERIOD_MS * 2U);

    if (time_to_empty_s != (float *)0)
    {
        err = max17320_read_scaled(MAX17320_REG_ATTTE, MAX17320_LSB_TIME_S, time_to_empty_s);
        if (err != MAX17320_OK)
        {
            return err;
        }
    }
    if (soc_percent != (float *)0)
    {
        err = max17320_read_scaled(MAX17320_REG_ATAVSOC, MAX17320_LSB_PERCENT, soc_percent);
        if (err != MAX17320_OK)
        {
            return err;
        }
    }
    if (capacity_ah != (float *)0)
    {
        err = max17320_read_sensed(
            MAX17320_REG_ATAVCAP, MAX17320_LSB_CAPACITY_VHR, 0U, capacity_ah);
        if (err != MAX17320_OK)
        {
            return err;
        }
    }
    return MAX17320_OK;
}

/* -------------------------------------------------------------------------
 * Alert thresholds
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_set_voltage_alert(const float min_volts, const float max_volts)
{
    if ((min_volts < 0.0F) || (max_volts < min_volts))
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    const int32_t vmin = max17320_clamp_round(min_volts / MAX17320_LSB_ALERT_VOLT_V, 0, 255);
    const int32_t vmax = max17320_clamp_round(max_volts / MAX17320_LSB_ALERT_VOLT_V, 0, 255);

    return max17320_write_protected(MAX17320_REG_VALRTTH,
                                    (uint16_t)(((uint16_t)vmax << 8U) | (uint16_t)vmin));
}

max17320_error_t max17320_set_temperature_alert(const float min_celsius, const float max_celsius)
{
    if (max_celsius < min_celsius)
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    const int32_t tmin = max17320_clamp_round(min_celsius, -128, 127);
    const int32_t tmax = max17320_clamp_round(max_celsius, -128, 127);

    return max17320_write_protected(
        MAX17320_REG_TALRTTH,
        (uint16_t)((((uint16_t)(uint8_t)(int8_t)tmax) << 8U) | (uint16_t)(uint8_t)(int8_t)tmin));
}

max17320_error_t max17320_set_soc_alert(const float min_percent, const float max_percent)
{
    if ((min_percent < 0.0F) || (max_percent < min_percent))
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    const int32_t smin = max17320_clamp_round(min_percent, 0, 255);
    const int32_t smax = max17320_clamp_round(max_percent, 0, 255);

    return max17320_write_protected(MAX17320_REG_SALRTTH,
                                    (uint16_t)(((uint16_t)smax << 8U) | (uint16_t)smin));
}

max17320_error_t max17320_set_current_alert(const float min_amps, const float max_amps)
{
    float r_sense = 0.0F;

    if (max_amps < min_amps)
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    const max17320_error_t err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        return err;
    }

    const float   scale = MAX17320_LSB_ALERT_CURR_VR / r_sense;
    const int32_t imin  = max17320_clamp_round(min_amps / scale, -128, 127);
    const int32_t imax  = max17320_clamp_round(max_amps / scale, -128, 127);

    return max17320_write_protected(
        MAX17320_REG_IALRTTH,
        (uint16_t)((((uint16_t)(uint8_t)(int8_t)imax) << 8U) | (uint16_t)(uint8_t)(int8_t)imin));
}

max17320_error_t max17320_set_alerts_enabled(const uint8_t alerts_enabled,
                                             const uint8_t protection_enabled)
{
    const uint16_t mask = (uint16_t)(MAX17320_CONFIG_AEN | MAX17320_CONFIG_PAEN);
    uint16_t       value = 0U;

    if (alerts_enabled != 0U)
    {
        value |= MAX17320_CONFIG_AEN;
    }
    if (protection_enabled != 0U)
    {
        value |= MAX17320_CONFIG_PAEN;
    }

    return max17320_update_protected(MAX17320_REG_CONFIG, mask, value);
}

/* -------------------------------------------------------------------------
 * FET and power-mode control
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_get_fet_state(max17320_fet_state_t *state)
{
    uint16_t raw = 0U;

    if (state == (max17320_fet_state_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }

    const max17320_error_t err = max17320_read_register(MAX17320_REG_HPROTCFG2, &raw);
    if (err != MAX17320_OK)
    {
        return err;
    }

    state->raw                      = raw;
    state->chg_fet_on               = ((raw & MAX17320_HPROTCFG2_CHGS) != 0U) ? 1U : 0U;
    state->dis_fet_on               = ((raw & MAX17320_HPROTCFG2_DISS) != 0U) ? 1U : 0U;
    state->pushbutton_enabled       = ((raw & MAX17320_HPROTCFG2_PBEN) != 0U) ? 1U : 0U;
    state->command_override_enabled = ((raw & MAX17320_HPROTCFG2_COMMOVRD) != 0U) ? 1U : 0U;
    return MAX17320_OK;
}

max17320_error_t max17320_set_fet_override(const uint8_t chg_off, const uint8_t dis_off)
{
    uint16_t prot_cfg = 0U;
    uint16_t bits     = 0U;

    max17320_error_t err = max17320_read_register(MAX17320_REG_NPROTCFG, &prot_cfg);
    if (err != MAX17320_OK)
    {
        return err;
    }
    if ((prot_cfg & MAX17320_NPROTCFG_CMOVRDEN) == 0U)
    {
        return MAX17320_ERR_FET_OVERRIDE_OFF;
    }

    if (chg_off != 0U)
    {
        bits |= MAX17320_COMMSTAT_CHGOFF;
    }
    if (dis_off != 0U)
    {
        bits |= MAX17320_COMMSTAT_DISOFF;
    }

    /*
     * The FET-off bits live in CommStat alongside the write-protection bits,
     * so they must be carried through both the unlocked and the re-locked
     * writes or the re-lock would clear them again.
     */
    err = max17320_write_comm_stat_twice((uint16_t)(MAX17320_COMMSTAT_UNLOCKED | bits));
    if (err != MAX17320_OK)
    {
        return err;
    }

    return max17320_write_comm_stat_twice((uint16_t)(MAX17320_COMMSTAT_LOCKED | bits));
}

max17320_error_t max17320_enter_ship_mode(void)
{
    return max17320_update_protected(
        MAX17320_REG_CONFIG, MAX17320_CONFIG_SHIP, MAX17320_CONFIG_SHIP);
}

max17320_error_t max17320_set_hibernate_enabled(const uint8_t enabled)
{
    return max17320_update_protected(MAX17320_REG_NHIBCFG,
                                     MAX17320_NHIBCFG_ENHIB,
                                     (enabled != 0U) ? MAX17320_NHIBCFG_ENHIB : 0U);
}

/* -------------------------------------------------------------------------
 * Pack configuration helpers
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_set_design_capacity(const float amp_hours)
{
    float r_sense = 0.0F;

    if (amp_hours <= 0.0F)
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    max17320_error_t err = max17320_sense(&r_sense);
    if (err != MAX17320_OK)
    {
        return err;
    }

    const float counts = (amp_hours * r_sense) / MAX17320_LSB_CAPACITY_VHR;
    if (counts > 65535.0F)
    {
        return MAX17320_ERR_BAD_PARAM;
    }
    const uint16_t raw = (uint16_t)max17320_clamp_round(counts, 0, 65535);

    err = max17320_write_protected(MAX17320_REG_NDESIGNCAP, raw);
    if (err != MAX17320_OK)
    {
        return err;
    }
    return max17320_write_protected(MAX17320_REG_DESIGNCAP, raw);
}

max17320_error_t max17320_set_nv_sense_resistor(const float ohms)
{
    if (ohms <= 0.0F)
    {
        return MAX17320_ERR_BAD_SENSE_RESISTOR;
    }

    const float counts = ohms / MAX17320_LSB_NRSENSE_OHM;
    if (counts > 65535.0F)
    {
        return MAX17320_ERR_BAD_PARAM;
    }

    return max17320_write_protected(MAX17320_REG_NRSENSE,
                                    (uint16_t)max17320_clamp_round(counts, 1, 65535));
}

/* -------------------------------------------------------------------------
 * Aggregate read
 * ------------------------------------------------------------------------- */

max17320_error_t max17320_read_measurements(max17320_measurements_t *out)
{
    uint8_t          cell = 0U;
    max17320_error_t err  = MAX17320_OK;

    if (out == (max17320_measurements_t *)0)
    {
        return MAX17320_ERR_NULL_PARAM;
    }
    if (s_cell_count == 0U)
    {
        return MAX17320_ERR_NOT_INITIALISED;
    }

    for (cell = 0U; cell < 4U; cell++)
    {
        out->cell_v[cell]     = 0.0F;
        out->avg_cell_v[cell] = 0.0F;
    }
    for (cell = 1U; cell <= s_cell_count; cell++)
    {
        err = max17320_get_cell_voltage(cell, &out->cell_v[cell - 1U]);
        if (err != MAX17320_OK)
        {
            return err;
        }
        err = max17320_get_avg_cell_voltage(cell, &out->avg_cell_v[cell - 1U]);
        if (err != MAX17320_OK)
        {
            return err;
        }
    }

    err = max17320_get_vcell(&out->vcell_v);
    if (err == MAX17320_OK)
    {
        err = max17320_get_avg_vcell(&out->avg_vcell_v);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_pack_voltage(&out->batt_v);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_pckp_voltage(&out->pckp_v);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_open_circuit_voltage(&out->vfocv_v);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_ripple_voltage(&out->vripple_v);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_current(&out->current_a);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_avg_current(&out->avg_current_a);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_max_min_current(&out->max_current_a, &out->min_current_a);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_power(&out->power_w);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_avg_power(&out->avg_power_w);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_temperature(&out->temperature_c);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_avg_temperature(&out->avg_temperature_c);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_die_temperature(&out->die_temperature_c);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_max_min_temperature(&out->max_temperature_c, &out->min_temperature_c);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_reported_capacity(&out->rep_capacity_ah);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_full_capacity(&out->full_capacity_ah);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_full_capacity_nominal(&out->full_capacity_nom_ah);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_design_capacity(&out->design_capacity_ah);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_available_capacity(&out->available_capacity_ah);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_reported_soc(&out->rep_soc_pct);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_available_soc(&out->av_soc_pct);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_mix_soc(&out->mix_soc_pct);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_vf_soc(&out->vf_soc_pct);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_age(&out->age_pct);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_time_to_empty(&out->time_to_empty_s);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_time_to_full(&out->time_to_full_s);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_cycles(&out->cycles);
    }

    if (err == MAX17320_OK)
    {
        err = max17320_get_charging_voltage(&out->charging_voltage_v);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_charging_current(&out->charging_current_a);
    }
    if (err == MAX17320_OK)
    {
        err = max17320_get_cell_resistance(&out->cell_resistance_ohm);
    }

    return err;
}
