#include "ssd1309z.h"

/* -------------------------------------------------------------------------
 * Internal state.
 *
 * The controller is write-only over SPI, so anything the driver needs to
 * reason about has to be tracked here and kept in step with what has been
 * sent.  All of it is reset by ssd1309z_reset().
 * ------------------------------------------------------------------------- */

static uint8_t              s_initialised    = 0U;
static uint8_t              s_scroll_active  = 0U;
static uint8_t              s_command_locked = 0U;
static uint8_t              s_multiplex      = SSD1309Z_HEIGHT;
static ssd1309z_addr_mode_t s_addr_mode      = SSD1309Z_ADDR_MODE_PAGE;

/* Scratch used to stream a constant pattern one page-row at a time. */
#define SSD1309Z_FILL_CHUNK (SSD1309Z_WIDTH)

/* -------------------------------------------------------------------------
 * Weak platform hooks.  The application overrides these; the defaults report
 * a distinct error so a missing port cannot masquerade as working hardware.
 * ------------------------------------------------------------------------- */

ssd1309z_error_t __attribute__((weak))
ssd1309z_spi_write(const uint8_t *tx, const uint32_t len)
{
    (void)tx;
    (void)len;
    return SSD1309Z_ERR_NO_PLATFORM_SPI;
}

ssd1309z_error_t __attribute__((weak)) ssd1309z_set_dc(const uint8_t data_mode)
{
    (void)data_mode;
    return SSD1309Z_ERR_NO_PLATFORM_DC;
}

ssd1309z_error_t __attribute__((weak)) ssd1309z_set_cs(const uint8_t selected)
{
    (void)selected;
    return SSD1309Z_ERR_NO_PLATFORM_CS;
}

ssd1309z_error_t __attribute__((weak)) ssd1309z_set_reset(const uint8_t asserted)
{
    (void)asserted;
    return SSD1309Z_ERR_NO_PLATFORM_RESET;
}

ssd1309z_error_t __attribute__((weak)) ssd1309z_set_vcc(const uint8_t enabled)
{
    (void)enabled;
    return SSD1309Z_ERR_NO_PLATFORM_VCC;
}

void __attribute__((weak)) ssd1309z_delay_ms(const uint32_t ms) { (void)ms; }

/* -------------------------------------------------------------------------
 * Transfer primitives
 * ------------------------------------------------------------------------- */

/*
 * D/C# is driven before CS# falls so the level is already settled when the
 * controller latches it, and CS# is always released even if the write fails.
 */
static ssd1309z_error_t
ssd1309z_transfer(const uint8_t dc_mode, const uint8_t *buf, const uint32_t len)
{
    ssd1309z_error_t err = ssd1309z_set_dc(dc_mode);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_set_cs(SSD1309Z_CS_SELECTED);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    const ssd1309z_error_t write_err  = ssd1309z_spi_write(buf, len);
    const ssd1309z_error_t release_err = ssd1309z_set_cs(SSD1309Z_CS_RELEASED);

    return (write_err != SSD1309Z_OK) ? write_err : release_err;
}

/* Command write that bypasses the command-lock guard. */
static ssd1309z_error_t ssd1309z_send(const uint8_t *cmd, const uint32_t len)
{
    return ssd1309z_transfer(SSD1309Z_DC_COMMAND, cmd, len);
}

static ssd1309z_error_t ssd1309z_send1(const uint8_t cmd)
{
    const uint8_t buf[1] = {cmd};
    return ssd1309z_send(buf, 1U);
}

static ssd1309z_error_t ssd1309z_send2(const uint8_t cmd, const uint8_t arg)
{
    const uint8_t buf[2] = {cmd, arg};
    return ssd1309z_send(buf, 2U);
}

static ssd1309z_error_t
ssd1309z_send3(const uint8_t cmd, const uint8_t arg0, const uint8_t arg1)
{
    const uint8_t buf[3] = {cmd, arg0, arg1};
    return ssd1309z_send(buf, 3U);
}

/* Guard applied to every command issued through the public API. */
static ssd1309z_error_t ssd1309z_check_unlocked(void)
{
    return (s_command_locked != 0U) ? SSD1309Z_ERR_COMMAND_LOCKED : SSD1309Z_OK;
}

/* -------------------------------------------------------------------------
 * Argument validation
 * ------------------------------------------------------------------------- */

static ssd1309z_error_t ssd1309z_check_page_range(const uint8_t start, const uint8_t end)
{
    if ((start > SSD1309Z_PAGE_MAX) || (end > SSD1309Z_PAGE_MAX))
    {
        return SSD1309Z_ERR_BAD_PAGE;
    }
    if (start > end)
    {
        return SSD1309Z_ERR_BAD_RANGE;
    }
    return SSD1309Z_OK;
}

static ssd1309z_error_t ssd1309z_check_column_range(const uint8_t start, const uint8_t end)
{
    if ((start > SSD1309Z_COLUMN_MAX) || (end > SSD1309Z_COLUMN_MAX))
    {
        return SSD1309Z_ERR_BAD_COLUMN;
    }
    if (start > end)
    {
        return SSD1309Z_ERR_BAD_RANGE;
    }
    return SSD1309Z_OK;
}

static ssd1309z_error_t ssd1309z_check_scroll_interval(const ssd1309z_scroll_interval_t interval)
{
    switch (interval)
    {
        case SSD1309Z_SCROLL_INTERVAL_5_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_64_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_128_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_256_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_2_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_3_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_4_FRAMES:
        case SSD1309Z_SCROLL_INTERVAL_1_FRAME:
            return SSD1309Z_OK;
        default:
            return SSD1309Z_ERR_BAD_SCROLL_INTERVAL;
    }
}

static ssd1309z_error_t ssd1309z_check_scroll_dir(const ssd1309z_scroll_dir_t direction)
{
    return ((direction == SSD1309Z_SCROLL_RIGHT) || (direction == SSD1309Z_SCROLL_LEFT))
               ? SSD1309Z_OK
               : SSD1309Z_ERR_BAD_SCROLL_DIR;
}

/* -------------------------------------------------------------------------
 * Raw transfers
 * ------------------------------------------------------------------------- */

ssd1309z_error_t ssd1309z_write_command(const uint8_t *cmd, const uint32_t len)
{
    if (cmd == (const uint8_t *)0)
    {
        return SSD1309Z_ERR_NULL_PARAM;
    }
    if (len == 0U)
    {
        return SSD1309Z_ERR_BAD_LENGTH;
    }
    /* FDh is the only command the controller still answers while locked. */
    if ((s_command_locked != 0U) && (cmd[0] != SSD1309Z_CMD_SET_COMMAND_LOCK))
    {
        return SSD1309Z_ERR_COMMAND_LOCKED;
    }

    return ssd1309z_send(cmd, len);
}

ssd1309z_error_t ssd1309z_write_data(const uint8_t *data, const uint32_t len)
{
    if (data == (const uint8_t *)0)
    {
        return SSD1309Z_ERR_NULL_PARAM;
    }
    if (len == 0U)
    {
        return SSD1309Z_ERR_BAD_LENGTH;
    }
    if (s_command_locked != 0U)
    {
        return SSD1309Z_ERR_COMMAND_LOCKED;
    }
    /* Section 10.26: RAM access is prohibited once scrolling is running. */
    if (s_scroll_active != 0U)
    {
        return SSD1309Z_ERR_SCROLL_ACTIVE;
    }

    return ssd1309z_transfer(SSD1309Z_DC_DATA, data, len);
}

/* -------------------------------------------------------------------------
 * Fundamental settings
 * ------------------------------------------------------------------------- */

ssd1309z_error_t ssd1309z_set_contrast(const uint8_t contrast)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send2(SSD1309Z_CMD_SET_CONTRAST, contrast);
}

ssd1309z_error_t ssd1309z_set_display_enabled(const uint8_t enabled)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1((enabled != 0U) ? SSD1309Z_CMD_DISPLAY_ON
                                          : SSD1309Z_CMD_DISPLAY_OFF);
}

ssd1309z_error_t ssd1309z_set_entire_display_on(const uint8_t all_on)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1((all_on != 0U) ? SSD1309Z_CMD_ENTIRE_DISPLAY_ON
                                         : SSD1309Z_CMD_RESUME_TO_RAM);
}

ssd1309z_error_t ssd1309z_set_inverse(const uint8_t inverse)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1((inverse != 0U) ? SSD1309Z_CMD_INVERSE_DISPLAY
                                          : SSD1309Z_CMD_NORMAL_DISPLAY);
}

ssd1309z_error_t ssd1309z_nop(void)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1(SSD1309Z_CMD_NOP);
}

ssd1309z_error_t ssd1309z_set_command_lock(const uint8_t locked)
{
    const uint8_t          arg = (locked != 0U) ? SSD1309Z_ARG_COMMAND_LOCK
                                                : SSD1309Z_ARG_COMMAND_UNLOCK;
    const ssd1309z_error_t err = ssd1309z_send2(SSD1309Z_CMD_SET_COMMAND_LOCK, arg);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    s_command_locked = (locked != 0U) ? 1U : 0U;
    return SSD1309Z_OK;
}

/* -------------------------------------------------------------------------
 * Addressing
 * ------------------------------------------------------------------------- */

/* Mode change without the lock guard, for use inside init and the blitters. */
static ssd1309z_error_t ssd1309z_apply_addressing_mode(const ssd1309z_addr_mode_t mode)
{
    const ssd1309z_error_t err =
        ssd1309z_send2(SSD1309Z_CMD_SET_ADDRESSING_MODE, (uint8_t)mode);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    s_addr_mode = mode;
    return SSD1309Z_OK;
}

ssd1309z_error_t ssd1309z_set_addressing_mode(const ssd1309z_addr_mode_t mode)
{
    if ((mode != SSD1309Z_ADDR_MODE_HORIZONTAL) && (mode != SSD1309Z_ADDR_MODE_VERTICAL) &&
        (mode != SSD1309Z_ADDR_MODE_PAGE))
    {
        return SSD1309Z_ERR_BAD_ADDR_MODE;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_apply_addressing_mode(mode);
}

ssd1309z_error_t ssd1309z_set_column_range(const uint8_t start, const uint8_t end)
{
    ssd1309z_error_t err = ssd1309z_check_column_range(start, end);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    /* Commands 21h and 22h have no effect in page addressing mode. */
    if (s_addr_mode == SSD1309Z_ADDR_MODE_PAGE)
    {
        return SSD1309Z_ERR_WRONG_ADDR_MODE;
    }

    return ssd1309z_send3(SSD1309Z_CMD_SET_COLUMN_ADDRESS, start, end);
}

ssd1309z_error_t ssd1309z_set_page_range(const uint8_t start, const uint8_t end)
{
    ssd1309z_error_t err = ssd1309z_check_page_range(start, end);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_addr_mode == SSD1309Z_ADDR_MODE_PAGE)
    {
        return SSD1309Z_ERR_WRONG_ADDR_MODE;
    }

    return ssd1309z_send3(SSD1309Z_CMD_SET_PAGE_ADDRESS, start, end);
}

ssd1309z_error_t ssd1309z_set_page_start(const uint8_t page)
{
    if (page > SSD1309Z_PAGE_MAX)
    {
        return SSD1309Z_ERR_BAD_PAGE;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_addr_mode != SSD1309Z_ADDR_MODE_PAGE)
    {
        return SSD1309Z_ERR_WRONG_ADDR_MODE;
    }

    return ssd1309z_send1((uint8_t)(SSD1309Z_CMD_SET_PAGE_START_BASE | page));
}

ssd1309z_error_t ssd1309z_set_column_start(const uint8_t column)
{
    if (column > SSD1309Z_COLUMN_MAX)
    {
        return SSD1309Z_ERR_BAD_COLUMN;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_addr_mode != SSD1309Z_ADDR_MODE_PAGE)
    {
        return SSD1309Z_ERR_WRONG_ADDR_MODE;
    }

    const uint8_t buf[2] = {
        (uint8_t)(SSD1309Z_CMD_SET_COL_LOW_BASE | (column & 0x0FU)),
        (uint8_t)(SSD1309Z_CMD_SET_COL_HIGH_BASE | (column >> 4U)),
    };
    return ssd1309z_send(buf, 2U);
}

/* -------------------------------------------------------------------------
 * Hardware configuration
 * ------------------------------------------------------------------------- */

ssd1309z_error_t ssd1309z_set_display_start_line(const uint8_t line)
{
    if (line >= SSD1309Z_HEIGHT)
    {
        return SSD1309Z_ERR_BAD_LINE;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1((uint8_t)(SSD1309Z_CMD_SET_START_LINE_BASE | line));
}

ssd1309z_error_t ssd1309z_set_segment_remap(const uint8_t remapped)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1((remapped != 0U) ? SSD1309Z_CMD_SEG_REMAP_REVERSE
                                           : SSD1309Z_CMD_SEG_REMAP_NORMAL);
}

ssd1309z_error_t ssd1309z_set_multiplex_ratio(const uint8_t ratio)
{
    if ((ratio < 16U) || (ratio > SSD1309Z_HEIGHT))
    {
        return SSD1309Z_ERR_BAD_MUX_RATIO;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /* The register holds N where the ratio is N + 1. */
    const ssd1309z_error_t send_err =
        ssd1309z_send2(SSD1309Z_CMD_SET_MULTIPLEX_RATIO, (uint8_t)(ratio - 1U));
    if (send_err != SSD1309Z_OK)
    {
        return send_err;
    }

    s_multiplex = ratio;
    return SSD1309Z_OK;
}

ssd1309z_error_t ssd1309z_set_com_scan_remap(const uint8_t remapped)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send1((remapped != 0U) ? SSD1309Z_CMD_COM_SCAN_REMAP
                                           : SSD1309Z_CMD_COM_SCAN_NORMAL);
}

ssd1309z_error_t ssd1309z_set_display_offset(const uint8_t offset)
{
    if (offset >= SSD1309Z_HEIGHT)
    {
        return SSD1309Z_ERR_BAD_LINE;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send2(SSD1309Z_CMD_SET_DISPLAY_OFFSET, offset);
}

ssd1309z_error_t ssd1309z_set_com_pins(const uint8_t alternative, const uint8_t lr_remap)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /* Second byte is 0 0 A5 A4 0 0 1 0. */
    uint8_t arg = 0x02U;
    if (alternative != 0U)
    {
        arg |= 0x10U;
    }
    if (lr_remap != 0U)
    {
        arg |= 0x20U;
    }
    return ssd1309z_send2(SSD1309Z_CMD_SET_COM_PINS, arg);
}

ssd1309z_error_t ssd1309z_set_gpio(const ssd1309z_gpio_t mode)
{
    if ((mode != SSD1309Z_GPIO_HIZ_INPUT_DISABLED) && (mode != SSD1309Z_GPIO_HIZ_INPUT_ENABLED) &&
        (mode != SSD1309Z_GPIO_OUTPUT_LOW) && (mode != SSD1309Z_GPIO_OUTPUT_HIGH))
    {
        return SSD1309Z_ERR_BAD_GPIO_MODE;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send2(SSD1309Z_CMD_SET_GPIO, (uint8_t)mode);
}

/* -------------------------------------------------------------------------
 * Timing and driving scheme
 * ------------------------------------------------------------------------- */

ssd1309z_error_t ssd1309z_set_clock(const uint8_t divide_ratio, const uint8_t osc_frequency)
{
    if ((divide_ratio < 1U) || (divide_ratio > 16U))
    {
        return SSD1309Z_ERR_BAD_CLOCK_DIVIDE;
    }
    if (osc_frequency > 15U)
    {
        return SSD1309Z_ERR_BAD_OSC_FREQUENCY;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /* A[3:0] holds divide ratio - 1, A[7:4] the oscillator setting. */
    const uint8_t arg = (uint8_t)(((uint8_t)(osc_frequency << 4U)) | (uint8_t)(divide_ratio - 1U));
    return ssd1309z_send2(SSD1309Z_CMD_SET_CLOCK, arg);
}

ssd1309z_error_t ssd1309z_set_precharge(const uint8_t phase1, const uint8_t phase2)
{
    /* Zero is an invalid entry for either phase. */
    if ((phase1 < 1U) || (phase1 > 15U) || (phase2 < 1U) || (phase2 > 15U))
    {
        return SSD1309Z_ERR_BAD_PRECHARGE;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    const uint8_t arg = (uint8_t)(((uint8_t)(phase2 << 4U)) | phase1);
    return ssd1309z_send2(SSD1309Z_CMD_SET_PRECHARGE, arg);
}

ssd1309z_error_t ssd1309z_set_vcomh_deselect(const ssd1309z_vcomh_t level)
{
    if ((level != SSD1309Z_VCOMH_0_64_VCC) && (level != SSD1309Z_VCOMH_0_78_VCC) &&
        (level != SSD1309Z_VCOMH_0_84_VCC))
    {
        return SSD1309Z_ERR_BAD_VCOMH;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send2(SSD1309Z_CMD_SET_VCOMH_DESELECT, (uint8_t)level);
}

/* -------------------------------------------------------------------------
 * Scrolling
 * ------------------------------------------------------------------------- */

ssd1309z_error_t ssd1309z_scroll_horizontal(const ssd1309z_scroll_dir_t       direction,
                                            const uint8_t                    start_page,
                                            const uint8_t                    end_page,
                                            const ssd1309z_scroll_interval_t interval,
                                            const uint8_t                    start_col,
                                            const uint8_t                    end_col)
{
    ssd1309z_error_t err = ssd1309z_check_scroll_dir(direction);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_page_range(start_page, end_page);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_column_range(start_col, end_col);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_scroll_interval(interval);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    /* Section 10.23: the scroll must be deactivated before it is set up. */
    if (s_scroll_active != 0U)
    {
        return SSD1309Z_ERR_SCROLL_ACTIVE;
    }

    const uint8_t buf[8] = {
        (direction == SSD1309Z_SCROLL_LEFT) ? SSD1309Z_CMD_SCROLL_LEFT : SSD1309Z_CMD_SCROLL_RIGHT,
        0x00U, /* A: dummy */
        start_page,
        (uint8_t)interval,
        end_page,
        0x00U, /* E: dummy */
        start_col,
        end_col,
    };
    return ssd1309z_send(buf, 8U);
}

ssd1309z_error_t ssd1309z_scroll_diagonal(const ssd1309z_scroll_dir_t       direction,
                                          const uint8_t                    horizontal_step,
                                          const uint8_t                    start_page,
                                          const uint8_t                    end_page,
                                          const ssd1309z_scroll_interval_t interval,
                                          const uint8_t                    vertical_offset,
                                          const uint8_t                    start_col,
                                          const uint8_t                    end_col)
{
    ssd1309z_error_t err = ssd1309z_check_scroll_dir(direction);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_page_range(start_page, end_page);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_column_range(start_col, end_col);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_scroll_interval(interval);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (vertical_offset >= SSD1309Z_HEIGHT)
    {
        return SSD1309Z_ERR_BAD_SCROLL_OFFSET;
    }
    err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_scroll_active != 0U)
    {
        return SSD1309Z_ERR_SCROLL_ACTIVE;
    }

    const uint8_t buf[8] = {
        (direction == SSD1309Z_SCROLL_LEFT) ? SSD1309Z_CMD_SCROLL_VERT_LEFT
                                            : SSD1309Z_CMD_SCROLL_VERT_RIGHT,
        (horizontal_step != 0U) ? 0x01U : 0x00U, /* A[0]: columns per step */
        start_page,
        (uint8_t)interval,
        end_page,
        vertical_offset, /* E[5:0]: rows per step */
        start_col,
        end_col,
    };
    return ssd1309z_send(buf, 8U);
}

ssd1309z_error_t ssd1309z_set_vertical_scroll_area(const uint8_t top_fixed_rows,
                                                   const uint8_t scroll_rows)
{
    /* A[5:0] + B[6:0] must both fit inside the active multiplex ratio. */
    if (top_fixed_rows >= SSD1309Z_HEIGHT)
    {
        return SSD1309Z_ERR_BAD_SCROLL_AREA;
    }
    if ((scroll_rows > s_multiplex) ||
        (((uint16_t)top_fixed_rows + (uint16_t)scroll_rows) > (uint16_t)s_multiplex))
    {
        return SSD1309Z_ERR_BAD_SCROLL_AREA;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send3(SSD1309Z_CMD_SET_VERT_SCROLL_AREA, top_fixed_rows, scroll_rows);
}

ssd1309z_error_t ssd1309z_scroll_activate(void)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    const ssd1309z_error_t send_err = ssd1309z_send1(SSD1309Z_CMD_SCROLL_ACTIVATE);
    if (send_err != SSD1309Z_OK)
    {
        return send_err;
    }

    s_scroll_active = 1U;
    return SSD1309Z_OK;
}

/* Scroll teardown without the lock guard, for use inside init. */
static ssd1309z_error_t ssd1309z_apply_scroll_deactivate(void)
{
    const ssd1309z_error_t err = ssd1309z_send1(SSD1309Z_CMD_SCROLL_DEACTIVATE);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    s_scroll_active = 0U;
    return SSD1309Z_OK;
}

ssd1309z_error_t ssd1309z_scroll_deactivate(void)
{
    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_apply_scroll_deactivate();
}

ssd1309z_error_t ssd1309z_content_scroll_step(const ssd1309z_scroll_dir_t direction,
                                              const uint8_t               start_page,
                                              const uint8_t               end_page,
                                              const uint8_t               start_col,
                                              const uint8_t               end_col)
{
    ssd1309z_error_t err = ssd1309z_check_scroll_dir(direction);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_page_range(start_page, end_page);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_column_range(start_col, end_col);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_scroll_active != 0U)
    {
        return SSD1309Z_ERR_SCROLL_ACTIVE;
    }

    const uint8_t buf[8] = {
        (direction == SSD1309Z_SCROLL_LEFT) ? SSD1309Z_CMD_CONTENT_SCROLL_LEFT
                                            : SSD1309Z_CMD_CONTENT_SCROLL_RIGHT,
        0x00U, /* A: dummy */
        start_page,
        0x01U, /* C: dummy, fixed at 01h */
        end_page,
        0x00U, /* E: dummy */
        start_col,
        end_col,
    };
    return ssd1309z_send(buf, 8U);
}

/* -------------------------------------------------------------------------
 * Framebuffer transfer
 * ------------------------------------------------------------------------- */

/*
 * Point the address pointer at a page/column window in horizontal addressing
 * mode.  Callers restore the configured mode afterwards.
 */
static ssd1309z_error_t ssd1309z_open_window(const uint8_t start_page,
                                             const uint8_t end_page,
                                             const uint8_t start_col,
                                             const uint8_t end_col)
{
    ssd1309z_error_t err = SSD1309Z_OK;

    if (s_addr_mode != SSD1309Z_ADDR_MODE_HORIZONTAL)
    {
        err = ssd1309z_apply_addressing_mode(SSD1309Z_ADDR_MODE_HORIZONTAL);
        if (err != SSD1309Z_OK)
        {
            return err;
        }
    }

    err = ssd1309z_send3(SSD1309Z_CMD_SET_COLUMN_ADDRESS, start_col, end_col);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    return ssd1309z_send3(SSD1309Z_CMD_SET_PAGE_ADDRESS, start_page, end_page);
}

/* Blit into an already-open window and restore the caller's addressing mode. */
static ssd1309z_error_t ssd1309z_close_window(const ssd1309z_addr_mode_t restore_mode)
{
    if (s_addr_mode == restore_mode)
    {
        return SSD1309Z_OK;
    }
    return ssd1309z_apply_addressing_mode(restore_mode);
}

/* Whole-RAM pattern fill used by both ssd1309z_fill() and init. */
static ssd1309z_error_t ssd1309z_fill_unchecked(const uint8_t pattern)
{
    uint8_t chunk[SSD1309Z_FILL_CHUNK];
    uint8_t page  = 0U;
    uint16_t index = 0U;

    for (index = 0U; index < (uint16_t)SSD1309Z_FILL_CHUNK; index++)
    {
        chunk[index] = pattern;
    }

    const ssd1309z_addr_mode_t restore_mode = s_addr_mode;

    ssd1309z_error_t err =
        ssd1309z_open_window(0U, (uint8_t)SSD1309Z_PAGE_MAX, 0U, (uint8_t)SSD1309Z_COLUMN_MAX);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    for (page = 0U; page < (uint8_t)SSD1309Z_PAGES; page++)
    {
        err = ssd1309z_transfer(SSD1309Z_DC_DATA, chunk, (uint32_t)SSD1309Z_FILL_CHUNK);
        if (err != SSD1309Z_OK)
        {
            return err;
        }
    }

    return ssd1309z_close_window(restore_mode);
}

ssd1309z_error_t ssd1309z_write_frame(const uint8_t *frame)
{
    if (frame == (const uint8_t *)0)
    {
        return SSD1309Z_ERR_NULL_PARAM;
    }
    if (s_initialised == 0U)
    {
        return SSD1309Z_ERR_NOT_INITIALISED;
    }

    return ssd1309z_write_region(
        0U, (uint8_t)SSD1309Z_PAGE_MAX, 0U, (uint8_t)SSD1309Z_COLUMN_MAX, frame);
}

ssd1309z_error_t ssd1309z_write_region(const uint8_t  start_page,
                                       const uint8_t  end_page,
                                       const uint8_t  start_col,
                                       const uint8_t  end_col,
                                       const uint8_t *data)
{
    if (data == (const uint8_t *)0)
    {
        return SSD1309Z_ERR_NULL_PARAM;
    }
    if (s_initialised == 0U)
    {
        return SSD1309Z_ERR_NOT_INITIALISED;
    }

    ssd1309z_error_t err = ssd1309z_check_page_range(start_page, end_page);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_column_range(start_col, end_col);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_scroll_active != 0U)
    {
        return SSD1309Z_ERR_SCROLL_ACTIVE;
    }

    const uint32_t pages   = (uint32_t)(end_page - start_page) + 1U;
    const uint32_t columns = (uint32_t)(end_col - start_col) + 1U;

    const ssd1309z_addr_mode_t restore_mode = s_addr_mode;

    err = ssd1309z_open_window(start_page, end_page, start_col, end_col);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_transfer(SSD1309Z_DC_DATA, data, pages * columns);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    return ssd1309z_close_window(restore_mode);
}

ssd1309z_error_t ssd1309z_fill(const uint8_t pattern)
{
    if (s_initialised == 0U)
    {
        return SSD1309Z_ERR_NOT_INITIALISED;
    }

    const ssd1309z_error_t err = ssd1309z_check_unlocked();
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    if (s_scroll_active != 0U)
    {
        return SSD1309Z_ERR_SCROLL_ACTIVE;
    }

    return ssd1309z_fill_unchecked(pattern);
}

ssd1309z_error_t ssd1309z_clear(void) { return ssd1309z_fill(0x00U); }

/* -------------------------------------------------------------------------
 * Initialisation, reset and power sequencing
 * ------------------------------------------------------------------------- */

ssd1309z_error_t ssd1309z_get_default_config(ssd1309z_config_t *cfg)
{
    if (cfg == (ssd1309z_config_t *)0)
    {
        return SSD1309Z_ERR_NULL_PARAM;
    }

    cfg->multiplex_ratio    = SSD1309Z_HEIGHT;
    cfg->display_offset     = 0U;
    cfg->display_start_line = 0U;
    cfg->segment_remap      = 0U;
    cfg->com_scan_remap     = 0U;
    cfg->com_alternative    = 1U; /* DAh reset value is 12h */
    cfg->com_lr_remap       = 0U;
    cfg->contrast           = SSD1309Z_RESET_CONTRAST;
    cfg->clock_divide       = 1U;
    cfg->osc_frequency      = 0x07U; /* D5h reset value is 70h */
    cfg->precharge_phase1   = 2U;
    cfg->precharge_phase2   = 2U;
    cfg->inverse            = 0U;
    cfg->vcomh              = SSD1309Z_VCOMH_0_78_VCC;
    cfg->addressing_mode    = SSD1309Z_ADDR_MODE_HORIZONTAL;

    return SSD1309Z_OK;
}

/* Bring the cached state back in line with the controller's reset defaults. */
static void ssd1309z_forget_state(void)
{
    s_initialised    = 0U;
    s_scroll_active  = 0U;
    s_command_locked = 0U;
    s_multiplex      = SSD1309Z_HEIGHT;
    s_addr_mode      = SSD1309Z_ADDR_MODE_PAGE;
}

ssd1309z_error_t ssd1309z_reset(void)
{
    ssd1309z_forget_state();

    ssd1309z_error_t err = ssd1309z_set_cs(SSD1309Z_CS_RELEASED);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_set_reset(1U);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    ssd1309z_delay_ms(SSD1309Z_T_RESET_LOW_MS);

    err = ssd1309z_set_reset(0U);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    ssd1309z_delay_ms(SSD1309Z_T_RESET_HOLD_MS);

    return SSD1309Z_OK;
}

/* Reject an out-of-range configuration before any of it reaches the panel. */
static ssd1309z_error_t ssd1309z_validate_config(const ssd1309z_config_t *cfg)
{
    if ((cfg->multiplex_ratio < 16U) || (cfg->multiplex_ratio > SSD1309Z_HEIGHT))
    {
        return SSD1309Z_ERR_BAD_MUX_RATIO;
    }
    if ((cfg->display_offset >= SSD1309Z_HEIGHT) || (cfg->display_start_line >= SSD1309Z_HEIGHT))
    {
        return SSD1309Z_ERR_BAD_LINE;
    }
    if ((cfg->clock_divide < 1U) || (cfg->clock_divide > 16U))
    {
        return SSD1309Z_ERR_BAD_CLOCK_DIVIDE;
    }
    if (cfg->osc_frequency > 15U)
    {
        return SSD1309Z_ERR_BAD_OSC_FREQUENCY;
    }
    if ((cfg->precharge_phase1 < 1U) || (cfg->precharge_phase1 > 15U) ||
        (cfg->precharge_phase2 < 1U) || (cfg->precharge_phase2 > 15U))
    {
        return SSD1309Z_ERR_BAD_PRECHARGE;
    }
    if ((cfg->vcomh != SSD1309Z_VCOMH_0_64_VCC) && (cfg->vcomh != SSD1309Z_VCOMH_0_78_VCC) &&
        (cfg->vcomh != SSD1309Z_VCOMH_0_84_VCC))
    {
        return SSD1309Z_ERR_BAD_VCOMH;
    }
    if ((cfg->addressing_mode != SSD1309Z_ADDR_MODE_HORIZONTAL) &&
        (cfg->addressing_mode != SSD1309Z_ADDR_MODE_VERTICAL) &&
        (cfg->addressing_mode != SSD1309Z_ADDR_MODE_PAGE))
    {
        return SSD1309Z_ERR_BAD_ADDR_MODE;
    }
    return SSD1309Z_OK;
}

/* Program every configuration register while the panel is still blanked. */
static ssd1309z_error_t ssd1309z_apply_config(const ssd1309z_config_t *cfg)
{
    const uint8_t clock_arg =
        (uint8_t)(((uint8_t)(cfg->osc_frequency << 4U)) | (uint8_t)(cfg->clock_divide - 1U));
    const uint8_t precharge_arg =
        (uint8_t)(((uint8_t)(cfg->precharge_phase2 << 4U)) | cfg->precharge_phase1);

    uint8_t com_pins_arg = 0x02U;
    if (cfg->com_alternative != 0U)
    {
        com_pins_arg |= 0x10U;
    }
    if (cfg->com_lr_remap != 0U)
    {
        com_pins_arg |= 0x20U;
    }

    ssd1309z_error_t err = ssd1309z_send2(SSD1309Z_CMD_SET_CLOCK, clock_arg);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send2(SSD1309Z_CMD_SET_MULTIPLEX_RATIO, (uint8_t)(cfg->multiplex_ratio - 1U));
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    s_multiplex = cfg->multiplex_ratio;

    err = ssd1309z_send2(SSD1309Z_CMD_SET_DISPLAY_OFFSET, cfg->display_offset);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send1(
        (uint8_t)(SSD1309Z_CMD_SET_START_LINE_BASE | cfg->display_start_line));
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send1((cfg->segment_remap != 0U) ? SSD1309Z_CMD_SEG_REMAP_REVERSE
                                                    : SSD1309Z_CMD_SEG_REMAP_NORMAL);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send1((cfg->com_scan_remap != 0U) ? SSD1309Z_CMD_COM_SCAN_REMAP
                                                     : SSD1309Z_CMD_COM_SCAN_NORMAL);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send2(SSD1309Z_CMD_SET_COM_PINS, com_pins_arg);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send2(SSD1309Z_CMD_SET_CONTRAST, cfg->contrast);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send2(SSD1309Z_CMD_SET_PRECHARGE, precharge_arg);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send2(SSD1309Z_CMD_SET_VCOMH_DESELECT, (uint8_t)cfg->vcomh);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send1(SSD1309Z_CMD_RESUME_TO_RAM);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send1((cfg->inverse != 0U) ? SSD1309Z_CMD_INVERSE_DISPLAY
                                              : SSD1309Z_CMD_NORMAL_DISPLAY);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    return ssd1309z_apply_scroll_deactivate();
}

ssd1309z_error_t ssd1309z_init(const ssd1309z_config_t *cfg)
{
    if (cfg == (const ssd1309z_config_t *)0)
    {
        return SSD1309Z_ERR_NULL_PARAM;
    }

    ssd1309z_error_t err = ssd1309z_validate_config(cfg);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /* Pulse RES# with VCC still off, per the section 8.8 power-on sequence. */
    err = ssd1309z_set_vcc(0U);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_reset();
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_set_vcc(1U);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    ssd1309z_delay_ms(SSD1309Z_T_VCC_STABLE_MS);

    /* Keep the panel blanked until GDDRAM holds something deliberate. */
    err = ssd1309z_send1(SSD1309Z_CMD_DISPLAY_OFF);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_apply_config(cfg);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /*
     * Select the requested addressing mode before the clear, so the fill
     * restores straight to it instead of bouncing through page mode.
     */
    err = ssd1309z_apply_addressing_mode(cfg->addressing_mode);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /* Clear first so no reset-state noise is shown when the panel lights. */
    err = ssd1309z_fill_unchecked(0x00U);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_send1(SSD1309Z_CMD_DISPLAY_ON);
    if (err != SSD1309Z_OK)
    {
        return err;
    }
    ssd1309z_delay_ms(SSD1309Z_T_SEG_COM_ON_MS);

    s_initialised = 1U;
    return SSD1309Z_OK;
}

ssd1309z_error_t ssd1309z_power_off(void)
{
    ssd1309z_error_t err = ssd1309z_send1(SSD1309Z_CMD_DISPLAY_OFF);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    err = ssd1309z_set_vcc(0U);
    if (err != SSD1309Z_OK)
    {
        return err;
    }

    /* Hold for tOFF so the caller may drop VDD once this returns. */
    ssd1309z_delay_ms(SSD1309Z_T_VCC_OFF_MS);

    ssd1309z_forget_state();
    return SSD1309Z_OK;
}
