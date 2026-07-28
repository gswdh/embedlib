#ifndef SSD1309Z_H
#define SSD1309Z_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * SSD1309 — 128 x 64 dot matrix OLED/PLED segment/common driver with
 * controller (Solomon Systech, Rev 1.1).
 *
 * Portable driver for the 4-wire SPI interface (BS[2:0] = 000b).  All command
 * encoding lives here; the application supplies the platform layer by
 * implementing the weakly defined hooks declared below.  The driver performs
 * no printing, logging or allocation, and owns no framebuffer: every failure
 * is reported through a distinct ssd1309z_error_t code and the caller keeps
 * ownership of the pixel data.
 *
 * The controller is write-only over SPI — it cannot be read back — so the
 * driver tracks the state it needs (addressing mode, scroll activity) itself.
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Panel geometry
 * ------------------------------------------------------------------------- */
#define SSD1309Z_WIDTH       (128U)
#define SSD1309Z_HEIGHT      (64U)
#define SSD1309Z_PAGES       (8U)   /* 8 pages of 8 rows each */
#define SSD1309Z_COLUMN_MAX  (127U)
#define SSD1309Z_PAGE_MAX    (7U)

/* Bytes in a full GDDRAM image: one byte per column per page. */
#define SSD1309Z_FRAME_SIZE  (1024U)

/* -------------------------------------------------------------------------
 * Fundamental commands (Table 9-1)
 * ------------------------------------------------------------------------- */
#define SSD1309Z_CMD_SET_CONTRAST        (0x81U) /* + A[7:0], RESET = 7Fh */
#define SSD1309Z_CMD_RESUME_TO_RAM       (0xA4U) /* output follows RAM (RESET) */
#define SSD1309Z_CMD_ENTIRE_DISPLAY_ON   (0xA5U) /* all pixels on, ignore RAM */
#define SSD1309Z_CMD_NORMAL_DISPLAY      (0xA6U) /* RESET */
#define SSD1309Z_CMD_INVERSE_DISPLAY     (0xA7U)
#define SSD1309Z_CMD_DISPLAY_OFF         (0xAEU) /* sleep mode (RESET) */
#define SSD1309Z_CMD_DISPLAY_ON          (0xAFU)
#define SSD1309Z_CMD_NOP                 (0xE3U)
#define SSD1309Z_CMD_SET_COMMAND_LOCK    (0xFDU) /* + 12h unlock / 16h lock */

#define SSD1309Z_ARG_COMMAND_UNLOCK      (0x12U)
#define SSD1309Z_ARG_COMMAND_LOCK        (0x16U)

/* -------------------------------------------------------------------------
 * Scrolling commands (Table 9-2)
 * ------------------------------------------------------------------------- */
#define SSD1309Z_CMD_SCROLL_RIGHT        (0x26U) /* + 7 parameter bytes */
#define SSD1309Z_CMD_SCROLL_LEFT         (0x27U) /* + 7 parameter bytes */
#define SSD1309Z_CMD_CONTENT_SCROLL_RIGHT (0x2CU) /* + 7 parameter bytes */
#define SSD1309Z_CMD_CONTENT_SCROLL_LEFT (0x2DU) /* + 7 parameter bytes */
#define SSD1309Z_CMD_SCROLL_DEACTIVATE   (0x2EU)
#define SSD1309Z_CMD_SCROLL_ACTIVATE     (0x2FU)
#define SSD1309Z_CMD_SCROLL_VERT_RIGHT   (0x29U) /* + 7 parameter bytes */
#define SSD1309Z_CMD_SCROLL_VERT_LEFT    (0x2AU) /* + 7 parameter bytes */
#define SSD1309Z_CMD_SET_VERT_SCROLL_AREA (0xA3U) /* + A[5:0], B[6:0] */

/* -------------------------------------------------------------------------
 * Addressing setting commands (Table 9-3)
 * ------------------------------------------------------------------------- */
#define SSD1309Z_CMD_SET_COL_LOW_BASE    (0x00U) /* 00h-0Fh, page mode only */
#define SSD1309Z_CMD_SET_COL_HIGH_BASE   (0x10U) /* 10h-1Fh, page mode only */
#define SSD1309Z_CMD_SET_ADDRESSING_MODE (0x20U) /* + A[1:0] */
#define SSD1309Z_CMD_SET_COLUMN_ADDRESS  (0x21U) /* + start, end */
#define SSD1309Z_CMD_SET_PAGE_ADDRESS    (0x22U) /* + start, end */
#define SSD1309Z_CMD_SET_PAGE_START_BASE (0xB0U) /* B0h-B7h, page mode only */

/* -------------------------------------------------------------------------
 * Hardware configuration commands (Table 9-4)
 * ------------------------------------------------------------------------- */
#define SSD1309Z_CMD_SET_START_LINE_BASE (0x40U) /* 40h-7Fh */
#define SSD1309Z_CMD_SEG_REMAP_NORMAL    (0xA0U) /* column 0 -> SEG0 (RESET) */
#define SSD1309Z_CMD_SEG_REMAP_REVERSE   (0xA1U) /* column 127 -> SEG0 */
#define SSD1309Z_CMD_SET_MULTIPLEX_RATIO (0xA8U) /* + A[5:0], RESET = 3Fh */
#define SSD1309Z_CMD_COM_SCAN_NORMAL     (0xC0U) /* COM0 -> COM[N-1] (RESET) */
#define SSD1309Z_CMD_COM_SCAN_REMAP      (0xC8U) /* COM[N-1] -> COM0 */
#define SSD1309Z_CMD_SET_DISPLAY_OFFSET  (0xD3U) /* + A[5:0], RESET = 00h */
#define SSD1309Z_CMD_SET_COM_PINS        (0xDAU) /* + A[5:4], RESET = 12h */
#define SSD1309Z_CMD_SET_GPIO            (0xDCU) /* + A[1:0], RESET = 02h */

/* -------------------------------------------------------------------------
 * Timing and driving scheme commands (Table 9-5)
 * ------------------------------------------------------------------------- */
#define SSD1309Z_CMD_SET_CLOCK           (0xD5U) /* + A[7:0], RESET = 70h */
#define SSD1309Z_CMD_SET_PRECHARGE       (0xD9U) /* + A[7:0], RESET = 22h */
#define SSD1309Z_CMD_SET_VCOMH_DESELECT  (0xDBU) /* + A[5:2], RESET = 34h */

/* -------------------------------------------------------------------------
 * Reset-state register values (section 8.4 and the command tables)
 * ------------------------------------------------------------------------- */
#define SSD1309Z_RESET_CONTRAST          (0x7FU)
#define SSD1309Z_RESET_MULTIPLEX         (0x3FU) /* 64 MUX */
#define SSD1309Z_RESET_COM_PINS          (0x12U) /* alternative, no L/R remap */
#define SSD1309Z_RESET_CLOCK             (0x70U) /* divide 1, Fosc 0111b */
#define SSD1309Z_RESET_PRECHARGE         (0x22U) /* phase 1 = 2, phase 2 = 2 */
#define SSD1309Z_RESET_GPIO              (0x02U) /* pin output LOW */

/* -------------------------------------------------------------------------
 * Power sequencing times, milliseconds (section 8.8).
 *
 * t1 and t2 are specified as 3 us minimums; the driver rounds them up to the
 * 1 ms resolution of the delay hook.
 * ------------------------------------------------------------------------- */
#define SSD1309Z_T_RESET_LOW_MS   (1U)   /* t1: RES# held low            */
#define SSD1309Z_T_RESET_HOLD_MS  (1U)   /* t2: before VCC is applied    */
#define SSD1309Z_T_VCC_STABLE_MS  (10U)  /* allow the VCC rail to settle */
#define SSD1309Z_T_SEG_COM_ON_MS  (100U) /* tAF: SEG/COM on after AFh    */
#define SSD1309Z_T_VCC_OFF_MS     (100U) /* tOFF: typical, before VDD off*/

/* -------------------------------------------------------------------------
 * D/C# and CS# logical hook arguments
 * ------------------------------------------------------------------------- */
#define SSD1309Z_DC_COMMAND  (0U)
#define SSD1309Z_DC_DATA     (1U)
#define SSD1309Z_CS_RELEASED (0U)
#define SSD1309Z_CS_SELECTED (1U)

/* -------------------------------------------------------------------------
 * Enumerated settings
 * ------------------------------------------------------------------------- */

/** @brief GDDRAM address pointer behaviour (command 20h). */
typedef enum
{
    SSD1309Z_ADDR_MODE_HORIZONTAL = 0x00, /* column then page  */
    SSD1309Z_ADDR_MODE_VERTICAL   = 0x01, /* page then column  */
    SSD1309Z_ADDR_MODE_PAGE       = 0x02  /* column only (RESET) */
} ssd1309z_addr_mode_t;

/** @brief VCOMH regulator output level (command DBh). */
typedef enum
{
    SSD1309Z_VCOMH_0_64_VCC = 0x00, /* ~0.64 x VCC          */
    SSD1309Z_VCOMH_0_78_VCC = 0x34, /* ~0.78 x VCC (RESET)  */
    SSD1309Z_VCOMH_0_84_VCC = 0x3C  /* ~0.84 x VCC          */
} ssd1309z_vcomh_t;

/** @brief GPIO pin state (command DCh). */
typedef enum
{
    SSD1309Z_GPIO_HIZ_INPUT_DISABLED = 0x00,
    SSD1309Z_GPIO_HIZ_INPUT_ENABLED  = 0x01,
    SSD1309Z_GPIO_OUTPUT_LOW         = 0x02, /* RESET */
    SSD1309Z_GPIO_OUTPUT_HIGH        = 0x03
} ssd1309z_gpio_t;

/** @brief Horizontal scroll direction. */
typedef enum
{
    SSD1309Z_SCROLL_RIGHT = 0,
    SSD1309Z_SCROLL_LEFT  = 1
} ssd1309z_scroll_dir_t;

/**
 * @brief Frames between scroll steps (Table 9-2).
 *
 * The encoding is not monotonic; use these names rather than raw numbers.
 */
typedef enum
{
    SSD1309Z_SCROLL_INTERVAL_5_FRAMES   = 0x00,
    SSD1309Z_SCROLL_INTERVAL_64_FRAMES  = 0x01,
    SSD1309Z_SCROLL_INTERVAL_128_FRAMES = 0x02,
    SSD1309Z_SCROLL_INTERVAL_256_FRAMES = 0x03,
    SSD1309Z_SCROLL_INTERVAL_2_FRAMES   = 0x04,
    SSD1309Z_SCROLL_INTERVAL_3_FRAMES   = 0x05,
    SSD1309Z_SCROLL_INTERVAL_4_FRAMES   = 0x06,
    SSD1309Z_SCROLL_INTERVAL_1_FRAME    = 0x07
} ssd1309z_scroll_interval_t;

/* -------------------------------------------------------------------------
 * Error codes.  Every distinct failure condition has its own value so the
 * caller can act on it without the driver emitting any diagnostic output.
 * ------------------------------------------------------------------------- */
typedef enum
{
    SSD1309Z_OK = 0,

    /* Returned by the weak hook defaults when the port is missing. */
    SSD1309Z_ERR_NO_PLATFORM_SPI,
    SSD1309Z_ERR_NO_PLATFORM_DC,
    SSD1309Z_ERR_NO_PLATFORM_CS,
    SSD1309Z_ERR_NO_PLATFORM_RESET,
    SSD1309Z_ERR_NO_PLATFORM_VCC,

    /* Returned by the application's hooks on a hardware failure. */
    SSD1309Z_ERR_SPI_WRITE,
    SSD1309Z_ERR_GPIO_DC,
    SSD1309Z_ERR_GPIO_CS,
    SSD1309Z_ERR_GPIO_RESET,
    SSD1309Z_ERR_GPIO_VCC,

    /* Argument validation */
    SSD1309Z_ERR_NULL_PARAM,
    SSD1309Z_ERR_BAD_LENGTH,        /* zero-length or oversized transfer   */
    SSD1309Z_ERR_BAD_COLUMN,        /* column outside 0-127                */
    SSD1309Z_ERR_BAD_PAGE,          /* page outside 0-7                    */
    SSD1309Z_ERR_BAD_RANGE,         /* start address above end address     */
    SSD1309Z_ERR_BAD_LINE,          /* start line or offset outside 0-63   */
    SSD1309Z_ERR_BAD_MUX_RATIO,     /* multiplex ratio outside 16-64       */
    SSD1309Z_ERR_BAD_CLOCK_DIVIDE,  /* divide ratio outside 1-16           */
    SSD1309Z_ERR_BAD_OSC_FREQUENCY, /* oscillator setting outside 0-15     */
    SSD1309Z_ERR_BAD_PRECHARGE,     /* phase 1 or 2 outside 1-15           */
    SSD1309Z_ERR_BAD_VCOMH,         /* not an ssd1309z_vcomh_t value       */
    SSD1309Z_ERR_BAD_ADDR_MODE,     /* not an ssd1309z_addr_mode_t value   */
    SSD1309Z_ERR_BAD_GPIO_MODE,     /* not an ssd1309z_gpio_t value        */
    SSD1309Z_ERR_BAD_SCROLL_DIR,    /* not an ssd1309z_scroll_dir_t value  */
    SSD1309Z_ERR_BAD_SCROLL_INTERVAL,
    SSD1309Z_ERR_BAD_SCROLL_OFFSET, /* vertical scroll offset outside 0-63 */
    SSD1309Z_ERR_BAD_SCROLL_AREA,   /* fixed + scroll rows above MUX ratio */

    /* Sequencing */
    SSD1309Z_ERR_NOT_INITIALISED,   /* ssd1309z_init has not succeeded yet */
    SSD1309Z_ERR_WRONG_ADDR_MODE,   /* command needs page addressing mode  */
    SSD1309Z_ERR_SCROLL_ACTIVE,     /* RAM access is barred while scrolling*/
    SSD1309Z_ERR_COMMAND_LOCKED     /* driver IC is locked to FDh only     */
} ssd1309z_error_t;

/* -------------------------------------------------------------------------
 * Panel configuration applied by ssd1309z_init()
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t multiplex_ratio;    /* 16..64; 64 for a full-height panel   */
    uint8_t display_offset;     /* 0..63, vertical COM shift            */
    uint8_t display_start_line; /* 0..63                                */
    uint8_t segment_remap;      /* 0 = column 0 to SEG0, 1 = reversed   */
    uint8_t com_scan_remap;     /* 0 = COM0 first, 1 = COM[N-1] first   */
    uint8_t com_alternative;    /* DAh A[4]: 1 = alternative (RESET)    */
    uint8_t com_lr_remap;       /* DAh A[5]: 1 = enable COM L/R remap   */
    uint8_t contrast;           /* 0..255                               */
    uint8_t clock_divide;       /* 1..16                                */
    uint8_t osc_frequency;      /* 0..15, higher is faster              */
    uint8_t precharge_phase1;   /* 1..15 DCLK                           */
    uint8_t precharge_phase2;   /* 1..15 DCLK                           */
    uint8_t inverse;            /* 1 = inverse display (A7h)            */

    ssd1309z_vcomh_t     vcomh;
    ssd1309z_addr_mode_t addressing_mode;
} ssd1309z_config_t;

/* -------------------------------------------------------------------------
 * Platform hooks — implemented by the application.
 *
 * Each has a weak default in the driver that returns a distinct
 * SSD1309Z_ERR_NO_PLATFORM_* code, so a build that forgets to provide them
 * fails loudly at run time instead of silently doing nothing.
 *
 * The pin hooks take logical arguments, not pin levels.  All three control
 * pins are active low on the controller, so a port drives the pin low when
 * asked for the asserted state.
 * ------------------------------------------------------------------------- */

/**
 * @brief Clock @p len bytes out on SDIN, MSB first, sampling on the rising
 *        edge of SCLK (SPI mode 0 or 3).
 *
 * Must block until the last bit has been shifted out, because the driver
 * releases CS# as soon as this returns.
 *
 * @return SSD1309Z_OK, or SSD1309Z_ERR_SPI_WRITE on any bus error.
 */
ssd1309z_error_t ssd1309z_spi_write(const uint8_t *tx, const uint32_t len);

/**
 * @brief Drive the D/C# pin.
 * @param[in] data_mode SSD1309Z_DC_DATA to select display data (pin high),
 *                      SSD1309Z_DC_COMMAND to select a command (pin low).
 * @return SSD1309Z_OK or SSD1309Z_ERR_GPIO_DC.
 */
ssd1309z_error_t ssd1309z_set_dc(const uint8_t data_mode);

/**
 * @brief Drive the CS# pin.
 * @param[in] selected SSD1309Z_CS_SELECTED to select the chip (pin low),
 *                     SSD1309Z_CS_RELEASED to release it (pin high).
 * @return SSD1309Z_OK or SSD1309Z_ERR_GPIO_CS.
 */
ssd1309z_error_t ssd1309z_set_cs(const uint8_t selected);

/**
 * @brief Drive the RES# pin.
 * @param[in] asserted Non-zero to hold the controller in reset (pin low).
 * @return SSD1309Z_OK or SSD1309Z_ERR_GPIO_RESET.
 */
ssd1309z_error_t ssd1309z_set_reset(const uint8_t asserted);

/**
 * @brief Enable or disable the VCC panel-driving rail.
 *
 * VCC must float rather than be pulled to ground when off.  A board that
 * hard-wires VCC can implement this as a no-op returning SSD1309Z_OK, but
 * the datasheet power sequence is then not fully honoured.
 *
 * @param[in] enabled Non-zero to turn the rail on.
 * @return SSD1309Z_OK or SSD1309Z_ERR_GPIO_VCC.
 */
ssd1309z_error_t ssd1309z_set_vcc(const uint8_t enabled);

/**
 * @brief Block for at least @p ms milliseconds.
 *
 * Required by the power-on and power-off sequences (t1, t2, tAF, tOFF).
 */
void ssd1309z_delay_ms(const uint32_t ms);

/* -------------------------------------------------------------------------
 * Initialisation, reset and power sequencing
 * ------------------------------------------------------------------------- */

/**
 * @brief Fill @p cfg with the settings for a standard 128 x 64 panel.
 *
 * Mirrors the controller reset state except for the addressing mode, which
 * is set to horizontal so that whole-frame blits work without further setup.
 *
 * @return SSD1309Z_OK or SSD1309Z_ERR_NULL_PARAM.
 */
ssd1309z_error_t ssd1309z_get_default_config(ssd1309z_config_t *cfg);

/**
 * @brief Run the datasheet power-on sequence and apply @p cfg.
 *
 * Pulses RES#, brings up VCC, programs every configuration register, clears
 * GDDRAM so no random pixels appear, then turns the display on and waits
 * tAF for SEG/COM to come up.  VDD is assumed to be already stable.
 *
 * @param[in] cfg Panel configuration; copied internally.
 * @return SSD1309Z_OK, SSD1309Z_ERR_NULL_PARAM, one of the argument
 *         validation codes for an out-of-range field, or a platform error.
 */
ssd1309z_error_t ssd1309z_init(const ssd1309z_config_t *cfg);

/**
 * @brief Pulse RES# to return the controller to its reset state.
 *
 * Leaves the display off and the driver's cached state in sync with the
 * hardware; ssd1309z_init() must be run again before drawing.
 */
ssd1309z_error_t ssd1309z_reset(void);

/**
 * @brief Run the datasheet power-off sequence.
 *
 * Sends AEh, drops VCC, then waits tOFF so the caller may safely remove VDD.
 */
ssd1309z_error_t ssd1309z_power_off(void);

/* -------------------------------------------------------------------------
 * Raw transfers
 * ------------------------------------------------------------------------- */

/**
 * @brief Send @p len command bytes with D/C# low.
 */
ssd1309z_error_t ssd1309z_write_command(const uint8_t *cmd, const uint32_t len);

/**
 * @brief Send @p len display-data bytes with D/C# high.
 *
 * Refused with SSD1309Z_ERR_SCROLL_ACTIVE while hardware scrolling is
 * running, because the datasheet prohibits RAM access in that state.
 */
ssd1309z_error_t ssd1309z_write_data(const uint8_t *data, const uint32_t len);

/* -------------------------------------------------------------------------
 * Fundamental settings
 * ------------------------------------------------------------------------- */

/** @brief Set one of the 256 contrast steps (command 81h). */
ssd1309z_error_t ssd1309z_set_contrast(const uint8_t contrast);

/** @brief Turn the panel on or off (AFh / AEh).  Off is sleep mode. */
ssd1309z_error_t ssd1309z_set_display_enabled(const uint8_t enabled);

/**
 * @brief Force every pixel on, or resume following GDDRAM (A5h / A4h).
 *
 * Useful as a panel self-test; does not alter GDDRAM contents.
 */
ssd1309z_error_t ssd1309z_set_entire_display_on(const uint8_t all_on);

/** @brief Select normal or inverse pixel polarity (A6h / A7h). */
ssd1309z_error_t ssd1309z_set_inverse(const uint8_t inverse);

/** @brief Send the no-operation command (E3h). */
ssd1309z_error_t ssd1309z_nop(void);

/**
 * @brief Lock or unlock the MCU interface (FDh).
 *
 * While locked the controller ignores every command except FDh, so the
 * driver refuses other calls with SSD1309Z_ERR_COMMAND_LOCKED.
 */
ssd1309z_error_t ssd1309z_set_command_lock(const uint8_t locked);

/* -------------------------------------------------------------------------
 * Addressing
 * ------------------------------------------------------------------------- */

/** @brief Select horizontal, vertical or page addressing (command 20h). */
ssd1309z_error_t ssd1309z_set_addressing_mode(const ssd1309z_addr_mode_t mode);

/**
 * @brief Set the column window (command 21h).
 *
 * Horizontal and vertical addressing modes only.
 */
ssd1309z_error_t ssd1309z_set_column_range(const uint8_t start, const uint8_t end);

/**
 * @brief Set the page window (command 22h).
 *
 * Horizontal and vertical addressing modes only.
 */
ssd1309z_error_t ssd1309z_set_page_range(const uint8_t start, const uint8_t end);

/**
 * @brief Set the page start address (commands B0h-B7h).
 *
 * Page addressing mode only; returns SSD1309Z_ERR_WRONG_ADDR_MODE otherwise.
 */
ssd1309z_error_t ssd1309z_set_page_start(const uint8_t page);

/**
 * @brief Set the column start address (commands 00h-0Fh and 10h-1Fh).
 *
 * Page addressing mode only; sends both address nibbles.
 */
ssd1309z_error_t ssd1309z_set_column_start(const uint8_t column);

/* -------------------------------------------------------------------------
 * Hardware configuration
 * ------------------------------------------------------------------------- */

/** @brief Set the display RAM start line, 0-63 (commands 40h-7Fh). */
ssd1309z_error_t ssd1309z_set_display_start_line(const uint8_t line);

/** @brief Map column 0 to SEG0 or to SEG127 (A0h / A1h). */
ssd1309z_error_t ssd1309z_set_segment_remap(const uint8_t remapped);

/** @brief Set the multiplex ratio, 16-64 (command A8h). */
ssd1309z_error_t ssd1309z_set_multiplex_ratio(const uint8_t ratio);

/** @brief Set the COM output scan direction (C0h / C8h). */
ssd1309z_error_t ssd1309z_set_com_scan_remap(const uint8_t remapped);

/** @brief Set the vertical COM shift, 0-63 (command D3h). */
ssd1309z_error_t ssd1309z_set_display_offset(const uint8_t offset);

/**
 * @brief Set the COM pin hardware configuration (command DAh).
 *
 * @param[in] alternative Non-zero for alternative COM pin layout (reset
 *                        default), zero for sequential.
 * @param[in] lr_remap    Non-zero to enable COM left/right remap.
 */
ssd1309z_error_t ssd1309z_set_com_pins(const uint8_t alternative, const uint8_t lr_remap);

/** @brief Set the GPIO pin state (command DCh). */
ssd1309z_error_t ssd1309z_set_gpio(const ssd1309z_gpio_t mode);

/* -------------------------------------------------------------------------
 * Timing and driving scheme
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the display clock (command D5h).
 *
 * @param[in] divide_ratio  Display clock divide ratio, 1-16.
 * @param[in] osc_frequency Oscillator frequency setting, 0-15.
 */
ssd1309z_error_t ssd1309z_set_clock(const uint8_t divide_ratio, const uint8_t osc_frequency);

/**
 * @brief Set the pre-charge period in DCLKs (command D9h).
 *
 * @param[in] phase1 Phase 1 period, 1-15.
 * @param[in] phase2 Phase 2 period, 1-15.
 */
ssd1309z_error_t ssd1309z_set_precharge(const uint8_t phase1, const uint8_t phase2);

/** @brief Set the VCOMH deselect level (command DBh). */
ssd1309z_error_t ssd1309z_set_vcomh_deselect(const ssd1309z_vcomh_t level);

/* -------------------------------------------------------------------------
 * Scrolling
 * ------------------------------------------------------------------------- */

/**
 * @brief Configure a continuous horizontal scroll (commands 26h / 27h).
 *
 * Scrolling must be deactivated before this is issued or GDDRAM may be
 * corrupted; the driver enforces that. Call ssd1309z_scroll_activate() to
 * start the motion.
 *
 * @param[in] direction  Scroll left or right.
 * @param[in] start_page First page of the scrolled band, 0-7.
 * @param[in] end_page   Last page of the scrolled band, 0-7, >= start_page.
 * @param[in] interval   Frames between one-column steps.
 * @param[in] start_col  First column of the scrolled band, 0-127.
 * @param[in] end_col    Last column, 0-127, >= start_col.
 */
ssd1309z_error_t ssd1309z_scroll_horizontal(const ssd1309z_scroll_dir_t      direction,
                                            const uint8_t                   start_page,
                                            const uint8_t                   end_page,
                                            const ssd1309z_scroll_interval_t interval,
                                            const uint8_t                   start_col,
                                            const uint8_t                   end_col);

/**
 * @brief Configure a continuous vertical and horizontal scroll (29h / 2Ah).
 *
 * Setting @p horizontal_step to 0 gives pure vertical scrolling; setting
 * @p vertical_offset to 0 gives pure horizontal scrolling.
 *
 * @param[in] direction       Horizontal component direction.
 * @param[in] horizontal_step 0 for no horizontal shift, 1 to shift a column.
 * @param[in] start_page      First page of the scrolled band, 0-7.
 * @param[in] end_page        Last page, 0-7, >= start_page.
 * @param[in] interval        Frames between steps.
 * @param[in] vertical_offset Rows shifted per step, 0-63.
 * @param[in] start_col       First column, 0-127.
 * @param[in] end_col         Last column, 0-127, >= start_col.
 */
ssd1309z_error_t ssd1309z_scroll_diagonal(const ssd1309z_scroll_dir_t      direction,
                                          const uint8_t                   horizontal_step,
                                          const uint8_t                   start_page,
                                          const uint8_t                   end_page,
                                          const ssd1309z_scroll_interval_t interval,
                                          const uint8_t                   vertical_offset,
                                          const uint8_t                   start_col,
                                          const uint8_t                   end_col);

/**
 * @brief Define the vertical scroll area (command A3h).
 *
 * @param[in] top_fixed_rows Rows held fixed at the top, referenced to
 *                           GDDRAM row 0.
 * @param[in] scroll_rows    Rows taking part in the scroll, starting on the
 *                           first row below the fixed area.
 */
ssd1309z_error_t ssd1309z_set_vertical_scroll_area(const uint8_t top_fixed_rows,
                                                   const uint8_t scroll_rows);

/** @brief Start the configured scroll (command 2Fh). */
ssd1309z_error_t ssd1309z_scroll_activate(void);

/**
 * @brief Stop scrolling (command 2Eh).
 *
 * The datasheet requires GDDRAM to be rewritten afterwards; the driver
 * re-enables RAM access but does not repaint for you.
 */
ssd1309z_error_t ssd1309z_scroll_deactivate(void);

/**
 * @brief Shift the displayed content by one column (commands 2Ch / 2Dh).
 *
 * Unlike the continuous scrolls this performs a single step and leaves RAM
 * access available, so the caller can refill the column entering the window.
 * Consecutive calls must be spaced by at least two frame periods.
 */
ssd1309z_error_t ssd1309z_content_scroll_step(const ssd1309z_scroll_dir_t direction,
                                              const uint8_t               start_page,
                                              const uint8_t               end_page,
                                              const uint8_t               start_col,
                                              const uint8_t               end_col);

/* -------------------------------------------------------------------------
 * Framebuffer transfer.  The caller owns the pixel data.
 *
 * A framebuffer is laid out as GDDRAM is: one byte per column per page,
 * pages ascending, columns ascending within a page.  Bit 0 of a byte is the
 * topmost row of that page and bit 7 the bottom row.
 * ------------------------------------------------------------------------- */

/**
 * @brief Write a full 1024-byte frame to GDDRAM.
 *
 * Switches to horizontal addressing and restores the configured mode
 * afterwards.
 *
 * @param[in] frame Exactly SSD1309Z_FRAME_SIZE bytes.
 */
ssd1309z_error_t ssd1309z_write_frame(const uint8_t *frame);

/**
 * @brief Write a rectangular page-aligned region of GDDRAM.
 *
 * @param[in] start_page First page, 0-7.
 * @param[in] end_page   Last page, 0-7, >= start_page.
 * @param[in] start_col  First column, 0-127.
 * @param[in] end_col    Last column, 0-127, >= start_col.
 * @param[in] data       (end_page - start_page + 1) * (end_col - start_col + 1)
 *                       bytes, ordered column-major within each page.
 */
ssd1309z_error_t ssd1309z_write_region(const uint8_t  start_page,
                                       const uint8_t  end_page,
                                       const uint8_t  start_col,
                                       const uint8_t  end_col,
                                       const uint8_t *data);

/**
 * @brief Fill the whole of GDDRAM with a repeating byte pattern.
 *
 * Needs no caller buffer; the pattern is streamed a page at a time.
 */
ssd1309z_error_t ssd1309z_fill(const uint8_t pattern);

/** @brief Clear GDDRAM to all-off pixels. */
ssd1309z_error_t ssd1309z_clear(void);

#endif /* SSD1309Z_H */
