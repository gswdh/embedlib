#ifndef _AR0822_H_
#define _AR0822_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    AR_RESET = 0,
    AR_RUN
} ar_reset_t;

typedef enum
{
    AR_OK = 0,
    AR_ERROR_INIT_FAIL,
    AR_ERROR_INVALID_CONFIG,
    AR_ERROR_I2C_FAIL,
    AR_ERROR_SYNC_TO,
    AR_ERROR_INVALID_PARAM,
} ar_error_t;

typedef struct
{
    uint16_t addr;
    uint16_t data;
    uint16_t mask;
} ar_reg_write_t;

typedef enum
{
    AR_COLOUR_RGB = 0,
    AR_COLOUR_R,
    AR_COLOUR_G,
    AR_COLOUR_B,
} ar_colour_t;

/* GPIO Pin enumeration */
typedef enum
{
    AR_PIN_GPIO0 = 0,
    AR_PIN_GPIO1,
    AR_PIN_GPIO2,
    AR_PIN_GPIO3,
    AR_PIN_MAX
} ar_pin_t;

/* GPIO Function enumeration */
typedef enum
{
    /* Input functions */
    AR_FUNC_NO_INPUT = 0,
    AR_FUNC_OUTPUT_ENABLE_N,
    AR_FUNC_TRIGGER,
    AR_FUNC_STANDBY,

    /* Output functions */
    AR_FUNC_BOOT_STATUS_0,
    AR_FUNC_BOOT_STATUS_1,
    AR_FUNC_BOOT_STATUS_2,
    AR_FUNC_SHUTTER_READOUT,
    AR_FUNC_FLASH,
    AR_FUNC_SHUTTER,
    AR_FUNC_LINE_VALID,
    AR_FUNC_FRAME_VALID,
    AR_FUNC_PIXCLK,
    AR_FUNC_MD_MOTION,
    AR_FUNC_MD_STOP,
    AR_FUNC_NEW_ROW_PULSE,
    AR_FUNC_NEW_FRAME_PULSE,
    AR_FUNC_MAX
} ar_function_t;

/* Trigger modes. The pin modes are the slave modes of grr_control1 R0x30CE
 * (AND90149 "Slave Mode", Table 13): they take TRIGGER from a GPIO mapped to
 * AR_FUNC_TRIGGER, and ar_set_trigger_mode() sets gpi_en (R0x301A[8]),
 * without which no GPIO input function works. AR_TRIGGER_SOFTWARE needs no
 * pin: the sensor waits in standby, ar_trigger_frame() starts streaming over
 * I2C, and ar_trigger_readout_started() returns it to standby at the end of
 * that frame (standby_eof) - one frame per trigger. */
typedef enum
{
    AR_TRIGGER_OFF = 0,           /* free-running ERS stream, GPIO inputs ignored */
    AR_TRIGGER_INTEGRATION_START, /* standby until TRIGGER: integrate, read out, standby */
    AR_TRIGGER_DETERMINISTIC,     /* as integration start, readout starts FLL after TRIGGER */
    AR_TRIGGER_READOUT_START,     /* streaming: each TRIGGER starts the next readout */
    AR_TRIGGER_SURROUND_VIEW,     /* streaming but idle: TRIGGER starts one FLL-long frame */
    AR_TRIGGER_SOFTWARE,          /* standby until ar_trigger_frame(), over I2C */
    AR_TRIGGER_MAX
} ar_trigger_mode_t;

/* FLASH (strobe) output behaviour, R0x3046, on a GPIO mapped to
 * AR_FUNC_FLASH. */
typedef enum
{
    AR_FLASH_OFF = 0, /* output held low */
    AR_FLASH_LED,     /* high while every row integrates: the exposure window */
    AR_FLASH_XENON,   /* fixed-width pulse at the start of that window */
    AR_FLASH_MAX
} ar_flash_mode_t;

#define AR_I2C_DEV_ADDR (0x10)

#define AR_INIT_SYNC_TO_MS (100U)

/* Longest wait for a stream/standby transition: standby is taken at the end
 * of the frame in progress (R0x301A[4]), which can be a long exposure. */
#define AR_STATE_SYNC_TO_MS (1000U)

/* TRIGGER pulse width. The sensor needs >= 3 EXTCLK periods (156 ns at
 * 19.2 MHz); the margin covers slow edges through level shifting. */
#define AR_TRIGGER_PULSE_US (10U)

#define AR_REG_GLOBAL_GAIN_MAX  (0x77)
#define AR_REG_GLOBAL_GAIN_STEP (0.375)

#define AR_REG_FRAME_STATUS (0x2008)
#define AR_REG_GPIO_SELECT  (0x340E)

/* FRAME_STATUS bit 3 (PLL_LOCKED): streaming with the PLL locked. Bit 1:
 * the sensor has reached standby. */
#define AR_REG_FRAME_STATUS_STREAM_BIT  (0x0008)
#define AR_REG_FRAME_STATUS_STANDBY_BIT (0x0002)

/* Identification and status */
#define AR_REG_CHIP_VERSION    (0x3000)
#define AR_REG_REVISION_NUMBER (0x300E)
#define AR_REG_CUSTOMER_REV    (0x31FE)
#define AR_REG_GPI_STATUS      (0x2006)
#define AR_REG_FLASH_STATUS    (0x200C)

/* reset_register: stream (bit 2), standby_eof (bit 4: standby waits for the
 * end of the frame in progress) and gpi_en (bit 8) */
#define AR_REG_RESET_REGISTER         (0x301A)
#define AR_RESET_REGISTER_STREAM      (0x0004)
#define AR_RESET_REGISTER_STANDBY_EOF (0x0010)
#define AR_RESET_REGISTER_GPI_EN      (0x0100)

/* grr_control1 slave-mode bits (Table 13) */
#define AR_REG_GRR_CONTROL1    (0x30CE)
#define AR_GRR_SLAVE_MODE      (0x0010) /* bit 4 */
#define AR_GRR_FRAME_START     (0x0020) /* bit 5 */
#define AR_GRR_SLAVE_SH_SYNC   (0x0100) /* bit 8: surround view */
#define AR_GRR_SLAVE_MODE_MASK (AR_GRR_SLAVE_MODE | AR_GRR_FRAME_START | AR_GRR_SLAVE_SH_SYNC)

/* FLASH: en_flash (bit 8), invert (bit 7), xenon frames [5:3] and delay
 * [2:0]. FLASH2 is the xenon pulse width in pixel clocks. */
#define AR_REG_FLASH       (0x3046)
#define AR_REG_FLASH2      (0x3048)
#define AR_FLASH_EN_FLASH  (0x0100)
#define AR_FLASH_XENON_ALL (0x0038)
#define AR_FLASH_MODE_MASK (0x01BF)

/* Frame timing and test patterns (R0x3070: 0 off, 1 solid, 2 colour bars,
 * 3 fade-to-grey bars, 256 walking 1s) */
#define AR_REG_FRAME_LENGTH_LINES (0x300A)
#define AR_REG_TEST_PATTERN_MODE  (0x3070)

#define AR_REG_GLOBAL_GAIN (0x5900)
#define AR_REG_COARSE_INT  (0x3012)
#define AR_REG_HDR_CONTROL (0x3110)

#define AR_REG_GAIN_G1 (0x3056)
#define AR_REG_GAIN_B  (0x3058)
#define AR_REG_GAIN_R  (0x305A)
#define AR_REG_GAIN_G2 (0x305C)

/* GPIO Control Registers */
#define AR_REG_GPIO_CONTROL1 (0x340A)
#define AR_REG_GPIO_CONTROL2 (0x340C)
#define AR_REG_GPIO_SELECT   (0x340E)

/* Clock and timing registers for row time calculation */
#define AR_REG_LINE_LENGTH_PCK (0x300C)
#define AR_REG_VT_PIX_CLK_DIV  (0x302A)
#define AR_REG_VT_SYS_CLK_DIV  (0x302C)
#define AR_REG_PRE_PLL_CLK_DIV (0x302E)
#define AR_REG_PLL_MULTIPLIER  (0x3030)

// Interface functions
void       ar_set_nrst(const bool en);
void       ar_set_xshutdown(const bool en);
void       ar_enable_clock(const bool en);
void       ar_set_trigger(const bool en);
uint8_t    ar_get_gpio(void);
ar_error_t ar_i2c_write(const uint16_t reg, const uint8_t *data, const uint32_t len);
ar_error_t ar_i2c_read(const uint16_t reg, uint8_t *data, const uint32_t len);
void       ar_delay_ms(const uint32_t time_ms);
void       ar_delay_us(const uint32_t time_us);
uint32_t   ar_tick_ms(void);

// Driver functions
ar_error_t ar_init(const ar_reg_write_t *config, uint32_t len);
ar_error_t ar_read_register(const uint16_t reg, uint16_t *const value);
ar_error_t ar_write_register(const uint16_t reg, const uint16_t value);
ar_error_t ar_modify_register(const uint16_t reg, const uint16_t value, const uint16_t mask);
ar_error_t ar_frame_status(uint16_t *const status);
ar_error_t ar_gpio_config(uint16_t *const config);
ar_error_t ar_set_stream(const bool en);
ar_error_t ar_set_trigger_mode(const ar_trigger_mode_t mode);
ar_error_t ar_trigger_frame(void);
ar_error_t ar_trigger_readout_started(void);
ar_error_t ar_set_flash(const ar_flash_mode_t mode, const uint16_t xenon_width_pck);
ar_error_t ar_get_frame_length_lines(uint16_t *const lines);
ar_error_t ar_set_frame_length_lines(const uint16_t lines);
ar_error_t ar_get_gain(float *gain_db);
ar_error_t ar_set_gain(const float gain);
ar_error_t ar_set_colour_gain(const float gain, const ar_colour_t c);
ar_error_t ar_get_shutter_time_s(float *time_s);
ar_error_t ar_set_shutter_time_s(const float time_s);
ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y);
ar_error_t ar_set_pin_function(const ar_pin_t pin, const ar_function_t function);
ar_error_t ar_get_row_time_ns(uint32_t *time_ns);

// Debugging
char *ar_debug_gpio_state(const uint8_t gpio_state);

#endif