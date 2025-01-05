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

#define AR_I2C_DEV_ADDR (0x10)

#define AR_INIT_SYNC_TO_MS (100U)

#define AR_ROW_TIME_S (0.000014524)

#define AR_REG_GLOBAL_GAIN_MAX (0x77)
#define AR_REG_GLOBAL_GAIN_STEP (0.375)

#define AR_REG_FRAME_STATUS (0x2008)
#define AR_REG_GPIO_SELECT (0x340E)

#define AR_REG_FRAME_STATUS_STREAM_BIT (0x0008)

#define AR_REG_GLOBAL_GAIN (0x5900)
#define AR_REG_COARSE_INT (0x3012)
#define AR_REG_HDR_CONTROL (0x3110)

#define AR_REG_GAIN_G1 (0x3056)
#define AR_REG_GAIN_B (0x3058)
#define AR_REG_GAIN_R (0x305A)
#define AR_REG_GAIN_G2 (0x305C)

// Interface functions
void ar_set_nrst(const bool en);
void ar_set_xshutdown(const bool en);
void ar_enable_clock(const bool en);
uint8_t ar_get_gpio(void);
ar_error_t ar_i2c_write(const uint16_t reg, const uint8_t *data, const uint32_t len);
ar_error_t ar_i2c_read(const uint16_t reg, const uint8_t *data, const uint32_t len);
void ar_delay_ms(const uint32_t time_ms);
uint32_t ar_tick_ms(void);

// Driver functions
ar_error_t ar_init(const ar_reg_write_t *config, uint32_t len);
ar_error_t ar_frame_status(uint16_t *const status);
ar_error_t ar_gpio_config(uint16_t *const config);
ar_error_t ar_trigger_frame(void);
ar_error_t ar_get_gain(float *gain_db);
ar_error_t ar_set_gain(const float gain);
ar_error_t ar_set_colour_gain(const float gain, const ar_colour_t c);
ar_error_t ar_get_shutter_time_s(float *time_s);
ar_error_t ar_set_shutter_time_s(const float time_s);
ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y);

// Debugging
char *ar_debug_gpio_state(const uint8_t gpio_state);

#endif