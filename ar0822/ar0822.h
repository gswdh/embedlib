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
} ar_error_t;

typedef struct
{
    uint16_t addr;
    uint16_t data;
    uint16_t mask;
} ar_reg_write_t;

#define AR_I2C_DEV_ADDR (0x10)

#define AR_INIT_SYNC_TO_MS (100U)

#define AR_REG_FRAME_STATUS (0x2008)
#define AR_REG_GPIO_SELECT (0x340E)

#define AR_REG_FRAME_STATUS_STREAM_BIT (0x0008)

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
ar_error_t ar_set_gain(const float gain);
ar_error_t ar_set_shutter_time_s(const float time_s);
ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y);

// Debugging
char *ar_debug_gpio_state(const uint8_t gpio_state);

#endif