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
    AR_INIT_FAIL,
    AR_INVALID_CONFIG,
    AR_I2C_FAIL,
} ar_error_t;

typedef struct
{
    uint16_t addr;
    uint16_t data;
    uint16_t mask;
} ar_reg_write_t;

#define AR_I2C_DEV_ADDR (0x10)

// Interface functions
void ar_set_nrst(const bool en);
void ar_set_xshutdown(const bool en);
void ar_enable_clock(const bool en);
uint8_t ar_get_gpio(void);
ar_error_t ar_i2c_write(const uint16_t reg, const uint8_t *data, const uint32_t len);
ar_error_t ar_i2c_read(const uint16_t reg, const uint8_t *data, const uint32_t len);
void ar_delay_ms(const uint32_t time_ms);

// Driver functions
ar_error_t ar_init(const ar_reg_write_t *config, uint32_t len);
ar_error_t ar_trigger_frame(void);
ar_error_t ar_set_gain(const float gain);
ar_error_t ar_set_shutter_time_s(const float time_s);
ar_error_t ar_set_resolution(const uint32_t x, const uint32_t y);

#endif