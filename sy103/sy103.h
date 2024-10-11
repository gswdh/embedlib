#ifndef __SY103_H__
#define __SY103_H__

#include <stdint.h>
#include <stdbool.h>

#define SY103_I2C_ADDR (0x4C)

typedef enum
{
    SY103_OK = 0,
    SY103_MIPI_INIT_ERROR,
} sy103_error_t;

// Interface functions
void sy103_delay_ms(const uint32_t time_ms);
void sy103_i2c_write(const uint8_t *const data, const uint32_t len);
void sy103_enable_avdd(const bool enable);
void sy103_enable_avee(const bool enable);
void sy103_reset(const bool reset);
sy103_error_t sy103_init_mipi(const uint8_t *const buffer);

// Public functions
sy103_error_t sy103_init(const uint8_t *const buffer);

#endif