#include "sy103.h"

#include "xil_printf.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

void __attribute__((weak)) sy103_delay_ms(const uint32_t time_ms)
{
    assert(false && "sy103_delay_ms weakly def function not implemented.");
}

void __attribute__((weak)) sy103_i2c_write(const uint8_t *const data, const uint32_t len)
{
    assert(false && "sy103_i2c_write weakly def function not implemented.");
}

void __attribute__((weak)) sy103_enable_avdd(const bool enable)
{
    assert(false && "sy103_enable_avdd weakly def function not implemented.");
}

void __attribute__((weak)) sy103_enable_avee(const bool enable)
{
    assert(false && "sy103_enable_avee weakly def function not implemented.");
}

void __attribute__((weak)) sy103_reset(const bool reset)
{
    assert(false && "sy103_reset weakly def function not implemented.");
}

sy103_error_t __attribute__((weak)) sy103_init_mipi(const uint8_t *const buffer)
{
    assert(false && "sy103_init_mipi weakly def function not implemented.");
}

static void sy103_power_up(void)
{
    // Put the EVF into reset
    sy103_reset(true);
    sy103_delay_ms(10);

    // Start the power up sequence
    sy103_enable_avdd(true);
    sy103_delay_ms(2);

    sy103_enable_avee(true);
    sy103_delay_ms(2);

    // Release the reset, wait for OTP loading (20ms)
    sy103_reset(false);
    sy103_delay_ms(20);
}

static void sy103_power_down(void)
{
    // Put the EVF into reset
    sy103_reset(true);
    sy103_delay_ms(10);

    // Power down sequence
    sy103_enable_avee(false);
    sy103_delay_ms(2);

    sy103_enable_avdd(false);
}

sy103_error_t sy103_init(const uint8_t *const buffer)
{
    // Power up the EVF
    sy103_power_up();

    sy103_i2c_write((uint8_t[]){0x03, 0x00, 0x00, 0x80}, 4);
    sy103_i2c_write((uint8_t[]){0x53, 0x00, 0x00, 0x29}, 4);
    sy103_i2c_write((uint8_t[]){0x51, 0x00, 0x00, 0xFF}, 4);
    sy103_i2c_write((uint8_t[]){0x51, 0x01, 0x00, 0x01}, 4);
    sy103_i2c_write((uint8_t[]){0x69, 0x00, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x6B, 0x00, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x80, 0x00, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x80, 0x01, 0x00, 0xA0}, 4);
    sy103_i2c_write((uint8_t[]){0x80, 0x02, 0x00, 0xA0}, 4);
    sy103_i2c_write((uint8_t[]){0x80, 0x03, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x00, 0x00, 0x01}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x01, 0x00, 0xE1}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x02, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x03, 0x00, 0x10}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x04, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x05, 0x00, 0x10}, 4);
    sy103_i2c_write((uint8_t[]){0x81, 0x06, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x00, 0x00, 0x01}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x01, 0x00, 0xE1}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x02, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x03, 0x00, 0x10}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x04, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x05, 0x00, 0x10}, 4);
    sy103_i2c_write((uint8_t[]){0x82, 0x06, 0x00, 0x00}, 4);
    sy103_i2c_write((uint8_t[]){0x25, 0x00, 0x00, 0x01}, 4);
    sy103_i2c_write((uint8_t[]){0x2A, 0x00, 0x00, 0x02}, 4);
    sy103_i2c_write((uint8_t[]){0x2A, 0x01, 0x00, 0x80}, 4);
    sy103_i2c_write((uint8_t[]){0x2B, 0x00, 0x00, 0x02}, 4);
    sy103_i2c_write((uint8_t[]){0x2B, 0x01, 0x00, 0x80}, 4);
    sy103_i2c_write((uint8_t[]){0xF0, 0x00, 0x00, 0xAA}, 4);
    sy103_i2c_write((uint8_t[]){0xF0, 0x01, 0x00, 0x11}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x00, 0x00, 0x03}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x01, 0x00, 0xFF}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x02, 0x00, 0x03}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x03, 0x00, 0xFF}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x04, 0x00, 0x03}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x05, 0x00, 0xFF}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x06, 0x00, 0x03}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x07, 0x00, 0xFF}, 4);
    sy103_i2c_write((uint8_t[]){0xC2, 0x08, 0x00, 0x82}, 4);
    sy103_i2c_write((uint8_t[]){0xF0, 0x00, 0x00, 0xAA}, 4);
    sy103_i2c_write((uint8_t[]){0xF0, 0x01, 0x00, 0x12}, 4);
    sy103_i2c_write((uint8_t[]){0xD3, 0x00, 0x00, 0x20}, 4);
    sy103_i2c_write((uint8_t[]){0xBF, 0x00, 0x00, 0x17}, 4);
    sy103_i2c_write((uint8_t[]){0xBF, 0x01, 0x00, 0xBE}, 4);
    sy103_i2c_write((uint8_t[]){0xFF, 0x00, 0x00, 0x5A}, 4);
    sy103_i2c_write((uint8_t[]){0xFF, 0x01, 0x00, 0x81}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x0B, 0x00, 0x58}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x0C, 0x00, 0x5F}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x0D, 0x00, 0x66}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x0E, 0x00, 0x6D}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x0F, 0x00, 0x74}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x10, 0x00, 0x7B}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x11, 0x00, 0x82}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x12, 0x00, 0x89}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x13, 0x00, 0x90}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x14, 0x00, 0x97}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x15, 0x00, 0x9E}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x16, 0x00, 0xA5}, 4);
    sy103_i2c_write((uint8_t[]){0xF9, 0x17, 0x00, 0xAC}, 4);
    sy103_delay_ms(20);
    sy103_i2c_write((uint8_t[]){0x11, 0x00}, 2);
    sy103_delay_ms(100);
    sy103_i2c_write((uint8_t[]){0x29, 0x00}, 2);
    sy103_delay_ms(20);
    sy103_i2c_write((uint8_t[]){0xF0, 0x00, 0x00, 0xAA}, 4);
    sy103_i2c_write((uint8_t[]){0xF0, 0x01, 0x00, 0x11}, 4);
    sy103_i2c_write((uint8_t[]){0xC0, 0x00, 0x00, 0xFF}, 4);

    // Init the MIPI
    sy103_error_t error = sy103_init_mipi(buffer);
    if (error != SY103_OK)
    {
        return error;
    }

    return SY103_OK;
}