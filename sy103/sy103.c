#include "sy103.h"

#include "xil_printf.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

void __attribute__((weak)) sy103_delay_ms(const uint32_t time_ms)
{
    assert(false && "sy103_delay_ms weakly def function not implemented.");
}

void __attribute__((weak)) sy103_i2c_write(const uint8_t reg, const uint8_t *const data, const uint32_t len)
{
    assert(false && "sy103_i2c_write weakly def function not implemented.");
}

void __attribute__((weak)) sy103_i2c_read(const uint8_t reg, uint8_t *const data, const uint32_t len)
{
    assert(false && "sy103_i2c_read weakly def function not implemented.");
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

    // // CMD1
    sy103_i2c_write(0x03, (uint8_t[]){0x80}, 1);
    sy103_i2c_write(0x53, (uint8_t[]){0x29}, 1);
    sy103_i2c_write(0x51, (uint8_t[]){0xFF, 0x01}, 2);
    sy103_i2c_write(0x69, (uint8_t[]){0x00}, 1);
    sy103_i2c_write(0x6B, (uint8_t[]){0x00}, 1);
    sy103_i2c_write(0x80, (uint8_t[]){0x00, 0xA0, 0xA0, 0x00}, 4);
    sy103_i2c_write(0x81, (uint8_t[]){0x01, 0xE1, 0x00, 0x10, 0x00, 0x10, 0x00}, 7);
    sy103_i2c_write(0x82, (uint8_t[]){0x01, 0xE1, 0x00, 0x10, 0x00, 0x10, 0x00}, 7);
    sy103_i2c_write(0x25, (uint8_t[]){0x01}, 1);

    sy103_i2c_write(0x2A, (uint8_t[]){0x02, 0x80}, 2);
    sy103_i2c_write(0x2B, (uint8_t[]){0x02, 0x80}, 2);

    // CMD2 P1
    sy103_i2c_write(0xF0, (uint8_t[]){0xAA, 0x11}, 2);
    sy103_i2c_write(0xC2, (uint8_t[]){0x03, 0xFF, 0x03, 0xFF, 0x03, 0xFF, 0x03, 0xFF, 0x82}, 9);

    // CMD2 P2
    sy103_i2c_write(0xF0, (uint8_t[]){0xAA, 0x12}, 2);
    sy103_i2c_write(0xD3, (uint8_t[]){0x20}, 1);
    sy103_i2c_write(0xBF, (uint8_t[]){0x17, 0xBE}, 2);

    // CMD3 P1
    sy103_i2c_write(0xFF, (uint8_t[]){0x5A, 0x81}, 2);
    sy103_i2c_write(0x65, (uint8_t[]){0x0B}, 1);
    sy103_i2c_write(0xF9, (uint8_t[]){0x58, 0x5F, 0x66, 0x6D, 0x74, 0x7B, 0x82, 0x89, 0x90, 0x97, 0x9E, 0xA5, 0xAC}, 13);

    sy103_delay_ms(20);
    sy103_i2c_write(0x11, NULL, 0);
    sy103_delay_ms(100);
    sy103_i2c_write(0x29, NULL, 0);
    sy103_delay_ms(20);
    sy103_i2c_write(0xF0, (uint8_t[]){0xAA, 0x11}, 2);
    sy103_i2c_write(0xC0, (uint8_t[]){0xFF}, 1);

    // Init the MIPI
    sy103_error_t error = sy103_init_mipi(buffer);
    if (error != SY103_OK)
    {
        return error;
    }

    return SY103_OK;
}