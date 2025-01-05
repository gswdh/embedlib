#include "gmax0505.h"

#include <assert.h>

#include "gpio.h"
#include "xil_printf.h"

#include <stdio.h>

#define GMAX_T_LINE   (11.16e-6)
#define GMAX_T_CLKPIX (30e-9)

static const uint8_t gmax_base_config[256] = {
    0x23, 0x05, 0x00, 0xF4, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x14, 0x01, 0x00,
    0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x32, 0x14, 0x00, 0x00, 0x06, 0x02, 0x02, 0x04, 0x04, 0x00, 0x20, 0x23, 0x01,
    0xc4, 0x35, 0xcb, 0x3b, 0x35, 0xfc, 0xff, 0xff, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x34, 0xD8,
    0x10, 0x4F, 0x3D, 0x00, 0x08, 0x90, 0xC0, 0x1D, 0x09, 0xC4, 0x32, 0x4B, 0x5B, 0x05, 0xD0, 0xA0,
    0xC3, 0x44, 0x02, 0x10, 0x70, 0x43, 0x0E, 0x30, 0xD0, 0xF0, 0xD0, 0x44, 0x07, 0xD0, 0xC0, 0xC3,
    0x44, 0x34, 0xF0, 0x80, 0xC3, 0xFF, 0x4B, 0xA4, 0xA1, 0x40, 0x0E, 0x41, 0xFC, 0x1F, 0x81, 0x0C,
    0x7C, 0x00, 0x20, 0x2A, 0x01, 0x00, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x07, 0xD0, 0xC0, 0xC3,
    0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x1B, 0x21, 0x80, 0xF3,
    0x83, 0x84, 0x9D, 0xCC, 0x77, 0xF3, 0xDD, 0x0D, 0xD3, 0x00, 0x00, 0x1C, 0xB0, 0x0D, 0xFE, 0x62,
    0xC1, 0x1F, 0x8A, 0xCA, 0xE7, 0xA8, 0x3C, 0x8A, 0xFE, 0x82, 0x34, 0x7F, 0xC1, 0x20, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void __attribute__((weak)) gmax_delay_ms(const uint32_t time_ms)
{
    assert(false && "gmax_delay_ms weakly def function not implemented.");
}

bool __attribute__((weak)) gmax_pin_read(const gmax_pin_t pin)
{
    assert(false && "gmax_pin_read weakly def function not implemented.");
    return 0;
}

void __attribute__((weak)) gmax_pin_write(const gmax_pin_t pin, const bool value)
{
    assert(false && "gmax_pin_write weakly def function not implemented.");
}

void __attribute__((weak)) gmax_fpga_interface_init(void)
{
    assert(false && "gmax_fpga_interface_init weakly def function not implemented.");
}

uint16_t __attribute__((weak)) gmax_get_training_word(const uint8_t *const gmax_config)
{
    assert(false && "gmax_get_training_word weakly def function not implemented.");
    return 0;
}

void __attribute__((weak)) gmax_sync_word_write(const uint16_t sync_word)
{
    assert(false && "gmax_sync_word_write weakly def function not implemented.");
}

bool __attribute__((weak)) gmax_sync_complete(void)
{
    assert(false && "gmax_synced weakly def function not implemented.");
    return false;
}

void __attribute__((weak)) gmax_sync_mode(const bool en)
{
    assert(false && "gmax_sync_mode weakly def function not implemented.");
}

void __attribute__((weak)) gmax_initiate_frame(void)
{
    assert(false && "gmax_initiate_frame weakly def function not implemented.");
}

static void gmax_reset(const bool enable) { gmax_pin_write(SEN_NRESET, !enable); }

static bool gmax_is_frame_ready(void) { return gmax_pin_read(FRAME_RDY); }

static bool gmax_frame_in_progress(void) { return gmax_pin_read(FRAME_PROG); }

static void gmax_power_up(void)
{
    gpio_reset(SEN_SYS_NRESET);
    gmax_delay_ms(10);
    gpio_set(PWR_SEN_1V3_EN);
    gmax_delay_ms(10);
    gpio_set(PWR_SEN_3V3A_EN);
    gmax_delay_ms(10);
    gpio_set(PWR_SEN_3V6_EN);
    gpio_set(PWR_SEN_4V1_EN);
    gmax_delay_ms(10);
    gpio_set(PWR_SEN_0V7_EN);
    gmax_delay_ms(10);
    gpio_set(PWR_SEN_N1V3A_EN);
    gmax_delay_ms(10);
    gpio_set(SEN_SYS_NRESET);
}

static void gmax_power_down(void)
{
    gpio_reset(SEN_SYS_NRESET);
    gmax_delay_ms(10);
    gpio_reset(PWR_SEN_N1V3A_EN);
    gmax_delay_ms(10);
    gpio_reset(PWR_SEN_0V7_EN);
    gmax_delay_ms(10);
    gpio_reset(PWR_SEN_3V6_EN);
    gpio_reset(PWR_SEN_4V1_EN);
    gmax_delay_ms(10);
    gpio_reset(PWR_SEN_3V3A_EN);
    gmax_delay_ms(10);
    gpio_reset(PWR_SEN_1V3_EN);
}

static void write_data_bit(bool value)
{
    gpio_write(SEN_SPI_MOSI, value);
    gpio_reset(SEN_SPI_CLK);
    gpio_set(SEN_SPI_CLK);
}

static void write_data_byte(uint8_t value)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        write_data_bit(value & (1 << (7 - i)));
    }
}

static bool read_data_bit(void)
{
    gpio_reset(SEN_SPI_CLK);
    gpio_set(SEN_SPI_CLK);
    return gpio_read(SEN_SPI_MISO);
}

static uint8_t read_data_byte(void)
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        value |= read_data_bit() << (7 - i);
    }

    return value;
}

static void gmax_spi_write(const uint8_t addr, const uint8_t *data, const uint32_t len)
{
    // Select is active high
    gpio_set(SEN_SPI_NCS);

    // Do the RW bit (1 for write)
    write_data_bit(true);

    // Output the register address
    write_data_byte(addr);

    // Clock the data out
    for (uint32_t i = 0; i < len; i++)
    {
        write_data_byte(data[i]);
    }

    // Finish with reseting the select line
    gpio_reset(SEN_SPI_NCS);
}

static void gmax_spi_read(const uint8_t addr, uint8_t *data, const uint32_t len)
{
    // Select is active high
    gpio_set(SEN_SPI_NCS);

    // Do the RW bit (0 for read)
    write_data_bit(false);

    // Output the register address
    write_data_byte(addr);

    // Need a dummy bit
    write_data_bit(false);

    // Clock the data out
    for (uint32_t i = 0; i < len; i++)
    {
        data[i] = read_data_byte();
    }

    // Finish with reseting the select line
    gpio_reset(SEN_SPI_NCS);
}

static uint32_t gmax_calc_texp0c(const float exp_t_s)
{
    // Limits

    return (uint32_t)((exp_t_s - (636 * GMAX_T_CLKPIX)) / GMAX_T_LINE);
}

static void gmax_set_int_time(const uint32_t int_time_us)
{
    uint32_t reg_value = gmax_calc_texp0c((float)int_time_us / 10e6);

    uint8_t data[3] = {0};
    data[0]         = reg_value & 0xFF;
    data[1]         = (reg_value >> 8) & 0xFF;
    data[2]         = (reg_value >> 16) & 0xFF;
    gmax_spi_write(GMAX_REG_EXP0C, data, 3);
}

gmax_error_t gmax_init(void)
{
    // Init the FPGA IP interface
    gmax_fpga_interface_init();

    // Set the trainging word so the IP knows what to loook for
    gmax_sync_word_write(gmax_get_training_word(gmax_base_config));

    // Power the sensor up
    gmax_power_up();

    // Wait 1ms (in datasheet)
    gmax_delay_ms(1);

    // Set the base config
    gmax_spi_write(0x00, gmax_base_config, sizeof(gmax_base_config));

    // Wait 1ms (in datasheet)
    gmax_delay_ms(2);

    // Set reg 209 to 0x80 as it says in the datasheet
    uint8_t data = gmax_base_config[209] & 0xF0;
    gmax_spi_write(209, &data, 1);

    // Wait 6ms (in datasheet)
    gmax_delay_ms(6);

    // Enable the FPGA syncing
    gmax_sync_mode(true);

    // Wait for the syncing to be done (1s)
    for (uint32_t i = 0; i < 100; i++)
    {
        if (gmax_sync_complete() == true)
        {
            // Disable the FPGA syncing otherwise image data will trigger the sync module
            gmax_sync_mode(false);

            return GMAX_OK;
        }

        gmax_delay_ms(10);
    }

    return GMAX_SYNC_FAIL;
}

gmax_error_t gmax_deinit(void)
{
    // Power the sensor down
    gmax_power_down();

    return GMAX_OK;
}

gmax_error_t
gmax_frame_request(const uint32_t int_time_us, const gmax_gain_t gain, const gmax_resolution_t res)
{
    // The integration time
    gmax_set_int_time(int_time_us);

    // Actually get the frame underway
    gmax_initiate_frame();

    return GMAX_OK;
}

gmax_error_t gmax_sensor_temperature(float *const temp_c)
{
    uint8_t data[2] = {0};
    gmax_spi_read(GMAX_REG_DTEMP, data, sizeof(data));

    const uint16_t reg_value = (data[1] << 8) + data[0];
    *temp_c                  = ((float)reg_value * 0.1479) - 310.33;

    return GMAX_OK;
}

char *gmax_error_string(const gmax_error_t error_code)
{
    switch (error_code)
    {
    case GMAX_OK:
        return "GMAX_OK";
        break;
    case GMAX_SYNC_FAIL:
        return "GMAX_SYNC_FAIL";
        break;
    default:
        return NULL;
        break;
    }

    return NULL;
}