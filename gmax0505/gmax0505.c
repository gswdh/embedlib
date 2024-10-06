#include "gmax0505.h"

#include <assert.h>

#define GMAX_SPI_TYPE_READ (false)
#define GMAX_SPI_TYPE_WRITE (true)

static const uint8_t gmax_base_config[256] = {0x22, 0x05, 0x00, 0xF4, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x14, 0x01, 0x00, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x14, 0x00, 0x00, 0x06, 0x02, 0x02, 0x04, 0x04, 0x00, 0x20, 0x23, 0x01, 0xc4, 0x35, 0xcb, 0x3b, 0x35, 0xfc, 0xff, 0xff, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x34, 0xD8, 0x10, 0x4F, 0x3D, 0x00, 0x08, 0x90, 0xC0, 0x1D, 0x09, 0xC4, 0x32, 0x4B, 0x5B, 0x05, 0xD0, 0xA0, 0xC3, 0x44, 0x02, 0x10, 0x70, 0x43, 0x0E, 0x30, 0xD0, 0xF0, 0xD0, 0x44, 0x07, 0xD0, 0xC0, 0xC3, 0x44, 0x34, 0xF0, 0x80, 0xC3, 0xFF, 0x4B, 0xA4, 0xA1, 0x40, 0x0E, 0x41, 0xFC, 0x1F, 0x81, 0x0C, 0x7C, 0x00, 0x18, 0x1B, 0x01, 0x00, 0x02, 0x00, 0x05, 0x00, 0x00, 0x00, 0x07, 0xD0, 0xC0, 0xC3, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x1B, 0x21, 0x80, 0xF3, 0x83, 0x84, 0x9D, 0xCC, 0x77, 0xF3, 0xDD, 0x0D, 0xD3, 0x00, 0x00, 0x1C, 0xB0, 0x0D, 0xFE, 0x62, 0xC1, 0x1F, 0x8A, 0xCA, 0xE7, 0xA8, 0x3C, 0x8A, 0xFE, 0x82, 0x34, 0x7F, 0xC1, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void __attribute__((weak)) gmax_delay(const uint32_t time_ms)
{
    assert(false && "gmax_delay weakly def function not implemented.");
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

void __attribute__((weak)) gmax_sync_word_write(const uint16_t sync_word)
{
    assert(false && "gmax_sync_word_write weakly def function not implemented.");
}

static void gmax_reset(const bool enable)
{
    gmax_pin_write(SEN_NRESET, !enable);
}

static void gmax_enable_sync(const bool enable)
{
    gmax_pin_write(SYNC_EN, enable);
}

static void gmax_request_frame(void)
{
    gmax_pin_write(FRAME_REQ, false);
    gmax_delay(1);
    gmax_pin_write(FRAME_REQ, true);
}

static bool gmax_sync_status(void)
{
    return gmax_pin_read(SYNC_STATUS);
}

static bool gmax_is_frame_ready(void)
{
    return gmax_pin_read(FRAME_RDY);
}

static bool gmax_frame_in_progress(void)
{
    return gmax_pin_read(FRAME_PROG);
}

static void gmax_power_up(void)
{
    // Keep the sensor in reset
    gmax_reset(true);
    gmax_delay(10);

    // Bring up the power rails
    gmax_pin_write(PWR_1V3_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_3V3A_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_3V6_EN, true);
    gmax_pin_write(PWR_4V1_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_0V7_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_N1V3A_EN, true);
    gmax_delay(1);

    // Bring out of reset
    gmax_reset(false);
}

static void gmax_power_down(void)
{
    // Keep the sensor in reset
    gmax_reset(true);
    gmax_delay(10);

    // Take the power rails down
    gmax_pin_write(PWR_N1V3A_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_0V7_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_3V6_EN, true);
    gmax_pin_write(PWR_4V1_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_3V3A_EN, true);
    gmax_delay(1);
    gmax_pin_write(PWR_1V3_EN, true);
    gmax_delay(1);
}

static bool gmax_spi_read_bit(void)
{
    // Data comes out on the falling edge
    gmax_pin_write(SPI_CLK, false);

    // Read the data on the rising edge
    gmax_pin_write(SPI_CLK, true);
    return gmax_pin_read(SPI_MOSI);
}

static void gmax_spi_write_bit(const bool bit)
{
    // Clock out the data on the falling edge
    gmax_pin_write(SPI_CLK, false);
    gmax_pin_write(SPI_MOSI, bit);

    // Latch the data into the slave on the rising edge
    gmax_pin_write(SPI_CLK, true);
}

static uint8_t gmax_spi_read_byte(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        data |= (gmax_spi_read_bit() << (7 - i));
    }

    return data;
}

static void gmax_spi_write_byte(const uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        gmax_spi_write_bit(b & (1 << (7 - i)));
    }
}

static void gmax_spi_write(const uint8_t addr, const uint8_t *data, const uint32_t len)
{
    // Set the SPI enable line (qactive high)
    gmax_pin_write(SPI_NCS, true);

    // Set the transaction type (write)
    gmax_spi_write_bit(GMAX_SPI_TYPE_WRITE);

    // Send the address
    gmax_spi_write_byte(addr);

    // Clock out the data
    for (uint32_t i = 0; i < len; i++)
    {
        gmax_spi_write_byte(data[i]);
    }

    // Reset the SPI enable line (qactive high)
    gmax_pin_write(SPI_NCS, false);
}

static void gmax_spi_read(const uint8_t addr, uint8_t *const data, const uint32_t len)
{
    // Set the SPI enable line (qactive high)
    gmax_pin_write(SPI_NCS, true);

    // Set the transaction type (write)
    gmax_spi_write_bit(GMAX_SPI_TYPE_READ);

    // Send the address
    gmax_spi_write_byte(addr);

    // Insert a dummy clock
    gmax_spi_write_bit(false);

    // Read the data
    for (uint32_t i = 0; i < len; i++)
    {
        data[i] = gmax_spi_read_byte();
    }

    // Reset the SPI enable line (qactive high)
    gmax_pin_write(SPI_NCS, false);
}

gmax_error_t gmax_init(void)
{
    // Power the sensor up
    gmax_power_up();

    // Set the base config
    gmax_spi_write(0x00, gmax_base_config, sizeof(gmax_base_config));

    // Wait 1ms (in datasheet)
    gmax_delay(1);

    // Set reg 209 to 0x80 as it says in the datasheet
    uint8_t data = 0x80;
    gmax_spi_write(209, &data, 1);

    // Wait 6ms (in datasheet)
    gmax_delay(6);

    // Enable the stream
    data = gmax_base_config[0] | GMAX_STREAM_EN_BIT;
    gmax_spi_write(0x00, &data, 1);

    // TODO: read some registers back to see if we were successful

    return GMAX_OK;
}

gmax_error_t gmax_deinit(void)
{
    // Power the sensor down
    gmax_power_down();

    return GMAX_OK;
}

gmax_error_t gmax_frame_request(const uint32_t int_time_us, const gmax_gain_t gain, const gmax_resolution_t res)
{
    assert(false && "Construction in place.");

    return GMAX_OK;
}