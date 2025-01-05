#include "trion.h"

#include "esp_log.h"

bool __attribute__((weak)) trion_get_cdone() { return false; }

void __attribute__((weak)) trion_set_cnrst(bool level) { return; }

void __attribute__((weak)) trion_set_cnss(bool level) { return; }

void __attribute__((weak)) trion_set_csi(bool level) { return; }

void __attribute__((weak)) trion_spi_tx(uint8_t *data, uint32_t len) { return; }

void __attribute__((weak)) trion_sleep_ms(uint32_t time_ms) { return; }

uint32_t __attribute__((weak)) trion_get_tick() { return 0; }

bool trion_configure(uint8_t *bitstream, uint32_t len)
{
    // Put the FPGA into programming mode
    trion_set_cnrst(false);
    trion_sleep_ms(1);
    trion_set_cnss(false);
    trion_set_csi(true);
    trion_sleep_ms(1);
    trion_set_cnrst(true);

    // Provide some clock cycles to satisfy t_dmin
    uint8_t data[128] = {0};
    trion_spi_tx(data, 128);

    // Send the bitstream
    uint32_t sent = 0;
    while (sent != len)
    {
        uint32_t remainder = len - sent;

        if (remainder > 1024)
            remainder = 1024;

        trion_spi_tx(&bitstream[sent], remainder);

        sent += remainder;
    }

    // Send (at least) 1000 extra clock cycles for configuration to happen
    trion_spi_tx(data, 128);

    // Wait for config
    trion_sleep_ms(10);

    // Return the status of CDONE
    return trion_get_cdone();
}
