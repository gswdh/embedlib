#include "adxl355.h"

/* Implement this function to perform a full-duplex SPI exchange.
 * Wire to your platform's SPI HAL: assert CS, transfer `len` bytes
 * simultaneously from tx into rx, then deassert CS.
 * Return ADXL_OK on success or ADXL_ERR_SPI_XFER_FAIL on any error. */
adxl_error_t adxl_spi_transfer(const uint8_t *tx, uint8_t *rx, const uint8_t len)
{
    (void)tx;
    (void)rx;
    (void)len;
    return ADXL_ERR_SPI_XFER_FAIL;
}

/* Implement this function to block for at least `ms` milliseconds. */
void adxl_delay_ms(const uint32_t ms)
{
    (void)ms;
}
