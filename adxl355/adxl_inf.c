#include "adxl355.h"

#include "main.h"
#include "spi.h"

#define ADXL355_SPI_ID 1U

adxl_error_t adxl_spi_transfer(const uint8_t *tx, uint8_t *rx, const uint8_t len)
{
    const spi_error_t err = spi_transfer(ADXL355_SPI_ID, tx, rx, (uint16_t)len);
    return (err == SPI_OKAY) ? ADXL_OK : ADXL_ERR_SPI_XFER_FAIL;
}

void adxl_delay_ms(const uint32_t ms)
{
    HAL_Delay(ms);
}
