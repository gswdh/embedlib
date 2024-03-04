#include "lis3dsh.h"

void __attribute__((weak)) lis_spi_transfer(uint8_t *tx, uint8_t *rx, uint8_t len)
{
    return;
}

void lis_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t tx_data[len + 1];
    uint8_t rx_data[len + 1];

    tx_data[0] = addr & ~0x80;
    memcpy(tx_data + 1, data, len);

    lis_spi_transfer(tx_data, rx_data, len + 1);
}

void lis_read(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t tx_data[len + 1];
    uint8_t rx_data[len + 1];

    tx_data[0] = addr | 0x80;
    memcpy(tx_data + 1, data, len);

    lis_spi_transfer(tx_data, rx_data, len + 1);

    memcpy(data, rx_data + 1, len);
}

bool lis_init()
{
    uint8_t wai = 0;
    lis_read(LIS_WHO_I_AM, &wai, 1);

    if (wai != LIS_WAI_VALUE)
    {
        return false;
    }

    // Configuration registers
    uint8_t reg = 0;

    reg = LIS_DATARATE_400 | LIS_XYZ_ENABLE;
    lis_write(LIS_CTRL_REG4, &reg, 1);

    reg = LIS_FILTER_BW_50;
    lis_write(LIS_CTRL_REG5, &reg, 1);

    reg = 0x00; // FIFO disabled
    lis_write(LIS_FIFO_CONT, &reg, 1);

    reg = LIS_ADDR_INC;
    lis_write(LIS_CTRL_REG6, &reg, 1);

    return true;
}

void lis_get_angles(float *x, float *y, float *z)
{
    uint8_t data[6] = {0};
    lis_read(LIS_OUT_X_L, data, 6);

    *x = (float)((int16_t)((data[1] << 8) + data[0])) / 16383;
    *y = (float)((int16_t)((data[3] << 8) + data[2])) / 16383;
    *z = (float)((int16_t)((data[5] << 8) + data[4])) / 16383;
}
