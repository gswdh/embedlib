#include "max17049.h"

bool __attribute__((weak)) max_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    return false;
}

bool __attribute__((weak)) max_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    return false;
}

bool max_read_reg(uint8_t reg_addr, uint16_t *data) { return max_read_burst(reg_addr, data, 2); }

bool max_write_reg(uint8_t reg_addr, uint16_t data) { return max_write_burst(reg_addr, &data, 2); }

bool max_get_id(uint8_t *id)
{
    uint8_t data[2] = {0};
    if (!max_read_reg(MAX_VRESET_ID, (uint16_t *)data))
    {
        return false;
    }

    if (data[0] != 0x96)
    {
        return false;
    }

    *id = data[0];

    return true;
}

float max_get_soc(void)
{
    uint8_t data[2] = {0};
    max_read_reg(MAX_SOC, (uint16_t *)data);
    return (float)data[0] + (float)data[1] / 256.0;
}

float max_get_voltage(void)
{
    uint8_t data[2] = {0};
    max_read_reg(MAX_VCELL, (uint16_t *)data);
    return ((float)data[0] * 256 + (float)data[1]) * 0.00015625;
}
