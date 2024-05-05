#include "max17049.h"

bool __attribute__((weak)) emc_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    return false;
}

bool __attribute__((weak)) emc_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    return false;
}

bool emc_read_reg(uint8_t reg_addr, uint16_t *data)
{
    return emc_read_burst(reg_addr, data, 2);
}

bool emc_write_reg(uint8_t reg_addr, uint16_t data)
{
    return emc_write_burst(reg_addr, &data, 2);
}
