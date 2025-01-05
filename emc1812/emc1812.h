#ifndef _EMC1812_H_
#define _EMC1812_H_

#include <stdbool.h>
#include <stdint.h>

#define EMC_I2C_ADDR (0x36)

bool emc_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
bool emc_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
bool emc_read_reg(uint8_t reg_addr, uint16_t *data);
bool emc_write_reg(uint8_t reg_addr, uint16_t data);

#endif