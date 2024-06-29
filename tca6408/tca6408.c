#include "tca6408.h"

void __attribute__((weak)) tca_i2c_write(uint8_t reg, uint8_t *data, uint32_t len)
{
	return;
}

void __attribute__((weak)) tca_i2c_read(uint8_t reg, uint8_t *data, uint32_t len)
{
	return;
}

void tca_set_dir(uint8_t dir)
{
	tca_i2c_write(TCA_REG_CONFIG, &dir, 1);
}

uint8_t tca_read_port(void)
{
	uint8_t input_port = 0;
	tca_i2c_read(TCA_REG_INPUT_PORT, &input_port, 1);
	return input_port;
}

void tca_write_port(uint8_t value)
{
	tca_i2c_write(TCA_REG_OUTPUT_PORT, &value, 1);
}
