#ifndef __TCA__
#define __TCA__

#include <stdint.h>

#define TCA_I2C_ADDR (0x40)

#define TCA_REG_INPUT_PORT  (0x00)
#define TCA_REG_OUTPUT_PORT (0x01)
#define TCA_REG_POLAIRTY    (0x02)
#define TCA_REG_CONFIG      (0x03)

void    tca_i2c_write(uint8_t reg, uint8_t *data, uint32_t len);
void    tca_i2c_read(uint8_t reg, uint8_t *data, uint32_t len);
void    tca_init(void);
void    tca_set_dir(uint8_t dir);
uint8_t tca_read_port(void);
void    tca_write_port(uint8_t value);

#endif