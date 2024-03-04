#ifndef _MAX17049_H_
#define _MAX17049_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_I2C_ADDR (0x36)

#define MAX_VCELL 0x02     // R - 12-bit A/D measurement of battery voltage
#define MAX_SOC 0x04       // R - 16-bit state of charge (SOC)
#define MAX_MODE 0x06      // W - Sends special commands to IC
#define MAX_VERSION 0x08   // R - Returns IC version
#define MAX_HIBRT 0x0A     // R/W - (MAX17048/49) Thresholds for entering hibernate
#define MAX_CONFIG 0x0C    // R/W - Battery compensation (default 0x971C)
#define MAX_CVALRT 0x14    // R/W - (MAX17048/49) Configures vcell range to generate alerts (default 0x00FF)
#define MAX_CRATE 0x16     // R - (MAX17048/49) Charge rate 0.208%/hr
#define MAX_VRESET_ID 0x18 // R/W - (MAX17048/49) Reset voltage and ID (default 0x96__)
#define MAX_STATUS 0x1A    // R/W - (MAX17048/49) Status of ID (default 0x01__)
#define MAX_COMMAND 0xFE   // W - Sends special comands to IC

bool max_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
bool max_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
bool max_read_reg(uint8_t reg_addr, uint16_t *data);
bool max_write_reg(uint8_t reg_addr, uint16_t data);
bool max_get_id(uint8_t * id);
float max_get_soc(void);
float max_get_voltage(void);

#endif