#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Register locations
#define LIS_TEMP_REG (0x0C)
#define LIS_WHO_I_AM (0x0F)
#define LIS_CTRL_REG4 (0x20)
#define LIS_CTRL_REG1 (0x21)
#define LIS_CTRL_REG2 (0x22)
#define LIS_CTRL_REG3 (0x23)
#define LIS_CTRL_REG5 (0x24)
#define LIS_CTRL_REG6 (0x25)
#define LIS_OUT_X_L (0x28)
#define LIS_OUT_X_H (0x29)
#define LIS_OUT_Y_L (0x2A)
#define LIS_OUT_Y_H (0x2B)
#define LIS_OUT_Z_L (0x2C)
#define LIS_OUT_Z_H (0x2D)
#define LIS_FIFO_CONT (0x2E)

// Reg config options
#define LIS_DATARATE_400 ((uint8_t)0x70)
#define LIS_FULLSCALE_2 ((uint8_t)0x00)
#define LIS_FULLSCALE_4 ((uint8_t)0x08)
#define LIS_FULLSCALE_6 ((uint8_t)0x10)
#define LIS_FULLSCALE_8 ((uint8_t)0x18)
#define LIS_FULLSCALE_16 ((uint8_t)0x20)
#define LIS_FULLSCALE_SELECTION ((uint8_t)0x38)
#define LIS_FILTER_BW_800 ((uint8_t)0x00)
#define LIS_FILTER_BW_400 ((uint8_t)0x40)
#define LIS_FILTER_BW_200 ((uint8_t)0x80)
#define LIS_FILTER_BW_50 ((uint8_t)(0xC0))
#define LIS_SELFTEST_NORMAL ((uint8_t)0x00)
#define LIS_XYZ_ENABLE ((uint8_t)0x07)
#define LIS_SERINTER_4WIRE ((uint8_t)0x00)
#define LIS_SM_ENABLE ((uint8_t)0x01)
#define LIS_SM_DISABLE ((uint8_t)0x00)
#define LIS_ADDR_INC ((uint8_t)0x10)
#define LIS_WAI_VALUE ((uint8_t)0x3F)



void lis_spi_transfer(uint8_t *tx, uint8_t *rx, uint8_t len);

bool lis_init();
void lis_get_angles(float *x, float *y, float *z);
