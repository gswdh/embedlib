#ifndef _BMS_H_
#define _BMS_H_

#include <stdint.h>

#define BMS_I2C_L_ADDR (0x6C)
#define BMS_I2C_H_ADDR (0x16)

#define BMS_I2C_ADDR(reg) ((reg > 0xFF) ? BMS_I2C_H_ADDR : BMS_I2C_L_ADDR)

#define T_BMS_POR_TIMEOUT_MS 10000

#define BMS_REG_STATUS (0x0000)
#define BMS_REG_PROTSTATUS (0x00D9)
#define BMS_REG_PROTALRT (0x00AF)
#define BMS_REG_NBATTSTATUS (0x01A8)
#define BMS_REG_VBAT (0x00DA)
#define BMS_REG_CBAT (0x001C)
#define BMS_REG_AVG_CBAT (0x001D)
#define BMS_REG_COMMSTAT (0x0061)
#define BMS_REG_CMDREG (0x0060)
#define BMS_REG_CNFG2 (0x00AB)
#define BMS_REG_REPCAP (0x0005)
#define BMS_REG_FULLCAP (0x0035)

#define BMS_BIT_PERMFAIL (0x8000)
#define BMS_BIT_PROTALRT (0x8000)

#define BMS_BIT_CHGWDT (0x8000)
#define BMS_BIT_TOOHOTC (0x4000)
#define BMS_BIT_FULL (0x2000)
#define BMS_BIT_TOOCOLDC (0x1000)
#define BMS_BIT_OVP (0x0800)
#define BMS_BIT_OCCP (0x0400)
#define BMS_BIT_QOVFLW (0x0200)
#define BMS_BIT_PREQF (0x0100)
#define BMS_BIT_IMBALANCE (0x0080)
#define BMS_BIT_PMFAIL (0x0040)
#define BMS_BIT_DIEHOT (0x0020)
#define BMS_BIT_TOOHOTD (0x0010)
#define BMS_BIT_UVP (0x0008)
#define BMS_BIT_ODCP (0x0004)
#define BMS_BIT_RESDFAULT (0x0002)
#define BMS_BIT_LDET (0x0001)

typedef struct
{
	uint16_t protect_alert : 1;
	uint16_t max_soc : 1;
	uint16_t max_temp : 1;
	uint16_t max_voltage : 1;
	uint16_t rsvd1 : 1;
	uint16_t min_soc : 1;
	uint16_t min_temp : 1;
	uint16_t min_voltage : 1;
	uint16_t delta_soc : 1;
	uint16_t max_current : 1;
	uint16_t rsvd2 : 3;
	uint16_t min_current : 1;
	uint16_t por : 1;
	uint16_t rsvd3 : 1;
} bms_gen_status_t;

typedef struct
{
	uint16_t rsvd1 : 8;
	uint16_t too_hot : 1;
	uint16_t too_cold : 1;
	uint16_t ovp : 1;
	uint16_t occp : 1;
	uint16_t die_hot : 1;
	uint16_t imbalance : 1;
	uint16_t min_voltage : 1;
	uint16_t uvp : 1;
	uint16_t odcp : 1;
} bms_fault_status_t;

typedef struct
{
	bms_gen_status_t status;
	bms_fault_status_t fault;
	float volts;
	float amps;
	float soc;
	float cap;
} bms_stats_t;

// Error handling
typedef enum
{
	BMS_OK = 0,
	BMS_RESET_ERROR,
	BMS_I2C_MEM_WRITE_ERROR,
	BMS_I2C_MEM_READ_ERROR,
	BMS_CANNOT_REC_ERROR,
	BMS_PERM_FAIL_ERROR,
	BMS_PERM_FAIL_RESET_ERROR
} bms_error_t;

bms_error_t bms_reg_write(uint16_t reg, uint8_t *data, uint16_t len);
bms_error_t bms_reg_read(uint16_t reg, uint8_t *data, uint16_t len);
bms_error_t bms_bat_okay();
bms_error_t bms_get_voltage(float *voltage);
bms_error_t bms_get_current(float *current);
bms_error_t bms_get_soc(float *soc);
bms_error_t bms_get_full_capacity(float *full_cap);
bms_error_t bms_get_general_status(bms_gen_status_t *value);
bms_error_t bms_get_protection_status(bms_fault_status_t *value);
bms_error_t bms_get_stats(bms_stats_t *stats);

#endif