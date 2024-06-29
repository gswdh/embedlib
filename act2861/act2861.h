#ifndef _ACT2681_H_
#define _ACT2681_H_

#include <stdint.h>
#include <stdbool.h>

#define ACT_I2C_ADDR 0x48

#define ACT_MSTR_CNTRL_1 0x00
#define ACT_MSTR_CNTRL_2 0x01
#define ACT_STATUS_1 0x02
#define ACT_STATUS_2 0x03
#define ACT_TEMP_STATUS 0x04
#define ACT_FAULT_1 0x05
#define ACT_FAULT_2 0x06
#define ACT_ADC_1 0x07
#define ACT_ADC_2 0x08
#define ACT_ADC_CONF_1 0x09
#define ACT_ADC_CONF_2 0x0A
#define ACT_CHRG_VBAT_SHORT_V 0x0B
#define ACT_CHRG_CONT_1 0x0C
#define ACT_CHRG_CONT_2 0x0D
#define ACT_OTG_CONT_1 0x0E
#define ACT_OTG_CONT_2 0x0F
#define ACT_OTG_CONT_3 0x10
#define ACT_VBAT_REG_1 0x11
#define ACT_VBAT_REG_2 0x12
#define ACT_OTG_VOLTS_1 0x13
#define ACT_OTG_VOLTS_2 0X14
#define ACT_INPUT_AMPS_LIM 0x15
#define ACT_INPUT_VOLTS_LIM 0x16
#define ACT_OTG_AMPS_LIM 0x17
#define ACT_FAST_CHRG_AMPS 0x18
#define ACT_PRE_TERM_CHRG_AMPS 0x19
#define ACT_VBAT_LOW_VOLTS 0x1A
#define ACT_SAFETY_TIMER 0x1B
#define ACT_JEITA 0x1C
#define ACT_TEMP_SETTING 0x1D
#define ACT_IRQ_CONT_1 0x1E
#define ACT_IRQ_CONT_2 0x1F
#define ACT_OTG_STATUS 0x20

#define CHRG_STAT_RST (0x00)
#define CHRG_STAT_COND (0x01)
#define CHRG_STAT_SUS (0x02)
#define CHRG_STAT_PCOND (0x03)
#define CHRG_STAT_PCSUS (0x04)
#define CHRG_STAT_FAST (0x05)
#define CHRG_STAT_FSUS (0x06)
#define CHRG_STAT_FULL (0x07)
#define CHRG_STAT_CFSUS (0x08)
#define CHRG_STAT_TERM (0x09)
#define CHRG_STAT_TSUS (0x0A)
#define CHRG_STAT_FAULT (0x0B)

typedef enum
{
	ACT_OK = 0,
	ACT_I2C_MEM_WRITE_ERROR,
	ACT_I2C_MEM_ADDR_ERROR,
	ACT_I2C_MEM_POINTER_ERROR,
	ACT_OTG_VOLTAGE_OOR,
	ACT_FAILED_OTG_MODE,
	ACT_FAILED_CHG_MODE,
	ACT_HIGH_VBAT_ERROR,
	ACT_CHG_DONE,
	ACT_INPUT_VOLTAGE_ERROR,
	ACT_SAFETY_TIMER_ERROR,
	ACT_HIZ_MODE_ERROR,
	ACT_LOW_VBAT_ERROR,
	ACT_BAT_TEMP_ERROR,
	ACT_FET_OC_ERROR,
	ACT_OVER_TEMP_ERROR,
	ACT_HIZMODE_ERROR,
	ACT_READ_VBAT_ERROR,
	ACT_LOW_VBAT,
	ACT_MUX_ERROR,
	ACT_CHG_IN_PROGRESS,
} act_error_t;

act_error_t act_write_regs(uint8_t addr, uint8_t *data, uint8_t len);
act_error_t act_read_regs(uint8_t addr, uint8_t *data, uint8_t len);
void act_set_otg_pin(uint8_t state);
void act_set_shipmode_pin(uint8_t state);
void act_set_vbat_sw(uint8_t state);
bool act_get_irq_pin();
void act_delay_ms(uint32_t time_ms);
uint32_t act_get_tick_ms();

#endif
