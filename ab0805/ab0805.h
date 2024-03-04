#ifndef _AB0805_H_
#define _AB0805_H_

#include <stdint.h>
#include <time.h>
#include <stdbool.h>

#define AB_I2C_ADDR 0x69

#define AB_REG_TIME 0x01
#define AB_REG_ALARM 0x09
#define AB_REG_STATUS 0x0F
#define AB_REG_CONT1 0x10
#define AB_REG_CONT2 0x11
#define AB_REG_INTMASK 0x12
#define AB_REG_TIMCONT 0x18
#define AB_REG_TIMVAL 0x19
#define AB_REG_TIMINIT 0x1A
#define AB_REG_SQW 0x13
#define AB_REG_CALXT 0x14
#define AB_REG_WDG 0x1B
#define AB_REG_OSCCONT 0x1C
#define AB_REG_OSCSTAT 0x1D
#define AB_REG_KEY 0x1F
#define AB_REG_TRICKLE 0x20
#define AB_REG_EX_RAM 0x3F
#define AB_REG_ALT_RAM 0x80


#define AB_TRK_KEY 0x9D
#define AB_TRK_TCS_EN 0xA0
#define AB_TRK_DID_SIL 0x08
#define AB_TRK_DID_SCH 0x04
#define AB_TRK_ROT_DIS 0x00
#define AB_TRK_ROT_3K 0x01
#define AB_TRK_ROT_6K 0x02
#define AB_TRK_ROT_11K 0x03

#define AB_WRITE_RTC (0x01)

void ab_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
void ab_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
void ab_read_reg(uint8_t reg_addr, uint8_t *data);
void ab_write_reg(uint8_t reg_addr, uint8_t data);
struct tm ab_bcd_to_tm(uint8_t * bcd);
void ab_tm_to_bcd(struct tm time, uint8_t * bcd);
void ab_enable_writes(bool en);
void ab_unlock(uint8_t key_value);
void ab_enable_trickle_charging();
void ab_time_get(struct tm *time);
void ab_time_set(struct tm time);
void ab_wdg_start(uint32_t timeout_ms);
void ab_wdg_stop();
void ab_wdg_pat();

#endif