#include "act2861.h"

#ifdef DEBUG
#include "log.h"
#define LOG_TAG "ACT"
#endif

act_error_t __attribute__((weak)) act_write_regs(uint8_t addr, uint8_t *data, uint8_t len)
{
    return ACT_OK;
}

act_error_t __attribute__((weak)) act_read_regs(uint8_t addr, uint8_t *data, uint8_t len)
{
    return ACT_OK;
}

void __attribute__((weak)) act_set_otg_pin(uint8_t state) {}

void __attribute__((weak)) act_set_shipmode_pin(uint8_t state) {}

void __attribute__((weak)) act_set_vbat_sw(uint8_t state) {}

bool __attribute__((weak)) act_get_irq_pin() { return false; }

void __attribute__((weak)) act_delay_ms(uint32_t time_ms) {}

uint32_t __attribute__((weak)) act_get_tick_ms() { return 0; }