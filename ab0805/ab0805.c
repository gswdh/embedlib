#include "ab0805.h"

void __attribute__((weak)) ab_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    return;
}

void __attribute__((weak)) ab_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    return;
}

void ab_read_reg(uint8_t reg_addr, uint8_t *data) { ab_read_burst(reg_addr, data, 1); }

void ab_write_reg(uint8_t reg_addr, uint8_t data) { ab_write_burst(reg_addr, &data, 1); }

struct tm ab_bcd_to_tm(uint8_t *bcd)
{
    struct tm time;
    time.tm_year  = ((bcd[5] & 0xF0) >> 4) * 10 + (bcd[5] & 0x0F) + 100;
    time.tm_mon   = ((bcd[4] & 0xF0) >> 4) * 10 + (bcd[4] & 0x0F) - 1;
    time.tm_mday  = ((bcd[3] & 0xF0) >> 4) * 10 + (bcd[3] & 0x0F);
    time.tm_hour  = ((bcd[2] & 0xF0) >> 4) * 10 + (bcd[2] & 0x0F);
    time.tm_min   = ((bcd[1] & 0xF0) >> 4) * 10 + (bcd[1] & 0x0F);
    time.tm_sec   = ((bcd[0] & 0xF0) >> 4) * 10 + (bcd[0] & 0x0F);
    time.tm_isdst = -1; // Daylight Saving Time information (unknown)
    return time;
}

void ab_tm_to_bcd(struct tm time, uint8_t *bcd)
{
    bcd[5] = ((time.tm_year - 100) / 10) << 4 | (time.tm_year - 100) % 10;
    bcd[4] = (time.tm_mon + 1) / 10 << 4 | (time.tm_mon + 1) % 10;
    bcd[3] = time.tm_mday / 10 << 4 | time.tm_mday % 10;
    bcd[2] = time.tm_hour / 10 << 4 | time.tm_hour % 10;
    bcd[1] = time.tm_min / 10 << 4 | time.tm_min % 10;
    bcd[0] = time.tm_sec / 10 << 4 | time.tm_sec % 10;
}

void ab_enable_writes(bool en)
{
    // Get the current control reg value
    uint8_t reg_value = 0;
    ab_read_reg(AB_REG_CONT1, &reg_value);

    if (en)
    {
        // Set the write enable bit
        reg_value |= AB_WRITE_RTC;
    }

    else
    {
        // Reset the write enable bit
        reg_value &= ~AB_WRITE_RTC;
    }

    // Set the new value
    ab_write_reg(AB_REG_CONT1, reg_value);
}

void ab_unlock(uint8_t key_value) { ab_write_reg(AB_REG_KEY, key_value); }

void ab_enable_trickle_charging()
{
    // Unlock the relevant regs (they are locked automatically on a write)
    ab_unlock(AB_TRK_KEY);

    // Enable, schottkey diode and 3K resistor load
    uint8_t value = AB_TRK_TCS_EN | AB_TRK_DID_SCH | AB_TRK_ROT_3K;
    ab_write_reg(AB_REG_TRICKLE, value);
}

void ab_time_get(struct tm *time)
{
    // Get the time
    uint8_t data[6] = {0};
    ab_read_burst(AB_REG_TIME, data, 6);

    // Convert from BCD to tm
    *time = ab_bcd_to_tm(data);
}

void ab_time_set(struct tm time)
{
    // Enable writes
    ab_enable_writes(true);

    // Do the conversion
    uint8_t bcd[6] = {0};
    ab_tm_to_bcd(time, bcd);

    // Write into the regs
    ab_write_burst(AB_REG_TIME, bcd, 6);

    // Disable writes for safety
    ab_enable_writes(false);
}

void ab_wdg_start(uint32_t timeout_ms) {}

void ab_wdg_stop() {}

void ab_wdg_pat() {}
