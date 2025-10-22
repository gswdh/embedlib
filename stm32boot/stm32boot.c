/**
 * @file    stm32boot.c
 * @brief   STM32 I²C Bootloader Driver Implementation (AN4221 compliant)
 * @version 2.0.0
 */

#include "stm32boot.h"
#include <string.h>

#ifdef STM32
#include "stm32g0xx_hal.h"
#endif

/* Read status byte with BUSY polling support */
static stm32bl_status_t read_status(bool allow_busy, uint32_t timeout_ms)
{
    uint8_t        status;
    uint32_t       poll_count = 0;
    const uint32_t max_polls  = (timeout_ms / STM32BL_DEFAULT_BUSY_POLL_INTERVAL) + 1;

    while (poll_count < max_polls)
    {
        stm32bl_status_t result = stm32bl_i2c_read(
            STM32BL_DEFAULT_I2C_ADDR, &status, 1, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
        if (result != STM32BL_OK)
        {
            return result;
        }

        if (status == STM32BL_ACK)
        {
            return STM32BL_OK;
        }

        if (status == STM32BL_NACK)
        {
            return STM32BL_ERR_NACK;
        }

        if (status == STM32BL_BUSY && allow_busy)
        {
            /* Poll BUSY with interval */
            stm32bl_delay(STM32BL_DEFAULT_BUSY_POLL_INTERVAL);
            poll_count++;
            continue;
        }

        /* Invalid response or BUSY not allowed */
        return STM32BL_ERR_PROTO;
    }

    return STM32BL_ERR_BUSY_TIMEOUT;
}

/* Send command with proper framing: [cmd, ~cmd] */
static stm32bl_status_t send_cmd(uint8_t opcode)
{
    uint8_t          frame[2] = {opcode, (uint8_t)(~opcode)};
    stm32bl_status_t result =
        stm32bl_i2c_write(STM32BL_DEFAULT_I2C_ADDR, frame, 2, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read status response */
    return read_status(false, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

/* Calculate XOR checksum of buffer */
static uint8_t xor_sum(const uint8_t *buf, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        checksum ^= buf[i];
    }
    return checksum;
}

/* Send 4-byte address with XOR checksum */
static stm32bl_status_t send_addr4(uint32_t addr)
{
    uint8_t frame[5];
    frame[0] = (uint8_t)((addr >> 24) & 0xFF); /* A3 */
    frame[1] = (uint8_t)((addr >> 16) & 0xFF); /* A2 */
    frame[2] = (uint8_t)((addr >> 8) & 0xFF);  /* A1 */
    frame[3] = (uint8_t)(addr & 0xFF);         /* A0 */
    frame[4] = xor_sum(frame, 4);              /* XOR checksum */

    stm32bl_status_t result =
        stm32bl_i2c_write(STM32BL_DEFAULT_I2C_ADDR, frame, 5, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
    if (result != STM32BL_OK)
    {
        return result;
    }

    return read_status(false, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

/* Write frame with inter-frame timeout */
static stm32bl_status_t write_frame(const uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    return stm32bl_i2c_write(
        STM32BL_DEFAULT_I2C_ADDR, buf, len, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

/* Read frame with inter-frame timeout */
static stm32bl_status_t read_frame(uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    return stm32bl_i2c_read(STM32BL_DEFAULT_I2C_ADDR, buf, len, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

/* Public API Implementation */
stm32bl_status_t stm32bl_enable_bootloader(void)
{
#ifdef STM32
    FLASH_OBProgramInitTypeDef option_bytes = {0};
    option_bytes.WRPArea                    = OB_WRPAREA_ZONE_A;

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    HAL_FLASHEx_OBGetConfig(&option_bytes);

    // Modify only what you need (for example, set nBOOT_SEL and BOOT0)
    option_bytes.USERConfig &= ~FLASH_OPTR_nBOOT_SEL;
    HAL_FLASHEx_OBProgram(&option_bytes);

    HAL_FLASH_OB_Launch();

    return STM32BL_OK;
#else
    return STM32BL_ERR_INVALID;
#endif
}

stm32bl_status_t stm32bl_enter_bootloader(uint8_t boot0_pin, uint8_t reset_pin)
{
    /* Set BOOT0 high */
    stm32bl_gpio_set(boot0_pin, true);

    /* Reset STM32 */
    stm32bl_gpio_set(reset_pin, false);
    stm32bl_delay(10); /* 10ms low pulse */
    stm32bl_gpio_set(reset_pin, true);

    /* Wait for bootloader to start */
    stm32bl_delay(100);

    /* Verify bootloader communication */
    uint8_t proto_ver;
    return stm32bl_get_version(&proto_ver);
}

stm32bl_status_t stm32bl_get(uint8_t *proto_ver, uint8_t *cmds, uint8_t *num_cmds)
{
    if (!proto_ver || !cmds || !num_cmds)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    /* Send GET command */
    stm32bl_status_t result = send_cmd(STM32BL_CMD_GET);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read response: [N][ProtocolVersion][N command codes] */
    uint8_t n;
    result = read_frame(&n, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    *num_cmds = n;

    /* Read protocol version */
    result = read_frame(proto_ver, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read command list */
    if (n > 0)
    {
        result = read_frame(cmds, n);
        if (result != STM32BL_OK)
        {
            return result;
        }
    }

    /* Read final ACK */
    return read_status(false, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

stm32bl_status_t stm32bl_get_version(uint8_t *proto_ver)
{
    if (!proto_ver)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    /* Send GET VERSION command */
    stm32bl_status_t result = send_cmd(STM32BL_CMD_GET_VERSION);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read protocol version */
    result = read_frame(proto_ver, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read final ACK */
    return read_status(false, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

stm32bl_status_t stm32bl_get_id(uint16_t *pid)
{
    if (!pid)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    /* Send GET ID command */
    stm32bl_status_t result = send_cmd(STM32BL_CMD_GET_ID);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read response: [N][PID_MSB][PID_LSB] */
    uint8_t n;
    result = read_frame(&n, 1);
    if (result != STM32BL_OK)
    {
        return result;
    }

    if (n != 1)
    {
        return STM32BL_ERR_PROTO;
    }

    uint8_t pid_bytes[2];
    result = read_frame(pid_bytes, 2);
    if (result != STM32BL_OK)
    {
        return result;
    }

    *pid = ((uint16_t)pid_bytes[0] << 8) | pid_bytes[1];

    /* Read final ACK */
    return read_status(false, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

stm32bl_status_t stm32bl_read(uint32_t addr, uint8_t *dst, uint16_t len)
{
    if (!dst || len == 0 || len > STM32BL_MAX_WRITE_LEN)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    /* Send READ MEMORY command */
    stm32bl_status_t result = send_cmd(STM32BL_CMD_READ_MEMORY);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send address */
    result = send_addr4(addr);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send length: [N][~N] where N = len - 1 */
    uint8_t n               = (uint8_t)(len - 1);
    uint8_t length_frame[2] = {n, (uint8_t)(~n)};
    result                  = write_frame(length_frame, 2);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read data */
    return read_frame(dst, len);
}

stm32bl_status_t stm32bl_write(uint32_t addr, const uint8_t *src, uint16_t len, bool nostretch)
{
    if (!src || len == 0 || len > STM32BL_MAX_WRITE_LEN)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    uint8_t cmd = nostretch ? STM32BL_CMD_NOSTRETCH_WRITE : STM32BL_CMD_WRITE_MEMORY;

    /* Send WRITE MEMORY command */
    stm32bl_status_t result = send_cmd(cmd);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send address */
    result = send_addr4(addr);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Build payload: [N][data...][CHK] where N = len - 1, CHK = XOR(N, data...) */
    uint8_t  n = (uint8_t)(len - 1);
    uint8_t  payload[STM32BL_MAX_WRITE_LEN + 2]; /* N + data + checksum */
    uint16_t payload_len = 0;

    payload[payload_len++] = n;
    memcpy(&payload[payload_len], src, len);
    payload_len += len;

    /* Calculate checksum: XOR of N and all data bytes */
    uint8_t checksum = n;
    for (uint16_t i = 0; i < len; i++)
    {
        checksum ^= src[i];
    }
    payload[payload_len++] = checksum;

    /* Send payload */
    result = write_frame(payload, payload_len);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read final status (with BUSY polling for No-Stretch) */
    return read_status(nostretch,
                       nostretch ? STM32BL_DEFAULT_BUSY_POLL_TIMEOUT
                                 : STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

stm32bl_status_t stm32bl_erase_pages(const uint16_t *pages, uint16_t count, bool nostretch)
{
    if (!pages || count == 0 || count > STM32BL_MAX_ERASE_ITEMS)
    {
        return STM32BL_ERR_INVALID_PARAM;
    }

    uint8_t cmd = nostretch ? STM32BL_CMD_NOSTRETCH_ERASE : STM32BL_CMD_ERASE;

    /* Send ERASE command */
    stm32bl_status_t result = send_cmd(cmd);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send count: [CountHi CountLo][CHK] where Count = count - 1 */
    uint16_t count_val = count - 1;
    uint8_t  count_frame[3];
    count_frame[0] = (uint8_t)((count_val >> 8) & 0xFF); /* CountHi */
    count_frame[1] = (uint8_t)(count_val & 0xFF);        /* CountLo */
    count_frame[2] = xor_sum(count_frame, 2);            /* CHK */

    result = write_frame(count_frame, 3);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Wait for ACK */
    result = read_status(false, STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send page IDs: 2*count bytes (MSB first) + checksum */
    uint8_t  page_frame[STM32BL_MAX_ERASE_ITEMS * 2 + 1];
    uint16_t frame_len = 0;

    for (uint16_t i = 0; i < count; i++)
    {
        page_frame[frame_len++] = (uint8_t)((pages[i] >> 8) & 0xFF); /* MSB */
        page_frame[frame_len++] = (uint8_t)(pages[i] & 0xFF);        /* LSB */
    }

    /* Add checksum */
    uint8_t checksum        = xor_sum(page_frame, frame_len);
    page_frame[frame_len++] = checksum;

    result = write_frame(page_frame, frame_len);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read final status (with BUSY polling for No-Stretch) */
    return read_status(nostretch,
                       nostretch ? STM32BL_DEFAULT_BUSY_POLL_TIMEOUT
                                 : STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

stm32bl_status_t stm32bl_erase_mass(bool nostretch)
{
    uint8_t cmd = nostretch ? STM32BL_CMD_NOSTRETCH_ERASE : STM32BL_CMD_ERASE;

    /* Send ERASE command */
    stm32bl_status_t result = send_cmd(cmd);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send mass erase: [0xFF 0xFF][0x00] */
    uint8_t mass_frame[3] = {0xFF, 0xFF, 0x00};
    result                = write_frame(mass_frame, 3);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Read final status (with BUSY polling for No-Stretch) */
    return read_status(nostretch,
                       nostretch ? STM32BL_DEFAULT_BUSY_POLL_TIMEOUT
                                 : STM32BL_DEFAULT_INTERFRAME_TIMEOUT);
}

stm32bl_status_t stm32bl_go(uint32_t addr)
{
    /* Send GO command */
    stm32bl_status_t result = send_cmd(STM32BL_CMD_GO);
    if (result != STM32BL_OK)
    {
        return result;
    }

    /* Send address */
    return send_addr4(addr);
}

const char *stm32bl_get_error_string(stm32bl_status_t status)
{
    switch (status)
    {
    case STM32BL_OK:
        return "Success";
    case STM32BL_ERR_INVALID_PARAM:
        return "Invalid parameter";
    case STM32BL_ERR_COMM:
        return "Communication failed";
    case STM32BL_ERR_TIMEOUT:
        return "Timeout";
    case STM32BL_ERR_NACK:
        return "Not acknowledged";
    case STM32BL_ERR_PROTO:
        return "Protocol error";
    case STM32BL_ERR_BUSY_TIMEOUT:
        return "Busy timeout";
    case STM32BL_ERR_VERIFY:
        return "Verification failed";
    case STM32BL_ERR_FLASH_UNLOCK:
        return "Flash unlock failed";
    case STM32BL_ERR_OB_UNLOCK:
        return "Option bytes unlock failed";
    case STM32BL_ERR_OB_GET_CONFIG:
        return "Get option bytes configuration failed";
    case STM32BL_ERR_OB_PROGRAM:
        return "Program option bytes failed";
    case STM32BL_ERR_OB_LAUNCH:
        return "Launch option bytes reload failed";
    default:
        return "Unknown error";
    }
}