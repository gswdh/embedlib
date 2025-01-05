#ifndef __TMC2208__
#define __TMC2208__

#include <stdbool.h>
#include <stdint.h>

#define TMC_FREQ_TO_RPM (1)

#define TMC_UART_TIMEOUT_MS (100)

#define TMC_SYNC_WORD (0x05)
#define TMC_WRITE_BIT (0x01)
#define TMC_READ_BIT  (0x00)

typedef enum
{
    TMC_ERROR_OK = 0,
    TMC_ERROR_INIT,
    TMC_ERROR_UART,
    TMC_ERROR_INVALID_PARAM,
    TMC_ERROR_CRC,
    TMC_ERROR_INVALID_RX,
    TMC_ERROR_TIMEOUT,
} tmc_error_t;

typedef enum
{
    TMC_MICROSTEP_2 = 0,
    TMC_MICROSTEP_4,
    TMC_MICROSTEP_8,
    TMC_MICROSTEP_16,
    TMC_MICROSTEP_32,
    TMC_MICROSTEP_64,
    TMC_MICROSTEP_128,
    TMC_MICROSTEP_256,
} tmc_microstepping_t;

typedef struct __attribute__((packed))
{
    uint8_t idx : 7;
    uint8_t write : 1;
} tmc_addr_t;

typedef struct __attribute__((packed))
{
    uint8_t    sync;
    uint8_t    slave;
    tmc_addr_t addr;
    uint32_t   payload;
    uint8_t    crc;
} tmc_write_datagram_t;

typedef struct __attribute__((packed))
{
    uint8_t    sync;
    uint8_t    slave;
    tmc_addr_t addr;
    uint8_t    crc;
} tmc_read_datagram_t;

_Static_assert(sizeof(tmc_read_datagram_t) == 4, "sizeof(tmc_read_datagram_t) != 4");
_Static_assert(sizeof(tmc_write_datagram_t) == 8, "sizeof(tmc_write_datagram_t) != 8");
_Static_assert(sizeof(tmc_addr_t) == 1, "sizeof(tmc_addr_t) != 1");

void        tmc_set_enable(bool en);
void        tmc_set_dir(bool en);
void        tmc_set_stp(uint32_t frequency_hz);
tmc_error_t tmc_uart_tx(const uint8_t *data, const uint32_t len);
tmc_error_t tmc_uart_rx(uint8_t *data, const uint32_t len, const uint32_t timeout_ms);
void        tmc_set_ms1(bool en);
void        tmc_set_ms2(bool en);

tmc_error_t tmc_init(const tmc_microstepping_t stepping, const uint32_t steps_per_rev);
tmc_error_t tmc_start(const float speed_rpm);
tmc_error_t tmc_stop(void);
tmc_error_t tmc_set_microstepping(const tmc_microstepping_t stepping);

#endif
