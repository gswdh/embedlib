#ifndef __TMC2209__
#define __TMC2209__

#include <stdbool.h>
#include <stdint.h>

#define TMC_FREQ_TO_RPM (1)

#define TMC_UART_TIMEOUT_MS (100)

#define TMC_SYNC_WORD (0x05)
#define TMC_WRITE_BIT (0x01)
#define TMC_READ_BIT  (0x00)

// TMC2209 Register Addresses
#define TMC_REG_GCONF         0x00
#define TMC_REG_GSTAT         0x01
#define TMC_REG_IFCNT         0x02
#define TMC_REG_SLAVECONF     0x03
#define TMC_REG_OTP_PROG      0x04
#define TMC_REG_OTP_READ      0x05
#define TMC_REG_FACTORY_CONF  0x06
#define TMC_REG_SHORT_CONF    0x09
#define TMC_REG_DRV_CONF      0x0A
#define TMC_REG_GLOBAL_SCALER 0x0B
#define TMC_REG_OFFSET_READ   0x0C
#define TMC_REG_IHOLD_IRUN    0x10
#define TMC_REG_TPOWERDOWN    0x11
#define TMC_REG_TSTEP         0x12
#define TMC_REG_TPWMTHRS      0x13
#define TMC_REG_TCOOLTHRS     0x14
#define TMC_REG_THIGH         0x15
#define TMC_REG_XDIRECT       0x2D
#define TMC_REG_VDCMODE       0x33
#define TMC_REG_CHOPCONF      0x6C
#define TMC_REG_COOLCONF      0x6D
#define TMC_REG_DCCTRL        0x6E
#define TMC_REG_DRV_STATUS    0x6F
#define TMC_REG_PWMCONF       0x70
#define TMC_REG_PWM_SCALE     0x71
#define TMC_REG_ENCM_CTRL     0x72
#define TMC_REG_LOST_STEPS    0x73

typedef enum
{
    TMC_OK = 0,
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

typedef enum
{
    TMC_STEALTHCHOP = 0,
    TMC_SPREADCYCLE = 1,
} tmc_mode_t;

typedef struct
{
    uint8_t             node_address;
    uint32_t            steps_per_rev;
    tmc_microstepping_t microstepping;
    tmc_mode_t          mode;
    uint16_t            current_hold;
    uint16_t            current_run;
    uint16_t            current_hold_delay;
    uint8_t             stealthchop_threshold;
    uint8_t             coolstep_threshold;
    uint8_t             stall_threshold;
    bool                enabled;
    bool                direction;
    uint32_t            step_frequency;
} tmc_config_t;

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

// External UART functions that need to be implemented by the user
tmc_error_t tmc_uart_tx(const uint8_t *data, const uint32_t len);
tmc_error_t tmc_uart_rx(uint8_t *data, const uint32_t len, const uint32_t timeout_ms);

// Configuration functions
tmc_error_t tmc_init(const tmc_microstepping_t stepping, const uint32_t steps_per_rev);
tmc_error_t tmc_configure(const tmc_config_t *config);
tmc_error_t tmc_set_current(uint16_t hold_current, uint16_t run_current, uint16_t hold_delay);
tmc_error_t tmc_set_mode(tmc_mode_t mode);
tmc_error_t tmc_set_microstepping(const tmc_microstepping_t stepping);
tmc_error_t tmc_set_thresholds(uint8_t stealthchop_threshold,
                               uint8_t coolstep_threshold,
                               uint8_t stall_threshold);

// Control functions
tmc_error_t tmc_enable(bool enable);
tmc_error_t tmc_set_direction(bool direction);
tmc_error_t tmc_set_speed(float speed_rpm);
tmc_error_t tmc_set_frequency(uint32_t frequency_hz);
tmc_error_t tmc_start(const float speed_rpm);
tmc_error_t tmc_stop(void);

// Status functions
tmc_error_t tmc_get_status(uint32_t *status);
tmc_error_t tmc_get_position(uint32_t *position);
tmc_error_t tmc_get_speed(uint32_t *speed);
tmc_error_t tmc_get_current(uint16_t *current);
tmc_error_t tmc_get_temperature(uint16_t *temperature);
tmc_error_t tmc_get_stall_status(bool *stalled);

// Register read/write functions
tmc_error_t tmc_write_reg(const uint8_t node, const uint8_t addr, const uint32_t value);
tmc_error_t tmc_read_reg(const uint8_t node, const uint8_t addr, uint32_t *value);

#endif
