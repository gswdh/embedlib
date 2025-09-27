/**
 * @file    tmc2209.h
 * @brief   TMC2209 stepper motor driver library header
 * @version 1.0.0
 *
 * This header file provides the complete API for the TMC2209 stepper motor
 * driver library. The TMC2209 is a silent stepper motor driver with advanced
 * features including StealthChop, SpreadCycle, stall detection, and speed ramping.
 *
 * The library supports:
 * - Silent stepper motor operation (StealthChop mode)
 * - High-torque operation (SpreadCycle mode)
 * - Microstepping from 1/2 to 1/256 steps
 * - Stall detection and protection
 * - Smooth speed ramping with linear interpolation
 * - Real-time status monitoring
 * - Temperature and current monitoring
 *
 * @author  TMC2209 Library Team
 * @date    2024
 */

#ifndef __TMC2209__
#define __TMC2209__

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/* CONSTANTS AND DEFINITIONS                                                  */
/* ========================================================================== */

/**
 * @brief Frequency to RPM conversion factor
 *
 * Used for frequency-based speed calculations.
 * This constant may be adjusted based on motor specifications.
 */
#define TMC_FREQ_TO_RPM (1)

/**
 * @brief UART communication timeout in milliseconds
 *
 * Maximum time to wait for UART responses from TMC2209.
 * Used for read operations to prevent indefinite blocking.
 */
#define TMC_UART_TIMEOUT_MS (100)

/**
 * @brief TMC2209 UART sync word
 *
 * Synchronization byte used to identify TMC2209 datagrams.
 * Must be 0x05 as specified in the TMC2209 datasheet.
 */
#define TMC_SYNC_WORD (0x05)

/**
 * @brief Write operation bit for address field
 *
 * Set in the address field to indicate a write operation.
 * Used to distinguish between read and write datagrams.
 */
#define TMC_WRITE_BIT (0x01)

/**
 * @brief Read operation bit for address field
 *
 * Clear in the address field to indicate a read operation.
 * Used to distinguish between read and write datagrams.
 */
#define TMC_READ_BIT (0x00)

/* ========================================================================== */
/* TMC2209 REGISTER ADDRESSES                                                 */
/* ========================================================================== */

/**
 * @brief TMC2209 register addresses
 *
 * These constants define the addresses of all TMC2209 registers
 * as specified in the TMC2209 datasheet. Used for register
 * read/write operations via UART communication.
 */

/* Global configuration and status registers */
#define TMC_REG_GCONF        0x00 /* Global configuration */
#define TMC_REG_GSTAT        0x01 /* Global status */
#define TMC_REG_IFCNT        0x02 /* Interface counter */
#define TMC_REG_SLAVECONF    0x03 /* Slave configuration */
#define TMC_REG_OTP_PROG     0x04 /* OTP programming */
#define TMC_REG_OTP_READ     0x05 /* OTP read */
#define TMC_REG_FACTORY_CONF 0x06 /* Factory configuration */

/* Driver configuration registers */
#define TMC_REG_SHORT_CONF    0x09 /* Short circuit configuration */
#define TMC_REG_DRV_CONF      0x0A /* Driver configuration */
#define TMC_REG_GLOBAL_SCALER 0x0B /* Global scaler */
#define TMC_REG_OFFSET_READ   0x0C /* Offset read */

/* Current and power management registers */
#define TMC_REG_IHOLD_IRUN 0x10 /* Hold and run current */
#define TMC_REG_TPOWERDOWN 0x11 /* Power down time */

/* Motion control registers */
#define TMC_REG_TSTEP     0x12 /* Step time */
#define TMC_REG_TPWMTHRS  0x13 /* StealthChop threshold */
#define TMC_REG_TCOOLTHRS 0x14 /* CoolStep threshold */
#define TMC_REG_THIGH     0x15 /* Stall detection threshold */
#define TMC_REG_XDIRECT   0x2D /* Direct control */

/* Advanced control registers */
#define TMC_REG_VDCMODE    0x33 /* VDC mode */
#define TMC_REG_CHOPCONF   0x6C /* Chopper configuration */
#define TMC_REG_COOLCONF   0x6D /* CoolStep configuration */
#define TMC_REG_DCCTRL     0x6E /* DC control */
#define TMC_REG_DRV_STATUS 0x6F /* Driver status */
#define TMC_REG_PWMCONF    0x70 /* PWM configuration */
#define TMC_REG_PWM_SCALE  0x71 /* PWM scale */
#define TMC_REG_ENCM_CTRL  0x72 /* Encoder control */
#define TMC_REG_LOST_STEPS 0x73 /* Lost steps counter */

/* ========================================================================== */
/* ERROR CODES AND ENUMS                                                      */
/* ========================================================================== */

/**
 * @brief TMC2209 error codes
 *
 * Defines all possible error conditions that can occur during
 * TMC2209 operations. Used for error reporting and debugging.
 */
typedef enum
{
    TMC_OK = 0,              /* Operation successful */
    TMC_ERROR_INIT,          /* Initialization failed */
    TMC_ERROR_UART,          /* UART communication error */
    TMC_ERROR_INVALID_PARAM, /* Invalid parameter */
    TMC_ERROR_CRC,           /* CRC checksum error */
    TMC_ERROR_INVALID_RX,    /* Invalid response */
    TMC_ERROR_TIMEOUT,       /* Operation timeout */
} tmc_error_t;

/**
 * @brief Microstepping resolution options
 *
 * Defines the available microstepping resolutions for the TMC2209.
 * Higher microstepping provides smoother motion but requires higher step rates.
 *
 * @note The actual microstepping value is 2^resolution (e.g., TMC_MICROSTEP_8 = 1/8 step)
 */
typedef enum
{
    TMC_MICROSTEP_2 = 0, /* 1/2 step (2 microsteps per full step) */
    TMC_MICROSTEP_4,     /* 1/4 step (4 microsteps per full step) */
    TMC_MICROSTEP_8,     /* 1/8 step (8 microsteps per full step) */
    TMC_MICROSTEP_16,    /* 1/16 step (16 microsteps per full step) */
    TMC_MICROSTEP_32,    /* 1/32 step (32 microsteps per full step) */
    TMC_MICROSTEP_64,    /* 1/64 step (64 microsteps per full step) */
    TMC_MICROSTEP_128,   /* 1/128 step (128 microsteps per full step) */
    TMC_MICROSTEP_256,   /* 1/256 step (256 microsteps per full step) */
} tmc_microstepping_t;

/**
 * @brief TMC2209 operating modes
 *
 * Defines the two main operating modes of the TMC2209 driver.
 * Each mode is optimized for different applications.
 */
typedef enum
{
    TMC_STEALTHCHOP = 0, /* Silent operation mode - optimized for quiet operation */
    TMC_SPREADCYCLE = 1, /* High torque mode - optimized for maximum torque */
} tmc_mode_t;

/* ========================================================================== */
/* DATA STRUCTURES                                                            */
/* ========================================================================== */

/**
 * @brief TMC2209 configuration structure
 *
 * Contains all configuration parameters for the TMC2209 driver.
 * This structure is used to store driver settings and state information.
 */
typedef struct
{
    /* Basic driver settings */
    uint8_t             node_address;  /* TMC2209 node address (0-3) */
    uint32_t            steps_per_rev; /* Motor steps per revolution */
    tmc_microstepping_t microstepping; /* Microstepping resolution */
    tmc_mode_t          mode;          /* Operating mode */

    /* Current settings */
    uint16_t current_hold;       /* Hold current in mA */
    uint16_t current_run;        /* Run current in mA */
    uint16_t current_hold_delay; /* Hold delay in steps */

    /* Threshold settings */
    uint8_t stealthchop_threshold; /* StealthChop threshold */
    uint8_t coolstep_threshold;    /* CoolStep threshold */
    uint8_t stall_threshold;       /* Stall detection threshold */

    /* Motor control state */
    bool     enabled;           /* Driver enable state */
    bool     direction;         /* Motor direction (true=forward) */
    uint32_t step_frequency;    /* Current step frequency in Hz */
    float    current_speed_rpm; /* Current speed in RPM */

    /* Speed ramping state variables */
    bool     ramping_active;   /* Ramping operation active flag */
    float    start_speed_rpm;  /* Starting speed for ramping */
    float    target_speed_rpm; /* Target speed for ramping */
    uint32_t ramp_end_tick;    /* Ramping end time */
    uint32_t ramp_start_tick;  /* Ramping start time */
    uint32_t ramp_tick;        /* Last ramping update time */
} tmc_config_t;

/**
 * @brief TMC2209 address field structure
 *
 * Packed structure for the address field in TMC2209 datagrams.
 * Contains the register index and read/write bit.
 */
typedef struct __attribute__((packed))
{
    uint8_t idx : 7;   /* Register index (7 bits) */
    uint8_t write : 1; /* Write bit (1 bit) */
} tmc_addr_t;

/**
 * @brief TMC2209 write datagram structure
 *
 * Structure for write datagrams sent to TMC2209.
 * Contains sync word, slave address, register address, payload, and CRC.
 */
typedef struct __attribute__((packed))
{
    uint8_t    sync;    /* Synchronization word (0x05) */
    uint8_t    slave;   /* Slave address */
    tmc_addr_t addr;    /* Register address and write bit */
    uint32_t   payload; /* 32-bit payload data */
    uint8_t    crc;     /* CRC8 checksum */
} tmc_write_datagram_t;

/**
 * @brief TMC2209 read datagram structure
 *
 * Structure for read datagrams sent to TMC2209.
 * Contains sync word, slave address, register address, and CRC.
 */
typedef struct __attribute__((packed))
{
    uint8_t    sync;  /* Synchronization word (0x05) */
    uint8_t    slave; /* Slave address */
    tmc_addr_t addr;  /* Register address and read bit */
    uint8_t    crc;   /* CRC8 checksum */
} tmc_read_datagram_t;

/* ========================================================================== */
/* STRUCTURE SIZE VALIDATION                                                  */
/* ========================================================================== */

/* Compile-time assertions to ensure correct structure sizes */
_Static_assert(sizeof(tmc_read_datagram_t) == 4, "sizeof(tmc_read_datagram_t) != 4");
_Static_assert(sizeof(tmc_write_datagram_t) == 8, "sizeof(tmc_write_datagram_t) != 8");
_Static_assert(sizeof(tmc_addr_t) == 1, "sizeof(tmc_addr_t) != 1");

/* ========================================================================== */
/* PLATFORM-SPECIFIC INTERFACE FUNCTIONS                                      */
/* ========================================================================== */

/**
 * @brief Platform-specific UART transmission function
 * @param data Pointer to data buffer
 * @param len Number of bytes to transmit
 * @return TMC_OK on success, error code otherwise
 *
 * This function must be implemented by the user to provide UART
 * transmission capability. The function should transmit the specified
 * number of bytes via UART to the TMC2209.
 */
tmc_error_t tmc_uart_tx(const uint8_t *data, const uint32_t len);

/**
 * @brief Platform-specific UART reception function
 * @param data Pointer to receive buffer
 * @param len Number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 * @return TMC_OK on success, error code otherwise
 *
 * This function must be implemented by the user to provide UART
 * reception capability. The function should receive the specified
 * number of bytes via UART from the TMC2209 within the timeout period.
 */
tmc_error_t tmc_uart_rx(uint8_t *data, const uint32_t len, const uint32_t timeout_ms);

/**
 * @brief Platform-specific hardware initialization function
 * @return TMC_OK on success, error code otherwise
 *
 * This function must be implemented by the user to initialize
 * the hardware interface for the TMC2209. Should configure GPIO
 * pins, timers, and other hardware resources.
 */
tmc_error_t tmc_hw_init(void);

/**
 * @brief Platform-specific GPIO direction control
 * @param dir Direction pin state (true=forward, false=reverse)
 *
 * This function must be implemented by the user to control
 * the direction GPIO pin connected to the TMC2209 DIR input.
 */
void tmc_gpio_dir_write(const bool dir);

/**
 * @brief Platform-specific GPIO enable control
 * @param val Enable pin state (true=disabled, false=enabled)
 *
 * This function must be implemented by the user to control
 * the enable GPIO pin connected to the TMC2209 NEN input.
 * Note: NEN is active low, so true means disabled.
 */
void tmc_gpio_nen_write(const bool val);

/**
 * @brief Platform-specific GPIO MS1 control
 * @param val MS1 pin state
 *
 * This function must be implemented by the user to control
 * the MS1 GPIO pin for TMC2209 node address configuration.
 */
void tmc_gpio_ms1_write(const bool val);

/**
 * @brief Platform-specific GPIO MS2 control
 * @param val MS2 pin state
 *
 * This function must be implemented by the user to control
 * the MS2 GPIO pin for TMC2209 node address configuration.
 */
void tmc_gpio_ms2_write(const bool val);

/**
 * @brief Platform-specific timer frequency control
 * @param freq_hz Step frequency in Hz
 *
 * This function must be implemented by the user to configure
 * the timer for step generation at the specified frequency.
 * Used for motor speed control.
 */
void tmc_tim_step_freq(const float freq_hz);

/* ========================================================================== */
/* INITIALIZATION AND CONFIGURATION FUNCTIONS                                */
/* ========================================================================== */

/**
 * @brief Initialize TMC2209 driver
 * @param stepping Microstepping mode
 * @param steps_per_rev Steps per revolution of the motor
 * @param address TMC2209 node address (0-3)
 * @return TMC_OK on success, error code otherwise
 *
 * Initializes the TMC2209 driver with default settings and verifies
 * communication. Sets up motor parameters and driver configuration.
 * This function must be called before using any other TMC2209 functions.
 */
tmc_error_t
tmc_init(const tmc_microstepping_t stepping, const uint32_t steps_per_rev, const uint8_t address);

/**
 * @brief Set TMC2209 node address
 * @param address Node address (0-3)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the TMC2209 node address by configuring MS1 and MS2 pins.
 * Address is encoded as: MS2(MSB) MS1(LSB)
 */
tmc_error_t tmc_set_address(uint8_t address);

/**
 * @brief Configure TMC2209 with custom settings
 * @param config Pointer to configuration structure
 * @return TMC_OK on success, error code otherwise
 *
 * Applies a complete configuration to the TMC2209 driver.
 * Configures chopper, current, thresholds, and PWM settings.
 */
tmc_error_t tmc_configure(const tmc_config_t *config);

/**
 * @brief Set motor current settings
 * @param hold_current Hold current in mA
 * @param run_current Run current in mA
 * @param hold_delay Hold delay in steps
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the motor current settings for hold and run modes.
 * Current values are automatically scaled by the TMC2209.
 */
tmc_error_t tmc_set_current(uint16_t hold_current, uint16_t run_current, uint16_t hold_delay);

/**
 * @brief Set TMC2209 operating mode
 * @param mode Operating mode (StealthChop or SpreadCycle)
 * @return TMC_OK on success, error code otherwise
 *
 * Switches between StealthChop (silent) and SpreadCycle (high torque) modes.
 * StealthChop provides silent operation, SpreadCycle provides maximum torque.
 */
tmc_error_t tmc_set_mode(tmc_mode_t mode);

/**
 * @brief Set microstepping resolution
 * @param stepping Microstepping mode
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the microstepping resolution from 1/2 to 1/256 steps.
 * Higher microstepping provides smoother motion but requires higher step rates.
 */
tmc_error_t tmc_set_microstepping(const tmc_microstepping_t stepping);

/**
 * @brief Set TMC2209 threshold values
 * @param stealthchop_threshold StealthChop threshold
 * @param coolstep_threshold CoolStep threshold
 * @param stall_threshold Stall detection threshold
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the threshold values for StealthChop, CoolStep, and stall detection.
 * These thresholds determine when the driver switches between different modes.
 */
tmc_error_t tmc_set_thresholds(uint8_t stealthchop_threshold,
                               uint8_t coolstep_threshold,
                               uint8_t stall_threshold);

/* ========================================================================== */
/* MOTOR CONTROL FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Enable or disable TMC2209 driver
 * @param enable true to enable, false to disable
 * @return TMC_OK on success, error code otherwise
 *
 * Controls the enable pin of the TMC2209 driver.
 * When disabled, the motor is in high-impedance state.
 */
tmc_error_t tmc_enable(bool enable);

/**
 * @brief Set motor direction
 * @param direction true for forward, false for reverse
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the direction pin to control motor rotation direction.
 * Direction is relative to the motor's wiring and microstepping configuration.
 */
tmc_error_t tmc_set_direction(bool direction);

/**
 * @brief Set motor speed in RPM
 * @param speed_rpm Speed in revolutions per minute (can be negative)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the motor speed in RPM. Negative values indicate reverse direction.
 * Automatically calculates step frequency and sets direction.
 */
tmc_error_t tmc_set_speed(float speed_rpm);

/**
 * @brief Set step frequency directly
 * @param frequency_hz Step frequency in Hz
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the step frequency directly in Hz. Used internally by tmc_set_speed()
 * and for direct frequency control.
 */
tmc_error_t tmc_set_frequency(uint32_t frequency_hz);

/**
 * @brief Start motor with specified speed
 * @param speed_rpm Speed in RPM
 * @return TMC_OK on success, error code otherwise
 *
 * Enables the driver and sets the motor speed. Combines tmc_enable()
 * and tmc_set_speed() for convenience.
 */
tmc_error_t tmc_start(const float speed_rpm);

/**
 * @brief Stop motor immediately
 * @return TMC_OK on success, error code otherwise
 *
 * Stops the motor by setting frequency to zero and disabling the driver.
 * Provides immediate stop without ramping.
 */
tmc_error_t tmc_stop(void);

/* ========================================================================== */
/* SPEED RAMPING FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Start speed ramping to target
 * @param target Target speed in RPM
 * @param ramp_rate_rpm_per_sec Ramp rate in RPM per second
 * @return TMC_OK on success, error code otherwise
 *
 * Initiates smooth speed ramping from current speed to target speed.
 * Uses linear interpolation to provide smooth acceleration/deceleration.
 * Handles direction changes automatically.
 */
tmc_error_t tmc_start_ramp(const float target, const float ramp_rate_rpm_per_sec);

/**
 * @brief Poll function for non-blocking speed ramping
 *
 * This function should be called regularly in the main loop to handle
 * speed ramping operations. It processes the ramping engine without
 * blocking the main execution thread.
 */
void tmc_poll(void);

/**
 * @brief Get system tick count for timing
 * @return System tick count in milliseconds
 *
 * This function must be implemented by the user to provide system
 * tick count for timing operations. Should return milliseconds since
 * system startup or a similar monotonic time source.
 */
uint32_t tmc_get_tick(void);

/* ========================================================================== */
/* STATUS AND MONITORING FUNCTIONS                                            */
/* ========================================================================== */

/**
 * @brief Get TMC2209 driver status
 * @param status Pointer to store status register value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the DRV_STATUS register containing motor status information
 * including stall detection, temperature, and current measurements.
 */
tmc_error_t tmc_get_status(uint32_t *status);

/**
 * @brief Get motor position
 * @param position Pointer to store position value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the XDIRECT register containing the current motor position.
 * Position is relative to the last reset or initialization.
 */
tmc_error_t tmc_get_position(uint32_t *position);

/**
 * @brief Get motor speed
 * @param speed Pointer to store speed value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the TSTEP register containing the current step timing.
 * Can be used to calculate actual motor speed.
 */
tmc_error_t tmc_get_speed(uint32_t *speed);

/**
 * @brief Get motor current
 * @param current Pointer to store current value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the actual motor current from the DRV_STATUS register.
 * Current is returned as a 5-bit value representing the current scale.
 */
tmc_error_t tmc_get_current(uint16_t *current);

/**
 * @brief Get motor temperature
 * @param temperature Pointer to store temperature value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the motor temperature from the DRV_STATUS register.
 * Temperature is returned as an 8-bit value representing the temperature scale.
 */
tmc_error_t tmc_get_temperature(uint16_t *temperature);

/**
 * @brief Get stall detection status
 * @param stalled Pointer to store stall status
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the stall detection status from the DRV_STATUS register.
 * Returns true if a stall condition is detected.
 */
tmc_error_t tmc_get_stall_status(bool *stalled);

/**
 * @brief Get human-readable status string
 * @param status Status register value
 * @return Pointer to formatted status string
 *
 * Converts the DRV_STATUS register value into a human-readable string
 * containing all status information including warnings and errors.
 */
char *tmc_get_status_string(uint32_t status);

/* ========================================================================== */
/* REGISTER READ/WRITE FUNCTIONS                                              */
/* ========================================================================== */

/**
 * @brief Write a register to TMC2209
 * @param node TMC2209 node address (0-3)
 * @param addr Register address
 * @param value 32-bit value to write
 * @return TMC_OK on success, error code otherwise
 *
 * Writes a 32-bit value to the specified TMC2209 register.
 * Handles datagram construction, CRC calculation, and UART transmission.
 */
tmc_error_t tmc_write_reg(const uint8_t node, const uint8_t addr, const uint32_t value);

/**
 * @brief Read a register from TMC2209
 * @param node TMC2209 node address (0-3)
 * @param addr Register address
 * @param value Pointer to store the read value
 * @return TMC_OK on success, error code otherwise
 *
 * Reads a 32-bit value from the specified TMC2209 register.
 * Handles datagram construction, CRC verification, and response validation.
 */
tmc_error_t tmc_read_reg(const uint8_t node, const uint8_t addr, uint32_t *value);

#endif /* __TMC2209__ */