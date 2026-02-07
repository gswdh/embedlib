/**
 * @file    tmc2209.h
 * @brief   TMC2209 stepper motor driver library header
 * @version 2.0.0
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

/* General Configuration Registers */
#define TMC_REG_GCONF      0x00 /* Global configuration */
#define TMC_REG_GSTAT      0x01 /* Global status */
#define TMC_REG_IFCNT      0x02 /* Interface counter */
#define TMC_REG_REPLYDELAY 0x03 /* Reply delay */
#define TMC_REG_IOIN       0x06 /* Input pins */

/* Velocity Dependent Driver Feature Control Register Set */
#define TMC_REG_IHOLD_IRUN 0x10 /* Hold and run current */
#define TMC_REG_TPOWERDOWN 0x11 /* Power down time */
#define TMC_REG_TSTEP      0x12 /* Step time */
#define TMC_REG_TPWMTHRS   0x13 /* StealthChop threshold */
#define TMC_REG_TCOOLTHRS  0x14 /* CoolStep threshold */
#define TMC_REG_THIGH      0x15 /* Stall detection threshold */
#define TMC_REG_VACTUAL    0x22 /* Actual velocity */

/* CoolStep and StallGuard Control Register Set */
#define TMC_REG_SGTHRS    0x40 /* StallGuard threshold */
#define TMC_REG_SG_RESULT 0x41 /* StallGuard result */
#define TMC_REG_COOLCONF  0x42 /* CoolStep configuration */

/* Microstepping Control Register Set */
#define TMC_REG_MSCNT    0x6A /* Microstep counter */
#define TMC_REG_MSCURACT 0x6B /* Microstep current */

/* Driver Register Set */
#define TMC_REG_CHOPCONF   0x6C /* Chopper configuration */
#define TMC_REG_DRV_STATUS 0x6F /* Driver status */

/* PWM Configuration Register Set */
#define TMC_REG_PWMCONF   0x70 /* PWM configuration */
#define TMC_REG_PWM_SCALE 0x71 /* PWM scale */
#define TMC_REG_PWM_AUTO  0x72 /* PWM auto */

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
 * @brief Microstep resolution options
 *
 * Defines the available microstep resolutions for the TMC2209.
 * These values correspond to the MRES field in the CHOPCONF register.
 */
typedef enum
{
    TMC_MICROSTEP_1   = 1,   /* 1 microstep per step */
    TMC_MICROSTEP_2   = 2,   /* 2 microsteps per step */
    TMC_MICROSTEP_4   = 4,   /* 4 microsteps per step */
    TMC_MICROSTEP_8   = 8,   /* 8 microsteps per step */
    TMC_MICROSTEP_16  = 16,  /* 16 microsteps per step */
    TMC_MICROSTEP_32  = 32,  /* 32 microsteps per step */
    TMC_MICROSTEP_64  = 64,  /* 64 microsteps per step */
    TMC_MICROSTEP_128 = 128, /* 128 microsteps per step */
    TMC_MICROSTEP_256 = 256, /* 256 microsteps per step */
} tmc_microstep_t;

/**
 * @brief Standstill mode options
 *
 * Defines the available standstill modes for the TMC2209.
 */
typedef enum
{
    TMC_STANDSTILL_NORMAL         = 0, /* Normal standstill */
    TMC_STANDSTILL_FREEWHEELING   = 1, /* Freewheeling */
    TMC_STANDSTILL_STRONG_BRAKING = 2, /* Strong braking */
    TMC_STANDSTILL_BRAKING        = 3, /* Braking */
} tmc_standstill_mode_t;

/**
 * @brief Current increment options for CoolStep
 */
typedef enum
{
    TMC_CURRENT_INCREMENT_1 = 0,
    TMC_CURRENT_INCREMENT_2 = 1,
    TMC_CURRENT_INCREMENT_4 = 2,
    TMC_CURRENT_INCREMENT_8 = 3,
} tmc_current_increment_t;

/**
 * @brief Measurement count options for CoolStep
 */
typedef enum
{
    TMC_MEASUREMENT_COUNT_32 = 0,
    TMC_MEASUREMENT_COUNT_8  = 1,
    TMC_MEASUREMENT_COUNT_2  = 2,
    TMC_MEASUREMENT_COUNT_1  = 3,
} tmc_measurement_count_t;

/* ========================================================================== */
/* DATA STRUCTURES                                                            */
/* ========================================================================== */

/**
 * @brief TMC2209 ramping parameters structure
 *
 * Contains all ramping parameters for smooth speed transitions.
 * This structure is used to store ramping state and configuration.
 */
typedef struct
{
    /* Ramping state variables */
    float    current_speed;
    bool     ramping_active;        /* Ramping operation active flag */
    float    start_speed_rpm;       /* Starting speed for ramping */
    float    target_speed_rpm;      /* Target speed for ramping */
    uint32_t ramp_end_tick;         /* Ramping end time */
    uint32_t ramp_start_tick;       /* Ramping start time */
    uint32_t ramp_tick;             /* Last ramping update time */
    float    ramp_rate_rpm_per_sec; /* Ramp rate in RPM per second */
} tmc_ramp_t;

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

/**
 * @brief TMC2209 settings structure
 *
 * Contains all current settings and status information.
 */
typedef struct
{
    bool     is_communicating;
    bool     is_setup;
    bool     software_enabled;
    uint16_t microsteps_per_step;
    bool     inverse_motor_direction_enabled;
    bool     stealth_chop_enabled;
    uint8_t  standstill_mode;
    uint8_t  irun_percent;
    uint8_t  irun_register_value;
    uint8_t  ihold_percent;
    uint8_t  ihold_register_value;
    uint8_t  iholddelay_percent;
    uint8_t  iholddelay_register_value;
    bool     automatic_current_scaling_enabled;
    bool     automatic_gradient_adaptation_enabled;
    uint8_t  pwm_offset;
    uint8_t  pwm_gradient;
    bool     cool_step_enabled;
    bool     analog_current_scaling_enabled;
    bool     internal_sense_resistors_enabled;
} tmc_settings_t;

/**
 * @brief TMC2209 status structure
 *
 * Contains detailed status information from the driver.
 */
typedef struct
{
    uint32_t over_temperature_warning : 1;
    uint32_t over_temperature_shutdown : 1;
    uint32_t short_to_ground_a : 1;
    uint32_t short_to_ground_b : 1;
    uint32_t low_side_short_a : 1;
    uint32_t low_side_short_b : 1;
    uint32_t open_load_a : 1;
    uint32_t open_load_b : 1;
    uint32_t over_temperature_120c : 1;
    uint32_t over_temperature_143c : 1;
    uint32_t over_temperature_150c : 1;
    uint32_t over_temperature_157c : 1;
    uint32_t reserved0 : 4;
    uint32_t current_scaling : 5;
    uint32_t reserved1 : 9;
    uint32_t stealth_chop_mode : 1;
    uint32_t standstill : 1;
} tmc_status_t;

/**
 * @brief TMC2209 global status structure
 *
 * Contains global status information from the driver.
 */
typedef struct
{
    uint32_t reset : 1;
    uint32_t drv_err : 1;
    uint32_t uv_cp : 1;
    uint32_t reserved : 29;
} tmc_global_status_t;

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
 * @brief Write to direction GPIO pin
 * @param dir Direction value (true/false)
 */
void tmc_gpio_dir_write(const bool dir);

/**
 * @brief Write to enable GPIO pin
 * @param val Enable value (true/false)
 */
void tmc_gpio_nen_write(const bool val);

/**
 * @brief Write to MS1 GPIO pin
 * @param val MS1 value (true/false)
 */
void tmc_gpio_ms1_write(const bool val);

/**
 * @brief Write to MS2 GPIO pin
 * @param val MS2 value (true/false)
 */
void tmc_gpio_ms2_write(const bool val);

/**
 * @brief Set step frequency
 * @param freq_hz Frequency in Hz
 */
void tmc_tim_step_freq(const float freq_hz);

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
 * @brief Get system tick count for timing
 * @return System tick count in milliseconds
 *
 * This function must be implemented by the user to provide system
 * tick count for timing operations. Should return milliseconds since
 * system startup or a similar monotonic time source.
 */
uint32_t tmc_get_tick(void);

void tmc_delay_ms(uint32_t delay_ms);

void tmc_step_timer_callback(void);

/* ========================================================================== */
/* INITIALIZATION AND CONFIGURATION FUNCTIONS                                */
/* ========================================================================== */

/**
 * @brief Initialize TMC2209 driver
 * @param serial_address TMC2209 node address (0-3)
 * @param microstepping Microstep resolution from tmc_microstep_t enum (1-256, must be power of 2)
 * @return TMC_OK on success, error code otherwise
 *
 * Initializes the TMC2209 driver with default settings and verifies
 * communication. Sets up motor parameters and driver configuration.
 * This function must be called before using any other TMC2209 functions.
 */
tmc_error_t tmc_init(const uint8_t         serial_address,
                     const tmc_microstep_t microstepping,
                     const uint32_t        steps_per_rev);

/**
 * @brief Set TMC2209 node address
 * @param address Node address (0-3)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the TMC2209 node address by configuring MS1 and MS2 pins.
 * Address is encoded as: MS2(MSB) MS1(LSB)
 */
tmc_error_t tmc_set_address(uint8_t address);

/* ========================================================================== */
/* MICROSTEP CONFIGURATION FUNCTIONS                                          */
/* ========================================================================== */

/**
 * @brief Set microsteps per step
 * @param microsteps Microstep resolution from tmc_microstep_t enum (1-256, must be power of 2)
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the microstep resolution for the TMC2209 driver.
 * Valid values are powers of 2 from 1 to 256 microsteps per step.
 * Higher microstep values provide smoother operation but reduce maximum speed.
 */
tmc_error_t tmc_set_microsteps_per_step(tmc_microstep_t microsteps);

/**
 * @brief Set microsteps per step using power of two
 * @param exponent Power of two exponent (0-8), microsteps = 2^exponent
 * @return TMC_OK on success, error code otherwise
 *
 * Convenience function to set microstep resolution using power of two notation.
 * Exponent 0 = 1 microstep, exponent 8 = 256 microsteps.
 */
tmc_error_t tmc_set_microsteps_per_step_power_of_two(uint8_t exponent);

/* ========================================================================== */
/* CURRENT CONTROL FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Set run current percentage
 * @param percent Run current percentage (0-100)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the motor current during movement operations.
 * Higher values provide more torque but generate more heat.
 * The actual current depends on the sense resistor configuration.
 */
tmc_error_t tmc_set_run_current(uint8_t percent);

/**
 * @brief Set hold current percentage
 * @param percent Hold current percentage (0-100)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the motor current when the motor is stationary (holding position).
 * Lower values reduce power consumption and heat generation.
 * Should typically be 10-50% of the run current.
 */
tmc_error_t tmc_set_hold_current(uint8_t percent);

/**
 * @brief Set hold delay percentage
 * @param percent Hold delay percentage (0-100)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the delay before switching from run current to hold current.
 * Prevents current switching during rapid step sequences.
 * Higher values maintain run current longer after movement stops.
 */
tmc_error_t tmc_set_hold_delay(uint8_t percent);

/**
 * @brief Set all current values at once
 * @param run_current_percent Run current percentage (0-100)
 * @param hold_current_percent Hold current percentage (0-100)
 * @param hold_delay_percent Hold delay percentage (0-100)
 * @return TMC_OK on success, error code otherwise
 *
 * Convenience function to configure all current-related settings in one call.
 * Atomically updates run current, hold current, and hold delay.
 */
tmc_error_t tmc_set_all_current_values(uint8_t run_current_percent,
                                       uint8_t hold_current_percent,
                                       uint8_t hold_delay_percent);

/**
 * @brief Set RMS current in milliamps
 * @param mA RMS current in milliamps
 * @param r_sense Sense resistor value in ohms
 * @param hold_multiplier Hold current multiplier (default 0.5)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets motor current based on actual RMS current and sense resistor value.
 * Provides more precise current control than percentage-based functions.
 * Automatically calculates appropriate register values.
 */
tmc_error_t tmc_set_rms_current(uint16_t mA, float r_sense, float hold_multiplier);

/* ========================================================================== */
/* CHOPPER CONFIGURATION FUNCTIONS                                            */
/* ========================================================================== */

/**
 * @brief Enable double edge chopping
 * @return TMC_OK on success, error code otherwise
 *
 * Enables double edge chopping for improved current regulation.
 * Provides more accurate current control and reduced current ripple.
 */
tmc_error_t tmc_enable_double_edge(void);

/**
 * @brief Disable double edge chopping
 * @return TMC_OK on success, error code otherwise
 *
 * Disables double edge chopping, using single edge chopping instead.
 * May provide better performance in some applications.
 */
tmc_error_t tmc_disable_double_edge(void);

/**
 * @brief Enable VSense voltage sensing
 * @return TMC_OK on success, error code otherwise
 *
 * Enables voltage sensing for improved current regulation.
 * Provides more accurate current control by monitoring supply voltage.
 */
tmc_error_t tmc_enable_vsense(void);

/**
 * @brief Disable VSense voltage sensing
 * @return TMC_OK on success, error code otherwise
 *
 * Disables voltage sensing, using fixed current regulation instead.
 * May be preferred in applications with stable supply voltage.
 */
tmc_error_t tmc_disable_vsense(void);

/* ========================================================================== */
/* MOTOR DIRECTION CONTROL FUNCTIONS                                          */
/* ========================================================================== */

/**
 * @brief Enable inverse motor direction
 * @return TMC_OK on success, error code otherwise
 *
 * Inverts the motor direction by swapping the direction pin logic.
 * Useful when motor wiring requires direction reversal.
 */
tmc_error_t tmc_enable_inverse_motor_direction(void);

/**
 * @brief Disable inverse motor direction
 * @return TMC_OK on success, error code otherwise
 *
 * Disables motor direction inversion, using normal direction logic.
 * This is the default configuration.
 */
tmc_error_t tmc_disable_inverse_motor_direction(void);

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
tmc_error_t tmc_enable_driver(bool enable);

/**
 * @brief Set motor direction
 * @param direction true for forward, false for reverse
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the direction pin to control motor rotation direction.
 * Direction is relative to the motor's wiring and microstepping configuration.
 */
void tmc_set_direction(bool direction);
/**
 * @brief Set motor speed
 * @param speed_rpm Speed in RPM (can be negative for reverse)
 *
 * Sets the motor speed without changing direction.
 * Positive values for forward, negative for reverse.
 */
void tmc_set_speed(const float speed_rpm);

/**
 * @brief Set motor velocity with direction
 * @param speed_rpm Speed in RPM (can be negative for reverse)
 *
 * Sets the motor velocity and automatically handles direction.
 * Combines speed setting with direction control.
 */
void tmc_set_velocity(const float speed_rpm);

/**
 * @brief Start motor movement
 * @param speed_rpm Speed in RPM (can be negative for reverse)
 *
 * Starts motor movement with the specified speed.
 * Automatically enables the driver and sets direction.
 * Combines speed setting, direction control, and driver enable.
 */
void tmc_go(const float speed_rpm);

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

/* ========================================================================== */
/* ANGLE ROTATION FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Rotate angle
 * @param turns Number of turns to rotate
 * @param turns_per_second Turns per second
 * @return TMC_OK on success, error code otherwise
 *
 * Rotates the motor by the specified number of turns at the specified speed.
 */
tmc_error_t tmc_start_rotate_angle(const float turns, const float turns_per_second);

/* ========================================================================== */
/* ENGINE FUNCTIONS                                                    */
/* ========================================================================== */

/**
 * @brief Poll function for non-blocking speed ramping
 *
 * This function should be called regularly in the main loop to handle
 * speed ramping operations. It processes the ramping engine without
 * blocking the main execution thread.
 */
void tmc_poll(void);

/* ========================================================================== */
/* REGISTER-BASED CONFIGURATION FUNCTIONS                                     */
/* ========================================================================== */

/**
 * @brief Set standstill mode
 * @param mode Standstill mode from tmc_standstill_mode_t enum
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the motor behavior when stationary.
 * Options include normal, freewheeling, strong braking, and braking modes.
 */
tmc_error_t tmc_set_standstill_mode(tmc_standstill_mode_t mode);

/**
 * @brief Enable automatic current scaling
 * @return TMC_OK on success, error code otherwise
 *
 * Enables automatic current scaling based on motor load.
 * Improves efficiency by reducing current when load is light.
 */
tmc_error_t tmc_enable_automatic_current_scaling(void);

/**
 * @brief Disable automatic current scaling
 * @return TMC_OK on success, error code otherwise
 *
 * Disables automatic current scaling, using fixed current values.
 * May be preferred for applications requiring consistent torque.
 */
tmc_error_t tmc_disable_automatic_current_scaling(void);

/**
 * @brief Enable automatic gradient adaptation
 * @return TMC_OK on success, error code otherwise
 *
 * Enables automatic gradient adaptation for improved motor control.
 * Automatically adjusts PWM parameters based on motor characteristics.
 */
tmc_error_t tmc_enable_automatic_gradient_adaptation(void);

/**
 * @brief Disable automatic gradient adaptation
 * @return TMC_OK on success, error code otherwise
 *
 * Disables automatic gradient adaptation, using fixed PWM parameters.
 * May be preferred for applications requiring predictable behavior.
 */
tmc_error_t tmc_disable_automatic_gradient_adaptation(void);

/**
 * @brief Set PWM offset
 * @param pwm_amplitude PWM amplitude (0-255)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the PWM offset for motor current control.
 * Higher values provide more current but may increase noise.
 */
tmc_error_t tmc_set_pwm_offset(uint8_t pwm_amplitude);

/**
 * @brief Set PWM gradient
 * @param pwm_amplitude PWM amplitude (0-255)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the PWM gradient for motor current control.
 * Controls the rate of current change during operation.
 */
tmc_error_t tmc_set_pwm_gradient(uint8_t pwm_amplitude);

/**
 * @brief Set power down delay
 * @param power_down_delay Power down delay (minimum 2 for StealthChop auto tuning)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the delay before powering down the motor driver.
 * Minimum value of 2 is required for StealthChop auto tuning.
 */
tmc_error_t tmc_set_power_down_delay(uint8_t power_down_delay);

/**
 * @brief Set reply delay
 * @param delay Reply delay (0-15, minimum 2 for bidirectional communication)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the delay before the TMC2209 responds to UART commands.
 * Minimum value of 2 is required for bidirectional communication.
 */
tmc_error_t tmc_set_reply_delay(uint8_t delay);

/**
 * @brief Move at velocity
 * @param microsteps_per_period Microsteps per period
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the motor velocity using the VACTUAL register.
 * Positive values for forward, negative for reverse.
 */
tmc_error_t tmc_move_at_velocity(int32_t microsteps_per_period);

/**
 * @brief Move using step/dir interface
 * @return TMC_OK on success, error code otherwise
 *
 * Switches to step/direction interface mode.
 * Motor control is then handled via step and direction pins.
 */
tmc_error_t tmc_move_using_step_dir_interface(void);

/**
 * @brief Enable StealthChop mode
 * @return TMC_OK on success, error code otherwise
 *
 * Enables StealthChop mode for silent motor operation.
 * Provides smooth, quiet operation at low to medium speeds.
 */
tmc_error_t tmc_enable_stealth_chop(void);

/**
 * @brief Disable StealthChop mode
 * @return TMC_OK on success, error code otherwise
 *
 * Disables StealthChop mode, enabling SpreadCycle mode.
 * SpreadCycle provides higher torque and better high-speed performance.
 */
tmc_error_t tmc_disable_stealth_chop(void);

/**
 * @brief Set StealthChop duration threshold
 * @param duration_threshold Duration threshold
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the threshold for switching between StealthChop and SpreadCycle.
 * Lower values favor StealthChop, higher values favor SpreadCycle.
 */
tmc_error_t tmc_set_stealth_chop_duration_threshold(uint32_t duration_threshold);

/**
 * @brief Set StallGuard threshold
 * @param stall_guard_threshold StallGuard threshold (0-255)
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the StallGuard threshold for stall detection.
 * Lower values provide more sensitive stall detection.
 */
tmc_error_t tmc_set_stall_guard_threshold(uint8_t stall_guard_threshold);

/**
 * @brief Enable CoolStep
 * @param lower_threshold Lower threshold (1-15)
 * @param upper_threshold Upper threshold (0-15, 0-2 recommended)
 * @return TMC_OK on success, error code otherwise
 *
 * Enables CoolStep for automatic current reduction based on load.
 * Improves efficiency by reducing current when load is light.
 */
tmc_error_t tmc_enable_cool_step(uint8_t lower_threshold, uint8_t upper_threshold);

/**
 * @brief Disable CoolStep
 * @return TMC_OK on success, error code otherwise
 *
 * Disables CoolStep, using fixed current values.
 * May be preferred for applications requiring consistent torque.
 */
tmc_error_t tmc_disable_cool_step(void);

/**
 * @brief Set CoolStep current increment
 * @param current_increment Current increment from tmc_current_increment_t enum
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the current increment for CoolStep operation.
 * Controls how much current is reduced when load is light.
 */
tmc_error_t tmc_set_cool_step_current_increment(tmc_current_increment_t current_increment);

/**
 * @brief Set CoolStep measurement count
 * @param measurement_count Measurement count from tmc_measurement_count_t enum
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the number of measurements for CoolStep operation.
 * More measurements provide more stable current control.
 */
tmc_error_t tmc_set_cool_step_measurement_count(tmc_measurement_count_t measurement_count);

/**
 * @brief Set CoolStep duration threshold
 * @param duration_threshold Duration threshold
 * @return TMC_OK on success, error code otherwise
 *
 * Sets the duration threshold for CoolStep operation.
 * Controls when CoolStep becomes active based on step duration.
 */
tmc_error_t tmc_set_cool_step_duration_threshold(uint32_t duration_threshold);

/**
 * @brief Enable analog current scaling
 * @return TMC_OK on success, error code otherwise
 *
 * Enables analog current scaling for improved current control.
 * Uses analog feedback for more accurate current regulation.
 */
tmc_error_t tmc_enable_analog_current_scaling(void);

/**
 * @brief Disable analog current scaling
 * @return TMC_OK on success, error code otherwise
 *
 * Disables analog current scaling, using digital current control.
 * May be preferred for applications with stable supply voltage.
 */
tmc_error_t tmc_disable_analog_current_scaling(void);

/**
 * @brief Use external sense resistors
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the TMC2209 to use external sense resistors.
 * Required when using external current sensing resistors.
 */
tmc_error_t tmc_use_external_sense_resistors(void);

/**
 * @brief Use internal sense resistors
 * @return TMC_OK on success, error code otherwise
 *
 * Configures the TMC2209 to use internal sense resistors.
 * Used when no external current sensing resistors are present.
 */
tmc_error_t tmc_use_internal_sense_resistors(void);

/* ========================================================================== */
/* STATUS AND MONITORING FUNCTIONS                                            */
/* ========================================================================== */

/**
 * @brief Get TMC2209 version
 * @return Version number (0x21 for TMC2209)
 *
 * Reads the version number from the TMC2209.
 * Returns 0x21 for TMC2209 devices.
 */
uint8_t tmc_get_version(void);

/**
 * @brief Check if driver is communicating
 * @return true if communicating, false otherwise
 *
 * Verifies communication with the TMC2209 by checking the version.
 * Returns true if the version matches the expected TMC2209 version.
 */
bool tmc_is_communicating(void);

/**
 * @brief Check if driver is setup and communicating
 * @return true if setup and communicating, false otherwise
 *
 * Checks if the TMC2209 is both communicating and properly configured.
 * Verifies that the driver is in serial operation mode.
 */
bool tmc_is_setup_and_communicating(void);

/**
 * @brief Check if driver is communicating but not setup
 * @return true if communicating but not setup, false otherwise
 *
 * Checks if the TMC2209 is communicating but not properly configured.
 * Useful for detecting when initialization is needed.
 */
bool tmc_is_communicating_but_not_setup(void);

/**
 * @brief Check if driver is hardware disabled
 * @return true if hardware disabled, false otherwise
 *
 * Checks if the TMC2209 enable pin is active (hardware disabled).
 * Returns true if the enable pin is high (disabled).
 */
bool tmc_hardware_disabled(void);

/**
 * @brief Get microsteps per step
 * @return Microsteps per step (1-256)
 *
 * Reads the current microstep resolution from the TMC2209.
 * Returns the number of microsteps per full step.
 */
uint16_t tmc_get_microsteps_per_step(void);

/**
 * @brief Get current settings
 * @param settings Pointer to store settings
 * @return TMC_OK on success, error code otherwise
 *
 * Reads all current settings and status from the TMC2209.
 * Provides a comprehensive view of the driver configuration.
 */
tmc_error_t tmc_get_settings(tmc_settings_t *settings);

/**
 * @brief Get driver status
 * @param status Pointer to store status
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the driver status register from the TMC2209.
 * Provides information about temperature, shorts, and other conditions.
 */
tmc_error_t tmc_get_status(tmc_status_t *status);

/**
 * @brief Get global status
 * @param global_status Pointer to store global status
 * @return TMC_OK on success, error code otherwise
 *
 * Reads the global status register from the TMC2209.
 * Provides information about reset, drive errors, and under-voltage.
 */
tmc_error_t tmc_get_global_status(tmc_global_status_t *global_status);

/**
 * @brief Clear reset flag
 * @return TMC_OK on success, error code otherwise
 *
 * Clears the reset flag in the global status register.
 * Should be called after a reset to clear the flag.
 */
tmc_error_t tmc_clear_reset(void);

/**
 * @brief Clear drive error flag
 * @return TMC_OK on success, error code otherwise
 *
 * Clears the drive error flag in the global status register.
 * Should be called after resolving drive errors.
 */
tmc_error_t tmc_clear_drive_error(void);

/**
 * @brief Get interface transmission counter
 * @return Interface transmission counter (0-255)
 *
 * Reads the interface transmission counter from the TMC2209.
 * Increments with each UART transmission for debugging purposes.
 */
uint8_t tmc_get_interface_transmission_counter(void);

/**
 * @brief Get interstep duration
 * @return Interstep duration in clock cycles
 *
 * Reads the interstep duration from the TMC2209.
 * Provides timing information for step generation.
 */
uint32_t tmc_get_interstep_duration(void);

/**
 * @brief Get StallGuard result
 * @return StallGuard result (0-1023)
 *
 * Reads the StallGuard result from the TMC2209.
 * Lower values indicate higher load, higher values indicate lighter load.
 */
uint16_t tmc_get_stall_guard_result(void);

/**
 * @brief Get PWM scale sum
 * @return PWM scale sum (0-255)
 *
 * Reads the PWM scale sum from the TMC2209.
 * Provides information about PWM scaling for current control.
 */
uint8_t tmc_get_pwm_scale_sum(void);

/**
 * @brief Get PWM scale auto
 * @return PWM scale auto (-512 to 511)
 *
 * Reads the automatic PWM scale from the TMC2209.
 * Provides information about automatic PWM scaling.
 */
int16_t tmc_get_pwm_scale_auto(void);

/**
 * @brief Get PWM offset auto
 * @return PWM offset auto (0-255)
 *
 * Reads the automatic PWM offset from the TMC2209.
 * Provides information about automatic PWM offset adjustment.
 */
uint8_t tmc_get_pwm_offset_auto(void);

/**
 * @brief Get PWM gradient auto
 * @return PWM gradient auto (0-255)
 *
 * Reads the automatic PWM gradient from the TMC2209.
 * Provides information about automatic PWM gradient adjustment.
 */
uint8_t tmc_get_pwm_gradient_auto(void);

/**
 * @brief Get microstep counter
 * @return Microstep counter (0-1023)
 *
 * Reads the microstep counter from the TMC2209.
 * Provides information about the current microstep position.
 */
uint16_t tmc_get_microstep_counter(void);

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