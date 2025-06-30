/**
 * @file tmc2209_example.c
 * @brief TMC2209 Stepper Motor Driver Example/Test Implementation
 *
 * This file demonstrates how to use the TMC2209 driver for controlling
 * a stepper motor through UART communication.
 */

#include "tmc2209.h"
#include <stdio.h>
#include <string.h>

// Example configuration
static const tmc_config_t example_config = {.node_address       = 0x00,
                                            .steps_per_rev      = 200, // Standard stepper motor
                                            .microstepping      = TMC_MICROSTEP_16,
                                            .mode               = TMC_STEALTHCHOP,
                                            .current_hold       = 500, // 500mA hold current
                                            .current_run        = 800, // 800mA run current
                                            .current_hold_delay = 10,  // 10ms delay
                                            .stealthchop_threshold =
                                                0, // Use StealthChop for all speeds
                                            .coolstep_threshold = 0, // Disable CoolStep
                                            .stall_threshold    = 0, // Disable stall detection
                                            .enabled            = false,
                                            .direction          = true,
                                            .step_frequency     = 0};

/**
 * @brief Example function to initialize TMC2209
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_init(void)
{
    printf("Initializing TMC2209...\n");

    // Initialize driver with 16 microsteps and 200 steps per revolution
    tmc_error_t error = tmc_init(TMC_MICROSTEP_16, 200);
    if (error != TMC_ERROR_OK)
    {
        printf("TMC2209 init failed with error: %d\n", error);
        return error;
    }

    // Configure motor settings
    error = tmc_configure(&example_config);
    if (error != TMC_ERROR_OK)
    {
        printf("TMC2209 configuration failed with error: %d\n", error);
        return error;
    }

    printf("TMC2209 initialized successfully!\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Example function to test motor control
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_motor_test(void)
{
    printf("Starting motor test...\n");

    // Test 1: Start motor at 60 RPM
    printf("Test 1: Starting motor at 60 RPM\n");
    tmc_error_t error = tmc_start(60.0f);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to start motor: %d\n", error);
        return error;
    }

    // Wait for 2 seconds
    // HAL_Delay(2000); // This would be used in actual STM32 code

    // Test 2: Change speed to 120 RPM
    printf("Test 2: Changing speed to 120 RPM\n");
    error = tmc_set_speed(120.0f);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to change speed: %d\n", error);
        return error;
    }

    // Wait for 2 seconds
    // HAL_Delay(2000); // This would be used in actual STM32 code

    // Test 3: Stop motor
    printf("Test 3: Stopping motor\n");
    error = tmc_stop();
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to stop motor: %d\n", error);
        return error;
    }

    printf("Motor test completed successfully!\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Example function to test status monitoring
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_status_test(void)
{
    printf("Testing status monitoring...\n");

    uint32_t status;
    uint16_t current, temperature;
    bool     stalled;

    // Test 1: Get driver status
    tmc_error_t error = tmc_get_status(&status);
    if (error == TMC_ERROR_OK)
    {
        printf("Driver status: 0x%08lX\n", status);
    }
    else
    {
        printf("Failed to get status: %d\n", error);
    }

    // Test 2: Get motor current
    error = tmc_get_current(&current);
    if (error == TMC_ERROR_OK)
    {
        printf("Motor current: %d mA\n", current);
    }
    else
    {
        printf("Failed to get current: %d\n", error);
    }

    // Test 3: Get driver temperature
    error = tmc_get_temperature(&temperature);
    if (error == TMC_ERROR_OK)
    {
        printf("Driver temperature: %d°C\n", temperature);
    }
    else
    {
        printf("Failed to get temperature: %d\n", error);
    }

    // Test 4: Check stall status
    error = tmc_get_stall_status(&stalled);
    if (error == TMC_ERROR_OK)
    {
        printf("Stall status: %s\n", stalled ? "STALLED" : "OK");
    }
    else
    {
        printf("Failed to get stall status: %d\n", error);
    }

    printf("Status monitoring test completed!\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Example function to test current configuration
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_current_test(void)
{
    printf("Testing current configuration...\n");

    // Test different current settings
    tmc_error_t error;

    // Set low current (200mA hold, 400mA run)
    printf("Setting low current (200mA hold, 400mA run)\n");
    error = tmc_set_current(200, 400, 5);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to set low current: %d\n", error);
        return error;
    }

    // Wait a moment
    // HAL_Delay(1000); // This would be used in actual STM32 code

    // Set medium current (500mA hold, 800mA run)
    printf("Setting medium current (500mA hold, 800mA run)\n");
    error = tmc_set_current(500, 800, 10);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to set medium current: %d\n", error);
        return error;
    }

    // Wait a moment
    // HAL_Delay(1000); // This would be used in actual STM32 code

    // Set high current (800mA hold, 1200mA run)
    printf("Setting high current (800mA hold, 1200mA run)\n");
    error = tmc_set_current(800, 1200, 15);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to set high current: %d\n", error);
        return error;
    }

    printf("Current configuration test completed!\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Example function to test microstepping configuration
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_microstepping_test(void)
{
    printf("Testing microstepping configuration...\n");

    tmc_error_t error;

    // Test different microstepping settings
    tmc_microstepping_t microsteps[] = {TMC_MICROSTEP_2,
                                        TMC_MICROSTEP_4,
                                        TMC_MICROSTEP_8,
                                        TMC_MICROSTEP_16,
                                        TMC_MICROSTEP_32,
                                        TMC_MICROSTEP_64,
                                        TMC_MICROSTEP_128,
                                        TMC_MICROSTEP_256};

    const char *microstep_names[] = {"2", "4", "8", "16", "32", "64", "128", "256"};

    for (int i = 0; i < 8; i++)
    {
        printf("Setting microstepping to 1/%s\n", microstep_names[i]);
        error = tmc_set_microstepping(microsteps[i]);
        if (error != TMC_ERROR_OK)
        {
            printf("Failed to set microstepping 1/%s: %d\n", microstep_names[i], error);
            return error;
        }

        // Wait a moment
        // HAL_Delay(500); // This would be used in actual STM32 code
    }

    // Reset to 16 microsteps
    printf("Resetting to 16 microsteps\n");
    error = tmc_set_microstepping(TMC_MICROSTEP_16);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to reset microstepping: %d\n", error);
        return error;
    }

    printf("Microstepping configuration test completed!\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Example function to test mode switching
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_mode_test(void)
{
    printf("Testing mode switching...\n");

    tmc_error_t error;

    // Test StealthChop mode
    printf("Switching to StealthChop mode\n");
    error = tmc_set_mode(TMC_STEALTHCHOP);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to set StealthChop mode: %d\n", error);
        return error;
    }

    // Wait a moment
    // HAL_Delay(1000); // This would be used in actual STM32 code

    // Test SpreadCycle mode
    printf("Switching to SpreadCycle mode\n");
    error = tmc_set_mode(TMC_SPREADCYCLE);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to set SpreadCycle mode: %d\n", error);
        return error;
    }

    // Wait a moment
    // HAL_Delay(1000); // This would be used in actual STM32 code

    // Switch back to StealthChop
    printf("Switching back to StealthChop mode\n");
    error = tmc_set_mode(TMC_STEALTHCHOP);
    if (error != TMC_ERROR_OK)
    {
        printf("Failed to set StealthChop mode: %d\n", error);
        return error;
    }

    printf("Mode switching test completed!\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Complete example function that runs all tests
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_run_all_tests(void)
{
    printf("=== TMC2209 Complete Test Suite ===\n\n");

    // Initialize
    tmc_error_t error = tmc2209_example_init();
    if (error != TMC_ERROR_OK)
    {
        printf("Initialization failed!\n");
        return error;
    }

    // Run all tests
    printf("\n--- Running Status Test ---\n");
    error = tmc2209_example_status_test();
    if (error != TMC_ERROR_OK)
    {
        printf("Status test failed!\n");
        return error;
    }

    printf("\n--- Running Current Test ---\n");
    error = tmc2209_example_current_test();
    if (error != TMC_ERROR_OK)
    {
        printf("Current test failed!\n");
        return error;
    }

    printf("\n--- Running Microstepping Test ---\n");
    error = tmc2209_example_microstepping_test();
    if (error != TMC_ERROR_OK)
    {
        printf("Microstepping test failed!\n");
        return error;
    }

    printf("\n--- Running Mode Test ---\n");
    error = tmc2209_example_mode_test();
    if (error != TMC_ERROR_OK)
    {
        printf("Mode test failed!\n");
        return error;
    }

    printf("\n--- Running Motor Control Test ---\n");
    error = tmc2209_example_motor_test();
    if (error != TMC_ERROR_OK)
    {
        printf("Motor control test failed!\n");
        return error;
    }

    printf("\n=== All Tests Completed Successfully! ===\n");
    return TMC_ERROR_OK;
}

/**
 * @brief Example function to demonstrate register-level access
 * @return tmc_error_t: TMC_ERROR_OK on success
 */
tmc_error_t tmc2209_example_register_test(void)
{
    printf("Testing register-level access...\n");

    uint32_t    value;
    tmc_error_t error;

    // Read GSTAT register
    error = tmc_read_reg(0x00, TMC_REG_GSTAT, &value);
    if (error == TMC_ERROR_OK)
    {
        printf("GSTAT register: 0x%08lX\n", value);
    }
    else
    {
        printf("Failed to read GSTAT: %d\n", error);
    }

    // Read DRV_STATUS register
    error = tmc_read_reg(0x00, TMC_REG_DRV_STATUS, &value);
    if (error == TMC_ERROR_OK)
    {
        printf("DRV_STATUS register: 0x%08lX\n", value);

        // Parse some status bits
        bool     sg_result   = (value >> 24) & 0x01;
        uint16_t cs_actual   = (value >> 16) & 0xFF;
        uint16_t temperature = (value >> 8) & 0xFF;
        bool     stall_guard = (value >> 0) & 0x01;

        printf("  SG_RESULT: %s\n", sg_result ? "Stall detected" : "No stall");
        printf("  CS_ACTUAL: %d mA\n", cs_actual * 32); // Convert to mA
        printf("  Temperature: %d°C\n", temperature);
        printf("  STALL_GUARD: %s\n", stall_guard ? "Stall" : "No stall");
    }
    else
    {
        printf("Failed to read DRV_STATUS: %d\n", error);
    }

    printf("Register-level access test completed!\n");
    return TMC_ERROR_OK;
}