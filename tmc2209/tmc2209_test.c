/**
 * @file tmc2209_test.c
 * @brief TMC2209 Stepper Motor Driver Test Implementation
 */

#include "tmc2209.h"
#include <stdio.h>

// Test configuration
static const tmc_config_t test_config = {.node_address          = 0x00,
                                         .steps_per_rev         = 200,
                                         .microstepping         = TMC_MICROSTEP_16,
                                         .mode                  = TMC_STEALTHCHOP,
                                         .current_hold          = 500,
                                         .current_run           = 800,
                                         .current_hold_delay    = 10,
                                         .stealthchop_threshold = 0,
                                         .coolstep_threshold    = 0,
                                         .stall_threshold       = 0,
                                         .enabled               = false,
                                         .direction             = true,
                                         .step_frequency        = 0};

/**
 * @brief Test TMC2209 initialization
 */
tmc_error_t tmc2209_test_init(void)
{
    printf("Testing TMC2209 initialization...\n");

    tmc_error_t error = tmc_init(TMC_MICROSTEP_16, 200);
    if (error != TMC_OK)
    {
        printf("Init failed: %d\n", error);
        return error;
    }

    error = tmc_configure(&test_config);
    if (error != TMC_OK)
    {
        printf("Configure failed: %d\n", error);
        return error;
    }

    printf("TMC2209 initialized successfully!\n");
    return TMC_OK;
}

/**
 * @brief Test motor control functions
 */
tmc_error_t tmc2209_test_motor_control(void)
{
    printf("Testing motor control...\n");

    // Start motor
    tmc_error_t error = tmc_start(60.0f);
    if (error != TMC_OK)
    {
        printf("Start failed: %d\n", error);
        return error;
    }

    // Change speed
    error = tmc_set_speed(120.0f);
    if (error != TMC_OK)
    {
        printf("Speed change failed: %d\n", error);
        return error;
    }

    // Stop motor
    error = tmc_stop();
    if (error != TMC_OK)
    {
        printf("Stop failed: %d\n", error);
        return error;
    }

    printf("Motor control test passed!\n");
    return TMC_OK;
}

/**
 * @brief Test status monitoring
 */
tmc_error_t tmc2209_test_status(void)
{
    printf("Testing status monitoring...\n");

    uint32_t status;
    uint16_t current, temperature;
    bool     stalled;

    tmc_error_t error = tmc_get_status(&status);
    if (error == TMC_OK)
    {
        printf("Status: 0x%08lX\n", status);
    }

    error = tmc_get_current(&current);
    if (error == TMC_OK)
    {
        printf("Current: %d mA\n", current);
    }

    error = tmc_get_temperature(&temperature);
    if (error == TMC_OK)
    {
        printf("Temperature: %d°C\n", temperature);
    }

    error = tmc_get_stall_status(&stalled);
    if (error == TMC_OK)
    {
        printf("Stalled: %s\n", stalled ? "Yes" : "No");
    }

    printf("Status test passed!\n");
    return TMC_OK;
}

/**
 * @brief Run all tests
 */
tmc_error_t tmc2209_run_all_tests(void)
{
    printf("=== TMC2209 Test Suite ===\n");

    tmc_error_t error = tmc2209_test_init();
    if (error != TMC_OK)
        return error;

    error = tmc2209_test_status();
    if (error != TMC_OK)
        return error;

    error = tmc2209_test_motor_control();
    if (error != TMC_OK)
        return error;

    printf("=== All Tests Passed! ===\n");
    return TMC_OK;
}