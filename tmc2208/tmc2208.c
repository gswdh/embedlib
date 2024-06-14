#include "tmc2208.h"

#include <math.h>

static tmc_microstepping_t tmc_microstepping = TMC_MICROSTEP_HALF;
static uint32_t tmc_steps_per_rev = 400;

void tmc_init(tmc_microstepping_t stepping, uint32_t steps_per_rev)
{
    tmc_microstepping = stepping;
    tmc_steps_per_rev = steps_per_rev;
}

void tmc_start(float speed_rpm)
{
    // Make sure the microstepping is correct
    tmc_set_microstepping(tmc_microstepping);

    // Set the direction
    tmc_set_dir((speed_rpm > 0) ? true : false);

    // Calculate the output PWM freuqency
    uint32_t frequency = (uint32_t)(fabs(speed_rpm) * tmc_steps_per_rev * pow(2, tmc_microstepping) / 60.0);

    // Set the output frequency
    tmc_set_stp(frequency);

    // Enable the driver (it's inverted)
    tmc_set_enable(false);
}

void tmc_stop(void)
{
    // Disable the driver (it's inverted)
    tmc_set_enable(true);
}

void tmc_set_microstepping(tmc_microstepping_t stepping)
{
    // Update the local
    tmc_microstepping = stepping;

    // Set the pins
    switch (tmc_microstepping)
    {
    case TMC_MICROSTEP_HALF:
        tmc_set_ms1(true);
        tmc_set_ms2(false);
        break;
    case TMC_MICROSTEP_QUARTER:
        tmc_set_ms1(false);
        tmc_set_ms2(true);
        break;
    case TMC_MICROSTEP_EIGHTH:
        tmc_set_ms1(false);
        tmc_set_ms2(false);
        break;
    case TMC_MICROSTEP_SIXTEENTH:
        tmc_set_ms1(true);
        tmc_set_ms2(true);
        break;
    default:
        break;
    }
}