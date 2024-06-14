#include "tmc2208.h"

#include <math.h>

static tmc_microstepping_t tmc_microstepping = TMC_MICROSTEP_HALF;
static float tmc_speed_rpm = 0;

void tmc_start(float speed_rpm)
{
    // Update the local
    tmc_speed_rpm = speed_rpm;

    // Make sure the microstepping is correct
    tmc_set_microstepping(tmc_microstepping);

    // Set the direction
    tmc_set_dir((speed_rpm > 0) ? true : false);

    // Calculate the output PWM freuqency
    uint32_t frequency = (uint32_t)(fabs(speed_rpm) * TMC_FREQ_TO_RPM * pow(2, tmc_microstepping));

    // Set the output frequency
    tmc_set_stp(frequency);

    // Enable the driver (it's inverted)
    tmc_set_enable(false);
}

void tmc_stop(void)
{
    // Disable the driver (it's inverted)
    tmc_set_enable(false);
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