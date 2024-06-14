#ifndef __TMC2208__
#define __TMC2208__

#include <stdint.h>
#include <stdbool.h>

#define TMC_FREQ_TO_RPM (1)

typedef enum {
    TMC_MICROSTEP_HALF = 1,
    TMC_MICROSTEP_QUARTER = 2,
    TMC_MICROSTEP_EIGHTH = 3,
    TMC_MICROSTEP_SIXTEENTH = 4,
} tmc_microstepping_t;

void tmc_set_enable(bool en);
void tmc_set_dir(bool en);
void tmc_set_stp(uint32_t frequency_hz);
void tmc_uart_tx(uint8_t * data, uint32_t len);
uint32_t tmc_uart_rx(uint8_t * data, uint32_t len);
void tmc_set_ms1(bool en);
void tmc_set_ms2(bool en);

void tmc_start(float speed_rpm);
void tmc_stop(void);
void tmc_set_microstepping(tmc_microstepping_t stepping);

#endif