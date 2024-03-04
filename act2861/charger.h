#ifndef _CHARGER_H_
#define _CHARGER_H_

#include <stdint.h>

#include "act2861.h"

#define CHRG_ILIM_A 3
#define CHRG_OLIM_A 5

#define CHRG_READ_MUX_MSK 0x38
#define CHRG_READ_MUX_IIN 0x00
#define CHRG_READ_MUX_VIN 0x01
#define CHRG_READ_MUX_VBT 0x02
#define CHRG_READ_MUX_IBT 0x03
#define CHRG_READ_MUX_VTH 0x04
#define CHRG_READ_MUX_DTH 0x05
#define CHRG_READ_MUX_AVN 0x06

// A results struct for the ACT ADC
struct CHRG_ADCResultsTag;
typedef struct CHRG_ADCResultsTag
{
	float i_in_amps;
	float v_in_volts;
	float v_bat_volts;
	float i_bat_amps;
	float t_th_celcius;
	float t_die_celcius;
	float v_adc_volts;
}
CHRG_ADCResults;

act_error CHRG_PollForOTGMode(uint32_t timeout);
act_error CHRG_VerifyOTGMode(uint32_t timeout);
act_error CHRG_VerifyCHGMode();

act_error CHRG_EnableCharging(float max_charge_current, float max_input_current);
act_error CHRG_EnterOTG(float voltage, float max_current);
act_error CHRG_EnterIdle();
act_error CHRG_EnterHiZ();
act_error CHRG_Reset();

act_error CHRG_ClearIRQs();
act_error CHRG_GetStatus();
act_error CHRG_IRQHandler(uint8_t state);

act_error CHRG_ADCReadMUX(uint8_t adc_read_channel);
act_error CHRG_ReadADC(uint8_t adc_read_channel, uint16_t *adc_val);
act_error CHRG_GetADCResults(CHRG_ADCResults *res);

void CHRG_PrintRegisters();

#endif
