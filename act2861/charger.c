#include "charger.h"

#include <string.h>

#include "act_configs.h"

#ifdef DEBUG
#include "logging.h"
#define LOG_TAG "CHRG"
#endif

const uint16_t otg_volts_conv[] = {10240, 5120, 2560, 1280, 640, 320, 160, 80, 40, 20, 0};

uint16_t get_otg_reg(float voltage)
{
	voltage -= 2.96;
	voltage *= 1000;

	uint16_t reg = 0;

	for (uint8_t i = 0; i < 11; i++)
	{
		if (voltage > otg_volts_conv[i])
		{
			reg |= (1 << (10 - i));

			voltage -= (float)otg_volts_conv[i];
		}
	}

	return reg;
}

act_error_t CHRG_PollForOTGMode(uint32_t timeout)
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_PollForOTGMode: Start OTG poll.\n");
#endif

	// Get all the data
	uint8_t data = 0;
	uint8_t status;

	// Get the time
	uint32_t t_start = act_get_tick_ms();

	while ((data & 0x07) != 0x02)
	{
		// Read the status register
		status = act_read_regs(ACT_OTG_STATUS, &data, 1);

		// Check result
		if (status != ACT_OK)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_PollForOTGMode: Could not read registers via I2C. %u.\n", status);
#endif
			return status;
		}

		if (act_get_tick_ms() > (t_start + timeout))
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_PollForOTGMode: Poll timed out, OTG error. State = %u\n", (data & 0x07));
#endif
			return ACT_FAILED_OTG_MODE;
		}
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_PollForOTGMode: OTG mode = OTG_REG.\n");
#endif
	return ACT_OK;
}

act_error_t CHRG_VerifyCHGMode()
{
	// Check the system status for the charging mode
	uint8_t chg_status = 0;
	uint8_t status = act_read_regs(ACT_STATUS_2, &chg_status, 1);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Could not read registers. Error code = %u.\n", status);
#endif
		return status;
	}

	// Just get the charge status
	chg_status &= 0x0F;

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_VerifyCHGMode: Verifying the charge mode. Mode = %u.\n", chg_status);
#endif

	// Check the charging status
	switch (chg_status)
	{
	case CHRG_STAT_RST:

		status = ACT_FAILED_CHG_MODE;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging failed, in reset state.\n");
#endif
		break;

	case CHRG_STAT_COND:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Battery being conditioned, low VBAT or short detected.\n");
#endif
		break;

	case CHRG_STAT_SUS:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Conditioning suspended.\n");
#endif

		break;

	case CHRG_STAT_PCOND:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Battery being preconditioned at IPRECHG.\n");
#endif
		break;

	case CHRG_STAT_PCSUS:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Preconditioned at IPRECHG suspended.\n");
#endif

		break;

	case CHRG_STAT_FAST:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging at full current.\n");
#endif

		break;

	case CHRG_STAT_FSUS:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging at full current suspended.\n");
#endif

		break;

	case CHRG_STAT_FULL:

		status = ACT_CHG_DONE;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging complete.\n");
#endif

		break;

	case CHRG_STAT_CFSUS:

		status = ACT_CHG_DONE;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging complete suspended.\n");
#endif

		break;

	case CHRG_STAT_TERM:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging complete, monitoring battery voltage.\n");
#endif

		break;

	case CHRG_STAT_TSUS:

		status = ACT_CHG_IN_PROGRESS;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: monitoring battery voltage suspended.\n");
#endif

		break;

	case CHRG_STAT_FAULT:

		status = ACT_FAILED_CHG_MODE;

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyCHGMode: Charging failed, fault in the state machine.\n");
#endif

		break;
	}

	return status;
}

act_error_t CHRG_VerifyOTGMode(uint32_t timeout)
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_VerifyOTGMode: Starting polling.\n");
#endif

	if (CHRG_PollForOTGMode(timeout) == ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyOTGMode: Charger is in OTG_REG state.\n");
#endif
		return ACT_OK;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_VerifyOTGMode: Charger has not transitioned to OTG mode.\n");
#endif

	// Get all the data
	uint8_t data[33];
	uint8_t status = act_read_regs(0x00, data, 33);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyOTGMode: Could not read registers via I2C. %u.\n", status);
#endif
		return status;
	}

	// Get the OTG state
	uint8_t otg_state = data[ACT_OTG_STATUS] & 0x07;

	// Is the device in OTG_REG state
	if (otg_state != 0x02)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyOTGMode: ACT not in OTG_REG mode. %u instead.\n", otg_state);
#endif

		if (otg_state == 0x03)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: ACT in OTG_HICCUP mode. Output power limit reached.\n");
#endif
		}

		if (data[ACT_OTG_STATUS] & 0x08)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: VBAT exceeded OV.\n");
#endif
		}

		if (data[ACT_OTG_STATUS] & 0x10)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: VBAT too low.\n");
#endif
		}

		if (data[ACT_STATUS_1] & 0x80)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: VBAT not good.\n");
#endif
		}

		if ((data[ACT_TEMP_STATUS] & 0x80) == 0)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: Output voltage below 92%% of OTG voltage setting.\n");
#endif
		}

		if (data[ACT_TEMP_STATUS] & 0x20)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: Battery is too cold.\n");
#endif
		}

		if (data[ACT_TEMP_STATUS] & 0x10)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: Battery is too hot.\n");
#endif
		}

		if (data[ACT_FAULT_1] & 0x08)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: Charger IC in thermal shutdown.\n");
#endif
		}

		if (data[ACT_FAULT_1] & 0x04)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: FETs over current error.\n");
#endif
		}

		if (data[ACT_FAULT_2] & 0x40)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: OTG Hiccup Mode fault.\n");
#endif
		}

		if (data[ACT_FAULT_2] & 0x20)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: OTG VBAT Cutoff fault.\n");
#endif
		}

		if (data[ACT_FAULT_2] & 0x10)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: OTG Vout Overvoltage fault.\n");
#endif
		}

		if (data[ACT_FAULT_2] & 0x04)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_VerifyOTGMode: OTG VBAT Overvoltage fault.\n");
#endif
		}

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_VerifyOTGMode: OTG not working.\n");
#endif

		// Return the Error
		return ACT_FAILED_OTG_MODE;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_VerifyOTGMode: OTG working ACT_OK.\n");
#endif

	return ACT_OK;
}

act_error_t CHRG_EnableCharging(float max_charge_current, float max_input_current)
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableCharging: Entering charging mode.\n");
#endif

	// Bring the charger out of ship mode
	act_set_shipmode_pin(0);

	// Delay to wait for the charger to wake
	act_delay_ms(35);

	// Copy over the configuration from the default
	uint8_t config[33];
	memcpy(config, act_chg_conf, 33);

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableCharging: Setting the max charge current to %fA.\n", max_charge_current);
#endif

	// Get the output current percentage (ILIM is used in OTG mode)
	float percentage = max_charge_current / (float)CHRG_OLIM_A * 100;

	// Cap the extents
	if (percentage > 100)
		percentage = 100;
	if (percentage < 0)
		percentage = 0;

	// Set in the register
	config[0x18] = (uint8_t)percentage & 0x7F;

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableCharging: Setting the max input current to %fA.\n", max_input_current);
#endif

	// Get the output current percentage (ILIM is used in OTG mode)
	percentage = max_input_current / (float)CHRG_ILIM_A * 100;

	// Cap the extents
	if (percentage > 100)
		percentage = 100;
	if (percentage < 0)
		percentage = 0;

	// Set in the register
	config[0x15] = (uint8_t)percentage & 0x7F;

	// Write the entire configuration to the device
	uint8_t status = act_write_regs(0x00, config, 33);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnableCharging: Charge mode commmand unsuccessful. Error code = %u.\n", status);
#endif
		return status;
	}

	// Wait for the charging mode to start
	act_delay_ms(300);

	// Get the status
	status = CHRG_VerifyCHGMode();

	if (status != ACT_OK && status != ACT_CHG_IN_PROGRESS)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnableCharging: Entered charge mode unsuccessfully. Error code = %u.\n", status);
#endif

		uint8_t data[5] = {0};
		act_read_regs(0x02, data, 5);

#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnableCharging: status bytes 0x02 = %u, 0x03 = %u, 0x04 = %u, 0x05 = %u, 0x06 = %u.\n", data[0], data[1], data[2], data[3], data[4]);
#endif
		return status;
	}

	// Clear interrupts
	CHRG_ClearIRQs();

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableCharging: Charging ACT_OK.\n");
#endif

	return ACT_OK;
}

act_error_t CHRG_EnterOTG(float voltage, float max_current)
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnterOTG: Entering On-The-Go mode.\n");
#endif

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnterOTG: SHIPM pin must be low.\n");
#endif

	// Bring the charger out of ship mode
	act_set_shipmode_pin(0);

	// Delay to wait for the charger to wake
	act_delay_ms(50);

	// Reset the device
	uint8_t status = CHRG_Reset();

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnterOTG: Could not reset the device. Error code = %u.\n", status);
#endif
		return status;
	}

	// Copy over the configuration from the default
	uint8_t config[33];
	memcpy(config, act_otg_conf, 33);

	// Check the desired output voltages
	if ((voltage > 15.5) || (voltage < 4.5))
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnterOTG: Error, set output voltage for OTG should be be between 4.5 and 15.5V, not %f.\n", voltage);
#endif
		return ACT_OTG_VOLTAGE_OOR;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnterOTG: Setting OTG voltage to %f.\n", voltage);
#endif

	// We need to set the maximum output voltage
	uint16_t reg = get_otg_reg(voltage);
	config[0x13] = (0xA0) | (((reg) >> 8) & 0x07);
	config[0x14] = reg & 0xFF;

	// Check the desired output voltages
	if (max_current > CHRG_ILIM_A)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnterOTG: Error, requested output current out of range, should be below 3.01A, not %f.\n", max_current);
#endif
		return ACT_OTG_VOLTAGE_OOR;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnterOTG: Setting maximum output current to %f.\n", max_current);
#endif

	// Get the output current percentage (ILIM is used in OTG mode)
	float percentage = max_current / (float)CHRG_ILIM_A * 100;

	// Cap the extents
	if (percentage > 100)
		percentage = 100;
	if (percentage < 0)
		percentage = 0;

	// Set in the register
	config[0x17] = (uint8_t)percentage & 0x7F;

	// Write the entire configuration to the device
	status = act_write_regs(0x00, config, 33);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnterOTG: Entered OTG mode unsuccessfully. Error code = %u.\n", status);
#endif
		return status;
	}

	// Now enable OTG
	act_read_regs(0x0E, config, 1);
	config[0] |= 0x80;
	act_write_regs(0x0E, config, 1);

	// Verify that the OTG mode is working correctly
	status = CHRG_VerifyOTGMode(10);
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnterOTG: Charger failed to enter OTG mode. Error = %u.\n", status);
#endif
		return status;
	}

	// Clear interrupts
	CHRG_ClearIRQs();

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnterOTG: Charger now in OTG mode.\n");
#endif

	// All ACT_OKay
	return ACT_OK;
}

act_error_t CHRG_EnterIdle()
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableIdle: Entering Idle mode.\n");
#endif

	// Set SHIPM pin high
	act_set_shipmode_pin(0);

	act_delay_ms(35);

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableIdle: SHIPM pin must be set high, first.\n");
#endif

	// Set SHIPM pin high
	act_set_shipmode_pin(1);

	act_delay_ms(35);

	// Simply go into SHIP MODE
	uint8_t data[] = {0x22};

	// Set the ship mode bit in the master control reg 1
	uint8_t status = act_write_regs(ACT_MSTR_CNTRL_1, data, 1);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnableIdle: Entered idle mode unsuccessfully. Error code = %u.\n", status);
#endif
		return status;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnableIdle: Charger now in idle mode.\n");
#endif

	// All ACT_OKay
	return ACT_OK;
}

act_error_t CHRG_EnterHiZ()
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_EnterHiZ: Entering HiZ mode.\n");
#endif

	CHRG_Reset();

	// Copy over the configuration from the default
	uint8_t config[33];
	memcpy(config, act_hiz_conf, 33);

	// Write the entire configuration to the device
	uint8_t status = act_write_regs(0x00, config, 33);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_EnterHiZ: Entered HiZ mode unsuccessfully. Error code = %u.\n", status);
#endif
		return status;
	}

	return ACT_OK;
}

act_error_t CHRG_Reset()
{
#ifdef DEBUG
	log_info(LOG_TAG, "ACT_Reset: Reseting device.\n");
#endif

	// Set the reset bit
	uint8_t data[] = {0x01};

	// Set the ship mode bit in the master control reg 1
	uint8_t status = act_write_regs(ACT_MSTR_CNTRL_1, data, 1);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "ACT_Reset: Reset was unsuccessful. Error code = %u.\n", status);
#endif
		return status;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "ACT_Reset: Charger has been reset.\n");
#endif

	// All ACT_OKay
	return ACT_OK;
}

act_error_t CHRG_GetStatus()
{

	return ACT_OK;
}

act_error_t CHRG_ClearIRQs()
{
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_ClearIRQs: Clearing IRQs.\n");
#endif

	// Get all the data
	uint8_t data = 0x80;
	uint8_t status = act_write_regs(ACT_FAULT_1, &data, 1);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_ClearIRQs: Could not write registers via I2C. %u.\n", status);
#endif
		return status;
	}

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_ClearIRQs: Cleared IRQs.\n");
#endif

	return ACT_OK;
}

act_error_t CHRG_IRQHandler(uint8_t state)
{
	// Read the status register
	uint8_t data[33];
	uint8_t status = act_read_regs(ACT_MSTR_CNTRL_1, data, 33);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_IRQHandler: Could not read registers via I2C. %u.\n", status);
#endif
		return status;
	}

	// Set the status to ACT_OK, the following will adjust it accordingly
	status = ACT_OK;
/*
	// Act based on the current state of the state machine
	switch (state)
	{
	case SYS_STATE_OFF:

		asm ("NOP");
		break;

	case SYS_STATE_IDL:

		asm ("NOP");
		break;

	case SYS_STATE_OTG:

		// Check the things that can cause an error
		if ((data[0x05] & 0x20) == 0x20)
			status = ACT_HIGH_VBAT_ERROR;
		if ((data[0x02] & 0x80) == 0x80)
			status = ACT_LOW_VBAT_ERROR;
		if ((data[0x04] & 0x20) == 0x20)
			status = ACT_BAT_TEMP_ERROR;
		if ((data[0x04] & 0x10) == 0x10)
			status = ACT_BAT_TEMP_ERROR;
		if ((data[0x05] & 0x40) == 0x40)
			status = ACT_FET_OC_ERROR;
		if ((data[0x05] & 0x80) == 0x80)
			status = ACT_OVER_TEMP_ERROR;

		break;

	case SYS_STATE_CHG:

		// Check the things that can cause an error
		if ((data[0x05] & 0x20) == 0x20)
			status = ACT_HIGH_VBAT_ERROR;
		if ((data[0x03] & 0x0F) == 0x06)
			status = ACT_CHG_DONE;
		if ((data[0x03] & 0x0F) == 0x09)
			status = ACT_CHG_DONE;
		if ((data[0x03] & 0x10) == 0x10)
			status = ACT_INPUT_VOLTAGE_ERROR;
		if ((data[0x05] & 0x40) == 0x40)
			status = ACT_SAFETY_TIMER_ERROR;
		if ((data[0x00] & 0x80) == 0x80)
			status = ACT_HIZ_MODE_ERROR;

		break;

	case SYS_STATE_HTR:

		// Check the things that can cause an error
		if ((data[0x05] & 0x20) == 0x20)
			status = ACT_HIGH_VBAT_ERROR;
		if ((data[0x02] & 0x80) == 0x80)
			status = ACT_LOW_VBAT_ERROR;
		if ((data[0x04] & 0x20) == 0x20)
			status = ACT_BAT_TEMP_ERROR;
		if ((data[0x04] & 0x10) == 0x10)
			status = ACT_BAT_TEMP_ERROR;
		if ((data[0x05] & 0x40) == 0x40)
			status = ACT_FET_OC_ERROR;
		if ((data[0x05] & 0x80) == 0x80)
			status = ACT_OVER_TEMP_ERROR;

		break;
	}
*/
#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_IRQHandler: Charger status = %u.\n", status);
#endif

#ifdef DEBUG
	log_info(LOG_TAG, "CHRG_IRQHandler: Clearing IRQs.\n");
#endif

	// Finally, clear the IRQs
	if (CHRG_ClearIRQs() != ACT_OK)
	{
		return ACT_I2C_MEM_WRITE_ERROR;
	}

	return status;
}

void CHRG_PrintRegisters()
{
#ifdef DEBUG
	uint8_t data[33];
	act_read_regs(0x00, data, 33);

	log_info(LOG_TAG, "%lu,", act_get_tick_ms());

	for (uint8_t i = 0; i < 32; i++)
	{
		log_info(LOG_TAG, "%u,", data[i]);
	}

	log_info(LOG_TAG, "%u\n", data[32]);
#endif
}
/*
   act_error_t CHRG_GetVBAT(float *vbat)
   {
		// Read the status register
		uint8_t data[2];
		uint8_t status = act_read_regs(ACT_ADC_1, 2, data);

		// Check result
		if (status != ACT_OK)
		{
 #ifdef DEBUG
				log_info(LOG_TAG, "CHRG_GetVBAT: Could not read registers via I2C. %u.\n", status);
 #endif
				return status;
		}

		// Work out output value
		uint16_t adc_out = (data[0] << 6) + data[1];
 * vbat = 0.01527 * ((adc_out >> 2) - 2048);

		return ACT_OK;
   }
 */

act_error_t CHRG_ADCReadMUX(uint8_t adc_read_channel)
{
	uint8_t status = ACT_OK;

	// Check the channel is ACT_OKay
	if (adc_read_channel > CHRG_READ_MUX_AVN)
	{
		return ACT_MUX_ERROR;
	}

	// Get the current value of the reg
	uint8_t reg = 0;
	status = act_read_regs(ACT_ADC_CONF_2, &reg, 1);

	// Set the correct mux selection
	reg &= ~CHRG_READ_MUX_MSK;
	reg |= (adc_read_channel << 3);

	// Write back
	status = act_write_regs(ACT_ADC_CONF_2, &reg, 1);

	return status;
}

act_error_t CHRG_ReadADC(uint8_t adc_read_channel, uint16_t *adc_val)
{
	uint8_t status = ACT_OK;

	// Set the correct mux
	status = CHRG_ADCReadMUX(adc_read_channel);

	// Read the status register
	uint8_t data[2];
	status = act_read_regs(ACT_ADC_1, data, 2);

	// Check result
	if (status != ACT_OK)
	{
#ifdef DEBUG
		log_info(LOG_TAG, "CHRG_GetVBAT: Could not read registers via I2C. %u.\n", status);
#endif
		return status;
	}

	// Work out output value
	*adc_val = (data[0] << 6) + data[1];

	return status;
}

act_error_t CHRG_GetADCResults(CHRG_ADCResults *res)
{
	uint16_t adc_values[7] = {0};
	uint8_t status = ACT_OK;

	// Go through all the channels
	for (uint8_t i = 0; i < 7; i++)
	{
		status = CHRG_ReadADC(i, &adc_values[i]);

		// Check result
		if (status != ACT_OK)
		{
#ifdef DEBUG
			log_info(LOG_TAG, "CHRG_GetADCResults: Error in reading ADC values. %u.\n", status);
#endif
			return status;
		}
	}

	// Get the right klim value
	float klim = 1;

	// Convert all the values
	res->i_in_amps = 0.7633 * ((float)(adc_values[0] >> 2) - 2048) / 0.01 / 33e3;
	res->v_in_volts = 0.02035 * ((float)(adc_values[1] >> 2) - 2048);
	res->v_bat_volts = 0.01527 * ((float)(adc_values[2] >> 2) - 2048);
	res->i_bat_amps = klim * 0.7633 * ((float)(adc_values[3] >> 2) - 2048) / 0.01 / 20e3;
	res->t_th_celcius = 0.003053 * ((float)(adc_values[4] >> 2) - 2048);									 // From ADC to volts
	res->t_th_celcius = (12.3 * res->t_th_celcius * res->t_th_celcius) + (-63 * res->t_th_celcius) + (70.5); // From volts to celcius
	res->t_die_celcius = (0.2707 * (float)(adc_values[5] >> 2)) - 809.49;
	res->v_adc_volts = 0.01527 * ((float)(adc_values[6] >> 2) - 2048);

	return status;
}

/*
   Input Current (IIN) 		IIN = 0.7633*(ADC_OUT[13:2]-2048)/RCS_IN/RlLIM
   CH1 Input Voltage (VIN) 	VIN = 0.02035*(ADC_OUT[13:2]-2048)
   CH2 Output Voltage (VBAT) 	VBAT = 0.01527*(ADC_OUT[13:2]-2048)
   CH3 Output Current (IBAT) 	IBAT = klim*0.7633*( ADC_OUT[13:2]-2048)/RCS_OUT/ROLIM
   CH4 TH 						VTH = 0.003053*( ADC_OUT[13:2]-2048)
   CH5 Die Temperature 		TJ = 0.2707* ADC_OUT[13:2] - 809.49
   CH6 ADC Input 				VADC = 0.01527*(ADC_OUT[13:2]-2048)
 */
