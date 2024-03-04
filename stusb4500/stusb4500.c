#include "stusb4500.h"

const uint8_t stusb_pdo_locs[3] = {STUSB_PDO_1, STUSB_PDO_2, STUSB_PDO_3};

void __attribute__((weak)) stusb_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
	return;
}

void __attribute__((weak)) stusb_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
	return;
}

void __attribute__((weak)) stusb_set_reset(bool en)
{
	return;
}

bool __attribute__((weak)) stusb_get_attach()
{
	return false;
}

bool __attribute__((weak)) stusb_get_nint()
{
	return false;
}

bool __attribute__((weak)) stusb_get_pok2()
{
	return false;
}

bool __attribute__((weak)) stusb_get_pok3()
{
	return false;
}

void __attribute__((weak)) stusb_delay_ms(uint32_t time_ms)
{
}

static bool stusb_pdo_n_in_range(pdo_select_t n)
{
	if (n > 3)
		return false;
	if (n < 1)
		return false;
	return true;
}

static float stusb_pdo_get_voltage(uint32_t pdo_raw)
{
	float voltage = 0;

	pdo_raw = (pdo_raw >> 10) & 0x3FF;
	voltage = (float)pdo_raw / 20.0;

	return voltage;
}

static float stusb_pdo_get_current(uint32_t pdo_raw)
{
	pdo_raw &= 0x3FF;
	return (float)pdo_raw * 0.01;
}

pdo_select_t stusb_get_selected_pdo()
{
	pdo_select_t pdo_selected = 1;
	pdo_selected = stusb_get_pok2() ? pdo_selected : 2;
	pdo_selected = stusb_get_pok3() ? pdo_selected : 3;
	return pdo_selected;
}

pdo_t stusb_read_pdo(pdo_select_t n)
{
	pdo_t pdo = {0};

	if (!stusb_pdo_n_in_range(n))
	{
		return pdo;
	}

	uint32_t pdo_raw[3] = {0};

	// PDO1:0x85, PDO2:0x89, PDO3:0x8D
	stusb_read_burst(STUSB_PDO_1, (uint8_t *)pdo_raw, 12);

	pdo.voltage = stusb_pdo_get_voltage(pdo_raw[n-1]);
	pdo.current = stusb_pdo_get_current(pdo_raw[n-1]);

	return pdo;
}

pdo_t stusb_read_pdo_selected()
{
	return stusb_read_pdo(stusb_get_selected_pdo());
}

void stusb_write_pdo(pdo_select_t n, pdo_t pdo)
{
	if (!stusb_pdo_n_in_range(n))
	{
		return;
	}

	uint32_t pdo_raw = 0;
	uint32_t voltage = pdo.voltage * 20.0;
	uint32_t current = pdo.current * 100;
	pdo_raw |= (voltage & 0x3FF) << 10;
	pdo_raw |= current & 0x3FF;

	// PDO1:0x85, PDO2:0x89, PDO3:0x8D
	stusb_write_burst(stusb_pdo_locs[n - 1], (uint8_t *)&pdo_raw, 4);
}

void stusb_soft_pd_reset()
{
	uint8_t data = 0;

	// Set the command to SOFT RESET
	data = STUSB_CMD_SOFT_RESET;
	stusb_write_burst(STUSB_TX_HEADER_LOW, &data, 1);

	// Send the command
	data = STUSB_CMD_SEND_CMD;
	stusb_write_burst(STUSB_PD_COMMAND_CTRL, &data, 1);
}
