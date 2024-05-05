#include "ssd1309z.h"

void __attribute__((weak)) ssd_spi_write(uint8_t *data, uint32_t len)
{
}

void __attribute__((weak)) ssd_set_dc(bool en)
{
}

void __attribute__((weak)) ssd_set_cs(bool en)
{
}

void __attribute__((weak)) ssd_set_rst(bool en)
{
}

void __attribute__((weak)) ssd_pwr_cont(bool en)
{
}

void __attribute__((weak)) ssd_delay_ms(uint32_t time_ms)
{
}

void ssd_init()
{
	// Make sure the CS pin is set before attempting to send anything
	ssd_set_cs(true);

	// Reset the screen
	ssd_reset();

	// Turn the screen PSU on
	ssd_pwr_cont(true);

	// Wait for it to settle
	ssd_delay_ms(110);

	// Space for the commands
	uint8_t cmd[2] = {0};

	// Turn the display off
	cmd[0] = SSD_CMD_SET_ON;
	ssd_write_cmd(cmd, 1);

	// Set the clock
	cmd[0] = SSD_CMD_CLK_DIV;
	cmd[1] = 0xA0;
	ssd_write_cmd(cmd, 2);

	// Set multiplexer ratio
	cmd[0] = SSD_CMD_MULT_RATIO;
	cmd[1] = 0x40;
	// ssd_write_cmd(cmd, 2);

	// Set start line
	cmd[0] = SSD_CMD_START_LINE;
	ssd_write_cmd(cmd, 1);

	// Set page addressing mode
	cmd[0] = SSD_CMD_MEM_ADDR_MD;
	cmd[1] = 0x00;
	ssd_write_cmd(cmd, 2);

	// Set segment mapping
	cmd[0] = SSD_CMD_SEG_REMAP + 1;
	ssd_write_cmd(cmd, 1);

	// Set the column mapping
	cmd[0] = SSD_CMD_COM_DIR + 8;
	ssd_write_cmd(cmd, 1);

	// Set the com pins
	cmd[0] = SSD_CMD_COM_PIN_HW;
	cmd[1] = 0x12;
	ssd_write_cmd(cmd, 2);

	// Set the contrast
	cmd[0] = SSD_CMD_SET_CONTRAST;
	cmd[1] = 0xFF;
	ssd_write_cmd(cmd, 2);

	// Set the pre charge period
	cmd[0] = SSD_CMD_PRE_PER;
	cmd[1] = 0xD3;
	ssd_write_cmd(cmd, 2);

	// Set the pre charge period
	cmd[0] = SSD_CMD_COMH_DESEL;
	cmd[1] = 0x20;
	ssd_write_cmd(cmd, 2);

	// Set multiplexer ratio
	cmd[0] = SSD_CMD_STOP_SCROLL;
	ssd_write_cmd(cmd, 1);

	// Set the entire display on
	cmd[0] = SSD_CMD_DISP_ON;
	ssd_write_cmd(cmd, 1);

	// Normal display output, no inversions
	cmd[0] = SSD_CMD_INVERT;
	ssd_write_cmd(cmd, 1);

	// Set multiplexer ratio
	cmd[0] = SSD_CMD_STOP_SCROLL;
	ssd_write_cmd(cmd, 1);

	// Set the entire display on
	cmd[0] = SSD_CMD_DISP_ON;
	ssd_write_cmd(cmd, 1);
}

void ssd_reset()
{
	ssd_set_rst(false);

	ssd_delay_ms(1);

	ssd_set_rst(true);

	ssd_delay_ms(1);
}

void ssd_write_cmd(uint8_t *cmd, uint32_t len)
{
	// Set the DC pin
	ssd_set_dc(SSD_DC_PIN_CMD);

	// Set the CS pin low
	ssd_set_cs(SSD_CS_PIN_LOW);

	// Send the data
	ssd_spi_write(cmd, len);

	// Set the CS pin high
	ssd_set_cs(SSD_CS_PIN_HIGH);
}

void ssd_write_data(uint8_t *data, uint32_t len)
{
	// Set the DC pin
	ssd_set_dc(SSD_DC_PIN_DATA);

	// Set the CS pin low
	ssd_set_cs(SSD_CS_PIN_LOW);

	// Send the data
	ssd_spi_write(data, len);

	// Set the CS pin high
	ssd_set_cs(SSD_CS_PIN_HIGH);
}

void ssd_reset_memptr()
{
	// Space for the commands
	uint8_t cmd[1] = {0};

	// Set the column start and end addresses
	cmd[0] = SSD_CMD_COL_STRT_LOW;
	ssd_write_cmd(cmd, 1);
	cmd[0] = SSD_CMD_COL_STRT_HI;
	ssd_write_cmd(cmd, 1);

	// Set start line
	cmd[0] = SSD_CMD_START_LINE | 0x00;
	ssd_write_cmd(cmd, 1);

	// Set the page start and end addresses
	cmd[0] = SSD_CMD_PAGE_START;
	ssd_write_cmd(cmd, 1);
}

void ssd_update_display(uint8_t *disp_buff)
{
	// Set the column and page start addresses
	ssd_reset_memptr();

	// Write the data over SPI
	ssd_write_data(disp_buff, SSD_DISP_BUFFER_LEN);

	// Set the entire display on
	uint8_t cmd[1];
	cmd[0] = SSD_CMD_DISP_ON;
	ssd_write_cmd(cmd, 1);
}
