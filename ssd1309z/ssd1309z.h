#ifndef _SSD_H_
#define _SSD_H_

#include <stdint.h>
#include <stdbool.h>

#define SSD_DISP_BUFFER_LEN (1024)

// General commands
#define SSD_CMD_SET_CONTRAST	0x81
#define SSD_CMD_DISP_ON			0xA4
#define SSD_CMD_DISP_OFF		0xA5
#define SSD_CMD_INVERT			0xA6
#define SSD_CMD_SET_ON			0xAF
#define SSD_CMD_SET_OFF			0xAE
#define SSD_CMD_NOP				0xE3
#define SSD_CMD_LOCK			0xFD

// Scrolling commands
#define SSD_CMD_HOR_STEP		0x26
#define SSD_CMD_VER_STEP		0x29
#define SSD_CMD_STOP_SCROLL		0x2E
#define SSD_CMD_ACT_SCROLL		0x2F
#define SSD_CMD_VER_AREA		0xA3
#define SSD_CMD_SCROLL_STP		0x2C

// Address setting commands
#define SSD_CMD_COL_STRT_LOW	0x00
#define SSD_CMD_COL_STRT_HI		0x10
#define SSD_CMD_MEM_ADDR_MD		0x20
#define SSD_CMD_COL_ADDR		0x21
#define SSD_CMD_PAGE_ADDR		0x22
#define SSD_CMD_PAGE_START		0xB0
#define SSD_CMD_START_LINE		0x40
#define SSD_CMD_SEG_REMAP		0xA0
#define SSD_CMD_MULT_RATIO		0xA8
#define SSD_CMD_COM_DIR			0xC0
#define SSD_CMD_DISP_OFFSET		0xD3
#define SSD_CMD_COM_PIN_HW		0xDA
#define SSD_CMD_SET_GPIO		0xDC

// Timing commands
#define SSD_CMD_CLK_DIV			0xD5
#define SSD_CMD_PRE_PER			0xD9
#define SSD_CMD_COMH_DESEL		0xDB

// DC pin definitions
#define SSD_DC_PIN_CMD			0x00
#define SSD_DC_PIN_DATA			0x01

// CS pin definitions
#define SSD_CS_PIN_LOW			0x00
#define SSD_CS_PIN_HIGH			0x01

// Configurations
#define SSD_COM_DIR_63_0		0x08

// Weakly typed driver interfaces
void ssd_spi_write(uint8_t * data, uint32_t len);
void ssd_set_dc(bool en);
void ssd_set_cs(bool en);
void ssd_set_rst(bool en);
void ssd_pwr_cont(bool en);
void ssd_delay_ms(uint32_t time_ms);

// Driver functions
void ssd_init();
void ssd_reset();
void ssd_write_cmd(uint8_t * cmd, uint32_t len);
void ssd_write_data(uint8_t * data, uint32_t len);
void ssd_reset_memptr();
void ssd_update_display(uint8_t * disp_buff);

#endif