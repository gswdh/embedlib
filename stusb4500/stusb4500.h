#ifndef __STUSB__
#define __STUSB__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STUSB_I2C_ADDR (0x50)

#define STUSB_FTP_CUST_PASSWORD_REG 0x95
#define STUSB_FTP_CUST_PASSWORD     0x47

#define STUSB_FTP_CTRL_0      0x96
#define STUSB_FTP_CUST_PWR    0x80
#define STUSB_FTP_CUST_RST_N  0x40
#define STUSB_FTP_CUST_REQ    0x10
#define STUSB_FTP_CUST_SECT   0x07
#define STUSB_FTP_CTRL_1      0x97
#define STUSB_FTP_CUST_SER    0xF8
#define STUSB_FTP_CUST_OPCODE 0x07
#define STUSB_RW_BUFFER       0x53
#define STUSB_TX_HEADER_LOW   0x51
#define STUSB_PD_COMMAND_CTRL 0x1A
#define STUSB_DPM_PDO_NUMB    0x70

#define STUSB_READ             0x00
#define STUSB_WRITE_PL         0x01
#define STUSB_WRITE_SER        0x02
#define STUSB_ERASE_SECTOR     0x05
#define STUSB_PROG_SECTOR      0x06
#define STUSB_SOFT_PROG_SECTOR 0x07

#define STUSB_SECTOR_0 0x01
#define STUSB_SECTOR_1 0x02
#define STUSB_SECTOR_2 0x04
#define STUSB_SECTOR_3 0x08
#define STUSB_SECTOR_4 0x10

#define STUSB_PDO_1 0x85
#define STUSB_PDO_2 0x89
#define STUSB_PDO_3 0x8D

#define STUSB_RDO_REG 0x91

#define STUSB_CMD_SOFT_RESET 0x0D
#define STUSB_CMD_SEND_CMD   0x26

typedef enum
{
    PDO1 = 1,
    PDO2 = 2,
    PDO3 = 3
} pdo_select_t;

typedef struct
{
    float voltage;
    float current;
} pdo_t;

// HW interfaces
void stusb_read_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
void stusb_write_burst(uint8_t reg_addr, uint8_t *data, uint16_t length);
void stusb_set_reset(bool en);
bool stusb_get_attach();
bool stusb_get_nint();
bool stusb_get_pok2();
bool stusb_get_pok3();

// Lib
pdo_select_t stusb_get_selected_pdo();
pdo_t        stusb_read_pdo(pdo_select_t n);
pdo_t        stusb_read_pdo_selected();
void         stusb_write_pdo(pdo_select_t n, pdo_t pdo);
void         stusb_soft_pd_reset();

#endif