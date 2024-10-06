#ifndef __GMAX_H__
#define __GMAX_H__

#include <stdint.h>
#include <stdbool.h>

#define GMAX_STREAM_EN_BIT (0x01)

#define GMAX_REG_DTEMP (192)
#define GMAX_REG_EXP0C (3)

typedef enum
{
    PWR_0V7_EN = 0,
    PWR_1V3_EN,
    PWR_3V3A_EN,
    PWR_3V6_EN,
    PWR_4V1_EN,
    PWR_N1V3A_EN,
    SPI_NCS,
    SPI_CLK,
    SPI_MOSI,
    SPI_MISO,
    SYNC_EN,
    FRAME_REQ,
    SYNC_STATUS,
    FRAME_RDY,
    FRAME_PROG,
    SEN_NRESET,
} gmax_pin_t;

typedef enum
{
    GMAX_GAIN_0DB = 0,
    GMAX_GAIN_3_5DB,
    GMAX_GAIN_6DB,
    GMAX_GAIN_8DB,
    GMAX_GAIN_9_5DB,
    GMAX_GAIN_11DB,
    GMAX_GAIN_12DB,
    GMAX_GAIN_13DB,
    GMAX_GAIN_14DB,
} gmax_gain_t;

typedef enum
{
    GMAX_RES_FULL = 0,
    GMAX_RES_VIDEO,
} gmax_resolution_t;

typedef enum
{
    GMAX_OK = 0,
    GMAX_SYNC_FAIL,
} gmax_error_t;

void gmax_delay_ms(const uint32_t time_ms);
bool gmax_pin_read(const gmax_pin_t pin);
void gmax_pin_write(const gmax_pin_t pin, const bool value);

void gmax_fpga_interface_init(void);
uint16_t gmax_get_training_word(const uint8_t *const gmax_config);
void gmax_sync_word_write(const uint16_t sync_word);
bool gmax_sync_complete(void);
void gmax_sync_mode(const bool en);
void gmax_initiate_frame(void);

gmax_error_t gmax_init(void);
gmax_error_t gmax_deinit(void);
gmax_error_t gmax_frame_request(const uint32_t int_time_us, const gmax_gain_t gain, const gmax_resolution_t res);
gmax_error_t gmax_sensor_temperature(float *const temp_c);

char *gmax_error_string(const gmax_error_t error_code);

#endif