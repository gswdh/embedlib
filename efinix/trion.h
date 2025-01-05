#ifndef _TRION_H_
#define _TRION_H_

#include <stdbool.h>
#include <stdint.h>

#define TRION_SPI_TX_BLOCK_SIZE (1024)

bool     trion_get_cdone();
void     trion_set_cnrst(bool level);
void     trion_set_cnss(bool level);
void     trion_set_csi(bool level);
void     trion_spi_tx(uint8_t *data, uint32_t len);
void     trion_sleep_ms(uint32_t time_ms);
uint32_t trion_get_tick();

bool trion_configure(uint8_t *bitstream, uint32_t len);

#endif