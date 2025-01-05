#ifndef _DMGUI_H_
#define _DMGUI_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Screen size
#define GUI_HEIGHT 64
#define GUI_WIDTH  128
#define GUI_SIZE   (GUI_HEIGHT * GUI_WIDTH)

#define GUI_CLR_BLANK 2
#define GUI_CLR_WHITE 1
#define GUI_CLR_BLACK 0

#define GUI_TXT_ALIGN_L 0
#define GUI_TXT_ALIGN_C 1
#define GUI_TXT_ALIGN_R 2

void dmgui_init(uint8_t *data_ptr);
void dmgui_set_pixel(uint8_t x, uint8_t y, uint8_t state);
void dmgui_draw_line(uint8_t x_1, uint8_t y_1, uint8_t x_2, uint8_t y_2, uint8_t state);
void dmgui_draw_rect(uint8_t x_1, uint8_t y_1, uint8_t x_2, uint8_t y_2, uint8_t filled);
void dmgui_add_image(const uint8_t *image);
void dmgui_add_text(char *text, uint8_t x, uint8_t y, uint8_t alignment);
void dmgui_fill_screen(uint8_t colour);
bool dmgui_update_needed();
void dmgui_update_done();

#endif