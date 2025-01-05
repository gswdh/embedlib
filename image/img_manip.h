#ifndef _IMG_H_
#define _IMG_H_

#include <math.h>
#include <stdint.h>
#include <string.h>

#define IMG_PI (3.14159265358979323846)

#define IMG_CLR_YELLOW (0xEFE0)
#define IMG_CLR_BLUE   (0x031F)
#define IMG_CLR_GREEN  (0x07E0)
#define IMG_CLR_BLACK  (0x0000)
#define IMG_CLR_WHITE  (0xFFFF)

#define IMG_PIXEL_TYPE uint8_t

struct image_struct
{
    IMG_PIXEL_TYPE *data;
    uint32_t        w, h;
    uint8_t         pixel_size;
};
typedef struct image_struct image_t;

void img_rectangle(
    image_t *img, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, IMG_PIXEL_TYPE colour);
void img_circle(image_t *img, uint32_t x, uint32_t y, uint32_t d, IMG_PIXEL_TYPE colour);
void img_rotate(IMG_PIXEL_TYPE *dst, IMG_PIXEL_TYPE *src, int width, int height, float theta);
void img_integer_scale(image_t *dst, image_t *src, uint32_t factor);
void img_darken(image_t *src, double factor);
void img_pat_watchdog();

#endif