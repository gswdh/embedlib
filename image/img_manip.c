#include "img_manip.h"

void img_rectangle(image_t *img, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, IMG_PIXEL_TYPE colour)
{
    // Check for out of range
    if (x1 > img->w)
        return;

    if (y1 > img->h)
        return;

    // Limit extremes
    if (x2 > img->w)
        x2 = img->w;

    if (y2 > img->h)
        y2 = img->h;

    // Draw in the pixels
    for (uint32_t x = x1; x < x2; x++)
    {
        for (uint32_t y = y1; y < y2; y++)
        {
            uint32_t loc = (y * img->w) + x;
            img->data[loc] = (IMG_PIXEL_TYPE)colour;
        }
    }
}

void img_circle(image_t *img, uint32_t x, uint32_t y, uint32_t d, IMG_PIXEL_TYPE colour)
{
    uint32_t r = d / 2;

    for (uint32_t i = 0; i < d; i++)
    {
        // Find the limits
        int32_t row = y - r + i;

        if (row < 0)
            continue;
        if (row >= img->h)
            continue;

        int32_t col = x - r;

        uint32_t h = 0;

        // Top half of the circle
        if (row < y)
            h = r - i;

        // Bottom half of the circle
        else
            h = i - r;

        uint32_t w = sqrt(pow(r, 2) - pow(h, 2));

        for (uint32_t j = 0; j < (w * 2); j++)
        {
            int32_t loc = col + r - w + j;

            // Is the column placement inbound
            if (loc > img->w)
                continue;
            if (loc < 0)
                continue;

            // Write the pixel
            img->data[(row * img->w) + loc] = (IMG_PIXEL_TYPE)colour;
        }
    }
}

void img_rotate(IMG_PIXEL_TYPE *dst, IMG_PIXEL_TYPE *src, int width, int height, float theta)
{
    // Set the destination image to all white
    // memset(dst, 0, sizeof(uint16_t) * width * height);

    // Calc the required angle in radians
    float phi = theta / 180.0 * IMG_PI;

    // A lookup table for which trig function to use
    float T_inv[2][2] = {{cos(phi), sin(phi)}, {-sin(phi), cos(phi)}};

    // Coords for source image
    int i_ = 0, j_ = 0;

    // Center coords from the source image
    int center_i = round(width / 2.0);
    int center_j = round(height / 2.0);

    // Go through all the pixels and make a new image...
    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            // Calc the position of the pixel for retrieval from the source image
            i_ = (T_inv[0][0] * (i - center_i) + T_inv[0][1] * (j - center_j)) + center_i;
            j_ = (T_inv[1][0] * (i - center_i) + T_inv[1][1] * (j - center_j)) + center_j;

            // If the coords are out of bounds, skip assignment
            if (i_ > 0 && j_ > 0)
            {
                if (i_ < width && j_ < height)
                {
                    // Assign the value
                    dst[i + j * width] = src[i_ + j_ * width];
                }
            }
        }
    }
}

void img_integer_scale(image_t *dst, image_t *src, uint32_t factor)
{
    // Get out of the real work
    if (factor == 1)
    {
        dst = src;
        return;
    }

    // Check for scalability
    if (src->w % factor || src->h % factor)
    {
        dst = NULL;
        return;
    }

    // Setup the new image
    dst->w = src->w / factor;
    dst->h = src->h / factor;
    dst->pixel_size = src->pixel_size;
    dst->data = calloc(dst->w * dst->h, dst->pixel_size);

    // Do the maths
    for (uint32_t x = 0; x < dst->w; x++)
    {
        for (uint32_t y = 0; y < dst->h; y++)
        {
            // Temp pixel
            uint32_t p = 0;

            // Get the data from the source
            for (uint32_t i = 0; i < factor; i++)
            {
                for (uint32_t j = 0; j < factor; j++)
                {
                    // Source coords
                    uint32_t x_t = (x * factor) + i;
                    uint32_t y_t = (y * factor) + j;

                    // Add the data up
                    p += src->data[x_t + (y_t * src->h)];
                }
            }

            // Average the pixel value for the dst
            dst->data[x + (y * dst->h)] = p / pow(factor, 2);
        }

        img_pat_watchdog();
    }
}

void img_darken(image_t *src, double factor)
{
    if(factor > 1) factor = 1;
    if(factor < 0) factor = 0;

    for(uint32_t i = 0; i < (src->w * src->h); i++)
    {
        src->data[i] = (uint8_t)((float)src->data[i] * factor);
    }
}

void __attribute__((weak)) img_pat_watchdog()
{
    return;
}