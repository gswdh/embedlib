#include "ssd1309gfx.h"

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

#define SSD1309GFX_PANEL_W ((int16_t)SSD1309GFX_WIDTH)
#define SSD1309GFX_PANEL_H ((int16_t)SSD1309GFX_HEIGHT)

static ssd1309gfx_error_t ssd1309gfx_check_color(const ssd1309gfx_color_t color)
{
    /* TRANSPARENT is a background-only value. */
    if ((color != SSD1309GFX_BLACK) && (color != SSD1309GFX_WHITE) &&
        (color != SSD1309GFX_INVERSE))
    {
        return SSD1309GFX_ERR_BAD_COLOR;
    }
    return SSD1309GFX_OK;
}

static ssd1309gfx_error_t ssd1309gfx_check_background(const ssd1309gfx_color_t background)
{
    if ((background != SSD1309GFX_BLACK) && (background != SSD1309GFX_WHITE) &&
        (background != SSD1309GFX_INVERSE) && (background != SSD1309GFX_TRANSPARENT))
    {
        return SSD1309GFX_ERR_BAD_BACKGROUND;
    }
    return SSD1309GFX_OK;
}

static ssd1309gfx_error_t ssd1309gfx_check_scale(const uint8_t scale)
{
    if ((scale == 0U) || (scale > (uint8_t)SSD1309GFX_MAX_SCALE))
    {
        return SSD1309GFX_ERR_BAD_SCALE;
    }
    return SSD1309GFX_OK;
}

static ssd1309gfx_error_t ssd1309gfx_check_font(const ssd1309gfx_font_t *font)
{
    if ((font == (const ssd1309gfx_font_t *)0) || (font->glyphs == (const uint8_t *)0))
    {
        return SSD1309GFX_ERR_NULL_FONT;
    }
    if ((font->width == 0U) || (font->width > 8U) || (font->height == 0U) ||
        (font->height > 8U) || (font->advance < font->width) ||
        (font->last_char < font->first_char))
    {
        return SSD1309GFX_ERR_BAD_FONT;
    }
    return SSD1309GFX_OK;
}

/* Unchecked single-pixel write; callers validate the buffer and colour. */
static void
ssd1309gfx_plot(uint8_t *fb, const int16_t x, const int16_t y, const ssd1309gfx_color_t color)
{
    if ((x < 0) || (x >= SSD1309GFX_PANEL_W) || (y < 0) || (y >= SSD1309GFX_PANEL_H))
    {
        return;
    }

    const uint16_t index = (uint16_t)(((uint16_t)(y >> 3) * (uint16_t)SSD1309GFX_PANEL_W) + (uint16_t)x);
    const uint8_t  mask  = (uint8_t)(1U << ((uint16_t)y & 7U));

    switch (color)
    {
        case SSD1309GFX_WHITE:
            fb[index] |= mask;
            break;
        case SSD1309GFX_BLACK:
            fb[index] &= (uint8_t)(~mask);
            break;
        case SSD1309GFX_INVERSE:
            fb[index] ^= mask;
            break;
        default:
            /* TRANSPARENT leaves the pixel alone. */
            break;
    }
}

/* Unchecked horizontal run, used by the span-filling primitives. */
static void ssd1309gfx_span(uint8_t                 *fb,
                            const int16_t            x,
                            const int16_t            y,
                            const int16_t            w,
                            const ssd1309gfx_color_t color)
{
    int16_t start = x;
    int16_t end   = (int16_t)(x + w); /* exclusive */

    if ((w <= 0) || (y < 0) || (y >= SSD1309GFX_PANEL_H))
    {
        return;
    }
    if (start < 0)
    {
        start = 0;
    }
    if (end > SSD1309GFX_PANEL_W)
    {
        end = SSD1309GFX_PANEL_W;
    }
    if (start >= end)
    {
        return;
    }

    const uint16_t row_base = (uint16_t)((uint16_t)(y >> 3) * (uint16_t)SSD1309GFX_PANEL_W);
    const uint8_t  mask     = (uint8_t)(1U << ((uint16_t)y & 7U));
    int16_t        column   = 0;

    for (column = start; column < end; column++)
    {
        const uint16_t index = (uint16_t)(row_base + (uint16_t)column);
        switch (color)
        {
            case SSD1309GFX_WHITE:
                fb[index] |= mask;
                break;
            case SSD1309GFX_BLACK:
                fb[index] &= (uint8_t)(~mask);
                break;
            case SSD1309GFX_INVERSE:
                fb[index] ^= mask;
                break;
            default:
                break;
        }
    }
}

/* -------------------------------------------------------------------------
 * Whole-buffer operations
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t ssd1309gfx_fill(uint8_t *fb, const ssd1309gfx_color_t color)
{
    uint16_t index = 0U;

    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    for (index = 0U; index < (uint16_t)SSD1309GFX_BUFFER_SIZE; index++)
    {
        switch (color)
        {
            case SSD1309GFX_WHITE:
                fb[index] = 0xFFU;
                break;
            case SSD1309GFX_BLACK:
                fb[index] = 0x00U;
                break;
            default:
                fb[index] = (uint8_t)(~fb[index]);
                break;
        }
    }
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_clear(uint8_t *fb)
{
    return ssd1309gfx_fill(fb, SSD1309GFX_BLACK);
}

ssd1309gfx_error_t ssd1309gfx_shift_vertical(uint8_t *fb, const int16_t rows)
{
    int16_t column = 0;

    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (rows == 0)
    {
        return SSD1309GFX_OK;
    }

    const int16_t magnitude = (rows < 0) ? (int16_t)(-rows) : rows;
    if (magnitude >= SSD1309GFX_PANEL_H)
    {
        return ssd1309gfx_fill(fb, SSD1309GFX_BLACK);
    }

    /*
     * Each column is a bit column spread across the pages: row r lives in bit
     * r of a notional 64-bit word, so a vertical shift is a shift of that
     * word.  Moving content up (toward row 0) is a right shift.
     */
    for (column = 0; column < SSD1309GFX_PANEL_W; column++)
    {
        uint64_t bits = 0U;
        uint16_t page = 0U;

        for (page = 0U; page < (uint16_t)SSD1309GFX_PAGES; page++)
        {
            const uint16_t index = (uint16_t)((page * (uint16_t)SSD1309GFX_PANEL_W) + (uint16_t)column);
            bits |= ((uint64_t)fb[index]) << (page * 8U);
        }

        bits = (rows > 0) ? (bits >> (uint32_t)magnitude) : (bits << (uint32_t)magnitude);

        for (page = 0U; page < (uint16_t)SSD1309GFX_PAGES; page++)
        {
            const uint16_t index = (uint16_t)((page * (uint16_t)SSD1309GFX_PANEL_W) + (uint16_t)column);
            fb[index]            = (uint8_t)((bits >> (page * 8U)) & 0xFFU);
        }
    }
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_shift_horizontal(uint8_t *fb, const int16_t columns)
{
    uint16_t page = 0U;

    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (columns == 0)
    {
        return SSD1309GFX_OK;
    }

    const int16_t magnitude = (columns < 0) ? (int16_t)(-columns) : columns;
    if (magnitude >= SSD1309GFX_PANEL_W)
    {
        return ssd1309gfx_fill(fb, SSD1309GFX_BLACK);
    }

    for (page = 0U; page < (uint16_t)SSD1309GFX_PAGES; page++)
    {
        uint8_t *const row  = &fb[page * (uint16_t)SSD1309GFX_PANEL_W];
        const int16_t  kept = (int16_t)(SSD1309GFX_PANEL_W - magnitude);
        int16_t        i    = 0;

        if (columns > 0)
        {
            for (i = 0; i < kept; i++)
            {
                row[i] = row[i + magnitude];
            }
            for (i = kept; i < SSD1309GFX_PANEL_W; i++)
            {
                row[i] = 0x00U;
            }
        }
        else
        {
            for (i = (int16_t)(SSD1309GFX_PANEL_W - 1); i >= magnitude; i--)
            {
                row[i] = row[i - magnitude];
            }
            for (i = 0; i < magnitude; i++)
            {
                row[i] = 0x00U;
            }
        }
    }
    return SSD1309GFX_OK;
}

/* -------------------------------------------------------------------------
 * Pixels
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t
ssd1309gfx_draw_pixel(uint8_t *fb, const int16_t x, const int16_t y, const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    ssd1309gfx_plot(fb, x, y, color);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t
ssd1309gfx_get_pixel(const uint8_t *fb, const int16_t x, const int16_t y, uint8_t *set)
{
    if (fb == (const uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (set == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_OUTPUT;
    }

    if ((x < 0) || (x >= SSD1309GFX_PANEL_W) || (y < 0) || (y >= SSD1309GFX_PANEL_H))
    {
        *set = 0U;
        return SSD1309GFX_OK;
    }

    const uint16_t index = (uint16_t)(((uint16_t)(y >> 3) * (uint16_t)SSD1309GFX_PANEL_W) + (uint16_t)x);
    const uint8_t  mask  = (uint8_t)(1U << ((uint16_t)y & 7U));

    *set = ((fb[index] & mask) != 0U) ? 1U : 0U;
    return SSD1309GFX_OK;
}

/* -------------------------------------------------------------------------
 * Lines
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t ssd1309gfx_draw_hline(uint8_t                 *fb,
                                         const int16_t            x,
                                         const int16_t            y,
                                         const int16_t            w,
                                         const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    ssd1309gfx_span(fb, x, y, w, color);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_draw_vline(uint8_t                 *fb,
                                         const int16_t            x,
                                         const int16_t            y,
                                         const int16_t            h,
                                         const ssd1309gfx_color_t color)
{
    int16_t row = 0;

    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if (h <= 0)
    {
        return SSD1309GFX_OK;
    }

    for (row = y; row < (int16_t)(y + h); row++)
    {
        ssd1309gfx_plot(fb, x, row, color);
    }
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_draw_line(uint8_t                 *fb,
                                        const int16_t            x0,
                                        const int16_t            y0,
                                        const int16_t            x1,
                                        const int16_t            y1,
                                        const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    /* Axis-aligned cases go through the cheaper span/column writers. */
    if (y0 == y1)
    {
        const int16_t left  = (x0 < x1) ? x0 : x1;
        const int16_t right = (x0 < x1) ? x1 : x0;
        ssd1309gfx_span(fb, left, y0, (int16_t)(right - left + 1), color);
        return SSD1309GFX_OK;
    }
    if (x0 == x1)
    {
        const int16_t top    = (y0 < y1) ? y0 : y1;
        const int16_t bottom = (y0 < y1) ? y1 : y0;
        int16_t       row    = 0;
        for (row = top; row <= bottom; row++)
        {
            ssd1309gfx_plot(fb, x0, row, color);
        }
        return SSD1309GFX_OK;
    }

    /* Bresenham, integer only, both endpoints inclusive. */
    int16_t       x    = x0;
    int16_t       y    = y0;
    const int16_t dx   = (x1 > x0) ? (int16_t)(x1 - x0) : (int16_t)(x0 - x1);
    const int16_t dy   = (y1 > y0) ? (int16_t)(y1 - y0) : (int16_t)(y0 - y1);
    const int16_t step_x = (x0 < x1) ? 1 : -1;
    const int16_t step_y = (y0 < y1) ? 1 : -1;
    int32_t       error  = (int32_t)dx - (int32_t)dy;

    for (;;)
    {
        ssd1309gfx_plot(fb, x, y, color);
        if ((x == x1) && (y == y1))
        {
            break;
        }

        const int32_t error2 = error * 2;
        if (error2 > -(int32_t)dy)
        {
            error -= (int32_t)dy;
            x = (int16_t)(x + step_x);
        }
        if (error2 < (int32_t)dx)
        {
            error += (int32_t)dx;
            y = (int16_t)(y + step_y);
        }
    }
    return SSD1309GFX_OK;
}

/* -------------------------------------------------------------------------
 * Rectangles
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t ssd1309gfx_draw_rect(uint8_t                 *fb,
                                        const int16_t            x,
                                        const int16_t            y,
                                        const int16_t            w,
                                        const int16_t            h,
                                        const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if ((w <= 0) || (h <= 0))
    {
        return SSD1309GFX_OK;
    }

    const int16_t bottom = (int16_t)(y + h - 1);
    const int16_t right  = (int16_t)(x + w - 1);
    int16_t       row    = 0;

    ssd1309gfx_span(fb, x, y, w, color);
    if (h > 1)
    {
        ssd1309gfx_span(fb, x, bottom, w, color);
    }
    /* Corners are already covered by the two spans. */
    for (row = (int16_t)(y + 1); row < bottom; row++)
    {
        ssd1309gfx_plot(fb, x, row, color);
        if (w > 1)
        {
            ssd1309gfx_plot(fb, right, row, color);
        }
    }
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_fill_rect(uint8_t                 *fb,
                                        const int16_t            x,
                                        const int16_t            y,
                                        const int16_t            w,
                                        const int16_t            h,
                                        const ssd1309gfx_color_t color)
{
    int16_t row = 0;

    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if ((w <= 0) || (h <= 0))
    {
        return SSD1309GFX_OK;
    }

    for (row = y; row < (int16_t)(y + h); row++)
    {
        ssd1309gfx_span(fb, x, row, w, color);
    }
    return SSD1309GFX_OK;
}

/* Unchecked vertical run, the transpose of ssd1309gfx_span. */
static void ssd1309gfx_vrun(uint8_t                 *fb,
                            const int16_t            x,
                            const int16_t            y,
                            const int16_t            h,
                            const ssd1309gfx_color_t color)
{
    int16_t row = 0;

    if (h <= 0)
    {
        return;
    }
    for (row = y; row < (int16_t)(y + h); row++)
    {
        ssd1309gfx_plot(fb, x, row, color);
    }
}

/* Plot the four mirrored points of one circle octant sample. */
static void ssd1309gfx_circle_points(uint8_t                 *fb,
                                     const int16_t            cx,
                                     const int16_t            cy,
                                     const int16_t            dx,
                                     const int16_t            dy,
                                     const uint8_t            quadrants,
                                     const ssd1309gfx_color_t color)
{
    /* quadrants is a bitmask: 1 top-right, 2 top-left, 4 bottom-left, 8 bottom-right. */
    if ((quadrants & 0x01U) != 0U)
    {
        ssd1309gfx_plot(fb, (int16_t)(cx + dx), (int16_t)(cy - dy), color);
        ssd1309gfx_plot(fb, (int16_t)(cx + dy), (int16_t)(cy - dx), color);
    }
    if ((quadrants & 0x02U) != 0U)
    {
        ssd1309gfx_plot(fb, (int16_t)(cx - dx), (int16_t)(cy - dy), color);
        ssd1309gfx_plot(fb, (int16_t)(cx - dy), (int16_t)(cy - dx), color);
    }
    if ((quadrants & 0x04U) != 0U)
    {
        ssd1309gfx_plot(fb, (int16_t)(cx - dx), (int16_t)(cy + dy), color);
        ssd1309gfx_plot(fb, (int16_t)(cx - dy), (int16_t)(cy + dx), color);
    }
    if ((quadrants & 0x08U) != 0U)
    {
        ssd1309gfx_plot(fb, (int16_t)(cx + dx), (int16_t)(cy + dy), color);
        ssd1309gfx_plot(fb, (int16_t)(cx + dy), (int16_t)(cy + dx), color);
    }
}

/* Midpoint circle outline, optionally restricted to some quadrants. */
static void ssd1309gfx_circle_outline(uint8_t                 *fb,
                                      const int16_t            cx,
                                      const int16_t            cy,
                                      const int16_t            r,
                                      const uint8_t            quadrants,
                                      const ssd1309gfx_color_t color)
{
    int16_t dx    = 0;
    int16_t dy    = r;
    int16_t error = (int16_t)(1 - r);

    ssd1309gfx_circle_points(fb, cx, cy, 0, r, quadrants, color);

    while (dx < dy)
    {
        dx = (int16_t)(dx + 1);
        if (error < 0)
        {
            error = (int16_t)(error + (2 * dx) + 1);
        }
        else
        {
            dy    = (int16_t)(dy - 1);
            error = (int16_t)(error + (2 * (dx - dy)) + 1);
        }
        ssd1309gfx_circle_points(fb, cx, cy, dx, dy, quadrants, color);
    }
}

/*
 * Fill the left and/or right half of a disc using vertical runs.
 *
 * @p delta stretches every run downward, which turns the two halves into the
 * rounded caps of a capsule — that is what fill_round_rect needs.  Filling
 * with horizontal spans instead cannot express the stretch, because the
 * straight section between the arcs would be left unpainted.
 */
static void ssd1309gfx_circle_halves(uint8_t                 *fb,
                                     const int16_t            cx,
                                     const int16_t            cy,
                                     const int16_t            r,
                                     const uint8_t            halves,
                                     const int16_t            delta,
                                     const ssd1309gfx_color_t color)
{
    int16_t dx    = 0;
    int16_t dy    = r;
    int16_t error = (int16_t)(1 - r);

    /* halves is a bitmask: 1 right half, 2 left half. */
    while (dx < dy)
    {
        if (error >= 0)
        {
            dy    = (int16_t)(dy - 1);
            error = (int16_t)(error - (2 * dy));
        }
        dx    = (int16_t)(dx + 1);
        error = (int16_t)(error + (2 * dx) + 1);

        if ((halves & 0x01U) != 0U)
        {
            ssd1309gfx_vrun(
                fb, (int16_t)(cx + dx), (int16_t)(cy - dy), (int16_t)((2 * dy) + 1 + delta), color);
            ssd1309gfx_vrun(
                fb, (int16_t)(cx + dy), (int16_t)(cy - dx), (int16_t)((2 * dx) + 1 + delta), color);
        }
        if ((halves & 0x02U) != 0U)
        {
            ssd1309gfx_vrun(
                fb, (int16_t)(cx - dx), (int16_t)(cy - dy), (int16_t)((2 * dy) + 1 + delta), color);
            ssd1309gfx_vrun(
                fb, (int16_t)(cx - dy), (int16_t)(cy - dx), (int16_t)((2 * dx) + 1 + delta), color);
        }
    }
}

ssd1309gfx_error_t ssd1309gfx_draw_round_rect(uint8_t                 *fb,
                                              const int16_t            x,
                                              const int16_t            y,
                                              const int16_t            w,
                                              const int16_t            h,
                                              const int16_t            r,
                                              const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (r < 0)
    {
        return SSD1309GFX_ERR_BAD_RADIUS;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if ((w <= 0) || (h <= 0))
    {
        return SSD1309GFX_OK;
    }

    /* A radius past half the shorter side would fold the corners over. */
    const int16_t shorter = (w < h) ? w : h;
    int16_t       radius  = r;
    if (radius > (int16_t)(shorter / 2))
    {
        radius = (int16_t)(shorter / 2);
    }
    if (radius == 0)
    {
        return ssd1309gfx_draw_rect(fb, x, y, w, h, color);
    }

    const int16_t right  = (int16_t)(x + w - 1);
    const int16_t bottom = (int16_t)(y + h - 1);
    const int16_t straight_w = (int16_t)(w - (2 * radius));
    const int16_t straight_h = (int16_t)(h - (2 * radius));

    ssd1309gfx_span(fb, (int16_t)(x + radius), y, straight_w, color);
    ssd1309gfx_span(fb, (int16_t)(x + radius), bottom, straight_w, color);

    int16_t row = 0;
    for (row = (int16_t)(y + radius); row < (int16_t)(y + radius + straight_h); row++)
    {
        ssd1309gfx_plot(fb, x, row, color);
        ssd1309gfx_plot(fb, right, row, color);
    }

    /* Corner arcs, drawn about the centres of the four corner circles. */
    ssd1309gfx_circle_outline(
        fb, (int16_t)(x + radius), (int16_t)(y + radius), radius, 0x02U, color);
    ssd1309gfx_circle_outline(
        fb, (int16_t)(right - radius), (int16_t)(y + radius), radius, 0x01U, color);
    ssd1309gfx_circle_outline(
        fb, (int16_t)(x + radius), (int16_t)(bottom - radius), radius, 0x04U, color);
    ssd1309gfx_circle_outline(
        fb, (int16_t)(right - radius), (int16_t)(bottom - radius), radius, 0x08U, color);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_fill_round_rect(uint8_t                 *fb,
                                              const int16_t            x,
                                              const int16_t            y,
                                              const int16_t            w,
                                              const int16_t            h,
                                              const int16_t            r,
                                              const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (r < 0)
    {
        return SSD1309GFX_ERR_BAD_RADIUS;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if ((w <= 0) || (h <= 0))
    {
        return SSD1309GFX_OK;
    }

    const int16_t shorter = (w < h) ? w : h;
    int16_t       radius  = r;
    if (radius > (int16_t)(shorter / 2))
    {
        radius = (int16_t)(shorter / 2);
    }
    if (radius == 0)
    {
        return ssd1309gfx_fill_rect(fb, x, y, w, h, color);
    }

    const int16_t straight_h = (int16_t)(h - (2 * radius));

    /*
     * Full-height centre slab, then the left and right caps stretched down by
     * the straight section so the four corners meet it exactly.
     */
    (void)ssd1309gfx_fill_rect(
        fb, (int16_t)(x + radius), y, (int16_t)(w - (2 * radius)), h, color);
    ssd1309gfx_circle_halves(fb,
                             (int16_t)(x + w - radius - 1),
                             (int16_t)(y + radius),
                             radius,
                             0x01U,
                             (int16_t)(straight_h - 1),
                             color);
    ssd1309gfx_circle_halves(fb,
                             (int16_t)(x + radius),
                             (int16_t)(y + radius),
                             radius,
                             0x02U,
                             (int16_t)(straight_h - 1),
                             color);
    return SSD1309GFX_OK;
}

/* -------------------------------------------------------------------------
 * Circles and triangles
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t ssd1309gfx_draw_circle(uint8_t                 *fb,
                                          const int16_t            cx,
                                          const int16_t            cy,
                                          const int16_t            r,
                                          const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (r < 0)
    {
        return SSD1309GFX_ERR_BAD_RADIUS;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if (r == 0)
    {
        ssd1309gfx_plot(fb, cx, cy, color);
        return SSD1309GFX_OK;
    }

    ssd1309gfx_circle_outline(fb, cx, cy, r, 0x0FU, color);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_fill_circle(uint8_t                 *fb,
                                          const int16_t            cx,
                                          const int16_t            cy,
                                          const int16_t            r,
                                          const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (r < 0)
    {
        return SSD1309GFX_ERR_BAD_RADIUS;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if (r == 0)
    {
        ssd1309gfx_plot(fb, cx, cy, color);
        return SSD1309GFX_OK;
    }

    /* Centre column, then both halves of the disc. */
    ssd1309gfx_vrun(fb, cx, (int16_t)(cy - r), (int16_t)((2 * r) + 1), color);
    ssd1309gfx_circle_halves(fb, cx, cy, r, 0x03U, 0, color);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_draw_triangle(uint8_t                 *fb,
                                            const int16_t            x0,
                                            const int16_t            y0,
                                            const int16_t            x1,
                                            const int16_t            y1,
                                            const int16_t            x2,
                                            const int16_t            y2,
                                            const ssd1309gfx_color_t color)
{
    ssd1309gfx_error_t err = ssd1309gfx_draw_line(fb, x0, y0, x1, y1, color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_draw_line(fb, x1, y1, x2, y2, color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    return ssd1309gfx_draw_line(fb, x2, y2, x0, y0, color);
}

ssd1309gfx_error_t ssd1309gfx_fill_triangle(uint8_t                 *fb,
                                            const int16_t            x0,
                                            const int16_t            y0,
                                            const int16_t            x1,
                                            const int16_t            y1,
                                            const int16_t            x2,
                                            const int16_t            y2,
                                            const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    /* Sort the vertices so ay <= by <= cy. */
    int16_t ax = x0;
    int16_t ay = y0;
    int16_t bx = x1;
    int16_t by = y1;
    int16_t cx = x2;
    int16_t cy = y2;
    int16_t swap = 0;

    if (ay > by)
    {
        swap = ay; ay = by; by = swap;
        swap = ax; ax = bx; bx = swap;
    }
    if (by > cy)
    {
        swap = by; by = cy; cy = swap;
        swap = bx; bx = cx; cx = swap;
    }
    if (ay > by)
    {
        swap = ay; ay = by; by = swap;
        swap = ax; ax = bx; bx = swap;
    }

    if (ay == cy)
    {
        /* Degenerate: the triangle is a single horizontal run. */
        int16_t left  = ax;
        int16_t right = ax;
        if (bx < left)  { left = bx; }
        if (bx > right) { right = bx; }
        if (cx < left)  { left = cx; }
        if (cx > right) { right = cx; }
        ssd1309gfx_span(fb, left, ay, (int16_t)(right - left + 1), color);
        return SSD1309GFX_OK;
    }

    const int32_t dx_ac = (int32_t)cx - (int32_t)ax;
    const int32_t dy_ac = (int32_t)cy - (int32_t)ay;
    const int32_t dx_ab = (int32_t)bx - (int32_t)ax;
    const int32_t dy_ab = (int32_t)by - (int32_t)ay;
    const int32_t dx_bc = (int32_t)cx - (int32_t)bx;
    const int32_t dy_bc = (int32_t)cy - (int32_t)by;

    int16_t scan = 0;

    for (scan = ay; scan <= cy; scan++)
    {
        /* Long edge A->C spans the whole height; the short edges swap at B. */
        const int32_t long_x =
            (int32_t)ax + ((dx_ac * ((int32_t)scan - (int32_t)ay)) / dy_ac);

        int32_t short_x = 0;
        if (scan < by)
        {
            short_x = (dy_ab == 0)
                          ? (int32_t)ax
                          : ((int32_t)ax + ((dx_ab * ((int32_t)scan - (int32_t)ay)) / dy_ab));
        }
        else
        {
            short_x = (dy_bc == 0)
                          ? (int32_t)bx
                          : ((int32_t)bx + ((dx_bc * ((int32_t)scan - (int32_t)by)) / dy_bc));
        }

        const int32_t left  = (long_x < short_x) ? long_x : short_x;
        const int32_t right = (long_x < short_x) ? short_x : long_x;
        ssd1309gfx_span(fb, (int16_t)left, scan, (int16_t)(right - left + 1), color);
    }
    return SSD1309GFX_OK;
}

/* -------------------------------------------------------------------------
 * Bitmaps
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t ssd1309gfx_draw_bitmap(uint8_t                 *fb,
                                          const int16_t            x,
                                          const int16_t            y,
                                          const uint8_t           *bitmap,
                                          const int16_t            w,
                                          const int16_t            h,
                                          const ssd1309gfx_color_t color,
                                          const ssd1309gfx_color_t background)
{
    int16_t row = 0;

    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (bitmap == (const uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BITMAP;
    }

    ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_background(background);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if ((w <= 0) || (h <= 0))
    {
        return SSD1309GFX_OK;
    }

    /* Rows are padded out to a whole number of bytes. */
    const uint16_t row_bytes = (uint16_t)(((uint16_t)w + 7U) / 8U);

    for (row = 0; row < h; row++)
    {
        int16_t column = 0;
        for (column = 0; column < w; column++)
        {
            const uint16_t index = (uint16_t)(((uint16_t)row * row_bytes) +
                                              ((uint16_t)column >> 3));
            const uint8_t  mask  = (uint8_t)(0x80U >> ((uint16_t)column & 7U));
            const uint8_t  lit   = ((bitmap[index] & mask) != 0U) ? 1U : 0U;

            const ssd1309gfx_color_t pixel = (lit != 0U) ? color : background;
            if (pixel != SSD1309GFX_TRANSPARENT)
            {
                ssd1309gfx_plot(fb, (int16_t)(x + column), (int16_t)(y + row), pixel);
            }
        }
    }
    return SSD1309GFX_OK;
}

/* -------------------------------------------------------------------------
 * Text
 * ------------------------------------------------------------------------- */

/* Resolve a code point to its glyph, substituting the font's fallback. */
static const uint8_t *ssd1309gfx_glyph_of(const ssd1309gfx_font_t *font, const char ch)
{
    uint8_t code = (uint8_t)ch;

    if ((code < font->first_char) || (code > font->last_char))
    {
        code = font->fallback;
        if ((code < font->first_char) || (code > font->last_char))
        {
            return (const uint8_t *)0;
        }
    }
    return &font->glyphs[(uint16_t)(code - font->first_char) * (uint16_t)font->width];
}

/* Render one already-validated glyph. */
static void ssd1309gfx_blit_glyph(uint8_t                 *fb,
                                  const int16_t            x,
                                  const int16_t            y,
                                  const uint8_t           *glyph,
                                  const ssd1309gfx_font_t *font,
                                  const uint8_t            scale,
                                  const ssd1309gfx_color_t color,
                                  const ssd1309gfx_color_t background)
{
    const int16_t step = (int16_t)scale;
    uint8_t       col  = 0U;

    /* Paint the whole advance cell first so the inter-glyph gap is covered. */
    if (background != SSD1309GFX_TRANSPARENT)
    {
        const int16_t cell_w = (int16_t)((int16_t)font->advance * step);
        const int16_t cell_h = (int16_t)((int16_t)font->height * step);
        int16_t       row    = 0;
        for (row = 0; row < cell_h; row++)
        {
            ssd1309gfx_span(fb, x, (int16_t)(y + row), cell_w, background);
        }
    }

    for (col = 0U; col < font->width; col++)
    {
        const uint8_t bits = glyph[col];
        uint8_t       bit  = 0U;

        for (bit = 0U; bit < font->height; bit++)
        {
            if ((bits & (uint8_t)(1U << bit)) == 0U)
            {
                continue;
            }

            const int16_t px = (int16_t)(x + ((int16_t)col * step));
            const int16_t py = (int16_t)(y + ((int16_t)bit * step));

            if (scale == 1U)
            {
                ssd1309gfx_plot(fb, px, py, color);
            }
            else
            {
                int16_t row = 0;
                for (row = 0; row < step; row++)
                {
                    ssd1309gfx_span(fb, px, (int16_t)(py + row), step, color);
                }
            }
        }
    }
}

ssd1309gfx_error_t ssd1309gfx_draw_char(uint8_t                 *fb,
                                        const int16_t            x,
                                        const int16_t            y,
                                        const char               ch,
                                        const ssd1309gfx_font_t *font,
                                        const uint8_t            scale,
                                        const ssd1309gfx_color_t color,
                                        const ssd1309gfx_color_t background)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    ssd1309gfx_error_t err = ssd1309gfx_check_font(font);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_scale(scale);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_background(background);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    const uint8_t *const glyph = ssd1309gfx_glyph_of(font, ch);
    if (glyph == (const uint8_t *)0)
    {
        return SSD1309GFX_ERR_BAD_FONT;
    }

    ssd1309gfx_blit_glyph(fb, x, y, glyph, font, scale, color, background);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_draw_string(uint8_t                 *fb,
                                          const int16_t            x,
                                          const int16_t            y,
                                          const char              *text,
                                          const ssd1309gfx_font_t *font,
                                          const uint8_t            scale,
                                          const ssd1309gfx_color_t color,
                                          const ssd1309gfx_color_t background,
                                          int16_t                 *end_x)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (text == (const char *)0)
    {
        return SSD1309GFX_ERR_NULL_STRING;
    }

    ssd1309gfx_error_t err = ssd1309gfx_check_font(font);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_scale(scale);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_background(background);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    const int16_t step   = (int16_t)((int16_t)font->advance * (int16_t)scale);
    int16_t       cursor = x;
    uint16_t      i      = 0U;

    for (i = 0U; text[i] != '\0'; i++)
    {
        /* Stop once the cell would start beyond the right edge. */
        if (cursor >= SSD1309GFX_PANEL_W)
        {
            break;
        }
        if (cursor > (int16_t)-step)
        {
            const uint8_t *const glyph = ssd1309gfx_glyph_of(font, text[i]);
            if (glyph == (const uint8_t *)0)
            {
                return SSD1309GFX_ERR_BAD_FONT;
            }
            ssd1309gfx_blit_glyph(fb, cursor, y, glyph, font, scale, color, background);
        }
        cursor = (int16_t)(cursor + step);
    }

    if (end_x != (int16_t *)0)
    {
        *end_x = cursor;
    }
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_text_width(const char              *text,
                                         const ssd1309gfx_font_t *font,
                                         const uint8_t            scale,
                                         int16_t                 *width)
{
    if (text == (const char *)0)
    {
        return SSD1309GFX_ERR_NULL_STRING;
    }
    if (width == (int16_t *)0)
    {
        return SSD1309GFX_ERR_NULL_OUTPUT;
    }

    ssd1309gfx_error_t err = ssd1309gfx_check_font(font);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_scale(scale);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    uint16_t count = 0U;
    while (text[count] != '\0')
    {
        count++;
    }

    *width = (int16_t)((int16_t)count * (int16_t)font->advance * (int16_t)scale);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_text_height(const ssd1309gfx_font_t *font,
                                          const uint8_t            scale,
                                          int16_t                 *height)
{
    if (height == (int16_t *)0)
    {
        return SSD1309GFX_ERR_NULL_OUTPUT;
    }

    ssd1309gfx_error_t err = ssd1309gfx_check_font(font);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_scale(scale);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    /* One blank row below the glyph keeps stacked lines legible. */
    *height = (int16_t)(((int16_t)font->height + 1) * (int16_t)scale);
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_draw_string_wrapped(uint8_t                 *fb,
                                                  const int16_t            left_margin,
                                                  const int16_t            y,
                                                  const char              *text,
                                                  const ssd1309gfx_font_t *font,
                                                  const uint8_t            scale,
                                                  const ssd1309gfx_color_t color,
                                                  const ssd1309gfx_color_t background,
                                                  int16_t                 *end_y)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }
    if (text == (const char *)0)
    {
        return SSD1309GFX_ERR_NULL_STRING;
    }

    ssd1309gfx_error_t err = ssd1309gfx_check_font(font);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_scale(scale);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    err = ssd1309gfx_check_background(background);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    const int16_t step        = (int16_t)((int16_t)font->advance * (int16_t)scale);
    const int16_t line_height = (int16_t)(((int16_t)font->height + 1) * (int16_t)scale);
    int16_t       cursor_x    = left_margin;
    int16_t       cursor_y    = y;
    uint16_t      i           = 0U;

    while (text[i] != '\0')
    {
        if (text[i] == '\n')
        {
            cursor_x = left_margin;
            cursor_y = (int16_t)(cursor_y + line_height);
            i++;
            continue;
        }

        /* Measure the run up to the next break so whole words move down. */
        uint16_t word_len = 0U;
        while ((text[i + word_len] != '\0') && (text[i + word_len] != ' ') &&
               (text[i + word_len] != '\n'))
        {
            word_len++;
        }

        const int16_t word_w = (int16_t)((int16_t)word_len * step);
        if ((cursor_x > left_margin) && ((int16_t)(cursor_x + word_w) > SSD1309GFX_PANEL_W))
        {
            cursor_x = left_margin;
            cursor_y = (int16_t)(cursor_y + line_height);
        }

        uint16_t n = 0U;
        for (n = 0U; n < word_len; n++)
        {
            /* A word wider than the line is broken rather than lost. */
            if ((int16_t)(cursor_x + step) > SSD1309GFX_PANEL_W)
            {
                cursor_x = left_margin;
                cursor_y = (int16_t)(cursor_y + line_height);
            }

            const uint8_t *const glyph = ssd1309gfx_glyph_of(font, text[i + n]);
            if (glyph == (const uint8_t *)0)
            {
                return SSD1309GFX_ERR_BAD_FONT;
            }
            ssd1309gfx_blit_glyph(fb, cursor_x, cursor_y, glyph, font, scale, color, background);
            cursor_x = (int16_t)(cursor_x + step);
        }
        i += word_len;

        /* Collapse the separating space, dropping it at a line break. */
        if (text[i] == ' ')
        {
            if ((int16_t)(cursor_x + step) <= SSD1309GFX_PANEL_W)
            {
                const uint8_t *const glyph = ssd1309gfx_glyph_of(font, ' ');
                if (glyph != (const uint8_t *)0)
                {
                    ssd1309gfx_blit_glyph(
                        fb, cursor_x, cursor_y, glyph, font, scale, color, background);
                }
                cursor_x = (int16_t)(cursor_x + step);
            }
            i++;
        }
    }

    if (end_y != (int16_t *)0)
    {
        *end_y = (int16_t)(cursor_y + line_height);
    }
    return SSD1309GFX_OK;
}

ssd1309gfx_error_t ssd1309gfx_draw_string_aligned(uint8_t                 *fb,
                                                  const int16_t            x,
                                                  const int16_t            y,
                                                  const int16_t            width,
                                                  const char              *text,
                                                  const ssd1309gfx_font_t *font,
                                                  const uint8_t            scale,
                                                  const ssd1309gfx_align_t align,
                                                  const ssd1309gfx_color_t color,
                                                  const ssd1309gfx_color_t background)
{
    int16_t text_w = 0;

    if ((align != SSD1309GFX_ALIGN_LEFT) && (align != SSD1309GFX_ALIGN_CENTRE) &&
        (align != SSD1309GFX_ALIGN_RIGHT))
    {
        return SSD1309GFX_ERR_BAD_ALIGN;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_text_width(text, font, scale, &text_w);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }

    /*
     * text_width counts the trailing gap of the final glyph; discount it so
     * centred and right-aligned runs sit where they look right.
     */
    const int16_t ink_w =
        (text_w > 0) ? (int16_t)(text_w - (((int16_t)font->advance - (int16_t)font->width) *
                                           (int16_t)scale))
                     : 0;

    int16_t start = x;
    if (align == SSD1309GFX_ALIGN_CENTRE)
    {
        start = (int16_t)(x + ((width - ink_w) / 2));
    }
    else if (align == SSD1309GFX_ALIGN_RIGHT)
    {
        start = (int16_t)(x + width - ink_w);
    }
    else
    {
        /* Left alignment starts at x. */
    }

    return ssd1309gfx_draw_string(
        fb, start, y, text, font, scale, color, background, (int16_t *)0);
}

/* -------------------------------------------------------------------------
 * Widgets
 * ------------------------------------------------------------------------- */

ssd1309gfx_error_t ssd1309gfx_draw_progress_bar(uint8_t                 *fb,
                                                const int16_t            x,
                                                const int16_t            y,
                                                const int16_t            w,
                                                const int16_t            h,
                                                const uint8_t            percent,
                                                const ssd1309gfx_color_t color)
{
    if (fb == (uint8_t *)0)
    {
        return SSD1309GFX_ERR_NULL_BUFFER;
    }

    const ssd1309gfx_error_t err = ssd1309gfx_check_color(color);
    if (err != SSD1309GFX_OK)
    {
        return err;
    }
    if ((w <= 2) || (h <= 2))
    {
        return SSD1309GFX_OK;
    }

    const uint8_t level = (percent > 100U) ? 100U : percent;

    (void)ssd1309gfx_draw_rect(fb, x, y, w, h, color);

    /* Leave a one-pixel gap between the border and the fill. */
    const int16_t track_w = (int16_t)(w - 4);
    if (track_w <= 0)
    {
        return SSD1309GFX_OK;
    }

    const int16_t filled = (int16_t)(((int32_t)track_w * (int32_t)level) / 100);
    if (filled > 0)
    {
        (void)ssd1309gfx_fill_rect(
            fb, (int16_t)(x + 2), (int16_t)(y + 2), filled, (int16_t)(h - 4), color);
    }
    return SSD1309GFX_OK;
}
