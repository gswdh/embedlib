#ifndef SSD1309GFX_H
#define SSD1309GFX_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Monochrome graphics primitives for the SSD1309 framebuffer layout.
 *
 * Every entry point takes a pointer to a caller-owned framebuffer; the
 * library holds no state, allocates nothing and prints nothing.  The buffer
 * it draws into is exactly what ssd1309z_write_frame() expects, so a typical
 * application keeps one 1024-byte buffer, draws into it here, and hands the
 * same pointer to the driver.
 *
 * Buffer layout (identical to the controller's GDDRAM):
 *
 *   index = (y / 8) * SSD1309GFX_WIDTH + x
 *   bit   = y % 8            bit 0 is the top row of a page
 *
 * Coordinates are signed and every primitive clips to the panel, so drawing
 * partly or wholly off-screen is legal and reports SSD1309GFX_OK.
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Geometry.  Override at build time for a differently sized panel; the
 * defaults match the SSD1309 and ssd1309z_write_frame().
 * ------------------------------------------------------------------------- */
#ifndef SSD1309GFX_WIDTH
#define SSD1309GFX_WIDTH  (128)
#endif

#ifndef SSD1309GFX_HEIGHT
#define SSD1309GFX_HEIGHT (64)
#endif

#define SSD1309GFX_PAGES      (SSD1309GFX_HEIGHT / 8)
#define SSD1309GFX_BUFFER_SIZE (SSD1309GFX_WIDTH * SSD1309GFX_PAGES)

/** @brief Largest integer text magnification the renderer accepts. */
#define SSD1309GFX_MAX_SCALE (8U)

/* -------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

/**
 * @brief Pixel operation applied by a primitive.
 *
 * SSD1309GFX_TRANSPARENT is only meaningful as a text background; passing it
 * as a foreground colour is rejected with SSD1309GFX_ERR_BAD_COLOR.
 */
typedef enum
{
    SSD1309GFX_BLACK       = 0, /* clear the pixel            */
    SSD1309GFX_WHITE       = 1, /* set the pixel              */
    SSD1309GFX_INVERSE     = 2, /* toggle the pixel           */
    SSD1309GFX_TRANSPARENT = 3  /* leave the pixel untouched  */
} ssd1309gfx_color_t;

/** @brief Horizontal placement used by the aligned text helpers. */
typedef enum
{
    SSD1309GFX_ALIGN_LEFT   = 0,
    SSD1309GFX_ALIGN_CENTRE = 1,
    SSD1309GFX_ALIGN_RIGHT  = 2
} ssd1309gfx_align_t;

/**
 * @brief A column-major bitmap font.
 *
 * Glyphs are stored back to back, @c width bytes each, one byte per column.
 * Within a column byte bit 0 is the topmost row, matching the framebuffer,
 * so a page-aligned glyph costs one byte store per column.  Only codes
 * first_char..last_char are present; anything else renders as @c fallback.
 */
typedef struct
{
    const uint8_t *glyphs;     /* (last_char - first_char + 1) * width bytes */
    uint8_t        width;      /* glyph width in columns, 1..8               */
    uint8_t        height;     /* glyph height in rows, 1..8                 */
    uint8_t        advance;    /* columns consumed per glyph, incl. spacing  */
    uint8_t        first_char; /* lowest code point present                  */
    uint8_t        last_char;  /* highest code point present                 */
    uint8_t        fallback;   /* substitute drawn for codes outside range   */
} ssd1309gfx_font_t;

/**
 * @brief 5x7 ASCII font on a 6x8 cell.
 *
 * Covers 0x20 to 0x7E, renders 21 characters across and 8 rows down on a
 * 128 x 64 panel, and unknown code points fall back to '?'.
 */
extern const ssd1309gfx_font_t ssd1309gfx_font_5x7;

/* -------------------------------------------------------------------------
 * Error codes.  Out-of-range geometry is clipped rather than rejected; these
 * report only argument mistakes the caller should fix.
 * ------------------------------------------------------------------------- */
typedef enum
{
    SSD1309GFX_OK = 0,

    SSD1309GFX_ERR_NULL_BUFFER,  /* framebuffer pointer was NULL          */
    SSD1309GFX_ERR_NULL_STRING,  /* text pointer was NULL                 */
    SSD1309GFX_ERR_NULL_FONT,    /* font pointer or its glyph data NULL   */
    SSD1309GFX_ERR_NULL_BITMAP,  /* bitmap pointer was NULL               */
    SSD1309GFX_ERR_NULL_OUTPUT,  /* mandatory output pointer was NULL     */

    SSD1309GFX_ERR_BAD_COLOR,      /* not a colour, or TRANSPARENT as fg  */
    SSD1309GFX_ERR_BAD_BACKGROUND, /* not an ssd1309gfx_color_t value     */
    SSD1309GFX_ERR_BAD_ALIGN,      /* not an ssd1309gfx_align_t value     */
    SSD1309GFX_ERR_BAD_SCALE,      /* scale 0 or above SSD1309GFX_MAX_SCALE */
    SSD1309GFX_ERR_BAD_FONT,       /* font descriptor is self-inconsistent */
    SSD1309GFX_ERR_BAD_RADIUS      /* negative corner or circle radius     */
} ssd1309gfx_error_t;

/* -------------------------------------------------------------------------
 * Whole-buffer operations
 * ------------------------------------------------------------------------- */

/** @brief Clear every pixel. */
ssd1309gfx_error_t ssd1309gfx_clear(uint8_t *fb);

/**
 * @brief Apply @p color to every pixel.
 *
 * SSD1309GFX_INVERSE flips the whole frame.
 */
ssd1309gfx_error_t ssd1309gfx_fill(uint8_t *fb, const ssd1309gfx_color_t color);

/**
 * @brief Shift the whole frame vertically, feeding in blank rows.
 *
 * @param[in] rows Positive shifts content up (toward row 0), negative down.
 *                 A magnitude at or above the panel height clears the frame.
 */
ssd1309gfx_error_t ssd1309gfx_shift_vertical(uint8_t *fb, const int16_t rows);

/**
 * @brief Shift the whole frame horizontally, feeding in blank columns.
 *
 * @param[in] columns Positive shifts content left, negative right.
 */
ssd1309gfx_error_t ssd1309gfx_shift_horizontal(uint8_t *fb, const int16_t columns);

/* -------------------------------------------------------------------------
 * Pixels
 * ------------------------------------------------------------------------- */

/** @brief Apply @p color to one pixel; off-screen coordinates are ignored. */
ssd1309gfx_error_t
ssd1309gfx_draw_pixel(uint8_t *fb, const int16_t x, const int16_t y, const ssd1309gfx_color_t color);

/**
 * @brief Read one pixel.
 * @param[out] set Non-zero if the pixel is lit; 0 for a clear or off-screen
 *                 pixel.
 */
ssd1309gfx_error_t
ssd1309gfx_get_pixel(const uint8_t *fb, const int16_t x, const int16_t y, uint8_t *set);

/* -------------------------------------------------------------------------
 * Lines
 * ------------------------------------------------------------------------- */

/** @brief Draw a horizontal run of @p w pixels starting at (@p x, @p y). */
ssd1309gfx_error_t ssd1309gfx_draw_hline(uint8_t                 *fb,
                                         const int16_t            x,
                                         const int16_t            y,
                                         const int16_t            w,
                                         const ssd1309gfx_color_t color);

/** @brief Draw a vertical run of @p h pixels starting at (@p x, @p y). */
ssd1309gfx_error_t ssd1309gfx_draw_vline(uint8_t                 *fb,
                                         const int16_t            x,
                                         const int16_t            y,
                                         const int16_t            h,
                                         const ssd1309gfx_color_t color);

/** @brief Draw an arbitrary line between two endpoints, both inclusive. */
ssd1309gfx_error_t ssd1309gfx_draw_line(uint8_t                 *fb,
                                        const int16_t            x0,
                                        const int16_t            y0,
                                        const int16_t            x1,
                                        const int16_t            y1,
                                        const ssd1309gfx_color_t color);

/* -------------------------------------------------------------------------
 * Rectangles
 * ------------------------------------------------------------------------- */

/** @brief Draw a one-pixel rectangle outline. */
ssd1309gfx_error_t ssd1309gfx_draw_rect(uint8_t                 *fb,
                                        const int16_t            x,
                                        const int16_t            y,
                                        const int16_t            w,
                                        const int16_t            h,
                                        const ssd1309gfx_color_t color);

/** @brief Fill a rectangle. */
ssd1309gfx_error_t ssd1309gfx_fill_rect(uint8_t                 *fb,
                                        const int16_t            x,
                                        const int16_t            y,
                                        const int16_t            w,
                                        const int16_t            h,
                                        const ssd1309gfx_color_t color);

/**
 * @brief Draw a rectangle outline with rounded corners.
 *
 * @param[in] r Corner radius; clamped to half the shorter side.
 */
ssd1309gfx_error_t ssd1309gfx_draw_round_rect(uint8_t                 *fb,
                                              const int16_t            x,
                                              const int16_t            y,
                                              const int16_t            w,
                                              const int16_t            h,
                                              const int16_t            r,
                                              const ssd1309gfx_color_t color);

/** @brief Fill a rectangle with rounded corners. */
ssd1309gfx_error_t ssd1309gfx_fill_round_rect(uint8_t                 *fb,
                                              const int16_t            x,
                                              const int16_t            y,
                                              const int16_t            w,
                                              const int16_t            h,
                                              const int16_t            r,
                                              const ssd1309gfx_color_t color);

/* -------------------------------------------------------------------------
 * Circles and triangles
 * ------------------------------------------------------------------------- */

/** @brief Draw a circle outline centred on (@p cx, @p cy). */
ssd1309gfx_error_t ssd1309gfx_draw_circle(uint8_t                 *fb,
                                          const int16_t            cx,
                                          const int16_t            cy,
                                          const int16_t            r,
                                          const ssd1309gfx_color_t color);

/** @brief Fill a circle centred on (@p cx, @p cy). */
ssd1309gfx_error_t ssd1309gfx_fill_circle(uint8_t                 *fb,
                                          const int16_t            cx,
                                          const int16_t            cy,
                                          const int16_t            r,
                                          const ssd1309gfx_color_t color);

/** @brief Draw a triangle outline through three vertices. */
ssd1309gfx_error_t ssd1309gfx_draw_triangle(uint8_t                 *fb,
                                            const int16_t            x0,
                                            const int16_t            y0,
                                            const int16_t            x1,
                                            const int16_t            y1,
                                            const int16_t            x2,
                                            const int16_t            y2,
                                            const ssd1309gfx_color_t color);

/** @brief Fill a triangle. */
ssd1309gfx_error_t ssd1309gfx_fill_triangle(uint8_t                 *fb,
                                            const int16_t            x0,
                                            const int16_t            y0,
                                            const int16_t            x1,
                                            const int16_t            y1,
                                            const int16_t            x2,
                                            const int16_t            y2,
                                            const ssd1309gfx_color_t color);

/* -------------------------------------------------------------------------
 * Bitmaps
 * ------------------------------------------------------------------------- */

/**
 * @brief Blit a 1-bit-per-pixel bitmap.
 *
 * The bitmap is row-major and MSB-first, with each row padded to a whole
 * byte — the layout produced by the common "horizontal" bitmap converters.
 * Set bits are drawn in @p color; clear bits are drawn in @p background, so
 * pass SSD1309GFX_TRANSPARENT to overlay the shape without a backing box.
 *
 * @param[in] w Bitmap width in pixels.
 * @param[in] h Bitmap height in pixels.
 */
ssd1309gfx_error_t ssd1309gfx_draw_bitmap(uint8_t                 *fb,
                                          const int16_t            x,
                                          const int16_t            y,
                                          const uint8_t           *bitmap,
                                          const int16_t            w,
                                          const int16_t            h,
                                          const ssd1309gfx_color_t color,
                                          const ssd1309gfx_color_t background);

/* -------------------------------------------------------------------------
 * Text
 * ------------------------------------------------------------------------- */

/**
 * @brief Draw one character with its top-left corner at (@p x, @p y).
 *
 * @param[in] font       Font to render with; ssd1309gfx_font_5x7 by default.
 * @param[in] scale      Integer magnification, 1..SSD1309GFX_MAX_SCALE.
 * @param[in] color      Foreground colour; TRANSPARENT is rejected.
 * @param[in] background Colour for unset glyph pixels, commonly
 *                       SSD1309GFX_TRANSPARENT or the inverse of @p color.
 */
ssd1309gfx_error_t ssd1309gfx_draw_char(uint8_t                 *fb,
                                        const int16_t            x,
                                        const int16_t            y,
                                        const char               ch,
                                        const ssd1309gfx_font_t *font,
                                        const uint8_t            scale,
                                        const ssd1309gfx_color_t color,
                                        const ssd1309gfx_color_t background);

/**
 * @brief Draw a NUL-terminated string on one line.
 *
 * No wrapping is performed; characters past the right edge are clipped.
 *
 * @param[out] end_x Optional; receives the x coordinate just past the last
 *                   glyph, so runs can be chained.  May be NULL.
 */
ssd1309gfx_error_t ssd1309gfx_draw_string(uint8_t                 *fb,
                                          const int16_t            x,
                                          const int16_t            y,
                                          const char              *text,
                                          const ssd1309gfx_font_t *font,
                                          const uint8_t            scale,
                                          const ssd1309gfx_color_t color,
                                          const ssd1309gfx_color_t background,
                                          int16_t                 *end_x);

/**
 * @brief Draw a string, wrapping at the right edge of the panel.
 *
 * Breaks on spaces where possible and falls back to a hard break for words
 * longer than a line.  A '\n' in the text forces a new line.
 *
 * @param[in]  left_margin Column that each wrapped line starts at.
 * @param[out] end_y       Optional; receives the y coordinate of the line
 *                         after the last one drawn.  May be NULL.
 */
ssd1309gfx_error_t ssd1309gfx_draw_string_wrapped(uint8_t                 *fb,
                                                  const int16_t            left_margin,
                                                  const int16_t            y,
                                                  const char              *text,
                                                  const ssd1309gfx_font_t *font,
                                                  const uint8_t            scale,
                                                  const ssd1309gfx_color_t color,
                                                  const ssd1309gfx_color_t background,
                                                  int16_t                 *end_y);

/**
 * @brief Draw a string positioned within a horizontal span.
 *
 * @param[in] x     Left edge of the span.
 * @param[in] width Span width; the text is placed inside it per @p align.
 */
ssd1309gfx_error_t ssd1309gfx_draw_string_aligned(uint8_t                 *fb,
                                                  const int16_t            x,
                                                  const int16_t            y,
                                                  const int16_t            width,
                                                  const char              *text,
                                                  const ssd1309gfx_font_t *font,
                                                  const uint8_t            scale,
                                                  const ssd1309gfx_align_t align,
                                                  const ssd1309gfx_color_t color,
                                                  const ssd1309gfx_color_t background);

/**
 * @brief Measure the pixel width a string would occupy.
 *
 * Counts the trailing inter-character gap of the last glyph, matching where
 * ssd1309gfx_draw_string() reports @c end_x.
 */
ssd1309gfx_error_t ssd1309gfx_text_width(const char              *text,
                                         const ssd1309gfx_font_t *font,
                                         const uint8_t            scale,
                                         int16_t                 *width);

/** @brief Report the line height of @p font at @p scale, in pixels. */
ssd1309gfx_error_t ssd1309gfx_text_height(const ssd1309gfx_font_t *font,
                                          const uint8_t            scale,
                                          int16_t                 *height);

/* -------------------------------------------------------------------------
 * Widgets built from the primitives
 * ------------------------------------------------------------------------- */

/**
 * @brief Draw a horizontal progress bar with a one-pixel border.
 *
 * @param[in] percent Fill level, 0..100; higher values are clamped.
 */
ssd1309gfx_error_t ssd1309gfx_draw_progress_bar(uint8_t                 *fb,
                                                const int16_t            x,
                                                const int16_t            y,
                                                const int16_t            w,
                                                const int16_t            h,
                                                const uint8_t            percent,
                                                const ssd1309gfx_color_t color);

#endif /* SSD1309GFX_H */
