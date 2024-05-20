#include "dmgui.h"

#include <assert.h>

const uint8_t dmgui_font[354] = {0, 0, 0, 0, 0, 0, 92, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 68, 32, 16, 8, 68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 68, 130, 0, 0, 130, 68, 56, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 124, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 68, 68, 56, 0, 0, 8, 124, 0, 0, 0, 0, 100, 84, 72, 0, 0, 0, 84, 84, 40, 0, 0, 0, 28, 16, 120, 16, 0, 0, 92, 84, 36, 0, 0, 0, 120, 84, 84, 32, 0, 0, 4, 100, 20, 12, 0, 0, 40, 84, 84, 40, 0, 0, 8, 84, 84, 56, 0, 0, 56, 68, 68, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 120, 20, 20, 120, 0, 0, 124, 84, 84, 40, 0, 0, 56, 68, 68, 68, 0, 0, 124, 68, 68, 56, 0, 0, 124, 84, 84, 68, 0, 0, 124, 20, 20, 4, 0, 0, 56, 68, 84, 48, 0, 0, 124, 16, 16, 124, 0, 0, 0, 0, 124, 0, 0, 0, 68, 68, 60, 4, 0, 0, 124, 16, 40, 68, 0, 0, 124, 64, 64, 64, 0, 124, 8, 16, 8, 124, 0, 0, 124, 8, 16, 124, 0, 0, 56, 68, 68, 56, 0, 0, 124, 20, 20, 8, 0, 56, 68, 84, 36, 88, 0, 0, 124, 20, 20, 104, 0, 0, 72, 84, 84, 36, 0, 4, 4, 124, 4, 4, 0, 0, 60, 64, 64, 60, 0, 12, 48, 96, 48, 12, 0, 124, 32, 56, 32, 124, 0, 68, 40, 16, 40, 68, 0, 4, 8, 112, 8, 4, 0, 100, 84, 84, 76, 0, 0};

static uint8_t *screen = NULL;

static bool screen_changed = false;

void dmgui_init(uint8_t *frame_ptr)
{
    assert(frame_ptr != NULL);

    // Capture the memory pointer to the display
    screen = frame_ptr;

    // Reset the screen variable
    dmgui_fill_screen(GUI_CLR_BLACK);
}

void dmgui_set_pixel(uint8_t x, uint8_t y, uint8_t state)
{
    // Make it's in range
    assert(x < 128);
    assert(y < 64);
    assert(screen != NULL);

    uint8_t row = y / 8;
    uint8_t pixel = y % 8;

    // Set or clear the pixels
    if (state)
    {
        screen[(row * GUI_WIDTH) + x] |= (1 << pixel);
    }

    else
    {
        screen[(row * GUI_WIDTH) + x] &= ~(1 << pixel);
    }

    screen_changed = true;
}

void dmgui_draw_line(uint8_t x_1, uint8_t y_1, uint8_t x_2, uint8_t y_2, uint8_t state)
{
    // Draw
    if (x_1 == x_2)
    {
        // Set the pixels
        for (uint8_t y = y_1; y < y_2; y++)
        {
            dmgui_set_pixel(x_1, y, state);
        }
    }

    else if (y_1 == y_2)
    {
        // Set the pixels
        for (uint8_t x = x_1; x < x_2; x++)
        {
            dmgui_set_pixel(x, y_1, state);
        }
    }
}

void dmgui_draw_rect(uint8_t x_1, uint8_t y_1, uint8_t x_2, uint8_t y_2, uint8_t filled)
{
    // Just go through all the pixels
    if (filled == GUI_CLR_WHITE || filled == GUI_CLR_BLACK)
    {
        for (uint8_t x = x_1; x < x_2; x++)
        {
            for (uint8_t y = y_1; y < y_2; y++)
            {
                dmgui_set_pixel(x, y, filled);
            }
        }
    }

    // Otherwise use lines
    else
    {
        dmgui_draw_line(x_1, y_1, x_1, y_2, GUI_CLR_WHITE);
        dmgui_draw_line(x_2, y_1, x_2, y_2 + 1, GUI_CLR_WHITE);
        dmgui_draw_line(x_1, y_1, x_2, y_1, GUI_CLR_WHITE);
        dmgui_draw_line(x_1, y_2, x_2, y_2, GUI_CLR_WHITE);
    }
}

void dmgui_add_image(const uint8_t *image)
{
    assert(screen != NULL);
    assert(image != NULL);
    memcpy(screen, image, 1024);

    screen_changed = true;
}

void dmgui_add_text(char *text, uint8_t x, uint8_t y, uint8_t alignment)
{
    // Make sure the coordinates are in range
    assert(screen != NULL);
    assert(text != NULL);
    assert(x < (127 - 5));
    assert(y < 7);

    // Make sure the string length isn't too long
    assert(strlen(text) < (21 - (x * 6)));

    // Shift the cursor every char
    int16_t char_offset = 0;

    // If we need to be center aligned, add some offset
    if (alignment == GUI_TXT_ALIGN_C)
    {
        char_offset = -((strlen(text) * 6) / 2);
    }

    // Add the text to memory
    for (uint8_t n = 0; n < strlen(text); n++)
    {
        // Make sure the char isn't out of range
        if ((text[n] < 90) && (text[n] > 32))
        {
            uint32_t font_offset = ((uint8_t)(text[n]) - 32) * 6;

            // Copy the char over
            memcpy(screen + ((y * GUI_WIDTH) + x) + char_offset, dmgui_font + font_offset, 5);
            screen[(y * GUI_WIDTH) + x + 5 + char_offset] = 0;
        }

        // Increment the offset
        char_offset += 6;
    }

    screen_changed = true;
}

void dmgui_fill_screen(uint8_t colour)
{
    assert(screen != NULL);
    memset(screen, colour ? 255 : 0, 1024);

    screen_changed = true;
}

bool dmgui_update_needed()
{
    return screen_changed;
}

void dmgui_update_done()
{
    screen_changed = false;
}