#include "log.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static int current_log_level = LOG_LEVEL_INFO;

static const char *log_level_strings[] = {"TRACE", "DEBUG", "INFO ", "WARN ", "ERROR", "FATAL"};

static const char *log_level_colors[] = {
    "\x1b[94m", // TRACE - Blue
    "\x1b[36m", // DEBUG - Cyan
    "\x1b[32m", // INFO  - Green
    "\x1b[33m", // WARN  - Yellow
    "\x1b[31m", // ERROR - Red
    "\x1b[35m"  // FATAL - Magenta
};

static const char *log_reset_color = "\x1b[0m";

void log_set_level(int level)
{
    if (level >= LOG_LEVEL_TRACE && level <= LOG_LEVEL_FATAL)
    {
        current_log_level = level;
    }
}

void log_message(int level, const char *tag, const char *fmt, ...)
{
    if (level < current_log_level)
    {
        return;
    }

    va_list args;
    va_start(args, fmt);

    const uint32_t uptime = log_get_time();

    char log_buffer[128];
    int  offset = 0;
    offset += snprintf(log_buffer + offset,
                       sizeof(log_buffer) - offset,
                       "%s%s [%010lu]   %s ",
                       log_level_colors[level],
                       log_level_strings[level],
                       uptime,
                       tag);

    offset += vsnprintf(log_buffer + offset, sizeof(log_buffer) - offset, fmt, args);

    // Append reset color and newline to the buffer
    snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "%s\n", log_reset_color);

    // Output the log message
    log_transmit((const char *)log_buffer);

    va_end(args);
}

void log_make_progress_bar(char          *bar_str,
                           const uint32_t bar_str_len,
                           const uint32_t num,
                           const uint32_t den,
                           const uint32_t width,
                           const char    *units_str)
{
    static const char empty_str[] = "";

    /* Parameter validation (MISRA: length checking, null checks) */
    if ((bar_str == NULL) || (bar_str_len == 0U))
    {
        return;
    }

    /* Bar needs at least 2 chars for '[' and ']' */
    if (width < 2U)
    {
        bar_str[0U] = '\0';
        return;
    }

    /* Buffer must hold bar (width chars) plus null terminator */
    if (bar_str_len <= width)
    {
        bar_str[0U] = '\0';
        return;
    }

    /* Avoid division by zero */
    if (den == 0U)
    {
        bar_str[0U] = '\0';
        return;
    }

    if (units_str == NULL)
    {
        units_str = empty_str;
    }

    /* Integer-only progress: filled segment count in [0, width-2] */
    uint32_t filled = (num * (width - 2U)) / den;
    if (filled > (width - 2U))
    {
        filled = width - 2U;
    }

    /* Clear buffer (bounded by bar_str_len) */
    (void)memset(bar_str, 0, (size_t)bar_str_len);

    /* Draw bar: '[' + filled '=' + (width-2-filled) ' ' + ']' */
    bar_str[0U] = '[';
    if (filled > 0U)
    {
        (void)memset(bar_str + 1U, '=', (size_t)filled);
    }
    {
        const uint32_t space_count = (width - 2U) - filled;
        if (space_count > 0U)
        {
            (void)memset(bar_str + 1U + filled, ' ', (size_t)space_count);
        }
    }
    bar_str[width - 1U] = ']';

    /* Append numeric part with length check (no sprintf/strcat) */
    {
        const uint32_t tail_start = width;
        const uint32_t tail_space = bar_str_len - width;

        if (tail_space > 0U)
        {
            (void)snprintf(bar_str + tail_start,
                           (size_t)tail_space,
                           " %" PRIu32 " / %" PRIu32 " %s",
                           num,
                           den,
                           units_str);
        }
    }
}