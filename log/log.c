#include "log.h"

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
