/**
 * Copyright (c) 2020 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdarg.h>
#include <stdint.h>

// Logging levels
#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_WARN  3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_FATAL 5

// Macros to simplify logging at different levels
#define log_trace(tag, ...) log_message(LOG_LEVEL_TRACE, tag, __VA_ARGS__)
#define log_debug(tag, ...) log_message(LOG_LEVEL_DEBUG, tag, __VA_ARGS__)
#define log_info(tag, ...)  log_message(LOG_LEVEL_INFO, tag, __VA_ARGS__)
#define log_warn(tag, ...)  log_message(LOG_LEVEL_WARN, tag, __VA_ARGS__)
#define log_error(tag, ...) log_message(LOG_LEVEL_ERROR, tag, __VA_ARGS__)
#define log_fatal(tag, ...) log_message(LOG_LEVEL_FATAL, tag, __VA_ARGS__)

void     log_transmit(const char *log);
uint32_t log_get_time();
void     log_set_level(int level);
void     log_message(int level, const char *tag, const char *fmt, ...);

#endif
