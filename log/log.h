/**
 * Copyright (c) 2020 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>

typedef enum
{
	LOG_TRACE,
	LOG_DEBUG,
	LOG_INFO,
	LOG_WARN,
	LOG_ERROR,
	LOG_FATAL,
	LOG_LEVEL_MAX
} log_level_t;

#define log_trace(TAG, ...) log_log(TAG, LOG_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#define log_debug(TAG, ...) log_log(TAG, LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define log_info(TAG, ...) log_log(TAG, LOG_INFO, __FILE__, __LINE__, __VA_ARGS__)
#define log_warn(TAG, ...) log_log(TAG, LOG_WARN, __FILE__, __LINE__, __VA_ARGS__)
#define log_error(TAG, ...) log_log(TAG, LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define log_fatal(TAG, ...) log_log(TAG, LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__)

void __attribute__((weak)) log_send_data(const char *msg, uint32_t msg_len);
uint32_t __attribute__((weak)) log_get_time();

void log_set_level(log_level_t level);
void log_log(const char *tag, log_level_t level, const char *file, int line, const char *fmt, ...);

void log_create_bar(const char * name, const char * units, float low_limit, float high_limit);
void log_set_bar(const char * name, float value);

#endif
