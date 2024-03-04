#include "log.h"

#include <stdlib.h>

#include "cJSON.h"

log_level_t log_level = LOG_INFO;

static const char *log_level_strings[] = {
	"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

static const char *log_level_colors[] = {
	"\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"};

void __attribute__((weak)) log_send_data(const char *msg, uint32_t msg_len)
{
	return;
}

uint32_t __attribute__((weak)) log_get_time()
{
	return 0;
}

void tx_log(char * log)
{
    cJSON *root = cJSON_CreateObject();
    char json[1024] = {0};
    cJSON_AddItemToObject(root, "log", cJSON_CreateString(log));
	cJSON_PrintPreallocated(root, json, 1024, false);
	cJSON_Minify(json);
	strcat(json, "***endjson***");
	log_send_data(json, strlen(json));
	cJSON_Delete(root);
}

void log_set_level(log_level_t level)
{
	if (level >= LOG_LEVEL_MAX)
	{
		level = LOG_LEVEL_MAX - 1;
	}
	log_level = level;
}

void log_log(const char *tag, log_level_t level, const char *file, int line, const char *fmt, ...)
{
	// Check if the level is valid
	if (level >= LOG_LEVEL_MAX)
	{
		return;
	}

	// Check if we have the level of logging for this level
	if(level < log_level)
	{
		return;
	}

	va_list args;
	va_start(args, fmt);

	// Determine the length of the formatted string
	uint32_t len = vsnprintf(NULL, 0, fmt, args);

	if (len > 1024)
	{
		va_end(args);
		return;
	}

	// Make the core user message
	char message_buf[256] = {0};
	vsnprintf(message_buf, len + 1, fmt, args);
	va_end(args);

	// Assemble the user message with the meta
	char str[1024] = {0};
	sprintf(str, "%s %10s [%010lu] %8s %s", log_level_colors[level], log_level_strings[level], log_get_time(), tag, message_buf);
	tx_log(str);
}

void log_create_bar(const char * name, const char * units, float low_limit, float high_limit)
{
	// Create the json
    cJSON *root = cJSON_CreateObject();
    cJSON *bar = cJSON_CreateObject();
    cJSON_AddItemToObject(bar, "name", cJSON_CreateString(name));
    cJSON_AddItemToObject(bar, "units", cJSON_CreateString(units));
    cJSON_AddItemToObject(bar, "low_limit", cJSON_CreateNumber((double)low_limit));
    cJSON_AddItemToObject(bar, "high_limit", cJSON_CreateNumber((double)high_limit));
    cJSON_AddItemToObject(root, "create_bar", bar);

    // Print and send
    char json[1024] = {0};
	cJSON_PrintPreallocated(root, json, 1024, false);
	cJSON_Minify(json);
	strcat(json, "***endjson***");
	log_send_data(json, strlen(json));

	// Release the memory
	cJSON_Delete(root);	
}

void log_set_bar(const char * name, float value)
{
	// Create the json
    cJSON *root = cJSON_CreateObject();
    cJSON *bar = cJSON_CreateObject();
    cJSON_AddItemToObject(bar, "name", cJSON_CreateString(name));
    cJSON_AddItemToObject(bar, "value", cJSON_CreateNumber(value));
    cJSON_AddItemToObject(root, "set_bar", bar);

    // Print and send
    char json[1024] = {0};
	cJSON_PrintPreallocated(root, json, 1024, false);
	cJSON_Minify(json);
	strcat(json, "***endjson***");
	log_send_data(json, strlen(json));

	// Release the memory
	cJSON_Delete(root);	
}

