/**
 * @file    events.c
 * @brief   Platform-agnostic Event Timer implementation
 * @version 1.0.0
 */

#include "events.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Maximum number of events */
#define MAX_EVENTS (16U)

/* Event structure */
typedef struct event_s
{
    bool              active;            /* Whether this event is active */
    event_type_t      type;              /* Type of event (flag or callback) */
    event_mode_t      mode;              /* Timing mode (relative or absolute) */
    event_recurring_t recurring;         /* Whether event recurs */
    uint32_t          period_ms;         /* Event period in milliseconds (for relative mode) */
    uint32_t          target_tick;       /* Target tick (for absolute mode) */
    uint32_t          next_trigger_time; /* Next trigger time (systick) */
    union
    {
        bool            *flag_ptr; /* Pointer to flag (for flag events) */
        event_callback_t callback; /* Callback function (for callback events) */
    } data;
} event_t;

/* Event pool */
static event_t event_pool[MAX_EVENTS];

/* Event handle type (just an index into the pool) */
struct event_handle_s
{
    uint8_t index;
};

/* Static event handles for the pool */
static struct event_handle_s event_handles[MAX_EVENTS];

/**
 * @brief Initialize the event timer system
 * @return EVENTS_OK if initialization successful, EVENTS_ERROR otherwise
 */
event_status_t events_init(void)
{
    /* Initialize event pool */
    for (uint8_t i = 0U; i < MAX_EVENTS; i++)
    {
        event_pool[i].active   = false;
        event_handles[i].index = i;
    }

    return EVENTS_OK;
}

/**
 * @brief Poll the event timer system (should be called in main loop)
 */
void events_poll(void)
{
    uint32_t current_time = events_get_tick_ms();

    /* Check all events */
    for (uint8_t i = 0U; i < MAX_EVENTS; i++)
    {
        if (event_pool[i].active)
        {
            /* Check if event should trigger */
            if (current_time >= event_pool[i].next_trigger_time)
            {
                /* Trigger the event */
                if (event_pool[i].type == EVNT_TYPE_FLAG)
                {
                    /* Set the flag */
                    if (event_pool[i].data.flag_ptr != NULL)
                    {
                        *(event_pool[i].data.flag_ptr) = true;
                    }
                }
                else if (event_pool[i].type == EVNT_TYPE_CALLBACK)
                {
                    /* Call the callback function */
                    if (event_pool[i].data.callback != NULL)
                    {
                        event_pool[i].data.callback();
                    }
                }

                /* Handle recurring events */
                if (event_pool[i].recurring == EVNT_RECURRING_YES)
                {
                    if (event_pool[i].mode == EVNT_MODE_RELATIVE)
                    {
                        /* Schedule next occurrence for relative mode */
                        event_pool[i].next_trigger_time = current_time + event_pool[i].period_ms;
                    }
                    else
                    {
                        /* Schedule next occurrence for absolute mode (add 1000ms period) */
                        event_pool[i].target_tick += 1000U;
                        event_pool[i].next_trigger_time = event_pool[i].target_tick;
                    }
                }
                else
                {
                    /* One-time event, deactivate it */
                    event_pool[i].active = false;
                }
            }
        }
    }
}

/**
 * @brief Create a new flag-based event
 * @param period_ms Event period in milliseconds
 * @param flag_ptr Pointer to flag that will be set when event occurs
 * @param recurring Whether the event should recur
 * @return Event handle if successful, NULL otherwise
 */
event_handle_t
events_new_flag_event(uint32_t period_ms, bool *flag_ptr, event_recurring_t recurring)
{
    /* Parameter validation */
    if ((period_ms == 0U) || (flag_ptr == NULL))
    {
        return NULL;
    }

    /* Find free event slot */
    uint8_t free_slot = MAX_EVENTS;
    for (uint8_t i = 0U; i < MAX_EVENTS; i++)
    {
        if (!event_pool[i].active)
        {
            free_slot = i;
            break;
        }
    }

    /* Check if we found a free slot */
    if (free_slot >= MAX_EVENTS)
    {
        return NULL; /* No free slots */
    }

    /* Initialize the event */
    event_pool[free_slot].active            = true;
    event_pool[free_slot].type              = EVNT_TYPE_FLAG;
    event_pool[free_slot].mode              = EVNT_MODE_RELATIVE;
    event_pool[free_slot].recurring         = recurring;
    event_pool[free_slot].period_ms         = period_ms;
    event_pool[free_slot].target_tick       = 0U; /* Not used in relative mode */
    event_pool[free_slot].next_trigger_time = events_get_tick_ms() + period_ms;
    event_pool[free_slot].data.flag_ptr     = flag_ptr;

    /* Clear the flag initially */
    *flag_ptr = false;

    return &event_handles[free_slot];
}

/**
 * @brief Create a new callback-based event
 * @param period_ms Event period in milliseconds
 * @param callback Function to call when event occurs
 * @param recurring Whether the event should recur
 * @return Event handle if successful, NULL otherwise
 */
event_handle_t events_new_callback_event(uint32_t          period_ms,
                                         event_callback_t  callback,
                                         event_recurring_t recurring)
{
    /* Parameter validation */
    if ((period_ms == 0U) || (callback == NULL))
    {
        return NULL;
    }

    /* Find free event slot */
    uint8_t free_slot = MAX_EVENTS;
    for (uint8_t i = 0U; i < MAX_EVENTS; i++)
    {
        if (!event_pool[i].active)
        {
            free_slot = i;
            break;
        }
    }

    /* Check if we found a free slot */
    if (free_slot >= MAX_EVENTS)
    {
        return NULL; /* No free slots */
    }

    /* Initialize the event */
    event_pool[free_slot].active            = true;
    event_pool[free_slot].type              = EVNT_TYPE_CALLBACK;
    event_pool[free_slot].mode              = EVNT_MODE_RELATIVE;
    event_pool[free_slot].recurring         = recurring;
    event_pool[free_slot].period_ms         = period_ms;
    event_pool[free_slot].target_tick       = 0U; /* Not used in relative mode */
    event_pool[free_slot].next_trigger_time = events_get_tick_ms() + period_ms;
    event_pool[free_slot].data.callback     = callback;

    return &event_handles[free_slot];
}

/**
 * @brief Create a new flag-based event at absolute time
 * @param target_tick System tick when event should occur
 * @param flag_ptr Pointer to flag that will be set when event occurs
 * @param recurring Whether the event should recur (if recurring, period is 1000ms)
 * @return Event handle if successful, NULL otherwise
 */
event_handle_t
events_new_flag_event_absolute(uint32_t target_tick, bool *flag_ptr, event_recurring_t recurring)
{
    /* Parameter validation */
    if (flag_ptr == NULL)
    {
        return NULL;
    }

    /* Find free event slot */
    uint8_t free_slot = MAX_EVENTS;
    for (uint8_t i = 0U; i < MAX_EVENTS; i++)
    {
        if (!event_pool[i].active)
        {
            free_slot = i;
            break;
        }
    }

    /* Check if we found a free slot */
    if (free_slot >= MAX_EVENTS)
    {
        return NULL; /* No free slots */
    }

    /* Initialize the event */
    event_pool[free_slot].active            = true;
    event_pool[free_slot].type              = EVNT_TYPE_FLAG;
    event_pool[free_slot].mode              = EVNT_MODE_ABSOLUTE;
    event_pool[free_slot].recurring         = recurring;
    event_pool[free_slot].period_ms         = 0U; /* Not used in absolute mode */
    event_pool[free_slot].target_tick       = target_tick;
    event_pool[free_slot].next_trigger_time = target_tick;
    event_pool[free_slot].data.flag_ptr     = flag_ptr;

    /* Clear the flag initially */
    *flag_ptr = false;

    return &event_handles[free_slot];
}

/**
 * @brief Create a new callback-based event at absolute time
 * @param target_tick System tick when event should occur
 * @param callback Function to call when event occurs
 * @param recurring Whether the event should recur (if recurring, period is 1000ms)
 * @return Event handle if successful, NULL otherwise
 */
event_handle_t events_new_callback_event_absolute(uint32_t          target_tick,
                                                  event_callback_t  callback,
                                                  event_recurring_t recurring)
{
    /* Parameter validation */
    if (callback == NULL)
    {
        return NULL;
    }

    /* Find free event slot */
    uint8_t free_slot = MAX_EVENTS;
    for (uint8_t i = 0U; i < MAX_EVENTS; i++)
    {
        if (!event_pool[i].active)
        {
            free_slot = i;
            break;
        }
    }

    /* Check if we found a free slot */
    if (free_slot >= MAX_EVENTS)
    {
        return NULL; /* No free slots */
    }

    /* Initialize the event */
    event_pool[free_slot].active            = true;
    event_pool[free_slot].type              = EVNT_TYPE_CALLBACK;
    event_pool[free_slot].mode              = EVNT_MODE_ABSOLUTE;
    event_pool[free_slot].recurring         = recurring;
    event_pool[free_slot].period_ms         = 0U; /* Not used in absolute mode */
    event_pool[free_slot].target_tick       = target_tick;
    event_pool[free_slot].next_trigger_time = target_tick;
    event_pool[free_slot].data.callback     = callback;

    return &event_handles[free_slot];
}

/**
 * @brief Delete an event
 * @param handle Event handle to delete
 * @return EVENTS_OK if successful, EVENTS_ERROR otherwise
 */
event_status_t events_delete_event(event_handle_t handle)
{
    /* Parameter validation */
    if (handle == NULL)
    {
        return EVENTS_INVALID_PARAM;
    }

    uint8_t index = handle->index;

    /* Check if index is valid */
    if (index >= MAX_EVENTS)
    {
        return EVENTS_INVALID_PARAM;
    }

    /* Deactivate the event */
    event_pool[index].active = false;

    return EVENTS_OK;
}

/**
 * @brief Reset an event timer (restart the countdown)
 * @param handle Event handle to reset
 * @return EVENTS_OK if successful, EVENTS_ERROR otherwise
 */
event_status_t events_reset_event(event_handle_t handle)
{
    /* Parameter validation */
    if (handle == NULL)
    {
        return EVENTS_INVALID_PARAM;
    }

    uint8_t index = handle->index;

    /* Check if index is valid and event is active */
    if ((index >= MAX_EVENTS) || (!event_pool[index].active))
    {
        return EVENTS_INVALID_PARAM;
    }

    /* Reset the timer */
    if (event_pool[index].mode == EVNT_MODE_RELATIVE)
    {
        event_pool[index].next_trigger_time = events_get_tick_ms() + event_pool[index].period_ms;
    }
    else
    {
        /* For absolute mode, reset to original target tick */
        event_pool[index].next_trigger_time = event_pool[index].target_tick;
    }

    return EVENTS_OK;
}

/**
 * @brief Get the remaining time for an event
 * @param handle Event handle
 * @return Remaining time in milliseconds, 0 if event not found
 */
uint32_t events_get_remaining_time(event_handle_t handle)
{
    /* Parameter validation */
    if (handle == NULL)
    {
        return 0U;
    }

    uint8_t index = handle->index;

    /* Check if index is valid and event is active */
    if ((index >= MAX_EVENTS) || (!event_pool[index].active))
    {
        return 0U;
    }

    uint32_t current_time = events_get_tick_ms();
    uint32_t remaining    = 0U;

    /* Calculate remaining time */
    if (event_pool[index].next_trigger_time > current_time)
    {
        remaining = event_pool[index].next_trigger_time - current_time;
    }

    return remaining;
}
