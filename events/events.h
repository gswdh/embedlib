/**
 * @file    events.h
 * @brief   Platform-agnostic Event Timer API
 * @version 1.0.0
 */

#ifndef __EVENTS_H__
#define __EVENTS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

    /* Event status type definition */
    typedef enum
    {
        EVENTS_OK              = 0, /* Operation successful */
        EVENTS_ERROR           = 1, /* General error */
        EVENTS_INVALID_PARAM   = 2, /* Invalid parameter */
        EVENTS_NOT_INITIALIZED = 3, /* Events system not initialized */
        EVENTS_ALREADY_INIT    = 4, /* Events system already initialized */
        EVENTS_POOL_FULL       = 5  /* Event pool is full */
    } event_status_t;

    /* Event types */
    typedef enum
    {
        EVNT_TYPE_FLAG,    /* Set a flag when event occurs */
        EVNT_TYPE_CALLBACK /* Call a function when event occurs */
    } event_type_t;

    /* Event timing modes */
    typedef enum
    {
        EVNT_MODE_RELATIVE, /* Event occurs after period_ms from creation */
        EVNT_MODE_ABSOLUTE  /* Event occurs when system tick equals target_tick */
    } event_mode_t;

    /* Recurring options */
    typedef enum
    {
        EVNT_RECURRING_NO, /* Event occurs once */
        EVNT_RECURRING_YES /* Event repeats every period */
    } event_recurring_t;

    /* Event callback function type */
    typedef void (*event_callback_t)(void);

    /* Event handle (opaque type) */
    typedef struct event_handle_s *event_handle_t;

    /* Event Timer API */

    /**
     * @brief Get the current system tick count in milliseconds
     * @return System tick count in milliseconds
     * @note This function must be implemented by the platform-specific code
     */
    uint32_t events_get_tick_ms(void);

    /**
     * @brief Initialize the event timer system
     * @return EVENTS_OK if initialization successful, EVENTS_ERROR otherwise
     */
    event_status_t events_init(void);

    /**
     * @brief Poll the event timer system (should be called in main loop)
     */
    void events_poll(void);

    /**
     * @brief Create a new flag-based event
     * @param period_ms Event period in milliseconds
     * @param flag_ptr Pointer to flag that will be set when event occurs
     * @param recurring Whether the event should recur
     * @return Event handle if successful, NULL otherwise
     */
    event_handle_t
    events_new_flag_event(uint32_t period_ms, bool *flag_ptr, event_recurring_t recurring);

    /**
     * @brief Create a new callback-based event
     * @param period_ms Event period in milliseconds
     * @param callback Function to call when event occurs
     * @param recurring Whether the event should recur
     * @return Event handle if successful, NULL otherwise
     */
    event_handle_t events_new_callback_event(uint32_t          period_ms,
                                             event_callback_t  callback,
                                             event_recurring_t recurring);

    /**
     * @brief Create a new flag-based event at absolute time
     * @param target_tick System tick when event should occur
     * @param flag_ptr Pointer to flag that will be set when event occurs
     * @param recurring Whether the event should recur (if recurring, period is 1000ms)
     * @return Event handle if successful, NULL otherwise
     */
    event_handle_t events_new_flag_event_absolute(uint32_t          target_tick,
                                                  bool             *flag_ptr,
                                                  event_recurring_t recurring);

    /**
     * @brief Create a new callback-based event at absolute time
     * @param target_tick System tick when event should occur
     * @param callback Function to call when event occurs
     * @param recurring Whether the event should recur (if recurring, period is 1000ms)
     * @return Event handle if successful, NULL otherwise
     */
    event_handle_t events_new_callback_event_absolute(uint32_t          target_tick,
                                                      event_callback_t  callback,
                                                      event_recurring_t recurring);

    /**
     * @brief Delete an event
     * @param handle Event handle to delete
     * @return EVENTS_OK if successful, EVENTS_ERROR otherwise
     */
    event_status_t events_delete_event(event_handle_t handle);

    /**
     * @brief Reset an event timer (restart the countdown)
     * @param handle Event handle to reset
     * @return EVENTS_OK if successful, EVENTS_ERROR otherwise
     */
    event_status_t events_reset_event(event_handle_t handle);

    /**
     * @brief Get the remaining time for an event
     * @param handle Event handle
     * @return Remaining time in milliseconds, 0 if event not found
     */
    uint32_t events_get_remaining_time(event_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __EVENTS_H__ */
