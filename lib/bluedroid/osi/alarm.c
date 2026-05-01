/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/
#include <stdbool.h>
#include "osi/alarm.h"
#include "osi/mutex.h"
#include "queue.h"
#include "system.h"
#include "thread.h"
#include "tick.h"
#include "bt_common.h"

enum {
    ALARM_STATE_IDLE,
    ALARM_STATE_OPEN,
};

#define OSI_ALARM_STACK_SIZE 4096

static osi_mutex_t alarm_mutex;
static struct event_queue alarm_queue;
static int alarm_state;
static unsigned int alarm_thread;
static uint32_t alarm_stack[OSI_ALARM_STACK_SIZE / sizeof(uint32_t)];

/* list of active timeout events */
static struct alarm_t alarm_list[ALARM_CBS_NUM];

static osi_alarm_err_t alarm_free(osi_alarm_t *alarm);
static osi_alarm_err_t alarm_set(osi_alarm_t *alarm, period_ms_t timeout, bool is_periodic);

static void alarm_tick(void)
{
    for(int i = 0; i < ALARM_CBS_NUM; i++)
    {
        if(!(alarm_list[i].valid && alarm_list[i].active))
            continue;

        if(TIME_BEFORE(current_tick, alarm_list[i].expires))
            continue;

        queue_post(&alarm_queue, 1, (intptr_t) &alarm_list[i]);

        if (alarm_list[i].period > 0) {
            alarm_list[i].expires += alarm_list[i].period;
        } else {
            alarm_list[i].active = false;
        }
    }
}

static void alarm_task(void)
{
    while (1) {
        struct queue_event ev;
        queue_wait(&alarm_queue, &ev);
        if (ev.id == 0) {
            queue_delete(&alarm_queue);
            thread_exit();
        } else if (ev.id == 1) {
            struct alarm_t *ptr = (struct alarm_t *)ev.data;
            ptr->callback(ptr->data);
        }
   }
}

int osi_alarm_create_mux(void)
{
    if (alarm_state != ALARM_STATE_IDLE) {
        OSI_TRACE_WARNING("%s, invalid state %d\n", __func__, alarm_state);
        return -1;
    }
    osi_mutex_new(&alarm_mutex);
    return 0;
}

int osi_alarm_delete_mux(void)
{
    if (alarm_state != ALARM_STATE_IDLE) {
        OSI_TRACE_WARNING("%s, invalid state %d\n", __func__, alarm_state);
        return -1;
    }
    osi_mutex_free(&alarm_mutex);
    return 0;
}

void osi_alarm_init(void)
{
    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    if (alarm_state != ALARM_STATE_IDLE) {
        OSI_TRACE_WARNING("%s, invalid state %d\n", __func__, alarm_state);
        goto end;
    }

    alarm_state = ALARM_STATE_OPEN;
    queue_init(&alarm_queue, false);
    alarm_thread = create_thread(
        alarm_task,
        alarm_stack,
        OSI_ALARM_STACK_SIZE,
        0, "bt_alarm"
        IF_PRIO(, PRIORITY_BLUETOOTH)
    );

    tick_add_task(alarm_tick);

end:
    osi_mutex_unlock(&alarm_mutex);
}

void osi_alarm_deinit(void)
{
    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    if (alarm_state != ALARM_STATE_OPEN) {
        OSI_TRACE_WARNING("%s, invalid state %d\n", __func__, alarm_state);
        goto end;
    }

    for (int i = 0; i < ALARM_CBS_NUM; i++) {
        if (alarm_list[i].valid) {
            alarm_free(&alarm_list[i]);
        }
    }

    queue_post(&alarm_queue, 0, (intptr_t) NULL);
    thread_wait(alarm_thread);

    tick_remove_task(alarm_tick);

    alarm_state = ALARM_STATE_IDLE;

    queue_delete(&alarm_queue);

end:
    osi_mutex_unlock(&alarm_mutex);
}

static struct alarm_t *alarm_cbs_lookfor_available(void)
{
    int i;

    for (i = 0; i < ALARM_CBS_NUM; i++) {
        if (!alarm_list[i].valid) { //available
            OSI_TRACE_DEBUG("%s %d %p\n", __func__, i, &alarm_list[i]);
            return &alarm_list[i];
        }
    }

    return NULL;
}

osi_alarm_t *osi_alarm_new(const char *alarm_name, osi_alarm_callback_t callback, void *data, period_ms_t timer_expire)
{
    struct alarm_t *timer_id = NULL;

    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    if (alarm_state != ALARM_STATE_OPEN) {
        OSI_TRACE_ERROR("%s, invalid state %d\n", __func__, alarm_state);
        timer_id = NULL;
        goto end;
    }

    timer_id = alarm_cbs_lookfor_available();

    if (!timer_id) {
        OSI_TRACE_ERROR("%s alarm_cbs exhausted\n", __func__);
        timer_id = NULL;
        goto end;
    }

    timer_id->active = false;
    timer_id->callback = callback;
    timer_id->data = data;
    
    int oldlevel = disable_irq_save();
    timer_id->valid = true;
    restore_irq(oldlevel);

end:
    osi_mutex_unlock(&alarm_mutex);
    return timer_id;
}

static osi_alarm_err_t alarm_free(osi_alarm_t *alarm)
{
    if (!alarm || !alarm->valid) {
        OSI_TRACE_ERROR("%s null\n", __func__);
        return OSI_ALARM_ERR_INVALID_ARG;
    }
    int oldlevel = disable_irq_save();
    alarm->valid = false;
    restore_irq(oldlevel);
    return OSI_ALARM_ERR_PASS;
}

void osi_alarm_free(osi_alarm_t *alarm)
{
    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    if (alarm_state != ALARM_STATE_OPEN) {
        OSI_TRACE_ERROR("%s, invalid state %d\n", __func__, alarm_state);
        goto end;
    }
    alarm_free(alarm);

end:
    osi_mutex_unlock(&alarm_mutex);
    return;
}

static osi_alarm_err_t alarm_set(osi_alarm_t *alarm, period_ms_t timeout, bool is_periodic)
{
    osi_alarm_err_t ret = OSI_ALARM_ERR_PASS;
    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    if (alarm_state != ALARM_STATE_OPEN) {
        OSI_TRACE_ERROR("%s, invalid state %d\n", __func__, alarm_state);
        ret = OSI_ALARM_ERR_INVALID_STATE;
        goto end;
    }

    if (!alarm || !alarm->valid) {
        OSI_TRACE_ERROR("%s null\n", __func__);
        ret = OSI_ALARM_ERR_INVALID_ARG;
        goto end;
    }

    unsigned long timeout_ticks = (timeout * HZ) / 1000;
    if (is_periodic) {
        alarm->expires = current_tick + timeout_ticks;
        alarm->period = timeout_ticks;
    } else {
        alarm->expires = current_tick + timeout_ticks;
        alarm->period = 0;
    }

    int oldlevel = disable_irq_save();
    alarm->active = true;
    restore_irq(oldlevel);

end:
    osi_mutex_unlock(&alarm_mutex);
    return ret;
}

osi_alarm_err_t osi_alarm_set(osi_alarm_t *alarm, period_ms_t timeout)
{
    return alarm_set(alarm, timeout, FALSE);
}

osi_alarm_err_t osi_alarm_set_periodic(osi_alarm_t *alarm, period_ms_t period)
{
    return alarm_set(alarm, period, TRUE);
}

osi_alarm_err_t osi_alarm_cancel(osi_alarm_t *alarm)
{
    int ret = OSI_ALARM_ERR_PASS;
    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    if (alarm_state != ALARM_STATE_OPEN) {
        OSI_TRACE_ERROR("%s, invalid state %d\n", __func__, alarm_state);
        ret = OSI_ALARM_ERR_INVALID_STATE;
        goto end;
    }

    if (!alarm || !alarm->valid) {
        OSI_TRACE_ERROR("%s null\n", __func__);
        ret = OSI_ALARM_ERR_INVALID_ARG;
        goto end;
    }

    int oldlevel = disable_irq_save();
    alarm->active = false;
    restore_irq(oldlevel);
end:
    osi_mutex_unlock(&alarm_mutex);
    return ret;
}

period_ms_t osi_alarm_get_remaining_ms(const osi_alarm_t *alarm)
{
    osi_mutex_lock(&alarm_mutex, OSI_MUTEX_MAX_TIMEOUT);
    int oldlevel = disable_irq_save();
    int64_t dt_ticks = alarm->expires - current_tick;
    restore_irq(oldlevel);
    osi_mutex_unlock(&alarm_mutex);

    return (dt_ticks > 0) ? (period_ms_t)((dt_ticks * 1000) / HZ) : 0;
}

uint32_t osi_time_get_os_boottime_ms(void)
{
    return (uint32_t)((current_tick * 1000) / HZ);
}

bool osi_alarm_is_active(osi_alarm_t *alarm)
{
    assert(alarm != NULL);

    return alarm->active;
}