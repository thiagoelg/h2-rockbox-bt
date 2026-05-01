/******************************************************************************
 *
 *  Copyright (C) 2015 Google, Inc.
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

#include "osi/mutex.h"


/* static section */
static osi_mutex_t gl_mutex; /* Recursive Type */


/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
int osi_mutex_new(osi_mutex_t *mutex)
{
    mutex_init(mutex);

    return 0;
}

/** Lock a mutex
 * @param mutex the mutex to lock */
int osi_mutex_lock(osi_mutex_t *mutex, uint32_t timeout)
{
    (void) timeout;
    
    mutex_lock(mutex);

    return 0;
}

/** Unlock a mutex
 * @param mutex the mutex to unlock */
void osi_mutex_unlock(osi_mutex_t *mutex)
{
    mutex_unlock(mutex);
}

/** Delete a mutex
 * @param mutex the mutex to delete
 * Note: Safe to call with NULL or uninitialized mutex (IDFGH-16853)
 */
void osi_mutex_free(osi_mutex_t *mutex)
{
    (void)mutex;
}

int osi_mutex_global_init(void)
{
    mutex_init(&gl_mutex);

    return 0;
}

void osi_mutex_global_deinit(void)
{
}

void osi_mutex_global_lock(void)
{
    mutex_lock(&gl_mutex);
}

void osi_mutex_global_unlock(void)
{
    mutex_unlock(&gl_mutex);
}
