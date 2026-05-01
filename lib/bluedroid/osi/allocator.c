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
#include <stdlib.h>
#include <string.h>
#include <tlsf.h>
#include "config.h"
#include "osi/allocator.h"

char *osi_strdup(const char *str)
{
    size_t size = strlen(str) + 1;  // + 1 for the null terminator
    char *new_string = (char *)osi_calloc(size);

    if (!new_string) {
        return NULL;
    }

    memcpy(new_string, str, size);
    return new_string;
}

#ifndef SIMULATOR

#if (CONFIG_PLATFORM & PLATFORM_HOSTED)
static unsigned char btbuf[BT_HEAP_SIZE];
#else
extern void *btbuf;
#endif

void *osi_malloc_func(size_t size) {
    void *ret = malloc_ex(size, btbuf);
    if (ret == NULL) {
        panicf("failed to allocate: osi_malloc(%d)", size);
    }
    return ret;
}
void *osi_calloc_func(size_t size) {
    void *ret = calloc_ex(1, size, btbuf);
    if (ret == NULL) {
        panicf("failed to allocate: osi_calloc(%d)", size);
    }
    return ret;
}
void osi_free_func(void *ptr) {
    return free_ex(ptr, btbuf);
}
#else
void *osi_malloc_func(size_t size) {
    return malloc(size);
}
void *osi_calloc_func(size_t size) {
    return calloc(1, size);
}
void osi_free_func(void *ptr) {
    return free(ptr);
}
#endif