#include <semaphore.h>
#include <kernel.h>
#include "osi/semaphore.h"

int osi_sem_new(osi_sem_t *sem, uint32_t max_count, uint32_t init_count)
{
    semaphore_init(sem, max_count, init_count);
    return 0;
}

int osi_sem_take(osi_sem_t *sem, uint32_t timeout)
{
    int ret = 0;

    if (timeout ==  OSI_SEM_MAX_TIMEOUT) {
        if (semaphore_wait(sem, TIMEOUT_BLOCK) != OBJ_WAIT_SUCCEEDED) {
            ret = -1;
        }
    } else {
        if (semaphore_wait(sem, (timeout * HZ) / 1000) != OBJ_WAIT_SUCCEEDED) {
            ret = -2;
        }
    }

    return ret;
}

void osi_sem_give(osi_sem_t *sem)
{
    semaphore_release(sem);
}


void osi_sem_free(osi_sem_t *sem)
{
    (void)sem;
}