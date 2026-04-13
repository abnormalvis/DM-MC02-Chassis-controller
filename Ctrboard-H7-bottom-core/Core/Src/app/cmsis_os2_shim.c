/**
 * @file cmsis_os2_shim.c
 * @brief CMSIS-RTOS2 minimal weak shim for bare-metal/debug link.
 */

#include "cmsis_os2.h"

#include <string.h>

#define SHIM_QUEUE_MAX_COUNT 16U
#define SHIM_QUEUE_MAX_MSG   192U

typedef struct
{
    uint32_t count;
    uint32_t msg_size;
    uint32_t head;
    uint32_t tail;
    uint32_t used;
    uint8_t storage[SHIM_QUEUE_MAX_COUNT * SHIM_QUEUE_MAX_MSG];
} shim_queue_t;

static shim_queue_t s_shim_queue;

__attribute__((weak)) osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr)
{
    (void)attr;

    if ((msg_count == 0U) || (msg_size == 0U))
    {
        return NULL;
    }

    s_shim_queue.count = (msg_count > SHIM_QUEUE_MAX_COUNT) ? SHIM_QUEUE_MAX_COUNT : msg_count;
    s_shim_queue.msg_size = (msg_size > SHIM_QUEUE_MAX_MSG) ? SHIM_QUEUE_MAX_MSG : msg_size;
    s_shim_queue.head = 0U;
    s_shim_queue.tail = 0U;
    s_shim_queue.used = 0U;

    return (osMessageQueueId_t)&s_shim_queue;
}

__attribute__((weak)) osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout)
{
    shim_queue_t *q = (shim_queue_t *)mq_id;

    (void)msg_prio;
    (void)timeout;

    if ((q == NULL) || (msg_ptr == NULL) || (q->msg_size == 0U) || (q->count == 0U))
    {
        return osErrorParameter;
    }

    if (q->used >= q->count)
    {
        return osErrorResource;
    }

    memcpy(&q->storage[q->tail * q->msg_size], msg_ptr, q->msg_size);
    q->tail = (q->tail + 1U) % q->count;
    q->used++;

    return osOK;
}

__attribute__((weak)) osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout)
{
    shim_queue_t *q = (shim_queue_t *)mq_id;

    (void)msg_prio;
    (void)timeout;

    if ((q == NULL) || (msg_ptr == NULL) || (q->msg_size == 0U) || (q->count == 0U))
    {
        return osErrorParameter;
    }

    if (q->used == 0U)
    {
        return osErrorResource;
    }

    memcpy(msg_ptr, &q->storage[q->head * q->msg_size], q->msg_size);
    q->head = (q->head + 1U) % q->count;
    q->used--;

    return osOK;
}
