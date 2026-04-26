#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#define RING_BUF_SIZE 128u

typedef struct {
    volatile uint8_t buf[RING_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

static inline void rb_init(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

static inline bool rb_push(RingBuffer_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1u) & (RING_BUF_SIZE - 1u);
    if (next == rb->tail) {
        return false;
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

static inline bool rb_pop(RingBuffer_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head) {
        return false;
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1u) & (RING_BUF_SIZE - 1u);
    return true;
}

#endif
