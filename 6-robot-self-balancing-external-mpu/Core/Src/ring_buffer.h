#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define RING_BUF_SIZE 128u   // must be power of 2

typedef struct {
    volatile uint8_t  buf[RING_BUF_SIZE];
    volatile uint16_t head;   // written by ISR
    volatile uint16_t tail;   // read  by main loop
} RingBuffer_t;

/* ── API ─────────────────────────────────────────────────────────────── */

static inline void rb_init(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

/* Called from ISR – push one byte. Returns false if buffer full. */
static inline bool rb_push(RingBuffer_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1u) & (RING_BUF_SIZE - 1u);
    if (next == rb->tail) return false;   // full → drop byte
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

/* Called from main loop – pop one byte. Returns false if empty. */
static inline bool rb_pop(RingBuffer_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head) return false;  // empty
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1u) & (RING_BUF_SIZE - 1u);
    return true;
}

static inline bool rb_empty(const RingBuffer_t *rb)
{
    return rb->tail == rb->head;
}

#endif /* RING_BUFFER_H */
