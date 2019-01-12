#ifndef BITARRAY_H
#define BITARRAY_H

#include <stdint.h>

static inline bool
bitarray_get(uint8_t *b, uint32_t i)
{
    return (b[i/8] >> (i % 8)) & 1;
}

static inline void
bitarray_set(uint8_t *b, uint32_t i)
{
    b[i/8] |= 1 << (i % 8);
}

static inline void
bitarray_clear(uint8_t *b, uint32_t i)
{
    b[i/8] &= ~(1 << (i % 8));
}

static inline void
bitarray_setv(uint8_t *b, uint32_t i, bool v)
{
    v ? bitarray_set(b, i) : bitarray_clear(b, i);
}

#endif

