/* Laptop stand-in: PSRAM allocations come from the normal heap. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM   (1 << 10)
#define MALLOC_CAP_INTERNAL (1 << 11)
#define MALLOC_CAP_8BIT     (1 << 2)
static inline void *heap_caps_calloc(size_t n, size_t size, uint32_t caps) { (void)caps; return calloc(n, size); }
static inline void *heap_caps_malloc(size_t size, uint32_t caps) { (void)caps; return malloc(size); }
static inline void heap_caps_free(void *p) { free(p); }
