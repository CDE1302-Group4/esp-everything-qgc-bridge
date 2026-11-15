/* Laptop stand-in: the simulator runs the firmware in lockstep (only one
 * thread executes at a time), so mutexes never contend and are no-ops. */
#pragma once
#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (SemaphoreHandle_t)1; }
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t m, TickType_t t) { (void)m; (void)t; return pdTRUE; }
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t m) { (void)m; return pdTRUE; }
