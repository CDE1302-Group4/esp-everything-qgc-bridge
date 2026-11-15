/* Laptop stand-in for ESP-IDF FreeRTOS: just the types and macros the
 * navigation code uses.  1 tick = 1 ms of simulated time. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;

#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define portMAX_DELAY      0xFFFFFFFFu
#define configTICK_RATE_HZ 1000
#define portTICK_PERIOD_MS 1
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))
#define configASSERT(x)    assert(x)
