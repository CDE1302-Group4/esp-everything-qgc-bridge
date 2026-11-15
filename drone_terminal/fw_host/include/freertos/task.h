/* Laptop stand-in: task timing runs on the simulator's clock (fw_host.c). */
#pragma once
#include "freertos/FreeRTOS.h"

typedef void *TaskHandle_t;

TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment);
void vTaskDelay(TickType_t ticks);
