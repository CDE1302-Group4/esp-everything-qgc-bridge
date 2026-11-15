/* Laptop stand-in: firmware log lines are collected for the simulator. */
#pragma once
void fw_log(char level, const char *tag, const char *fmt, ...);
#define ESP_LOGE(tag, fmt, ...) fw_log('E', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fw_log('W', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fw_log('I', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)
