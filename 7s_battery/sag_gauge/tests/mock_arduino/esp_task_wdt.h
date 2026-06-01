#pragma once
#include <cstdint>

inline void esp_task_wdt_init(uint32_t timeout, bool panic) {}
inline void esp_task_wdt_add(void* task) {}
inline void esp_task_wdt_reset() {}
