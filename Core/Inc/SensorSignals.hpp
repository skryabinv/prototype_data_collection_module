#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os2.h"
#ifdef __cplusplus
}
#endif

namespace SensorSignals {

constexpr uint32_t kFlagMmcMeasDone = 0x01U;
constexpr uint32_t kFlagBmpDrdy = 0x02U;
constexpr uint32_t kFlagUartRx = 0x04U;
constexpr uint32_t kFlagAnySensor = kFlagMmcMeasDone | kFlagBmpDrdy;
constexpr uint32_t kFlagTaskWake = kFlagAnySensor | kFlagUartRx;

void init();
void mmcMeasDoneFromIsr();
void bmpDrdyFromIsr();
void uartRxFromIsr();

/// @brief Блокирующее ожидание одного или нескольких флагов (osWaitForever или таймаут в тиках).
[[nodiscard]] uint32_t wait(uint32_t flags, uint32_t timeout);

} // namespace SensorSignals
