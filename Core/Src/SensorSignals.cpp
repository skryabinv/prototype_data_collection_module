#include "SensorSignals.hpp"

namespace {

osEventFlagsId_t sEventFlags = nullptr;

} // namespace

namespace SensorSignals {

void init()
{
    if (sEventFlags != nullptr) {
        return;
    }
    sEventFlags = osEventFlagsNew(nullptr);
}

void mmcMeasDoneFromIsr()
{
    if (sEventFlags != nullptr) {
        (void)osEventFlagsSet(sEventFlags, kFlagMmcMeasDone);
    }
}

void bmpDrdyFromIsr()
{
    if (sEventFlags != nullptr) {
        (void)osEventFlagsSet(sEventFlags, kFlagBmpDrdy);
    }
}

void uartRxFromIsr()
{
    if (sEventFlags != nullptr) {
        (void)osEventFlagsSet(sEventFlags, kFlagUartRx);
    }
}

uint32_t wait(const uint32_t flags, const uint32_t timeout)
{
    if (sEventFlags == nullptr) {
        return 0U;
    }
    return osEventFlagsWait(sEventFlags, flags, osFlagsWaitAny, timeout);
}

} // namespace SensorSignals
