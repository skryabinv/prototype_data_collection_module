#include "Uart4Link.hpp"

#include "SensorSignals.hpp"

#include <cstring>

extern "C" {
#include "stm32h7xx_hal.h"
}

namespace {

Uart4Link sUart4Link;

} // namespace

Uart4Link& Uart4Link::instance() {
  return sUart4Link;
}

bool Uart4Link::init() {
  if (mTxMutex == nullptr) {
    mTxMutex = osMutexNew(nullptr);
  }
  if (mTxDoneSem == nullptr) {
    mTxDoneSem = osSemaphoreNew(1U, 0U, nullptr);
  }
  if (mTxMutex == nullptr || mTxDoneSem == nullptr) {
    return false;
  }

  mRxRing.head = 0U;
  mRxRing.tail = 0U;
  mQueueHead = 0U;
  mQueueTail = 0U;
  for (PendingFrame& slot : mFrameQueue) {
    slot.valid = false;
  }
  mParser.reset();
  mTxDmaBusy = false;

  restartRxIt();
  return true;
}

void Uart4Link::restartRxIt() {
  (void)HAL_UART_Receive_IT(mUart, &mRxByte, 1U);
}

void Uart4Link::onRxCompleteFromIsr() {
  (void)ringPush(mRxByte);
  SensorSignals::uartRxFromIsr();
  restartRxIt();
}

void Uart4Link::onTxCompleteFromIsr() {
  mTxDmaBusy = false;
  (void)osSemaphoreRelease(mTxDoneSem);
}

bool Uart4Link::ringPush(const uint8_t byte) {
  const std::size_t nextHead = (mRxRing.head + 1U) % kRxRingCapacity;
  if (nextHead == mRxRing.tail) {
    return false;
  }
  mRxRing.data[mRxRing.head] = byte;
  mRxRing.head = nextHead;
  return true;
}

bool Uart4Link::ringPop(uint8_t& byte) {
  if (mRxRing.head == mRxRing.tail) {
    return false;
  }
  byte = mRxRing.data[mRxRing.tail];
  mRxRing.tail = (mRxRing.tail + 1U) % kRxRingCapacity;
  return true;
}

void Uart4Link::enqueueFrame(const UartProtocol::ReceivedFrame& frame) {
  const std::size_t nextHead = (mQueueHead + 1U) % kPendingFrameQueueDepth;
  if (nextHead == mQueueTail) {
    return;
  }
  mFrameQueue[mQueueHead].frame = frame;
  mFrameQueue[mQueueHead].valid = true;
  mQueueHead = nextHead;
}

void Uart4Link::pollRxParser() {
  uint8_t byte = 0U;
  UartProtocol::ReceivedFrame parsed{};
  while (ringPop(byte)) {
    const auto result = mParser.feed(byte, parsed);
    if (result == UartProtocol::ParseFeedResult::FrameReady) {
      enqueueFrame(parsed);
    }
  }
}

bool Uart4Link::popReceivedFrame(UartProtocol::ReceivedFrame& frame) {
  if (mQueueHead == mQueueTail) {
    return false;
  }
  if (!mFrameQueue[mQueueTail].valid) {
    mQueueTail = (mQueueTail + 1U) % kPendingFrameQueueDepth;
    return false;
  }
  frame = mFrameQueue[mQueueTail].frame;
  mFrameQueue[mQueueTail].valid = false;
  mQueueTail = (mQueueTail + 1U) % kPendingFrameQueueDepth;
  return true;
}

bool Uart4Link::transmitBlocking(const std::span<const uint8_t> frame) {
  if (frame.empty() || mTxMutex == nullptr) {
    return false;
  }

  if (osMutexAcquire(mTxMutex, osWaitForever) != osOK) {
    return false;
  }

  const HAL_StatusTypeDef status =
      HAL_UART_Transmit(mUart, const_cast<uint8_t*>(frame.data()),
                        static_cast<uint16_t>(frame.size()), kTxTimeoutMs);

  (void)osMutexRelease(mTxMutex);
  return status == HAL_OK;
}

bool Uart4Link::transmitDma(const std::span<const uint8_t> frame) {
  if (frame.empty() || frame.size() > mTxDmaBuffer.size() || mTxMutex == nullptr ||
      mTxDoneSem == nullptr) {
    return false;
  }

  if (osMutexAcquire(mTxMutex, osWaitForever) != osOK) {
    return false;
  }

  std::memcpy(mTxDmaBuffer.data(), frame.data(), frame.size());
  mTxDmaBusy = true;

  const HAL_StatusTypeDef startStatus = HAL_UART_Transmit_DMA(
      mUart, mTxDmaBuffer.data(), static_cast<uint16_t>(frame.size()));
  if (startStatus != HAL_OK) {
    mTxDmaBusy = false;
    (void)osMutexRelease(mTxMutex);
    return false;
  }

  const osStatus_t waitStatus = osSemaphoreAcquire(mTxDoneSem, kTxTimeoutMs);
  (void)osMutexRelease(mTxMutex);
  return waitStatus == osOK && !mTxDmaBusy;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart4) {
    Uart4Link::instance().onRxCompleteFromIsr();
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart4) {
    Uart4Link::instance().onTxCompleteFromIsr();
  }
}
