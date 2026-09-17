#pragma once

#include "UartProtocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#ifdef __cplusplus
extern "C" {
#endif
#include "cmsis_os2.h"
#include "usart.h"
#ifdef __cplusplus
}
#endif

/**
 * @brief UART4 к Jetson: приём по прерыванию, передача blocking или DMA.
 */
class Uart4Link {
public:
  static constexpr std::size_t kRxRingCapacity = 256U;
  static constexpr std::size_t kPendingFrameQueueDepth = 4U;
  static constexpr uint32_t kTxTimeoutMs = 50U;

  Uart4Link() = default;

  Uart4Link(const Uart4Link&) = delete;
  Uart4Link& operator=(const Uart4Link&) = delete;

  [[nodiscard]] bool init();

  /// Разбор кольцевого буфера RX (вызывать из задачи).
  void pollRxParser();

  [[nodiscard]] bool popReceivedFrame(UartProtocol::ReceivedFrame& frame);

  [[nodiscard]] bool transmitBlocking(std::span<const uint8_t> frame);

  [[nodiscard]] bool transmitDma(std::span<const uint8_t> frame);

  void onRxCompleteFromIsr();
  void onTxCompleteFromIsr();

  static Uart4Link& instance();

private:
  struct RingBuffer {
    std::array<uint8_t, kRxRingCapacity> data{};
    volatile std::size_t head = 0U;
    volatile std::size_t tail = 0U;
  };

  struct PendingFrame {
    UartProtocol::ReceivedFrame frame{};
    bool valid = false;
  };

  UART_HandleTypeDef* mUart = &huart4;
  osMutexId_t mTxMutex = nullptr;
  osSemaphoreId_t mTxDoneSem = nullptr;

  RingBuffer mRxRing{};
  UartProtocol::FrameParser mParser{};
  std::array<PendingFrame, kPendingFrameQueueDepth> mFrameQueue{};
  std::size_t mQueueHead = 0U;
  std::size_t mQueueTail = 0U;

  uint8_t mRxByte = 0U;
  std::array<uint8_t, UartProtocol::kMaxFrameSize> mTxDmaBuffer{};
  volatile bool mTxDmaBusy = false;

  void enqueueFrame(const UartProtocol::ReceivedFrame& frame);
  [[nodiscard]] bool ringPush(uint8_t byte);
  [[nodiscard]] bool ringPop(uint8_t& byte);
  void restartRxIt();
};
