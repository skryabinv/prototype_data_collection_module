#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

/**
 * @brief Кадры обмена STM32 ↔ Jetson (см. docs/Protocol.md).
 *
 * Формирование кадров без привязки к HAL UART — передача в UART отдельным шагом.
 */
namespace UartProtocol {

constexpr uint8_t kSync1 = 0xAAU;
constexpr uint8_t kSync2 = 0x55U;

constexpr std::size_t kHeaderSize = 4U; // SYNC_1, SYNC_2, MSG_ID, LENGTH
constexpr std::size_t kCrcSize = 1U;
constexpr std::size_t kMaxPayloadSize = 255U;
constexpr std::size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;

enum class MsgId : uint8_t {
  SensorDataBmp390 = 0x01U,
  SensorDataMmc5983ma = 0x02U,
  ConfigCmd = 0x03U,
  Ack = 0x04U,
  Error = 0x05U,
};

namespace Bmp390Flags {
constexpr uint8_t kOk = 0x01U;
constexpr uint8_t kDataReady = 0x02U;
} // namespace Bmp390Flags

namespace Mmc5983maFlags {
constexpr uint8_t kOk = 0x01U;
constexpr uint8_t kDataReady = 0x02U;
} // namespace Mmc5983maFlags

namespace ConfigTarget {
constexpr uint8_t kAll = 0x00U;
constexpr uint8_t kBmp390 = 0x01U;
constexpr uint8_t kMmc5983ma = 0x02U;
} // namespace ConfigTarget

namespace AckStatus {
constexpr uint8_t kOk = 0x00U;
constexpr uint8_t kError = 0x01U;
constexpr uint8_t kInvalidParam = 0x02U;
constexpr uint8_t kSensorBusy = 0x03U;
constexpr uint8_t kUnsupported = 0x04U;
} // namespace AckStatus

namespace ConfigCommandId {
constexpr uint8_t kSetOdr = 0x10U;
constexpr uint8_t kTriggerSetReset = 0x70U;
} // namespace ConfigCommandId

#pragma pack(push, 1)

struct Bmp390DataPayload {
  uint32_t timestampUs;
  float pressurePa;
  float tempC;
  uint8_t statusFlags;
};

struct Mmc5983maDataPayload {
  uint32_t timestampUs;
  float magXGauss;
  float magYGauss;
  float magZGauss;
  float tempC;
  uint8_t statusFlags;
};

struct ConfigCmdPayload {
  uint8_t targetSensor;
  uint8_t commandId;
  uint32_t paramValue;
};

struct AckPayload {
  uint8_t targetSensor;
  uint8_t commandId;
  uint8_t status;
};

struct ErrorPayload {
  uint8_t errorCode;
  uint8_t relatedMsgId;
};

#pragma pack(pop)

static_assert(sizeof(Bmp390DataPayload) == 13U);
static_assert(sizeof(Mmc5983maDataPayload) == 21U);
static_assert(sizeof(ConfigCmdPayload) == 6U);
static_assert(sizeof(AckPayload) == 3U);
static_assert(sizeof(ErrorPayload) == 2U);

constexpr std::size_t kBmp390FrameSize = kHeaderSize + sizeof(Bmp390DataPayload) + kCrcSize;
constexpr std::size_t kMmc5983maFrameSize = kHeaderSize + sizeof(Mmc5983maDataPayload) + kCrcSize;
constexpr std::size_t kConfigCmdFrameSize = kHeaderSize + sizeof(ConfigCmdPayload) + kCrcSize;
constexpr std::size_t kAckFrameSize = kHeaderSize + sizeof(AckPayload) + kCrcSize;
constexpr std::size_t kErrorFrameSize = kHeaderSize + sizeof(ErrorPayload) + kCrcSize;

/** CRC-8 Dallas/MAXIM, полином 0x8C (reflected 0x31). Данные: MSG_ID + LENGTH + PAYLOAD. */
[[nodiscard]] uint8_t crc8Maxim(std::span<const uint8_t> data);

/** Проверка CRC кадра (байты с индекса 2 до конца, без поля CRC). */
[[nodiscard]] bool verifyFrameCrc(std::span<const uint8_t> frame);

struct ReceivedFrame {
  MsgId msgId{};
  std::array<uint8_t, kMaxPayloadSize> payload{};
  std::size_t payloadSize = 0U;
};

enum class ParseFeedResult : uint8_t {
  NeedMore,
  FrameReady,
  SyncLost,
};

/** Потоковый разбор кадров из UART (состояние хранится в объекте). */
class FrameParser {
public:
  void reset();
  [[nodiscard]] ParseFeedResult feed(uint8_t byte, ReceivedFrame& outFrame);

private:
  enum class State : uint8_t {
    WaitSync1,
    WaitSync2,
    Header,
    Payload,
    Crc,
  };

  State mState = State::WaitSync1;
  uint8_t mMsgId = 0U;
  uint8_t mLength = 0U;
  std::size_t mPayloadIndex = 0U;
  std::array<uint8_t, kMaxPayloadSize> mPayload{};
};

/**
 * @brief Собрать кадр в @p frame.
 * @return Длина кадра в байтах или 0, если буфер мал или payload > 255.
 */
[[nodiscard]] std::size_t encodeFrame(MsgId msgId, std::span<const uint8_t> payload,
                                      std::span<uint8_t> frame);

[[nodiscard]] std::size_t encodeBmp390Frame(const Bmp390DataPayload& payload,
                                              std::span<uint8_t> frame);

[[nodiscard]] std::size_t encodeMmc5983maFrame(const Mmc5983maDataPayload& payload,
                                                 std::span<uint8_t> frame);

[[nodiscard]] std::size_t encodeConfigCmdFrame(const ConfigCmdPayload& payload,
                                                 std::span<uint8_t> frame);

[[nodiscard]] std::size_t encodeAckFrame(const AckPayload& payload, std::span<uint8_t> frame);

[[nodiscard]] std::size_t encodeErrorFrame(const ErrorPayload& payload, std::span<uint8_t> frame);

[[nodiscard]] inline std::array<uint8_t, kBmp390FrameSize> encodeBmp390Frame(
    const Bmp390DataPayload& payload) {
  std::array<uint8_t, kBmp390FrameSize> frame{};
  (void)encodeBmp390Frame(payload, std::span<uint8_t>{frame});
  return frame;
}

[[nodiscard]] inline std::array<uint8_t, kMmc5983maFrameSize> encodeMmc5983maFrame(
    const Mmc5983maDataPayload& payload) {
  std::array<uint8_t, kMmc5983maFrameSize> frame{};
  (void)encodeMmc5983maFrame(payload, std::span<uint8_t>{frame});
  return frame;
}

} // namespace UartProtocol
