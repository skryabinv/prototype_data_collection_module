#include "UartProtocol.hpp"

#include <cstring>

namespace UartProtocol {

namespace {

constexpr uint8_t kCrcPolyMaxim = 0x8CU;

uint8_t crc8MaximByte(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int bit = 0; bit < 8; ++bit) {
    if ((crc & 0x01U) != 0U) {
      crc = static_cast<uint8_t>((crc >> 1U) ^ kCrcPolyMaxim);
    } else {
      crc >>= 1U;
    }
  }
  return crc;
}

std::size_t encodeTypedPayload(MsgId msgId, const void* payload, std::size_t payloadSize,
                               std::span<uint8_t> frame) {
  return encodeFrame(msgId, std::span<const uint8_t>{static_cast<const uint8_t*>(payload),
                                                     payloadSize},
                     frame);
}

} // namespace

uint8_t crc8Maxim(std::span<const uint8_t> data) {
  uint8_t crc = 0U;
  for (const uint8_t byte : data) {
    crc = crc8MaximByte(crc, byte);
  }
  return crc;
}

std::size_t encodeFrame(MsgId msgId, std::span<const uint8_t> payload, std::span<uint8_t> frame) {
  if (payload.size() > kMaxPayloadSize) {
    return 0U;
  }

  const std::size_t frameSize = kHeaderSize + payload.size() + kCrcSize;
  if (frame.size() < frameSize) {
    return 0U;
  }

  frame[0] = kSync1;
  frame[1] = kSync2;
  frame[2] = static_cast<uint8_t>(msgId);
  frame[3] = static_cast<uint8_t>(payload.size());

  if (!payload.empty()) {
    std::memcpy(frame.data() + kHeaderSize, payload.data(), payload.size());
  }

  std::array<uint8_t, kMaxPayloadSize + 2U> crcInput{};
  crcInput[0] = frame[2];
  crcInput[1] = frame[3];
  if (!payload.empty()) {
    std::memcpy(crcInput.data() + 2U, payload.data(), payload.size());
  }
  frame[kHeaderSize + payload.size()] =
      crc8Maxim(std::span<const uint8_t>{crcInput.data(), 2U + payload.size()});

  return frameSize;
}

std::size_t encodeBmp390Frame(const Bmp390DataPayload& payload, std::span<uint8_t> frame) {
  return encodeTypedPayload(MsgId::SensorDataBmp390, &payload, sizeof(payload), frame);
}

std::size_t encodeMmc5983maFrame(const Mmc5983maDataPayload& payload, std::span<uint8_t> frame) {
  return encodeTypedPayload(MsgId::SensorDataMmc5983ma, &payload, sizeof(payload), frame);
}

std::size_t encodeConfigCmdFrame(const ConfigCmdPayload& payload, std::span<uint8_t> frame) {
  return encodeTypedPayload(MsgId::ConfigCmd, &payload, sizeof(payload), frame);
}

std::size_t encodeAckFrame(const AckPayload& payload, std::span<uint8_t> frame) {
  return encodeTypedPayload(MsgId::Ack, &payload, sizeof(payload), frame);
}

std::size_t encodeErrorFrame(const ErrorPayload& payload, std::span<uint8_t> frame) {
  return encodeTypedPayload(MsgId::Error, &payload, sizeof(payload), frame);
}

bool verifyFrameCrc(std::span<const uint8_t> frame) {
  if (frame.size() < kHeaderSize + kCrcSize) {
    return false;
  }

  const std::size_t payloadLen = frame[3];
  const std::size_t expectedSize = kHeaderSize + payloadLen + kCrcSize;
  if (frame.size() < expectedSize) {
    return false;
  }

  const uint8_t receivedCrc = frame[kHeaderSize + payloadLen];
  const uint8_t calculatedCrc =
      crc8Maxim(std::span<const uint8_t>{frame.data() + 2U, 2U + payloadLen});
  return receivedCrc == calculatedCrc;
}

void FrameParser::reset() {
  mState = State::WaitSync1;
  mMsgId = 0U;
  mLength = 0U;
  mPayloadIndex = 0U;
}

ParseFeedResult FrameParser::feed(const uint8_t byte, ReceivedFrame& outFrame) {
  switch (mState) {
  case State::WaitSync1:
    if (byte == kSync1) {
      mState = State::WaitSync2;
    }
    return ParseFeedResult::NeedMore;

  case State::WaitSync2:
    if (byte == kSync2) {
      mState = State::Header;
      mPayloadIndex = 0U;
      return ParseFeedResult::NeedMore;
    }
    mState = (byte == kSync1) ? State::WaitSync2 : State::WaitSync1;
    return ParseFeedResult::SyncLost;

  case State::Header:
    if (mPayloadIndex == 0U) {
      mMsgId = byte;
      mPayloadIndex = 1U;
      return ParseFeedResult::NeedMore;
    }
    mLength = byte;
    mPayloadIndex = 0U;
    if (mLength == 0U) {
      mState = State::Crc;
      return ParseFeedResult::NeedMore;
    }
    mState = State::Payload;
    return ParseFeedResult::NeedMore;

  case State::Payload:
    mPayload[mPayloadIndex++] = byte;
    if (mPayloadIndex >= mLength) {
      mState = State::Crc;
    }
    return ParseFeedResult::NeedMore;

  case State::Crc: {
    std::array<uint8_t, kMaxFrameSize> frame{};
    frame[0] = kSync1;
    frame[1] = kSync2;
    frame[2] = mMsgId;
    frame[3] = mLength;
    if (mLength > 0U) {
      std::memcpy(frame.data() + kHeaderSize, mPayload.data(), mLength);
    }
    frame[kHeaderSize + mLength] = byte;

    if (!verifyFrameCrc(std::span<const uint8_t>{frame.data(), kHeaderSize + mLength + kCrcSize})) {
      reset();
      return ParseFeedResult::SyncLost;
    }

    outFrame.msgId = static_cast<MsgId>(mMsgId);
    outFrame.payloadSize = mLength;
    if (mLength > 0U) {
      std::memcpy(outFrame.payload.data(), mPayload.data(), mLength);
    }
    reset();
    return ParseFeedResult::FrameReady;
  }
  }

  reset();
  return ParseFeedResult::SyncLost;
}

} // namespace UartProtocol
