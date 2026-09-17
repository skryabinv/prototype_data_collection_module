#include "ProtocolConfigHandler.hpp"

namespace ProtocolConfigHandler {

namespace {

UartProtocol::AckPayload makeAck(const UartProtocol::ConfigCmdPayload& cmd, const uint8_t status) {
  return UartProtocol::AckPayload{
      .targetSensor = cmd.targetSensor,
      .commandId = cmd.commandId,
      .status = status,
  };
}

[[nodiscard]] uint8_t applySetOdrBmp(const uint32_t paramValue, Bmp390& bmp390) {
  const uint8_t odr = static_cast<uint8_t>(paramValue & 0xFFU);
  if (bmp390.setOdr(odr) != Bmp390::Error::Ok) {
    return UartProtocol::AckStatus::kError;
  }
  return UartProtocol::AckStatus::kOk;
}

[[nodiscard]] uint8_t applySetOdrMmc(const uint32_t paramValue, Mmc5983ma& mmc5983ma) {
  const uint8_t cmFreq = static_cast<uint8_t>(paramValue & 0x07U);
  if (mmc5983ma.setCmFrequency(cmFreq) != Mmc5983ma::Status::Ok) {
    return UartProtocol::AckStatus::kError;
  }
  return UartProtocol::AckStatus::kOk;
}

[[nodiscard]] uint8_t applySetOdr(const UartProtocol::ConfigCmdPayload& cmd, Bmp390& bmp390,
                                Mmc5983ma& mmc5983ma) {
  switch (cmd.targetSensor) {
  case UartProtocol::ConfigTarget::kBmp390:
    return applySetOdrBmp(cmd.paramValue, bmp390);
  case UartProtocol::ConfigTarget::kMmc5983ma:
    return applySetOdrMmc(cmd.paramValue, mmc5983ma);
  case UartProtocol::ConfigTarget::kAll: {
    const uint8_t bmpStatus = applySetOdrBmp(cmd.paramValue, bmp390);
    if (bmpStatus != UartProtocol::AckStatus::kOk) {
      return bmpStatus;
    }
    return applySetOdrMmc(cmd.paramValue, mmc5983ma);
  }
  default:
    return UartProtocol::AckStatus::kInvalidParam;
  }
}

[[nodiscard]] uint8_t applyTriggerSetReset(const UartProtocol::ConfigCmdPayload& cmd,
                                           Mmc5983ma& mmc5983ma) {
  if (cmd.targetSensor != UartProtocol::ConfigTarget::kMmc5983ma &&
      cmd.targetSensor != UartProtocol::ConfigTarget::kAll) {
    return UartProtocol::AckStatus::kInvalidParam;
  }
  if (mmc5983ma.triggerSetResetCalibration() != Mmc5983ma::Status::Ok) {
    return UartProtocol::AckStatus::kError;
  }
  return UartProtocol::AckStatus::kOk;
}

} // namespace

UartProtocol::AckPayload handleConfigCommand(const UartProtocol::ConfigCmdPayload& cmd,
                                             Bmp390& bmp390, Mmc5983ma& mmc5983ma) {
  switch (cmd.commandId) {
  case UartProtocol::ConfigCommandId::kSetOdr:
    return makeAck(cmd, applySetOdr(cmd, bmp390, mmc5983ma));
  case UartProtocol::ConfigCommandId::kTriggerSetReset:
    return makeAck(cmd, applyTriggerSetReset(cmd, mmc5983ma));
  default:
    return makeAck(cmd, UartProtocol::AckStatus::kUnsupported);
  }
}

} // namespace ProtocolConfigHandler
