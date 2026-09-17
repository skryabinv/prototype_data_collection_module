#include "SensorTask.hpp"

#include "Bmp390.hpp"
#include "Mmc5983ma.hpp"
#include "ProtocolConfigHandler.hpp"
#include "SensorGpio.hpp"
#include "SensorSignals.hpp"
#include "Uart4Link.hpp"
#include "UartProtocol.hpp"
#include "cmsis_os2.h"

#include <array>
#include <cstdio>
#include <cstring>

extern I2C_HandleTypeDef hi2c1;

namespace {

[[noreturn]] void fatalError(const char* message) {
  printf("%s\n", message);
  for (;;) {
    osDelay(1000);
  }
}

uint32_t timestampUs() {
  const uint32_t tick = osKernelGetTickCount();
  const uint32_t tickHz = osKernelGetTickFreq();
  if (tickHz == 0U) {
    return 0U;
  }
  return (tick * 1000000U) / tickHz;
}

void processUartRx(Uart4Link& uart, Bmp390& bmp390, Mmc5983ma& mmc5983ma) {
  uart.pollRxParser();

  UartProtocol::ReceivedFrame frame{};
  while (uart.popReceivedFrame(frame)) {
    if (frame.msgId != UartProtocol::MsgId::ConfigCmd) {
      continue;
    }
    if (frame.payloadSize != sizeof(UartProtocol::ConfigCmdPayload)) {
      continue;
    }

    UartProtocol::ConfigCmdPayload cmd{};
    std::memcpy(&cmd, frame.payload.data(), sizeof(cmd));

    const UartProtocol::AckPayload ack =
        ProtocolConfigHandler::handleConfigCommand(cmd, bmp390, mmc5983ma);

    std::array<uint8_t, UartProtocol::kAckFrameSize> ackFrame{};
    if (UartProtocol::encodeAckFrame(ack, std::span<uint8_t>{ackFrame}) == 0U) {
      continue;
    }
    (void)uart.transmitBlocking(std::span<const uint8_t>{ackFrame});
  }
}

void sendBmp390Frame(Uart4Link& uart, const Bmp390::SensorData& data) {
  const UartProtocol::Bmp390DataPayload payload{
      .timestampUs = timestampUs(),
      .pressurePa = data.pressure,
      .tempC = data.temperature,
      .statusFlags =
          UartProtocol::Bmp390Flags::kOk | UartProtocol::Bmp390Flags::kDataReady,
  };

  std::array<uint8_t, UartProtocol::kBmp390FrameSize> frame{};
  if (UartProtocol::encodeBmp390Frame(payload, std::span<uint8_t>{frame}) == 0U) {
    return;
  }
  if (!uart.transmitDma(std::span<const uint8_t>{frame})) {
    (void)uart.transmitBlocking(std::span<const uint8_t>{frame});
  }
}

void sendMmc5983maFrame(Uart4Link& uart, const Mmc5983ma::SensorData& data) {
  const UartProtocol::Mmc5983maDataPayload payload{
      .timestampUs = timestampUs(),
      .magXGauss = data.field.x,
      .magYGauss = data.field.y,
      .magZGauss = data.field.z,
      .tempC = data.temperature,
      .statusFlags =
          UartProtocol::Mmc5983maFlags::kOk | UartProtocol::Mmc5983maFlags::kDataReady,
  };

  std::array<uint8_t, UartProtocol::kMmc5983maFrameSize> frame{};
  if (UartProtocol::encodeMmc5983maFrame(payload, std::span<uint8_t>{frame}) == 0U) {
    return;
  }
  if (!uart.transmitDma(std::span<const uint8_t>{frame})) {
    (void)uart.transmitBlocking(std::span<const uint8_t>{frame});
  }
}

} // namespace

void StartSensorTask(void* argument) {
  (void)argument;

  SensorSignals::init();

  Bmp390 bmp390(hi2c1, BMP3_ADDR_I2C_PRIM);
  Mmc5983ma mmc5983ma(hi2c1, 0x30);

  if (const auto err = bmp390.init(); err != Bmp390::Error::Ok) {
    fatalError("BMP390 init failed");
  }
  if (const auto err = mmc5983ma.init(); err != Mmc5983ma::Status::Ok) {
    fatalError("MMC5983MA init failed");
  }

  if (const auto err = bmp390.configure(Bmp390::Config::defaultNormal());
      err != Bmp390::Error::Ok) {
    fatalError("BMP390 configure failed");
  }
  if (const auto err = mmc5983ma.configure(Mmc5983ma::Config::defaultConfig());
      err != Mmc5983ma::Status::Ok) {
    fatalError("MMC5983MA configure failed");
  }
  if (const auto err = mmc5983ma.startContinuous(); err != Mmc5983ma::Status::Ok) {
    fatalError("MMC5983MA CMM start failed");
  }

  SensorGpio_initInterruptInputs();

  Uart4Link& uart = Uart4Link::instance();
  if (!uart.init()) {
    fatalError("UART4 link init failed");
  }

  printf("Sensors ready (BMP390 DRDY + MMC5983MA CMM on I2C1, UART4 protocol)\n");

  for (;;) {
    const uint32_t pending = SensorSignals::wait(SensorSignals::kFlagTaskWake, osWaitForever);
    if (pending == 0U) {
      continue;
    }

    processUartRx(uart, bmp390, mmc5983ma);

    if ((pending & SensorSignals::kFlagBmpDrdy) != 0U) {
      if (const auto data = bmp390.readData(); data.has_value()) {
        sendBmp390Frame(uart, *data);
        printf("BMP390  T=%.2f C  P=%.2f Pa\n", data->temperature, data->pressure);
      } else {
        printf("BMP390 read failed\n");
      }
    }

    if ((pending & SensorSignals::kFlagMmcMeasDone) != 0U) {
      if (const auto data = mmc5983ma.readData(); data.has_value()) {
        sendMmc5983maFrame(uart, *data);
        printf("MMC5983 H: X=%.3f Y=%.3f Z=%.3f G  T=%.1f C\n", data->field.x, data->field.y,
               data->field.z, data->temperature);
        printf("MMC5983 offset: X=%.3f Y=%.3f Z=%.3f G\n", data->offset.x, data->offset.y,
               data->offset.z);
      } else {
        printf("MMC5983MA read failed\n");
      }
    }
  }
}
