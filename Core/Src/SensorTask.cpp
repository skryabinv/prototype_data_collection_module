#include "SensorTask.hpp"
#include "Bmp390.hpp"
#include "Mmc5983ma.hpp"
#include "SensorGpio.hpp"
#include "SensorSignals.hpp"
#include "cmsis_os2.h"
#include <cstdio>

extern I2C_HandleTypeDef hi2c1;

namespace {

[[noreturn]] void fatalError(const char* message)
{
    printf("%s\n", message);
    for (;;) {
        osDelay(1000);
    }
}

} // namespace

void StartSensorTask(void* argument)
{
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

    if (const auto err = bmp390.configure(Bmp390::Config::defaultNormal()); err != Bmp390::Error::Ok) {
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

    printf("Sensors ready (BMP390 DRDY + MMC5983MA CMM on I2C1)\n");

    for (;;) {
        const uint32_t pending =
            SensorSignals::wait(SensorSignals::kFlagAnySensor, osWaitForever);
        if (pending == 0U) {
            continue;
        }

        if ((pending & SensorSignals::kFlagBmpDrdy) != 0U) {
            if (const auto data = bmp390.readData(); data.has_value()) {
                printf("BMP390  T=%.2f C  P=%.2f Pa\n", data->temperature, data->pressure);
            } else {
                printf("BMP390 read failed\n");
            }
        }

        if ((pending & SensorSignals::kFlagMmcMeasDone) != 0U) {
            if (const auto data = mmc5983ma.readData(); data.has_value()) {
                printf("MMC5983 H: X=%.3f Y=%.3f Z=%.3f G  T=%.1f C\n",
                       data->field.x, data->field.y, data->field.z, data->temperature);
                printf("MMC5983 offset: X=%.3f Y=%.3f Z=%.3f G\n",
                       data->offset.x, data->offset.y, data->offset.z);
            } else {
                printf("MMC5983MA read failed\n");
            }
        }
    }
}
