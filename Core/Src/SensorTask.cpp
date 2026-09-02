#include "SensorTask.hpp"
#include "Bmp390.hpp"
#include "cmsis_os2.h"
#include <cstdio>

extern I2C_HandleTypeDef hi2c1;

void StartSensorTask(void* argument)
{
    (void)argument;

    Bmp390 bmp390(hi2c1, BMP3_ADDR_I2C_PRIM);

    if (const auto err = bmp390.init(); err != Bmp390::Error::Ok) {
        printf("BMP390 Init Failed, Error: %d\n", static_cast<int>(err));
        for (;;) {
            osDelay(1000);
        }
    }

    Bmp390::Config config{};
    config.pressOversampling = BMP3_OVERSAMPLING_16X;
    config.tempOversampling = BMP3_OVERSAMPLING_2X;
    config.iirFilter = BMP3_IIR_FILTER_COEFF_3;
    config.odr = BMP3_ODR_25_HZ;
    config.opMode = BMP3_MODE_NORMAL;

    if (const auto err = bmp390.configure(config); err != Bmp390::Error::Ok) {
        printf("BMP390 Configure Failed, Error: %d\n", static_cast<int>(err));
        for (;;) {
            osDelay(1000);
        }
    }

    printf("BMP390 Initialized and configured\n");

    for (;;) {
        if (const auto data = bmp390.readData(); data.has_value()) {
            printf("Temp: %.2f C, Pressure: %.2f Pa\n",
                   data->temperature, data->pressure);
        } else {
            printf("BMP390 data not ready / read failed\n");
        }
        osDelay(100);
    }
}
