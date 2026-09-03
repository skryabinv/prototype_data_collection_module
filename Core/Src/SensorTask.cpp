#include "SensorTask.hpp"
#include "Bmp390.hpp"
#include "Mmc5983ma.hpp"
#include "cmsis_os2.h"
#include <cstdio>

extern I2C_HandleTypeDef hi2c1;

void StartSensorTask(void* argument)
{
    (void)argument;

    // TODO: проверить на железе I2C-адреса:
    // BMP390: 0x76 (SDO→GND) или 0x77 (SDO→VDD); MMC5983MA: обычно 0x30.
    Bmp390 bmp390(hi2c1, BMP3_ADDR_I2C_PRIM);
    Mmc5983ma mmc5983ma(hi2c1, 0x30);

    if (const auto err = bmp390.init(); err != Bmp390::Error::Ok) {
        printf("BMP390 init failed: %d\n", static_cast<int>(err));
        for (;;) {
            osDelay(1000);
        }
    }
    if (const auto err = mmc5983ma.init(); err != Mmc5983ma::Error::Ok) {
        printf("MMC5983MA init failed: %d\n", static_cast<int>(err));
        for (;;) {
            osDelay(1000);
        }
    }

    Bmp390::Config bmpConfig{};
    bmpConfig.pressOversampling = BMP3_OVERSAMPLING_16X;
    bmpConfig.tempOversampling = BMP3_OVERSAMPLING_2X;
    bmpConfig.iirFilter = BMP3_IIR_FILTER_COEFF_3;
    bmpConfig.odr = BMP3_ODR_25_HZ;
    bmpConfig.opMode = BMP3_MODE_NORMAL;

    Mmc5983ma::Config mmcConfig{};
    mmcConfig.autoSetReset = true;
    mmcConfig.bandWidth = 0x00; // ~100 Hz measurement time

    if (const auto err = bmp390.configure(bmpConfig); err != Bmp390::Error::Ok) {
        printf("BMP390 configure failed: %d\n", static_cast<int>(err));
        for (;;) {
            osDelay(1000);
        }
    }
    if (const auto err = mmc5983ma.configure(mmcConfig); err != Mmc5983ma::Error::Ok) {
        printf("MMC5983MA configure failed: %d\n", static_cast<int>(err));
        for (;;) {
            osDelay(1000);
        }
    }

    printf("Sensors ready (BMP390 + MMC5983MA on I2C1)\n");

    for (;;) {
        if (bmp390.hasUnreadData()) {
            if (const auto data = bmp390.readData(); data.has_value()) {
                printf("BMP390  T=%.2f C  P=%.2f Pa\n", data->temperature, data->pressure);
            } else {
                printf("BMP390 read failed\n");
            }
        }

        if (mmc5983ma.hasUnreadData()) {
            if (const auto data = mmc5983ma.readData(); data.has_value()) {
                printf("MMC5983 X=%.3f Y=%.3f Z=%.3f G  T=%.1f C\n",
                       data->x, data->y, data->z, data->temperature);
            } else {
                printf("MMC5983MA read failed\n");
            }
        }

        // Всегда отдаём CPU на 1 тик (configTICK_RATE_HZ = 1000)
        osDelay(1);
    }
}
