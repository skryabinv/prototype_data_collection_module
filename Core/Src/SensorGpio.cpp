#include "SensorGpio.hpp"
#include "Bmp390.hpp"
#include "Mmc5983ma.hpp"

extern "C" {
#include "main.h"
}

void SensorGpio_initInterruptInputs()
{
    // EXTI и NVIC настроены в MX_GPIO_Init() (gpio.c, CubeMX).
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == MMC5983_INT_Pin) {
        Mmc5983ma::notifyMeasurementDoneFromIsr();
    } else if (gpio_pin == BMP390_DRDY_Pin) {
        Bmp390::notifyDrdyFromIsr();
    }
}
