#include "Bmp390.hpp"

Bmp390::Bmp390(I2C_HandleTypeDef& i2c, uint8_t address)
    : mHi2c(i2c)
    , mAddress(address)
{
    mDev.intf = BMP3_I2C_INTF;
    mDev.intf_ptr = this;
    mDev.read = i2cRead;
    mDev.write = i2cWrite;
    mDev.delay_us = delayUs;
    mDev.dummy_byte = 0;
    mDev.chip_id = 0;
    mDev.intf_rslt = BMP3_INTF_RET_SUCCESS;
}

Bmp390::Error Bmp390::init()
{
    const int8_t result = bmp3_init(&mDev);
    mInitialized = (result == BMP3_OK);
    return toError(result);
}

Bmp390::Error Bmp390::configure(const Config& config)
{
    if (!mInitialized) {
        return Error::ENotInitialized;
    }

    mSettings = {};
    mSettings.press_en = config.enablePressure ? BMP3_ENABLE : BMP3_DISABLE;
    mSettings.temp_en = config.enableTemperature ? BMP3_ENABLE : BMP3_DISABLE;
    mSettings.odr_filter.press_os = config.pressOversampling;
    mSettings.odr_filter.temp_os = config.tempOversampling;
    mSettings.odr_filter.iir_filter = config.iirFilter;
    mSettings.odr_filter.odr = config.odr;
    mSettings.int_settings.drdy_en = BMP3_ENABLE;

    const uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                                  BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR |
                                  BMP3_SEL_DRDY_EN;

    int8_t result = bmp3_set_sensor_settings(settings_sel, &mSettings, &mDev);
    if (result != BMP3_OK) {
        return toError(result);
    }

    mSettings.op_mode = config.opMode;
    result = bmp3_set_op_mode(&mSettings, &mDev);
    return toError(result);
}

bool Bmp390::fetchStatus(bmp3_status& status)
{
    return mInitialized && (bmp3_get_status(&status, &mDev) == BMP3_OK);
}

bool Bmp390::isDataReady(const bmp3_status& status)
{
    return (status.intr.drdy == BMP3_ENABLE) ||
           (status.sensor.drdy_press == BMP3_ENABLE) ||
           (status.sensor.drdy_temp == BMP3_ENABLE);
}

bool Bmp390::hasUnreadData()
{
    bmp3_status status{};
    if (!fetchStatus(status)) {
        return false;
    }
    return isDataReady(status);
}

std::optional<Bmp390::SensorData> Bmp390::readData()
{
    bmp3_status status{};
    if (!fetchStatus(status) || !isDataReady(status)) {
        return std::nullopt;
    }

    bmp3_data data{};
    const int8_t result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &mDev);
    if (result != BMP3_OK) {
        return std::nullopt;
    }

    // Сброс флага data-ready
    (void)bmp3_get_status(&status, &mDev);

#ifdef BMP3_FLOAT_COMPENSATION
    return SensorData{
        .temperature = static_cast<float>(data.temperature),
        .pressure = static_cast<float>(data.pressure),
    };
#else
    // Integer compensation: temperature в 0.01 °C, pressure в 0.01 Pa
    return SensorData{
        .temperature = static_cast<float>(data.temperature) / 100.0f,
        .pressure = static_cast<float>(data.pressure) / 100.0f,
    };
#endif
}

Bmp390::Error Bmp390::toError(int8_t result)
{
    return static_cast<Error>(result);
}

int8_t Bmp390::i2cRead(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    auto* sensor = static_cast<Bmp390*>(intf_ptr);

    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&sensor->mHi2c,
                                                      static_cast<uint16_t>(sensor->mAddress << 1),
                                                      reg_addr,
                                                      I2C_MEMADD_SIZE_8BIT,
                                                      reg_data,
                                                      static_cast<uint16_t>(len),
                                                      kI2cTimeoutMs);

    return (status == HAL_OK) ? BMP3_OK : BMP3_E_COMM_FAIL;
}

int8_t Bmp390::i2cWrite(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    auto* sensor = static_cast<Bmp390*>(intf_ptr);

    const HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&sensor->mHi2c,
                                                       static_cast<uint16_t>(sensor->mAddress << 1),
                                                       reg_addr,
                                                       I2C_MEMADD_SIZE_8BIT,
                                                       const_cast<uint8_t*>(reg_data),
                                                       static_cast<uint16_t>(len),
                                                       kI2cTimeoutMs);

    return (status == HAL_OK) ? BMP3_OK : BMP3_E_COMM_FAIL;
}

void Bmp390::delayUs(uint32_t period, void* /*intf_ptr*/)
{
    // HAL_Delay — миллисекунды; округляем вверх
    uint32_t ms = (period + 999U) / 1000U;
    if (ms > 0U) {
        HAL_Delay(ms);
    }
}
