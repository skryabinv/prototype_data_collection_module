#include "Mmc5983ma.hpp"

namespace {
constexpr uint8_t REG_XOUT0 = 0x00;
constexpr uint8_t REG_STATUS = 0x08;
constexpr uint8_t REG_CTRL0 = 0x09;
constexpr uint8_t REG_CTRL1 = 0x0A;
constexpr uint8_t REG_PROD_ID = 0x2F;

constexpr uint8_t STATUS_MEAS_M_DONE = 0x01;
constexpr uint8_t STATUS_MEAS_T_DONE = 0x02;
constexpr uint8_t STATUS_MEAS_DONE = STATUS_MEAS_M_DONE | STATUS_MEAS_T_DONE;

constexpr uint8_t CTRL0_TM_M = 0x01;
constexpr uint8_t CTRL0_TM_T = 0x02;
constexpr uint8_t CTRL0_INT_MEAS_DONE_EN = 0x04;
constexpr uint8_t CTRL0_SET = 0x08;
constexpr uint8_t CTRL0_RESET = 0x10;
constexpr uint8_t CTRL0_AUTO_SR_EN = 0x20;

constexpr uint8_t PROD_ID_VALUE = 0x30;
} // namespace

Mmc5983ma::Mmc5983ma(I2C_HandleTypeDef& i2c, std::uint8_t address)
    : mI2c(i2c)
    , mAddress(address)
{
}

Mmc5983ma::Error Mmc5983ma::init()
{
    uint8_t prod_id = 0;
    if (readRegister(REG_PROD_ID, &prod_id) != Error::Ok || prod_id != PROD_ID_VALUE) {
        return Error::EDevNotFound;
    }

    if (performSetReset() != Error::Ok) {
        return Error::ECommFail;
    }

    mInitialized = true;
    mMeasurementPending = false;
    return Error::Ok;
}

Mmc5983ma::Error Mmc5983ma::configure(const Config& config)
{
    if (!mInitialized) {
        return Error::ENotInitialized;
    }

    mConfig = config;

    const uint8_t ctrl1_val = static_cast<uint8_t>(config.bandWidth & 0x03U);

    uint8_t ctrl0_val = CTRL0_INT_MEAS_DONE_EN;
    if (config.autoSetReset) {
        ctrl0_val |= CTRL0_AUTO_SR_EN;
    }

    if (writeRegister(REG_CTRL0, ctrl0_val) != Error::Ok) {
        return Error::ECommFail;
    }
    if (writeRegister(REG_CTRL1, ctrl1_val) != Error::Ok) {
        return Error::ECommFail;
    }

    mMeasurementPending = false;
    return startMeasurement();
}

Mmc5983ma::Error Mmc5983ma::startMeasurement()
{
    if (!mInitialized) {
        return Error::ENotInitialized;
    }
    if (mMeasurementPending) {
        return Error::Ok;
    }

    uint8_t ctrl0 = 0;
    if (readRegister(REG_CTRL0, &ctrl0) != Error::Ok) {
        return Error::ECommFail;
    }

    // Магнит + температура, one-shot
    ctrl0 |= static_cast<uint8_t>(CTRL0_TM_M | CTRL0_TM_T);
    if (writeRegister(REG_CTRL0, ctrl0) != Error::Ok) {
        return Error::ECommFail;
    }

    mMeasurementPending = true;
    return Error::Ok;
}

bool Mmc5983ma::fetchStatus(uint8_t& status)
{
    return mInitialized && (readRegister(REG_STATUS, &status) == Error::Ok);
}

bool Mmc5983ma::isDataReady(uint8_t status)
{
    // Ждём оба флага: иначе температура может быть от прошлого измерения
    return (status & STATUS_MEAS_DONE) == STATUS_MEAS_DONE;
}

bool Mmc5983ma::hasUnreadData()
{
    if (!mInitialized) {
        return false;
    }

    if (!mMeasurementPending) {
        (void)startMeasurement();
        return false;
    }

    uint8_t status = 0;
    if (!fetchStatus(status)) {
        return false;
    }
    return isDataReady(status);
}

std::optional<Mmc5983ma::SensorData> Mmc5983ma::readData()
{
    if (!mInitialized || !mMeasurementPending) {
        return std::nullopt;
    }

    // Один раз читаем STATUS здесь (без повторного hasUnreadData)
    uint8_t status = 0;
    if (!fetchStatus(status) || !isDataReady(status)) {
        return std::nullopt;
    }

    uint8_t buf[8] = {};
    const HAL_StatusTypeDef hal_status =
        HAL_I2C_Mem_Read(&mI2c,
                         static_cast<uint16_t>(mAddress << 1),
                         REG_XOUT0,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         sizeof(buf),
                         kI2cTimeoutMs);
    if (hal_status != HAL_OK) {
        mMeasurementPending = false;
        return std::nullopt;
    }

    // Чтение 0x00..0x07 сбрасывает Meas_M_Done / Meas_T_Done
    const uint32_t x_raw = (static_cast<uint32_t>(buf[0]) << 10) |
                           (static_cast<uint32_t>(buf[1]) << 2) |
                           ((buf[6] >> 6) & 0x03U);
    const uint32_t y_raw = (static_cast<uint32_t>(buf[2]) << 10) |
                           (static_cast<uint32_t>(buf[3]) << 2) |
                           ((buf[6] >> 4) & 0x03U);
    const uint32_t z_raw = (static_cast<uint32_t>(buf[4]) << 10) |
                           (static_cast<uint32_t>(buf[5]) << 2) |
                           ((buf[6] >> 2) & 0x03U);
    const uint8_t t_raw = buf[7];

    SensorData data{
        .x = static_cast<float>(static_cast<int32_t>(x_raw) - 131072) / 16384.0f,
        .y = static_cast<float>(static_cast<int32_t>(y_raw) - 131072) / 16384.0f,
        .z = static_cast<float>(static_cast<int32_t>(z_raw) - 131072) / 16384.0f,
        .temperature = -75.0f + (static_cast<float>(t_raw) * 0.8f),
    };

    mMeasurementPending = false;
    (void)startMeasurement();
    return data;
}

Mmc5983ma::Error Mmc5983ma::performSetReset()
{
    if (writeRegister(REG_CTRL0, CTRL0_SET) != Error::Ok) {
        return Error::ECommFail;
    }
    HAL_Delay(1);

    if (writeRegister(REG_CTRL0, CTRL0_RESET) != Error::Ok) {
        return Error::ECommFail;
    }
    HAL_Delay(1);

    return Error::Ok;
}

Mmc5983ma::Error Mmc5983ma::writeRegister(uint8_t reg, uint8_t value)
{
    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Write(&mI2c,
                          static_cast<uint16_t>(mAddress << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1,
                          kI2cTimeoutMs);
    return (status == HAL_OK) ? Error::Ok : Error::ECommFail;
}

Mmc5983ma::Error Mmc5983ma::readRegister(uint8_t reg, uint8_t* value)
{
    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(&mI2c,
                         static_cast<uint16_t>(mAddress << 1),
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         value,
                         1,
                         kI2cTimeoutMs);
    return (status == HAL_OK) ? Error::Ok : Error::ECommFail;
}
