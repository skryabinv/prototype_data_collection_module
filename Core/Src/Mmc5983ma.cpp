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

// Datasheet: 18-bit output, zero field = 2^17 counts, sensitivity = 16384 counts/G
constexpr uint32_t kMagNullOffsetCounts = 131072U;
constexpr float kCountsPerGauss = 16384.0f;

// Datasheet: T = -75 °C + 0.8 °C/LSB
constexpr float kTempOffsetCelsius = -75.0f;
constexpr float kTempLsbCelsius = 0.8f;

/// Собирает 18-битное значение оси из OUT_MSB, OUT_LSB и 2 бит в XOUT[6].
uint32_t parseAxisRaw(uint8_t out_msb, uint8_t out_lsb, uint8_t packed_lsb, uint8_t lsb_shift)
{
    return (static_cast<uint32_t>(out_msb) << 10) |
           (static_cast<uint32_t>(out_lsb) << 2) |
           ((packed_lsb >> lsb_shift) & 0x03U);
}

float rawMagToGauss(uint32_t raw_counts)
{
    const int32_t signed_counts = static_cast<int32_t>(raw_counts) - static_cast<int32_t>(kMagNullOffsetCounts);
    return static_cast<float>(signed_counts) / kCountsPerGauss;
}

float rawTempToCelsius(uint8_t raw_temp)
{
    return kTempOffsetCelsius + static_cast<float>(raw_temp) * kTempLsbCelsius;
}
} // namespace

Mmc5983ma::Config Mmc5983ma::Config::defaultConfig()
{
    Config config{};
    config.autoSetReset = true;
    config.bandWidth = 0x00; // ~100 Hz measurement time
    return config;
}

Mmc5983ma::Mmc5983ma(I2C_HandleTypeDef& i2c, std::uint8_t address)
    : mI2c(i2c)
    , mAddress(address)
{
}

Mmc5983ma::Status Mmc5983ma::init()
{
    uint8_t prod_id = 0;
    if (readRegister(REG_PROD_ID, &prod_id) != Status::Ok || prod_id != PROD_ID_VALUE) {
        return Status::EDevNotFound;
    }

    if (performSetReset() != Status::Ok) {
        return Status::ECommFail;
    }

    mInitialized = true;
    return Status::Ok;
}

Mmc5983ma::Status Mmc5983ma::configure(const Config& config)
{
    if (!mInitialized) {
        return Status::ENotInitialized;
    }

    mConfig = config;

    const uint8_t ctrl1_val = static_cast<uint8_t>(config.bandWidth & 0x03U);

    uint8_t ctrl0_val = CTRL0_INT_MEAS_DONE_EN;
    if (config.autoSetReset) {
        ctrl0_val |= CTRL0_AUTO_SR_EN;
    }

    if (writeRegister(REG_CTRL0, ctrl0_val) != Status::Ok) {
        return Status::ECommFail;
    }
    if (writeRegister(REG_CTRL1, ctrl1_val) != Status::Ok) {
        return Status::ECommFail;
    }

    return Status::Ok;
}

Mmc5983ma::Status Mmc5983ma::startMeasurement()
{
    if (!mInitialized) {
        return Status::ENotInitialized;
    }

    uint8_t ctrl0 = 0;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }

    ctrl0 |= static_cast<uint8_t>(CTRL0_TM_M | CTRL0_TM_T);
    if (writeRegister(REG_CTRL0, ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }

    return Status::Ok;
}

bool Mmc5983ma::fetchStatus(uint8_t& status)
{
    return mInitialized && (readRegister(REG_STATUS, &status) == Status::Ok);
}

bool Mmc5983ma::isDataReady(uint8_t status)
{
    return (status & STATUS_MEAS_DONE) == STATUS_MEAS_DONE;
}

bool Mmc5983ma::hasUnreadData()
{
    if (!mInitialized) {
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
    if (!mInitialized) {
        return std::nullopt;
    }

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
        return std::nullopt;
    }

    const uint32_t x_raw = parseAxisRaw(buf[0], buf[1], buf[6], 6);
    const uint32_t y_raw = parseAxisRaw(buf[2], buf[3], buf[6], 4);
    const uint32_t z_raw = parseAxisRaw(buf[4], buf[5], buf[6], 2);

    return SensorData{
        .x = rawMagToGauss(x_raw),
        .y = rawMagToGauss(y_raw),
        .z = rawMagToGauss(z_raw),
        .temperature = rawTempToCelsius(buf[7]),
    };
}

Mmc5983ma::Status Mmc5983ma::performSetReset()
{
    if (writeRegister(REG_CTRL0, CTRL0_SET) != Status::Ok) {
        return Status::ECommFail;
    }
    HAL_Delay(1);

    if (writeRegister(REG_CTRL0, CTRL0_RESET) != Status::Ok) {
        return Status::ECommFail;
    }
    HAL_Delay(1);

    return Status::Ok;
}

Mmc5983ma::Status Mmc5983ma::writeRegister(uint8_t reg, uint8_t value)
{
    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Write(&mI2c,
                          static_cast<uint16_t>(mAddress << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1,
                          kI2cTimeoutMs);
    return (status == HAL_OK) ? Status::Ok : Status::ECommFail;
}

Mmc5983ma::Status Mmc5983ma::readRegister(uint8_t reg, uint8_t* value)
{
    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(&mI2c,
                         static_cast<uint16_t>(mAddress << 1),
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         value,
                         1,
                         kI2cTimeoutMs);
    return (status == HAL_OK) ? Status::Ok : Status::ECommFail;
}
