#include "Mmc5983ma.hpp"
#include "SensorSignals.hpp"

extern "C" {
#include "stm32h7xx_hal.h"
}

namespace {
constexpr uint8_t REG_XOUT0 = 0x00;
constexpr uint8_t REG_STATUS = 0x08;
constexpr uint8_t REG_CTRL0 = 0x09;
constexpr uint8_t REG_CTRL1 = 0x0A;
constexpr uint8_t REG_CTRL2 = 0x0B;
constexpr uint8_t REG_PROD_ID = 0x2F;

constexpr uint8_t STATUS_MEAS_M_DONE = 0x01;

constexpr uint8_t CTRL0_TM_M = 0x01;
constexpr uint8_t CTRL0_TM_T = 0x02;
constexpr uint8_t CTRL0_INT_MEAS_DONE_EN = 0x04;
constexpr uint8_t CTRL0_SET = 0x08;
constexpr uint8_t CTRL0_RESET = 0x10;
constexpr uint8_t CTRL0_AUTO_SR_EN = 0x20;

constexpr uint8_t CTRL2_CMM_EN = 0x08;

constexpr uint8_t PROD_ID_VALUE = 0x30;

constexpr uint32_t kMagNullOffsetCounts = 131072U;
constexpr float kCountsPerGauss = 16384.0f;
constexpr float kTempOffsetCelsius = -75.0f;
constexpr float kTempLsbCelsius = 0.8f;

uint32_t parseAxisRaw(uint8_t out_msb, uint8_t out_lsb, uint8_t packed_lsb, uint8_t lsb_shift)
{
    return (static_cast<uint32_t>(out_msb) << 10) |
           (static_cast<uint32_t>(out_lsb) << 2) |
           ((packed_lsb >> lsb_shift) & 0x03U);
}

float rawMagToGauss(uint32_t raw_counts)
{
    const int32_t signed_counts =
        static_cast<int32_t>(raw_counts) - static_cast<int32_t>(kMagNullOffsetCounts);
    return static_cast<float>(signed_counts) / kCountsPerGauss;
}

float rawTempToCelsius(uint8_t raw_temp)
{
    return kTempOffsetCelsius + static_cast<float>(raw_temp) * kTempLsbCelsius;
}

Mmc5983ma::MagVector magVectorFromBuffer(const uint8_t* buf)
{
    return Mmc5983ma::MagVector{
        .x = rawMagToGauss(parseAxisRaw(buf[0], buf[1], buf[6], 6)),
        .y = rawMagToGauss(parseAxisRaw(buf[2], buf[3], buf[6], 4)),
        .z = rawMagToGauss(parseAxisRaw(buf[4], buf[5], buf[6], 2)),
    };
}
} // namespace

Mmc5983ma::Config Mmc5983ma::Config::defaultConfig()
{
    Config config{};
    config.bandWidth = 0x00;
    config.cmFrequency = 0x04;
    config.periodicSetEvery = 0x05;
    config.enablePeriodicSet = true;
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

    if (calibrateOffset() != Status::Ok) {
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
    if (config.enablePeriodicSet) {
        ctrl0_val |= CTRL0_AUTO_SR_EN;
    }

    if (writeRegister(REG_CTRL0, ctrl0_val) != Status::Ok) {
        return Status::ECommFail;
    }
    if (writeRegister(REG_CTRL1, ctrl1_val) != Status::Ok) {
        return Status::ECommFail;
    }
    if (writeCtrl2(false) != Status::Ok) {
        return Status::ECommFail;
    }

    return Status::Ok;
}

Mmc5983ma::Status Mmc5983ma::startContinuous()
{
    if (!mInitialized) {
        return Status::ENotInitialized;
    }
    return writeCtrl2(true);
}

void Mmc5983ma::notifyMeasurementDoneFromIsr()
{
    SensorSignals::mmcMeasDoneFromIsr();
}

std::optional<Mmc5983ma::SensorData> Mmc5983ma::readData()
{
    if (!mInitialized) {
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

    const MagVector raw = magVectorFromBuffer(buf);
    const SensorData data{
        .field =
            MagVector{
                .x = raw.x - mCalibratedOffset.x,
                .y = raw.y - mCalibratedOffset.y,
                .z = raw.z - mCalibratedOffset.z,
            },
        .offset = mCalibratedOffset,
        .temperature = rawTempToCelsius(buf[7]),
    };

    uint8_t status = 0;
    (void)readRegister(REG_STATUS, &status);

    return data;
}

Mmc5983ma::Status Mmc5983ma::calibrateOffset()
{
    if (issueSetPulse() != Status::Ok) {
        return Status::ECommFail;
    }
    HAL_Delay(kCoilSettleMs);

    if (startMagMeasurement() != Status::Ok) {
        return Status::ECommFail;
    }
    if (waitForMagDone() != Status::Ok) {
        return Status::ETimeout;
    }
    const auto after_set = readMagGauss();
    if (!after_set.has_value()) {
        return Status::ECommFail;
    }

    if (issueResetPulse() != Status::Ok) {
        return Status::ECommFail;
    }
    HAL_Delay(kCoilSettleMs);

    if (startMagMeasurement() != Status::Ok) {
        return Status::ECommFail;
    }
    if (waitForMagDone() != Status::Ok) {
        return Status::ETimeout;
    }
    const auto after_reset = readMagGauss();
    if (!after_reset.has_value()) {
        return Status::ECommFail;
    }

    mCalibratedOffset = offsetFromSetReset(*after_set, *after_reset);
    return Status::Ok;
}

Mmc5983ma::MagVector Mmc5983ma::offsetFromSetReset(const MagVector& after_set,
                                                   const MagVector& after_reset)
{
    return MagVector{
        .x = (after_set.x + after_reset.x) * 0.5f,
        .y = (after_set.y + after_reset.y) * 0.5f,
        .z = (after_set.z + after_reset.z) * 0.5f,
    };
}

Mmc5983ma::Status Mmc5983ma::writeCtrl2(const bool cmm_enable)
{
    uint8_t ctrl2 = static_cast<uint8_t>(mConfig.cmFrequency & 0x07U);
    if (cmm_enable) {
        ctrl2 |= CTRL2_CMM_EN;
    }
    if (mConfig.enablePeriodicSet) {
        ctrl2 |= static_cast<uint8_t>((mConfig.periodicSetEvery & 0x07U) << 4);
        ctrl2 |= 0x80U; // En_prd_set
    }
    return writeRegister(REG_CTRL2, ctrl2);
}

Mmc5983ma::Status Mmc5983ma::issueSetPulse()
{
    uint8_t ctrl0 = 0;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~CTRL0_SET & ~CTRL0_RESET) | CTRL0_SET);
    return writeRegister(REG_CTRL0, ctrl0);
}

Mmc5983ma::Status Mmc5983ma::issueResetPulse()
{
    uint8_t ctrl0 = 0;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~CTRL0_SET & ~CTRL0_RESET) | CTRL0_RESET);
    return writeRegister(REG_CTRL0, ctrl0);
}

Mmc5983ma::Status Mmc5983ma::startMagMeasurement()
{
    uint8_t ctrl0 = 0;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~(CTRL0_TM_M | CTRL0_TM_T)) | CTRL0_TM_M);
    return writeRegister(REG_CTRL0, ctrl0);
}

Mmc5983ma::Status Mmc5983ma::waitForMagDone()
{
    const uint32_t deadline = HAL_GetTick() + kMeasDoneTimeoutMs;
    while (HAL_GetTick() < deadline) {
        uint8_t status = 0;
        if (readRegister(REG_STATUS, &status) != Status::Ok) {
            return Status::ECommFail;
        }
        if ((status & STATUS_MEAS_M_DONE) != 0U) {
            return Status::Ok;
        }
    }
    return Status::ETimeout;
}

std::optional<Mmc5983ma::MagVector> Mmc5983ma::readMagGauss()
{
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
    return magVectorFromBuffer(buf);
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
