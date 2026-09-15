#include "Mmc5983ma.hpp"

extern "C" {
#include "stm32h7xx_hal.h"
}

namespace {
constexpr uint8_t REG_XOUT0 = 0x00;
constexpr uint8_t REG_STATUS = 0x08;
constexpr uint8_t REG_CTRL0 = 0x09;
constexpr uint8_t REG_CTRL1 = 0x0A;
constexpr uint8_t REG_TEMP = 0x07;
constexpr uint8_t REG_PROD_ID = 0x2F;

constexpr uint8_t STATUS_MEAS_M_DONE = 0x01;
constexpr uint8_t STATUS_MEAS_T_DONE = 0x02;

constexpr uint8_t CTRL0_TM_M = 0x01;
constexpr uint8_t CTRL0_TM_T = 0x02;
constexpr uint8_t CTRL0_INT_MEAS_DONE_EN = 0x04;
constexpr uint8_t CTRL0_SET = 0x08;
constexpr uint8_t CTRL0_RESET = 0x10;
constexpr uint8_t CTRL0_AUTO_SR_EN = 0x20;

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

Mmc5983ma::MagVector fieldFromSetReset(const Mmc5983ma::MagVector& after_set,
                                       const Mmc5983ma::MagVector& after_reset)
{
    return Mmc5983ma::MagVector{
        .x = (after_set.x - after_reset.x) * 0.5f,
        .y = (after_set.y - after_reset.y) * 0.5f,
        .z = (after_set.z - after_reset.z) * 0.5f,
    };
}

Mmc5983ma::MagVector offsetFromSetReset(const Mmc5983ma::MagVector& after_set,
                                        const Mmc5983ma::MagVector& after_reset)
{
    return Mmc5983ma::MagVector{
        .x = (after_set.x + after_reset.x) * 0.5f,
        .y = (after_set.y + after_reset.y) * 0.5f,
        .z = (after_set.z + after_reset.z) * 0.5f,
    };
}
} // namespace

Mmc5983ma::Config Mmc5983ma::Config::defaultConfig()
{
    Config config{};
    config.enableSetResetMeasurement = true;
    config.autoSetReset = false;
    config.bandWidth = 0x00;
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
    mState = CycleState::Idle;
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
    if (config.autoSetReset && !config.enableSetResetMeasurement) {
        ctrl0_val |= CTRL0_AUTO_SR_EN;
    }
    mCtrl0Base = ctrl0_val;

    if (writeRegister(REG_CTRL0, ctrl0_val) != Status::Ok) {
        return Status::ECommFail;
    }
    if (writeRegister(REG_CTRL1, ctrl1_val) != Status::Ok) {
        return Status::ECommFail;
    }

    mState = CycleState::Idle;
    return Status::Ok;
}

Mmc5983ma::Status Mmc5983ma::startMeasurement()
{
    if (!mInitialized) {
        return Status::ENotInitialized;
    }
    if (!mConfig.enableSetResetMeasurement) {
        return Status::ECommFail;
    }
    if (mState != CycleState::Idle) {
        return Status::EBusy;
    }

    if (issueSetPulse() != Status::Ok) {
        return Status::ECommFail;
    }

    mState = CycleState::WaitAfterSet;
    mStateDeadlineMs = HAL_GetTick() + kCoilSettleMs;
    return Status::Ok;
}

bool Mmc5983ma::hasUnreadData()
{
    if (!mInitialized) {
        return false;
    }
    poll();
    return mState == CycleState::Ready;
}

std::optional<Mmc5983ma::SensorData> Mmc5983ma::readData()
{
    if (!mInitialized || mState != CycleState::Ready) {
        return std::nullopt;
    }

    const SensorData data{
        .field = fieldFromSetReset(mResultAfterSet, mResultAfterReset),
        .offset = mLastOffset,
        .temperature = mLastTemperature,
    };

    mState = CycleState::Idle;
    return data;
}

void Mmc5983ma::poll()
{
    if (mState == CycleState::Idle || mState == CycleState::Ready) {
        return;
    }

    const uint32_t now = HAL_GetTick();

    switch (mState) {
    case CycleState::WaitAfterSet:
        if (now < mStateDeadlineMs) {
            return;
        }
        if (startMagMeasurement() != Status::Ok) {
            mState = CycleState::Idle;
            return;
        }
        mMagWaitStartMs = now;
        mState = CycleState::WaitMagAfterSet;
        return;

    case CycleState::WaitMagAfterSet: {
        uint8_t status = 0;
        if (!fetchStatus(status)) {
            return;
        }
        if (!isMagReady(status)) {
            if ((now - mMagWaitStartMs) > kMeasDoneTimeoutMs) {
                mState = CycleState::Idle;
            }
            return;
        }
        const auto sample = readMagGauss();
        if (!sample.has_value()) {
            mState = CycleState::Idle;
            return;
        }
        mResultAfterSet = *sample;
        if (issueResetPulse() != Status::Ok) {
            mState = CycleState::Idle;
            return;
        }
        mStateDeadlineMs = now + kCoilSettleMs;
        mState = CycleState::WaitAfterReset;
        return;
    }

    case CycleState::WaitAfterReset:
        if (now < mStateDeadlineMs) {
            return;
        }
        if (startMagMeasurement() != Status::Ok) {
            mState = CycleState::Idle;
            return;
        }
        mMagWaitStartMs = now;
        mState = CycleState::WaitMagAfterReset;
        return;

    case CycleState::WaitMagAfterReset: {
        uint8_t status = 0;
        if (!fetchStatus(status)) {
            return;
        }
        if (!isMagReady(status)) {
            if ((now - mMagWaitStartMs) > kMeasDoneTimeoutMs) {
                mState = CycleState::Idle;
            }
            return;
        }
        const auto sample = readMagGauss();
        if (!sample.has_value()) {
            mState = CycleState::Idle;
            return;
        }
        mResultAfterReset = *sample;
        mLastOffset = offsetFromSetReset(mResultAfterSet, mResultAfterReset);

        if (startTempMeasurement() != Status::Ok) {
            mLastTemperature = 0.0f;
            mState = CycleState::Ready;
            return;
        }
        mMagWaitStartMs = now;
        mState = CycleState::WaitTemperature;
        return;
    }

    case CycleState::WaitTemperature: {
        uint8_t status = 0;
        if (!fetchStatus(status)) {
            return;
        }
        if (!isTempReady(status)) {
            if ((now - mMagWaitStartMs) > kMeasDoneTimeoutMs) {
                mLastTemperature = 0.0f;
                mState = CycleState::Ready;
            }
            return;
        }
        uint8_t raw_temp = 0;
        if (readRegister(REG_TEMP, &raw_temp) != Status::Ok) {
            mLastTemperature = 0.0f;
        } else {
            mLastTemperature = rawTempToCelsius(raw_temp);
        }
        mState = CycleState::Ready;
        return;
    }

    default:
        return;
    }
}

Mmc5983ma::Status Mmc5983ma::issueSetPulse()
{
    uint8_t ctrl0 = mCtrl0Base;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~CTRL0_SET & ~CTRL0_RESET) | CTRL0_SET);
    return writeRegister(REG_CTRL0, ctrl0);
}

Mmc5983ma::Status Mmc5983ma::issueResetPulse()
{
    uint8_t ctrl0 = mCtrl0Base;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~CTRL0_SET & ~CTRL0_RESET) | CTRL0_RESET);
    return writeRegister(REG_CTRL0, ctrl0);
}

Mmc5983ma::Status Mmc5983ma::startMagMeasurement()
{
    uint8_t ctrl0 = mCtrl0Base;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    // Datasheet: TM_M и TM_T не могут быть 1 одновременно.
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~(CTRL0_TM_M | CTRL0_TM_T)) | CTRL0_TM_M);
    return writeRegister(REG_CTRL0, ctrl0);
}

Mmc5983ma::Status Mmc5983ma::startTempMeasurement()
{
    uint8_t ctrl0 = mCtrl0Base;
    if (readRegister(REG_CTRL0, &ctrl0) != Status::Ok) {
        return Status::ECommFail;
    }
    ctrl0 = static_cast<uint8_t>((ctrl0 & ~(CTRL0_TM_M | CTRL0_TM_T)) | CTRL0_TM_T);
    return writeRegister(REG_CTRL0, ctrl0);
}

std::optional<Mmc5983ma::MagVector> Mmc5983ma::readMagGauss()
{
    uint8_t buf[7] = {};
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

    return MagVector{
        .x = rawMagToGauss(parseAxisRaw(buf[0], buf[1], buf[6], 6)),
        .y = rawMagToGauss(parseAxisRaw(buf[2], buf[3], buf[6], 4)),
        .z = rawMagToGauss(parseAxisRaw(buf[4], buf[5], buf[6], 2)),
    };
}

bool Mmc5983ma::fetchStatus(uint8_t& status)
{
    return mInitialized && (readRegister(REG_STATUS, &status) == Status::Ok);
}

bool Mmc5983ma::isMagReady(uint8_t status)
{
    return (status & STATUS_MEAS_M_DONE) != 0U;
}

bool Mmc5983ma::isTempReady(uint8_t status)
{
    return (status & STATUS_MEAS_T_DONE) != 0U;
}

Mmc5983ma::Status Mmc5983ma::performSetReset()
{
    if (issueSetPulse() != Status::Ok) {
        return Status::ECommFail;
    }
    HAL_Delay(kCoilSettleMs);
    if (issueResetPulse() != Status::Ok) {
        return Status::ECommFail;
    }
    HAL_Delay(kCoilSettleMs);
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
