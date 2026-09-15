#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <cstdint>
#include <optional>

class Mmc5983ma {
public:
    enum class Status : std::int8_t {
        Ok = 0,
        ECommFail = -1,
        EDevNotFound = -2,
        ENotInitialized = -3,
        ETimeout = -4,
        EBusy = -5,
    };

    struct MagVector {
        float x = 0.0f; // Gauss
        float y = 0.0f;
        float z = 0.0f;
    };

    struct SensorData {
        MagVector field;          // H = (Result1 - Result2) / 2
        MagVector offset;         // Offset = (Result1 + Result2) / 2
        float temperature = 0.0f; // Celsius
    };

    struct Config {
        /// SET → measure → RESET → measure (даташит, наиболее точный режим).
        bool enableSetResetMeasurement = true;
        /// AUTO_SR_EN; не совмещать с enableSetResetMeasurement.
        bool autoSetReset = false;
        std::uint8_t bandWidth = 0x00;

        static Config defaultConfig();
    };

    explicit Mmc5983ma(I2C_HandleTypeDef& i2c, std::uint8_t address = 0x30);

    Mmc5983ma(const Mmc5983ma&) = delete;
    Mmc5983ma& operator=(const Mmc5983ma&) = delete;
    Mmc5983ma(Mmc5983ma&&) = delete;
    Mmc5983ma& operator=(Mmc5983ma&&) = delete;

    [[nodiscard]] Status init();
    [[nodiscard]] Status configure(const Config& config);

    /// Старт цикла SET/RESET (только из Idle).
    [[nodiscard]] Status startMeasurement();

    [[nodiscard]] bool hasUnreadData();
    [[nodiscard]] std::optional<SensorData> readData();

    [[nodiscard]] const MagVector& lastOffset() const { return mLastOffset; }
    [[nodiscard]] bool isInitialized() const { return mInitialized; }

private:
    enum class CycleState : std::uint8_t {
        Idle,
        WaitAfterSet,
        WaitMagAfterSet,
        WaitAfterReset,
        WaitMagAfterReset,
        WaitTemperature,
        Ready,
    };

    static constexpr std::uint32_t kI2cTimeoutMs = 100;
    static constexpr std::uint32_t kCoilSettleMs = 1;
    static constexpr std::uint32_t kMeasDoneTimeoutMs = 50;

    I2C_HandleTypeDef& mI2c;
    std::uint8_t mAddress;
    bool mInitialized = false;
    Config mConfig{};
    std::uint8_t mCtrl0Base = 0;
    MagVector mLastOffset{};
    MagVector mResultAfterSet{};
    MagVector mResultAfterReset{};
    float mLastTemperature = 0.0f;

    CycleState mState = CycleState::Idle;
    std::uint32_t mStateDeadlineMs = 0;
    std::uint32_t mMagWaitStartMs = 0;

    void poll();
    [[nodiscard]] Status issueSetPulse();
    [[nodiscard]] Status issueResetPulse();
    [[nodiscard]] Status startMagMeasurement();
    [[nodiscard]] Status startTempMeasurement();
    [[nodiscard]] std::optional<MagVector> readMagGauss();
    [[nodiscard]] Status performSetReset();
    [[nodiscard]] bool fetchStatus(std::uint8_t& status);
    [[nodiscard]] static bool isMagReady(std::uint8_t status);
    [[nodiscard]] static bool isTempReady(std::uint8_t status);

    [[nodiscard]] Status writeRegister(std::uint8_t reg, std::uint8_t value);
    [[nodiscard]] Status readRegister(std::uint8_t reg, std::uint8_t* value);
};

#endif // __cplusplus
