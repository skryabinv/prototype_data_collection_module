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
    };

    struct MagVector {
        float x = 0.0f; // Gauss
        float y = 0.0f;
        float z = 0.0f;
    };

    struct SensorData {
        MagVector field;          // H с вычетом калибровочного offset
        MagVector offset;         // Offset = (Result_Set + Result_Reset) / 2
        float temperature = 0.0f; // Celsius
    };

    /// CM_Freq[2:0] и Prd_set[2:0] — значения полей даташита (рег. 0x0B).
    struct Config {
        std::uint8_t bandWidth = 0x00;
        std::uint8_t cmFrequency = 0x04; // 50 Hz при BW=00
        std::uint8_t periodicSetEvery = 0x05; // 500 измерений между SET
        bool enablePeriodicSet = true;

        static Config defaultConfig();
    };

    explicit Mmc5983ma(I2C_HandleTypeDef& i2c, std::uint8_t address = 0x30);

    Mmc5983ma(const Mmc5983ma&) = delete;
    Mmc5983ma& operator=(const Mmc5983ma&) = delete;
    Mmc5983ma(Mmc5983ma&&) = delete;
    Mmc5983ma& operator=(Mmc5983ma&&) = delete;

    /// Chip ID + блокирующая калибровка SET/RESET offset.
    [[nodiscard]] Status init();

    /// INT_meas_done, AUTO_SR (для En_prd_set), BW; без включения CMM.
    [[nodiscard]] Status configure(const Config& config);

    /// Включить Continuous Measurement Mode (рег. 0x0B).
    [[nodiscard]] Status startContinuous();

    /// Чтение 0x00–0x07 после прерывания meas_done.
    [[nodiscard]] std::optional<SensorData> readData();

    /// Вызов из EXTI (ISR): только osEventFlagsSet, без I2C.
    static void notifyMeasurementDoneFromIsr();

    [[nodiscard]] const MagVector& calibratedOffset() const { return mCalibratedOffset; }
    [[nodiscard]] bool isInitialized() const { return mInitialized; }

private:
    static constexpr std::uint32_t kI2cTimeoutMs = 100;
    static constexpr std::uint32_t kCoilSettleMs = 1;
    static constexpr std::uint32_t kMeasDoneTimeoutMs = 50;

    I2C_HandleTypeDef& mI2c;
    std::uint8_t mAddress;
    bool mInitialized = false;
    Config mConfig{};
    MagVector mCalibratedOffset{};

    [[nodiscard]] Status calibrateOffset();
    [[nodiscard]] Status issueSetPulse();
    [[nodiscard]] Status issueResetPulse();
    [[nodiscard]] Status startMagMeasurement();
    [[nodiscard]] Status waitForMagDone();
    [[nodiscard]] std::optional<MagVector> readMagGauss();
    [[nodiscard]] Status writeCtrl2(bool cmmEnable);
    [[nodiscard]] static MagVector offsetFromSetReset(const MagVector& after_set,
                                                      const MagVector& after_reset);

    [[nodiscard]] Status writeRegister(std::uint8_t reg, std::uint8_t value);
    [[nodiscard]] Status readRegister(std::uint8_t reg, std::uint8_t* value);
};

#endif // __cplusplus
