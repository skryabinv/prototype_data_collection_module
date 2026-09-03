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

class Mmc5983ma {
public:
    enum class Error : std::int8_t {
        Ok = 0,
        ECommFail = -1,
        EDevNotFound = -2, // Product ID != 0x30
        ENotInitialized = -3,
    };

    struct SensorData {
        float x = 0.0f;           // Gauss
        float y = 0.0f;           // Gauss
        float z = 0.0f;           // Gauss
        float temperature = 0.0f; // Celsius
        bool valid = false;
    };

    struct Config {
        bool autoSetReset = true;
        // 0x00 = 100 Hz (8 ms), 0x03 = 800 Hz (0.5 ms)
        std::uint8_t bandWidth = 0x00;
    };

    // 7-bit I2C address; datasheet default 0x30
    explicit Mmc5983ma(I2C_HandleTypeDef& i2c, std::uint8_t address = 0x30);

    Mmc5983ma(const Mmc5983ma&) = delete;
    Mmc5983ma& operator=(const Mmc5983ma&) = delete;
    Mmc5983ma(Mmc5983ma&&) = delete;
    Mmc5983ma& operator=(Mmc5983ma&&) = delete;

    [[nodiscard]] Error init();
    [[nodiscard]] Error configure(const Config& config);

    /// Запуск one-shot измерения (неблокирующий).
    [[nodiscard]] Error startMeasurement();

    /// Meas_M_Done в STATUS — есть непрочитанный sample.
    [[nodiscard]] bool hasUnreadData();

    /// Чтение sample; после успеха сразу стартует следующее измерение.
    [[nodiscard]] SensorData readData();

    [[nodiscard]] bool isInitialized() const { return mInitialized; }

private:
    static constexpr std::uint32_t kI2cTimeoutMs = 100;

    I2C_HandleTypeDef& mI2c;
    std::uint8_t mAddress;
    bool mInitialized = false;
    bool mMeasurementPending = false;
    Config mConfig{};

    Error writeRegister(std::uint8_t reg, std::uint8_t value);
    Error readRegister(std::uint8_t reg, std::uint8_t* value);
    Error performSetReset();
};

#endif // __cplusplus
