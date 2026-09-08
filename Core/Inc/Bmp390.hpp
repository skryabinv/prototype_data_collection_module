#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "bmp3.h"
#include "main.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <cstdint>
#include <optional>

class Bmp390 {
public:
    enum class Error : std::int8_t {
        Ok = BMP3_OK,
        ENullPointer = BMP3_E_NULL_PTR,
        ECommFail = BMP3_E_COMM_FAIL,
        EDevNotFound = BMP3_E_DEV_NOT_FOUND,
        EInvalidOdr = BMP3_E_INVALID_ODR_OSR_SETTINGS,
        ECmdExecFailed = BMP3_E_CMD_EXEC_FAILED,
        EConfiguration = BMP3_E_CONFIGURATION_ERR,
        ENotInitialized = INT8_C(-100),
    };

    struct SensorData {
        float temperature; // °C
        float pressure;    // Pa
    };

    /// Параметры, которые плата может задать датчику.
    struct Config {
        uint8_t pressOversampling = BMP3_OVERSAMPLING_16X;
        uint8_t tempOversampling = BMP3_OVERSAMPLING_2X;
        uint8_t iirFilter = BMP3_IIR_FILTER_COEFF_3;
        uint8_t odr = BMP3_ODR_25_HZ;
        uint8_t opMode = BMP3_MODE_NORMAL;
        bool enablePressure = true;
        bool enableTemperature = true;

        static Config defaultNormal();
    };

    // BMP3_ADDR_I2C_PRIM = 0x76, BMP3_ADDR_I2C_SEC = 0x77
    explicit Bmp390(I2C_HandleTypeDef& i2c, uint8_t address = BMP3_ADDR_I2C_PRIM);
    ~Bmp390() = default;

    Bmp390(const Bmp390&) = delete;
    Bmp390& operator=(const Bmp390&) = delete;
    Bmp390(Bmp390&&) = delete;
    Bmp390& operator=(Bmp390&&) = delete;

    /// Связь с датчиком: chip-id и калибровочные коэффициенты.
    [[nodiscard]] Error init();

    /// Конфигурация режимов/OSR/ODR/фильтра (можно вызывать повторно).
    [[nodiscard]] Error configure(const Config& config);

    /// Есть непрочитанный sample (флаги DRDY в status).
    [[nodiscard]] bool hasUnreadData();

    /// Чтение температуры и давления (когда hasUnreadData() == true).
    [[nodiscard]] std::optional<SensorData> readData();

    [[nodiscard]] bool isInitialized() const { return mInitialized; }

private:
    static constexpr uint32_t kI2cTimeoutMs = 100;

    I2C_HandleTypeDef& mHi2c;
    uint8_t mAddress;
    bool mInitialized = false;
    bmp3_dev mDev{};
    bmp3_settings mSettings{};

    [[nodiscard]] bool fetchStatus(bmp3_status& status);
    [[nodiscard]] static bool isDataReady(const bmp3_status& status);

    static int8_t i2cRead(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
    static int8_t i2cWrite(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
    static void delayUs(uint32_t period, void* intf_ptr);

    static Error toError(int8_t result);
};

#endif // __cplusplus
