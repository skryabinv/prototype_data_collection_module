Ниже представлен **итоговый, полностью переработанный C++ Coding Standard**, адаптированный специально для вашего стека: **STM32 + STM32CubeMX (HAL) + RTOS + CMake + Modern C++**. 

Этот документ исключает «ловушки» десктопного C++ (исключения, куча, RTTI) и добавляет критически важные правила для микроконтроллеров.

---

# C++ Coding Standard for Embedded (STM32 + RTOS)
**Версия:** 2.0 (Embedded)  
**Стек:** C++17/20, STM32 HAL, FreeRTOS/ThreadX, CMake, ARM GCC.

## 1. Главные принципы и Жесткие ограничения (Embedded Specific)
Главный принцип: **Детерминированность, безопасность и предсказуемость использования памяти.**

### 1.1. Запрещено (STRICTLY FORBIDDEN)
1. **Исключения (`try`, `catch`, `throw`):** Запрещены. Используют неопределенное время выполнения и потребляют Flash.
2. **Динамическая память (`new`, `delete`, `malloc`, `free`):** Запрещена. Ведет к фрагментации кучи и `HardFault`.
3. **Тяжелый STL:** Запрещены `std::vector`, `std::string`, `std::map`, `std::list`, `std::shared_ptr`, `std::unique_ptr` (с дефолтным аллокатором).
4. **RTTI (`dynamic_cast`, `typeid`):** Запрещено. Потребляет Flash.
5. **`std::cout` / `printf`:** Запрещены в продакшн-коде. Используйте собственные легковесные функции логирования через UART.

### 1.2. Разрешено и Рекомендуется
1. **Static STL:** `std::array`, `std::span` (C++20), `std::string_view`, `std::optional`, `std::expected` (C++23).
2. **ETL (Embedded Template Library):** Рекомендуется к внедрению как замена `std::vector`/`std::string` (использует статическую память).
3. **`constexpr` и `const`:** Максимальное использование для размещения данных во Flash (`.rodata`).

---

## 2. Интеграция с STM32CubeMX и HAL
Поскольку CubeMX генерирует код на **C**, а вы пишете на **C++**, необходимо жесткое разграничение.

### 2.1. Правила работы с генератором
* **Запрещается** модифицировать сгенерированные файлы (`main.c`, `stm32f4xx_it.c`) вне блоков `USER CODE BEGIN` / `USER CODE END`.
* Файлы CubeMX компилируются **только как C** (через `gcc`), ваши файлы — **как C++** (через `g++`). Не пытайтесь компилировать HAL как C++.

### 2.2. Обертки над HAL (HAL Wrappers)
Не наследуйтесь от HAL-структур. Создавайте C++ классы, которые *владеют* или *ссылаются* на HAL-дескриптор.
```cpp
#pragma once
#include <cstdint>
#include <span>

// 1. Изолируем C-заголовки HAL
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
}
#endif

// 2. C++ обертка
class UartDriver {
public:
    // Передаем указатель на HAL-дескриптор, созданный в CubeMX
    explicit UartDriver(UART_HandleTypeDef* huart) : huart_(huart) {}

    // Используем std::span для безопасной работы с буферами
    enum class Status { Ok, Timeout, Error };
    Status transmit(std::span<const uint8_t> data, uint32_t timeout_ms);

private:
    UART_HandleTypeDef* huart_; 
};
```

---

## 3. RTOS и Прерывания (ISR)

### 3.1. Задачи (Tasks)
Точка входа в задачу RTOS должна быть `static` функцией. Указатель на C++ объект передается через `pvParameters`.
```cpp
class SensorTask {
public:
    void start() {
        // Передаем 'this' в RTOS
        xTaskCreate(taskEntryPoint, "SensorTask", 256, this, PRIORITY_NORMAL, &handle_);
    }
private:
    // Точка входа ОБЯЗАНО static
    static void taskEntryPoint(void* pvParameters) {
        auto* self = static_cast<SensorTask*>(pvParameters);
        self->run();
    }

    void run() {
        while (true) {
            // Логика задачи
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    TaskHandle_t handle_{};
};
```

### 3.2. Обработчики прерываний (ISR)
В функциях `HAL_..._IRQHandler` и `..._IRQHandler`:
* **ЗАПРЕЩЕНО:** Блокирующие вызовы RTOS, `printf`, долгие вычисления.
* **РАЗРЕШЕНО:** Только API с суффиксом `FromISR` (например, `xSemaphoreGiveFromISR`).
* Используйте `volatile` **только** для переменных, которые читаются в ISR и пишутся в основном коде (или наоборот). `volatile` не заменяет мьютексы!

### 3.3. RAII для RTOS
Запрещено использовать сырые мьютексы/семафоры. Всегда используйте RAII-обертки для предотвращения deadlocks.
```cpp
class RtosMutexGuard {
public:
    explicit RtosMutexGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    ~RtosMutexGuard() {
        xSemaphoreGive(mutex_);
    }
    // Запрещаем копирование
    RtosMutexGuard(const RtosMutexGuard&) = delete;
    RtosMutexGuard& operator=(const RtosMutexGuard&) = delete;
private:
    SemaphoreHandle_t mutex_;
};
```

---

## 4. Аппаратная специфика и Память

### 4.1. Выравнивание (Alignment)
Буферы для DMA, USB и сетевых интерфейсов **обязаны** быть выровнены.
```cpp
// Выравнивание по 4 байтам (32 бита) для DMA
alignas(4) uint8_t dmaRxBuffer[256]; 
```

### 4.2. Битовые операции и Регистры
* Избегайте магических чисел. Используйте `constexpr` или макросы CMSIS.
```cpp
constexpr uint32_t PIN_5_MASK = (1U << 5);
```
* При работе с регистрами памяти используйте `volatile` и точные типы (`uint32_t`).
```cpp
volatile uint32_t* const GPIOA_ODR = reinterpret_cast<volatile uint32_t*>(0x40020014);
```

---

## 5. Именование и Форматирование
*(Оставлено из вашего базового стандарта, так как оно отлично читается)*

* **Файлы:** `CamelCase` (например, `UartDriver.cpp`).
* **Классы/Структуры/Enum:** `CamelCase` (`SensorData`, `UartError`).
* **Переменные/Функции:** `camelCase` (`readTemperature`, `isActive`).
* **Поля класса:** `mCamelCase` (`mTemperature`, `mState`).
* **Константы/Макросы:** `SCREAMING_SNAKE_CASE` (`MAX_BUFFER_SIZE`).
* **Отступы:** 2 пробела. Табуляция запрещена.
* **Скобки:** K&R style (на той же строке).
* **Длина строки:** Максимум 100 символов.

---

## 6. Обработка ошибок (Без исключений)
Вместо `throw` используем `enum class` и `std::optional` / структуры результатов.

```cpp
enum class SensorError {
    Ok = 0,
    I2C_Timeout,
    InvalidData,
    HardwareFail
};

// Вариант 1: Если результат может отсутствовать
std::optional<float> readTemperature();

// Вариант 2: Если нужно вернуть и данные, и код ошибки (предпочтительно для C++17)
struct SensorResult {
    SensorError error;
    float value;
};

SensorResult readSensorData();
```

---

## 7. Заголовочные файлы и C/C++ Interop

### 7.1. Автономность и `#pragma once`
Всегда используйте `#pragma once`. Заголовочный файл должен компилироваться сам по себе.

### 7.2. Порядок включения (Include Order)
```cpp
#pragma once

// 1. Собственный заголовок (если это .cpp)
#include "MyClass.h"

// 2. C-заголовки HAL и CMSIS (обернутые в extern "C")
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#ifdef __cplusplus
}
#endif

// 3. Заголовки RTOS
#include "FreeRTOS.h"
#include "task.h"

// 4. Системные C++ заголовки (только разрешенные!)
#include <cstdint>
#include <array>
#include <span>

// 5. Заголовки проекта
#include "UartDriver.h"
```

---

## 8. Современный C++ (Разрешено и Запрещено)

### 8.1. Разрешено (Must Use)
* `constexpr` для всех вычислений на этапе компиляции.
* `std::array` вместо сырых C-массивов.
* `std::span` (C++20) для передачи массивов в функции вместо `pointer + size`.
* `std::string_view` для передачи строк без копирования.
* `auto` там, где тип очевиден (итераторы, `std::make_unique` - *если используется кастомный аллокатор*).
* `static_assert` для проверки размеров структур (критично для упаковки `__attribute__((packed))`).

### 8.2. Запрещено
* `std::function` (по умолчанию выделяет память в куче). Используйте шаблоны или C++20 lambdas без захвата.
* `std::shared_ptr` / `std::unique_ptr` (используйте RAII-обертки над ресурсами МК).
* `dynamic_cast`.

---

## 9. Сборка (CMake) и IDE

### 9.1. Структура проекта
```text
/project_root
  /Core          # Сгенерировано CubeMX (компилируется как C)
  /Drivers       # HAL, CMSIS (компилируется как C)
  /App           # Ваш C++ код (компилируется как C++)
  /CMake         # Toolchain файлы
  CMakeLists.txt
```

### 9.2. Обязательные флаги компиляции
В `CMakeLists.txt` для ваших C++ файлов **обязательно** укажите:
```cmake
target_compile_options(my_app PRIVATE
    -fno-exceptions       # Отключаем исключения
    -fno-rtti             # Отключаем RTTI
    -fno-threadsafe-statics # Отключаем потокобезопасную инициализацию static (экономия Flash/RAM, если инициализация происходит до старта RTOS)
    -Wall -Wextra -Wshadow -Wpedantic
)

# Линковщик
target_link_options(my_app PRIVATE
    -T${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld # Linker script
    -Wl,--gc-sections     # Удаляем неиспользуемый код
    -Wl,--print-memory-usage # Выводит размер Flash/RAM после сборки
)
```

### 9.3. Интеграция с IDE (VS Code / CLion)
Для работы автодополнения, подсветки ошибок и навигации по HAL, CMake **обязан** генерировать `compile_commands.json`:
```cmake
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```
*(В VS Code плагин Clangd подхватит этот файл автоматически).*

---

## 10. Документирование
* Используйте **Doxygen** для публичных API.
* Комментируйте **почему** (why), а не **что** (what).
* Обязательно документируйте требования к прерываниям и RTOS (например: `@note Must be called from ISR context`, `@warning Not thread-safe`).

```cpp
/**
 * @brief Инициализирует DMA для UART
 * @param buffer Указатель на выровненный буфер
 * @warning Буфер должен быть выровнен по 4 байтам (alignas(4))
 * @note Функция не потокобезопасна, требует блокировки мьютекса
 */
void initDma(std::span<uint8_t> buffer);
```