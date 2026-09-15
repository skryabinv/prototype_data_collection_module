# AGENTS.md — WeAct_H7_Sensors

Инструкции для AI-агентов и разработчиков. Полный стандарт: [`docs/CodingStandart.md`](docs/CodingStandart.md).

## Проект

- **Плата:** WeAct STM32H743 (Cortex-M7, hard FP).
- **Стек:** STM32CubeMX (HAL), FreeRTOS (CMSIS-RTOS v2), CMake + Ninja, ARM GCC 13, **C++20** (embedded-ограничения).
- **Назначение:** драйверы датчиков по I2C (BMP390, MMC5983MA), задача опроса `SensorTask`.

## Структура репозитория

```text
Core/           # CubeMX + пользовательский код (C и C++ в Core/Src, Core/Inc)
Drivers/        # HAL, CMSIS — только C, не править вручную
Middlewares/    # FreeRTOS
vendor/         # Сторонние C-API (bmp3-sensor-api)
cmake/          # toolchain
build/Debug/    # артефакты сборки, compile_commands.json
CMakeLists.txt
```

- **CubeMX-файлы** (`main.c`, `*_it.c`, `i2c.c`, …): правки **только** в `USER CODE BEGIN/END`.
- **Прикладной C++:** `Core/Src/*.cpp`, `Core/Inc/*.hpp` — основная зона изменений агента.
- HAL/Drivers **не** компилировать как C++.

## Сборка

```bash
cmake --preset Debug   # если настроены presets
cmake --build build/Debug
```

- `CMAKE_EXPORT_COMPILE_COMMANDS` включён — `build/Debug/compile_commands.json`.
- **clangd:** расширение `llvm-vs-code-extensions.vscode-clangd`, аргумент `--query-driver=**/arm-none-eabi-*` (обязательно для `<cstdint>` и libstdc++). После `cmake` — Reload Window.
- Флаги C++ уже заданы: `-fno-exceptions -fno-rtti -fno-threadsafe-statics`, `-Wall -Wextra -Wshadow -Wpedantic`.

## Жёсткие запреты (embedded)

Не добавлять и не предлагать:

| Запрещено | Причина |
|-----------|---------|
| `try` / `catch` / `throw` | Исключения отключены в сборке |
| `new` / `delete` / `malloc` / `free` | Куча, фрагментация, HardFault |
| `std::vector`, `std::string`, `std::map`, `std::list`, `shared_ptr`/`unique_ptr` (default alloc) | Динамическая память |
| `dynamic_cast`, `typeid` | RTTI отключён |
| `std::function` (default) | Аллокации в куче |
| Блокирующие RTOS-вызовы / `printf` в **ISR** | Детерминизм прерываний |

## Разрешено и предпочтительно

- `std::optional`, `std::array`, `std::span` (C++20), `std::string_view`, `enum class`, `constexpr`, `static_assert`.
- Ошибки: `enum class` + `std::optional` / явные коды (`Bmp390::Error`, `Mmc5983ma::Status`), **без** исключений.
- HAL: C++-классы со **ссылкой** на `I2C_HandleTypeDef&` / указателем на Cube-дескриптор, **без** наследования от HAL-структур.
- C-заголовки HAL/CMSIS — через `extern "C" { #include "main.h" }` в `.hpp`.
- Копирование/перемещение драйверов с `intf_ptr` / колбэками — **delete**, если не перенастроен HAL API.

## Именование и формат

См. [`docs/CodingStandart.md`](docs/CodingStandart.md) §5. Кратко:

| Сущность | Стиль | Пример |
|----------|--------|--------|
| Файлы | CamelCase | `Bmp390.cpp`, `SensorTask.hpp` |
| Классы / enum | CamelCase | `Mmc5983ma`, `Status::Ok` |
| Функции, локальные | camelCase | `readData`, `hasUnreadData` |
| Поля класса | `m` + CamelCase | `mI2c`, `mAddress` |
| Константы | SCREAMING_SNAKE или `k` + CamelCase | `kI2cTimeoutMs` |
| Отступы | 2 пробела, без табов | |
| Строка | ≤ 100 символов | |
| Скобки | K&R | `if (x) {` |

Форматирование: **clang-format** (`.clang-format`), format on save через clangd.

## RTOS и I2C

- Одна шина `hi2c1`, несколько датчиков — последовательный доступ из задачи; при втором клиенте шины — мьютекс.
- Цикл задачи: **не** фиксированный «опрос раз в N мс» вместо DRDY; при отсутствии данных — `osDelay(1)` (тик 1 ms).
- **MMC5983MA** (точный режим SET/RESET, `Config::enableSetResetMeasurement = true`, `autoSetReset = false`):
  - Протокол в драйвере: SET → измерение mag → RESET → измерение mag → **H = (R1−R2)/2**, **Offset = (R1+R2)/2**, затем температура.
  - API **неблокирующий**: `startMeasurement()` (только из `Idle`) → в цикле задачи `hasUnreadData()` (внутри `poll()` по состояниям) → `readData()` → снова `startMeasurement()`.
  - `SensorData`: `field` (H), `offset`, `temperature`. Кэш offset: `lastOffset()`.
  - Не вызывать `startMeasurement()` из `hasUnreadData()` / `readData()` — только из вызывающего кода (см. `SensorTask.cpp`).
- **BMP390** (normal mode): данные по ODR, `hasUnreadData()` + `readData()`.

## CubeMX / отладочный вывод

- `printf` в `SensorTask` — временно для bring-up; в продакшн — лёгкий лог через UART (см. стандарт §1.1). Не размножать `printf` без необходимости.
- Не включать UART/DMA в Cube без запроса пользователя.

## CMake

- Новые `.cpp` добавлять в `USER_SOURCES` в корневом `CMakeLists.txt`.
- Не ломать линковку CubeMX (`stm32cubemx` target).

## Документация и комментарии

- Публичный API — краткий Doxygen (`@brief`, `@note`, `@warning` для ISR/RTOS).
- Комментарии — **почему**, не пересказ кода.
- `TODO` с контекстом (железо, pull-up I2C, адреса) — ок.

## Чеклист перед завершением задачи агентом

1. Сборка `cmake --build build/Debug` без ошибок (если среда доступна).
2. Соответствие embedded-запретам (нет `new`, exceptions, тяжёлого STL).
3. CubeMX-файлы не тронуты вне USER CODE.
4. Минимальный diff — без рефакторинга «заодно».
5. Именование и `.clang-format` соблюдены.

## Ссылки

- Стандарт: [`docs/CodingStandart.md`](docs/CodingStandart.md)
- Датчики: `Core/Inc/Bmp390.hpp`, `Core/Inc/Mmc5983ma.hpp`, `Core/Src/SensorTask.cpp`
