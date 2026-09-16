/**
 * @brief Retarget newlib stdout to USB CDC (Virtual COM).
 */

#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "stm32h7xx_hal.h"

#include <errno.h>
#include <unistd.h>

int _write(int file, char* ptr, int len)
{
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        errno = EBADF;
        return -1;
    }
    if (ptr == NULL || len <= 0) {
        return 0;
    }

    int written = 0;
    while (written < len) {
        const int remaining = len - written;
        const uint16_t chunk =
            (remaining > APP_TX_DATA_SIZE) ? APP_TX_DATA_SIZE : (uint16_t)remaining;

        uint32_t retries = 10000U;
        uint8_t result = USBD_BUSY;
        while (result == USBD_BUSY && retries > 0U) {
            result = CDC_Transmit_FS((uint8_t*)&ptr[written], chunk);
            if (result == USBD_BUSY) {
                HAL_Delay(1);
                --retries;
            }
        }
        if (result != USBD_OK) {
            errno = EIO;
            return (written > 0) ? written : -1;
        }
        written += chunk;
    }

    return written;
}
