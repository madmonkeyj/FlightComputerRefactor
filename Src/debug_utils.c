/**
  ******************************************************************************
  * @file    debug_utils.c
  * @brief   Debug utilities - SAFE NON-BLOCKING version
  ******************************************************************************
  */

#include "debug_utils.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

/* Static buffer - persists after function returns (DMA safe) */
static char debug_buffer[256];
static volatile bool transmit_in_progress = false;
static uint32_t last_transmit_time = 0;

/**
 * @brief Safe non-blocking debug print
 * @note Uses static buffer to prevent DMA corruption
 */
void DebugPrint(const char *format, ...) {
    uint32_t now = HAL_GetTick();

    // Minimum 5ms between transmits to let USB complete
    if ((now - last_transmit_time) < 5) {
        return;
    }

    // Format message into static buffer
    va_list args;
    va_start(args, format);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    if (len <= 0) return;
    if (len >= (int)sizeof(debug_buffer)) {
        len = sizeof(debug_buffer) - 1;
    }

    // Try to transmit
    if (CDC_Transmit_FS((uint8_t*)debug_buffer, len) == USBD_OK) {
        last_transmit_time = now;
    }
}

/**
 * @brief Simple debug print (non-blocking)
 */
void DebugPrintSimple(const char *str) {
    if (!str) return;

    uint32_t now = HAL_GetTick();
    if ((now - last_transmit_time) < 5) {
        return;
    }

    size_t len = strlen(str);
    if (len >= sizeof(debug_buffer)) {
        len = sizeof(debug_buffer) - 1;
    }

    memcpy(debug_buffer, str, len);
    debug_buffer[len] = '\0';

    if (CDC_Transmit_FS((uint8_t*)debug_buffer, len) == USBD_OK) {
        last_transmit_time = now;
    }
}

/**
 * @brief Get dropped message count (for diagnostics)
 */
uint32_t DebugGetDroppedCount(void) {
    return 0;  // Not tracking in this version
}
