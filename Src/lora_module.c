/* ============================================================================ */
/* CLEAN LORA MODULE - WITH INTEGRATED HAL CALLBACK */
/* ============================================================================ */

#include "lora_module.h"
#include "debug_utils.h"
#include <string.h>
#include <stdio.h>
#include "inter_mcu_comm.h"


/* Forward declaration to avoid circular include */
extern void TelemetryManager_TransmissionComplete(bool success);

/* Private variables - encapsulated within module */
static volatile bool lora_tx_busy = false;
static volatile uint32_t lora_tx_start_time = 0;
static volatile uint32_t lora_tx_count = 0;
static volatile bool packet_buffer_busy = false;
static bool lora_initialized = false;
static uint32_t successful_transmissions = 0;
static uint32_t failed_transmissions = 0;
static uint32_t timeout_transmissions = 0;
static uint32_t last_transmission_time = 0;

/* Transmission timeout configuration */
#define LORA_TRANSMISSION_TIMEOUT_MS 150

/* Fixed configuration for rocket telemetry */
#define ROCKET_CONFIG_SPED    0x1B  // 9600 baud UART, 4.8kbps air rate
#define ROCKET_CONFIG_CHANNEL 0x35  // Channel 53 (915MHz for Australia)
#define ROCKET_CONFIG_OPTION  0x40  // 20dBm power, FEC on

/* Private function prototypes */
static bool LoRa_Configure(void);
static volatile bool lora_transmission_completed = false;
static volatile bool lora_transmission_success = false;

/**
 * @brief HAL UART TX completion callback - handles LoRa transmission completion
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        /*LoRa_TransmissionComplete(true);*/
        InterMCU_UART_RxCpltCallback(huart);

    }
}

/**
 * @brief HAL UART error callback - handles transmission errors
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        /*LoRa_TransmissionComplete(false);*/
        InterMCU_UART_ErrorCallback(huart);
    }
}

/**
 * @brief Set LoRa operating mode
 */
bool LoRa_SetMode(LoRa_Mode_t mode) {
    GPIO_PinState m0_state, m1_state;

    switch(mode) {
        case LORA_MODE_NORMAL:
            m0_state = GPIO_PIN_RESET; // M0 = 0
            m1_state = GPIO_PIN_RESET; // M1 = 0
            break;
        case LORA_MODE_WAKEUP:
            m0_state = GPIO_PIN_SET;   // M0 = 1
            m1_state = GPIO_PIN_RESET; // M1 = 0
            break;
        case LORA_MODE_POWERSAVE:
            m0_state = GPIO_PIN_RESET; // M0 = 0
            m1_state = GPIO_PIN_SET;   // M1 = 1
            break;
        case LORA_MODE_SLEEP:
            m0_state = GPIO_PIN_SET;   // M0 = 1
            m1_state = GPIO_PIN_SET;   // M1 = 1
            break;
        default:
            return false;
    }

    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, m0_state);
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, m1_state);

    // Wait for mode switch
    uint32_t timeout = HAL_GetTick() + 200;
    while(HAL_GetTick() < timeout) {
        if(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == GPIO_PIN_SET) {
            HAL_Delay(2);
            return true;
        }
        HAL_Delay(1);
    }

    return false;
}

/**
 * @brief Check if LoRa hardware is ready
 */
static bool LoRa_IsHardwareReady(void) {
    return HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == GPIO_PIN_SET;
}

/**
 * @brief Configure LoRa for rocket telemetry (4.8kbps, 915MHz)
 */
static bool LoRa_Configure(void) {
    // Enter sleep mode for configuration
    if (!LoRa_SetMode(LORA_MODE_SLEEP)) {
        DebugPrint("LoRa: ERROR - Failed to enter sleep mode\r\n");
        return false;
    }
    HAL_Delay(200);

    // Send configuration
    uint8_t config_cmd[] = {
        0xC0,                      // Save permanently
        0x00, 0x00,               // Address 0x0000
        ROCKET_CONFIG_SPED,       // 4.8kbps
        ROCKET_CONFIG_CHANNEL,    // 915MHz
        ROCKET_CONFIG_OPTION      // 20dBm, FEC on
    };

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, config_cmd, 6, 1000);
    if (status != HAL_OK) {
        DebugPrint("LoRa: ERROR - Failed to send configuration\r\n");
        return false;
    }

    HAL_Delay(500); // Wait for flash save

    // Return to normal mode
    if (!LoRa_SetMode(LORA_MODE_NORMAL)) {
        DebugPrint("LoRa: ERROR - Failed to return to normal mode\r\n");
        return false;
    }
    HAL_Delay(200);

    return true;
}

/**
 * @brief Initialize LoRa module
 */
bool LoRa_Init(void) {
    DebugPrint("LoRa: Initializing module...\r\n");

    /* Reset private variables */
    lora_tx_busy = false;
    lora_tx_start_time = 0;
    lora_tx_count = 0;
    packet_buffer_busy = false;
    successful_transmissions = 0;
    failed_transmissions = 0;
    timeout_transmissions = 0;
    last_transmission_time = HAL_GetTick();

    // Set initial normal mode
    if (!LoRa_SetMode(LORA_MODE_NORMAL)) {
        DebugPrint("LoRa: ERROR - Failed to set initial mode\r\n");
        lora_initialized = false;
        return false;
    }

    // Wait for module ready
    uint32_t timeout = HAL_GetTick() + 1000;
    while (!LoRa_IsHardwareReady() && HAL_GetTick() < timeout) {
        HAL_Delay(10);
    }

    if (!LoRa_IsHardwareReady()) {
        DebugPrint("LoRa: ERROR - Module not ready\r\n");
        lora_initialized = false;
        return false;
    }

    // Apply rocket configuration
    if (!LoRa_Configure()) {
        DebugPrint("LoRa: ERROR - Configuration failed\r\n");
        lora_initialized = false;
        return false;
    }

    lora_initialized = true;
    DebugPrint("LoRa: Module initialized (4.8k, 915MHz)\r\n");
    return true;
}

/**
 * @brief Send data via LoRa
 */
bool LoRa_SendData(uint8_t *data, uint16_t length) {
    if (!lora_initialized || !LoRa_IsHardwareReady() || data == NULL || length == 0 || lora_tx_busy) {
        return false;
    }

    lora_tx_busy = true;
    lora_tx_start_time = HAL_GetTick();

    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart2, data, length);

    if (status != HAL_OK) {
        lora_tx_busy = false;
        lora_tx_start_time = 0;
        failed_transmissions++;
        return false;
    }

    return true;
}

/**
 * @brief Check if LoRa module is ready for new transmission
 */
bool LoRa_IsReady(void) {
    return lora_initialized && LoRa_IsHardwareReady() && !lora_tx_busy;
}

/**
 * @brief Check if LoRa is currently transmitting
 */
bool LoRa_IsTransmitting(void) {
    return lora_tx_busy;
}

/**
 * @brief Get LoRa module status
 */
LoRa_Status_t LoRa_GetStatus(void) {
    if (!lora_initialized) {
        return LORA_STATUS_NOT_INITIALIZED;
    }

    if (lora_tx_busy) {
        /* Check for timeout */
        if (LoRa_GetTransmissionAge() > LORA_TRANSMISSION_TIMEOUT_MS) {
            return LORA_STATUS_TIMEOUT;
        }
        return LORA_STATUS_TRANSMITTING;
    }

    if (!LoRa_IsHardwareReady()) {
        return LORA_STATUS_ERROR;
    }

    return LORA_STATUS_READY;
}

/**
 * @brief Get total transmission count
 */
uint32_t LoRa_GetTransmissionCount(void) {
    return lora_tx_count;
}

/**
 * @brief Get age of current transmission
 */
uint32_t LoRa_GetTransmissionAge(void) {
    if (!lora_tx_busy || lora_tx_start_time == 0) {
        return 0;
    }

    return HAL_GetTick() - lora_tx_start_time;
}

/**
 * @brief Get LoRa statistics
 */
bool LoRa_GetStatistics(LoRa_Statistics_t* stats) {
    if (!stats) {
        return false;
    }

    stats->total_transmissions = successful_transmissions + failed_transmissions + timeout_transmissions;
    stats->successful_transmissions = successful_transmissions;
    stats->failed_transmissions = failed_transmissions;
    stats->timeout_transmissions = timeout_transmissions;
    stats->last_transmission_time = last_transmission_time;
    stats->current_transmission_start = lora_tx_start_time;
    stats->transmission_active = lora_tx_busy;

    return true;
}

/**
 * @brief Reset LoRa transmission state
 */
bool LoRa_Reset(void) {
    if (lora_tx_busy) {
        timeout_transmissions++;
    }

    lora_tx_busy = false;
    lora_tx_start_time = 0;
    packet_buffer_busy = false;

    /* Try to reset hardware state */
    HAL_UART_AbortTransmit_IT(&huart2);

    return true;
}

/**
 * @brief Handle transmission completion - INTERRUPT-SAFE VERSION
 */
void LoRa_TransmissionComplete(bool success) {
    if (!lora_tx_busy) {
        return;
    }

    // MINIMAL work in interrupt context - just set flags
    lora_tx_busy = false;
    lora_tx_start_time = 0;
    packet_buffer_busy = false;

    // Set completion flags for main loop to process
    lora_transmission_success = success;
    lora_transmission_completed = true;
}

/**
 * @brief Update LoRa module - handles completion flags
 */
void LoRa_Update(void) {
    if (!lora_initialized) {
        return;
    }

    /* Handle completion flags set by interrupt */
    if (lora_transmission_completed) {
        lora_transmission_completed = false;
        last_transmission_time = HAL_GetTick();

        if (lora_transmission_success) {
            lora_tx_count++;
            successful_transmissions++;
        } else {
            failed_transmissions++;
        }

        // Call TelemetryManager from main loop context
        TelemetryManager_TransmissionComplete(lora_transmission_success);
    }

    /* Check for transmission timeout */
    if (lora_tx_busy && lora_tx_start_time != 0) {
        if (LoRa_GetTransmissionAge() > LORA_TRANSMISSION_TIMEOUT_MS) {
            LoRa_Reset();
        }
    }
}

/**
 * @brief Check if packet buffer is busy
 */
bool LoRa_IsPacketBufferBusy(void) {
    return packet_buffer_busy;
}

/**
 * @brief Set packet buffer busy state
 */
void LoRa_SetPacketBufferBusy(bool busy) {
    packet_buffer_busy = busy;
}
