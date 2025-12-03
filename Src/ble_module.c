/**
  ******************************************************************************
  * @file    ble_module.c
  * @brief   BLE module with DMA circular buffer (Cleaned Version)
  * @note    Converted from interrupt-based to DMA for consistency and efficiency
  ******************************************************************************
  */

#include "ble_module.h"
#include "debug_utils.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "data_logger.h"

/* External DMA handle for USART1 RX (defined in usart.c) */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* Private variables - fully encapsulated within module */
#define BLE_RX_BUFFER_SIZE 512  /* Circular DMA buffer size */

/* DMA circular buffer variables (same pattern as GPS) */
static uint8_t ble_rx_dma_buffer[BLE_RX_BUFFER_SIZE];
static uint16_t buffer_read_pos = 0;
static volatile uint16_t last_dma_write_pos = 0;  /* Updated by DMA idle callback */
static bool dma_active = false;

/* Command processing buffer */
static uint8_t rx_buffer[128];
static uint16_t rx_index = 0;

/* Status and statistics */
static uint32_t last_rx_time = 0;
static uint32_t last_tx_time = 0;
static uint32_t total_bytes_received = 0;
static uint32_t total_bytes_sent = 0;
static char uart_response_buffer[256];
static uint16_t uart_response_index = 0;
static uint32_t connection_count = 0;
static BLE_Status_t ble_status = BLE_STATUS_DISCONNECTED;
static bool ble_initialized = false;

/* Command processing variables */
static BLE_CommandCallback_t command_callback = NULL;
static bool data_transmission_enabled = true; /* Controls sensor data transmission */

/* Command timeout variables */
static uint32_t last_char_time = 0;
static uint32_t char_timeout_ms = 2000;  /* 2 second timeout for incomplete commands */

/* Private function prototypes */
static void ClearResponseBuffer(void);
static bool SendBleCommand(const char* cmd, const char* expectedResponse, uint32_t timeout);
static void ResetBleModule(void);
static void BLE_ProcessReceivedByte(uint8_t byte);
static bool BLE_StartDMA(void);
static void BLE_ProcessCommand(const char* command);
static void BLE_ProcessIncompleteBuffer(void);
static bool IsProtocolOverhead(const uint8_t* data, uint16_t length);
static bool IsInCommandMode(void);

/**
 * @brief Check if received data is likely BLE protocol overhead
 */
static bool IsProtocolOverhead(const uint8_t* data, uint16_t length) {
    if (!data || length == 0) return true;

    // Very short data is likely protocol overhead
    if (length < 3) return true;

    // Check for binary/non-printable data (common in BLE protocol)
    uint16_t non_printable = 0;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        // Count non-printable characters (excluding common line endings)
        if (byte < 32 && byte != '\r' && byte != '\n' && byte != '\t') {
            non_printable++;
        }
    }

    // If more than 25% is non-printable, likely protocol data
    return (non_printable * 100 / length) > 25;
}

/**
 * @brief Process incomplete buffer due to timeout - INTERNAL FUNCTION with filtering
 */
static void BLE_ProcessIncompleteBuffer(void) {
    if (rx_index > 0) {
        // First check if this looks like protocol overhead
        if (IsProtocolOverhead(rx_buffer, rx_index)) {
            rx_index = 0;
            return;
        }

        rx_buffer[rx_index] = '\0';

        /* Remove trailing whitespace */
        while (rx_index > 0 && (rx_buffer[rx_index-1] == '\r' ||
               rx_buffer[rx_index-1] == '\n' || rx_buffer[rx_index-1] == ' ')) {
            rx_buffer[--rx_index] = '\0';
        }

        if (rx_index >= 3) { /* Only process reasonable length commands */
            BLE_ProcessCommand((char*)rx_buffer);
        }

        rx_index = 0; /* Reset for next message */
    }
}

/**
 * @brief No-configuration BLE setup - just reset and use defaults
 */
bool BLE_Configure_NoConfig(void) {
    // Just hardware reset and let it boot in default Data mode
    ResetBleModule();
    return true;
}

/**
 * @brief Check if already in command mode
 */
static bool IsInCommandMode(void) {
    ClearResponseBuffer();

    // Send a simple command that returns the prompt
    // V command shows version, or if in data mode it just passes through
    if (SendBleCommand("V\r", "Ver", 1000)) {
        return true;
    }

    // Check if we got CMD> prompt in the response
    if (strstr(uart_response_buffer, "CMD>") != NULL) {
        return true;
    }

    return false;
}

/**
 * @brief Try multiple times to enter command mode
 */
static bool EnterCommandMode(void) {
    // First check if already in command mode
    if (IsInCommandMode()) {
        return true;
    }

    for (int attempt = 1; attempt <= 5; attempt++) {
        // Clear response buffer
        ClearResponseBuffer();

        // Try to enter command mode
        if (SendBleCommand("$$$", "CMD>", 3000)) {
            return true;
        }

        // If failed, try alternative approaches
        if (attempt == 2) {
            if (SendBleCommand("$", "CMD>", 2000)) {
                return true;
            }
        }

        if (attempt == 3) {
            if (SendBleCommand("V\r", "RN", 2000)) {
                return true;
            }
        }

        // Wait between attempts
        HAL_Delay(1000);
    }

    DebugPrint("BLE: ERROR - Failed to enter command mode after 5 attempts\r\n");
    return false;
}

/**
 * @brief Initialize BLE module with DMA
 */
bool BLE_Init(void) {
    /* Reset all private variables */
    buffer_read_pos = 0;
    last_dma_write_pos = 0;
    dma_active = false;
    rx_index = 0;
    last_rx_time = 0;
    last_tx_time = 0;
    total_bytes_received = 0;
    total_bytes_sent = 0;
    uart_response_index = 0;
    connection_count = 0;
    ble_status = BLE_STATUS_CONNECTING;
    command_callback = NULL;
    data_transmission_enabled = true;

    /* Initialize timeout variables */
    last_char_time = 0;
    char_timeout_ms = 2000;  // 2 second timeout

    /* Clear buffers */
    memset(ble_rx_dma_buffer, 0, sizeof(ble_rx_dma_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(uart_response_buffer, 0, sizeof(uart_response_buffer));

    /* Configure the BLE module */
    if (!BLE_Configure()) {
        ble_status = BLE_STATUS_ERROR;
        ble_initialized = false;
        return false;
    }

    /* Start DMA reception */
    if (!BLE_StartDMA()) {
        ble_status = BLE_STATUS_ERROR;
        ble_initialized = false;
        return false;
    }

    ble_status = BLE_STATUS_DISCONNECTED; /* Ready but not connected */
    ble_initialized = true;
    last_rx_time = HAL_GetTick();

    return true;
}

/**
 * @brief Register command callback function
 */
void BLE_RegisterCommandCallback(BLE_CommandCallback_t callback) {
    command_callback = callback;
}

/**
 * @brief Enable/disable data transmission over BLE
 */
void BLE_SetDataTransmissionEnabled(bool enabled) {
    data_transmission_enabled = enabled;
}

/**
 * @brief Check if data transmission is enabled
 */
bool BLE_IsDataTransmissionEnabled(void) {
    return data_transmission_enabled;
}

/**
 * @brief Send response message
 */
bool BLE_SendResponse(const char* response) {
    if (!ble_initialized || !response) {
        return false;
    }

    char response_msg[100];
    snprintf(response_msg, sizeof(response_msg), "%s\r\n", response);

    return BLE_SendString(response_msg);
}

/**
 * @brief Update BLE module - DMA version (same pattern as GPS)
 */
void BLE_Update(void) {
    /* DIAGNOSTIC: Track BLE_Update() call frequency */
    static uint32_t last_update_heartbeat = 0;
    static uint32_t update_call_count = 0;
    update_call_count++;

    if ((HAL_GetTick() - last_update_heartbeat) > 10000) {
        char debug_msg[200];
        snprintf(debug_msg, sizeof(debug_msg),
                "BLE_Update: Called %lu times in last 10s, init=%d, dma_active=%d\r\n",
                update_call_count, ble_initialized, dma_active);
        DebugPrint(debug_msg);
        update_call_count = 0;
        last_update_heartbeat = HAL_GetTick();
    }

    if (!ble_initialized || !dma_active) {
        return;
    }

    /* Get stable snapshot of DMA write position (updated by idle line callback) */
    uint16_t current_write_pos = BLE_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

    /* DIAGNOSTIC: Track buffer state periodically */
    static uint32_t last_buffer_report = 0;
    if ((HAL_GetTick() - last_buffer_report) > 10000) {
        char debug_msg[200];
        snprintf(debug_msg, sizeof(debug_msg),
                "BLE_Update: Buffer state - read_pos=%u, write_pos=%u, pending=%d\r\n",
                buffer_read_pos, current_write_pos,
                (current_write_pos >= buffer_read_pos) ?
                    (current_write_pos - buffer_read_pos) :
                    (BLE_RX_BUFFER_SIZE - buffer_read_pos + current_write_pos));
        DebugPrint(debug_msg);
        last_buffer_report = HAL_GetTick();
    }

    /* Process all bytes from DMA circular buffer */
    while (buffer_read_pos != current_write_pos) {
        uint8_t byte = ble_rx_dma_buffer[buffer_read_pos];
        BLE_ProcessReceivedByte(byte);
        buffer_read_pos = (buffer_read_pos + 1) % BLE_RX_BUFFER_SIZE;
    }

    /* Handle command timeout */
    if (rx_index > 0) {
        uint32_t time_since_last_char = HAL_GetTick() - last_char_time;

        if (time_since_last_char > char_timeout_ms) {
            BLE_ProcessIncompleteBuffer();
        }
    }

    /* Update connection status */
    uint32_t time_since_activity = HAL_GetTick() - last_rx_time;

    if (time_since_activity < 5000) { /* Active within 5 seconds */
        if (ble_status == BLE_STATUS_DISCONNECTED) {
            ble_status = BLE_STATUS_CONNECTED;
            connection_count++;
        }
    } else if (time_since_activity > 15000) { /* No activity for 15 seconds */
        if (ble_status == BLE_STATUS_CONNECTED) {
            ble_status = BLE_STATUS_DISCONNECTED;
        }
    }
}

/**
 * @brief Send data via BLE
 */
bool BLE_SendData(const char* data, uint16_t length) {
    if (!ble_initialized || !data || length == 0) {
        return false;
    }

    // Check if this is sensor data and if data transmission is disabled
    if (!data_transmission_enabled && strncmp(data, "NAV_DATA,", 9) == 0) {
        return true; // Silently ignore sensor data when disabled
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, length, 1000);

    if (status == HAL_OK) {
        total_bytes_sent += length;
        last_tx_time = HAL_GetTick();
        return true;
    }

    return false;
}

/**
 * @brief Send string via BLE
 */
bool BLE_SendString(const char* str) {
    if (!str) {
        return false;
    }

    return BLE_SendData(str, strlen(str));
}

/**
 * @brief Process a complete command - UPDATED with data logger support
 */
static void BLE_ProcessCommand(const char* command) {
    if (!command || strlen(command) == 0) {
        return;
    }

    /* Convert to lowercase */
    char cmd_lower[64];
    strncpy(cmd_lower, command, sizeof(cmd_lower) - 1);
    cmd_lower[sizeof(cmd_lower) - 1] = '\0';

    for (int i = 0; cmd_lower[i]; i++) {
        if (cmd_lower[i] >= 'A' && cmd_lower[i] <= 'Z') {
            cmd_lower[i] = cmd_lower[i] + 32;
        }
    }

    /* Built-in command processing */
    if (strcmp(cmd_lower, "start") == 0) {
        // Start data logging instead of just changing transmission mode
        if (DataLogger_StartRecording()) {
            data_transmission_enabled = false;  // Stop BLE sensor data transmission
            BLE_SendResponse("Recording started - data logging to flash");
        } else {
            BLE_SendResponse("ERROR: Failed to start recording");
            DebugPrint("BLE: ERROR - Failed to start recording\r\n");
        }
    }
    else if (strcmp(cmd_lower, "stop") == 0) {
        // Stop data logging and resume BLE transmission
        if (DataLogger_StopRecording()) {
            data_transmission_enabled = true;   // Resume BLE sensor data transmission
            BLE_SendResponse("Recording stopped - BLE transmission resumed");
        } else {
            BLE_SendResponse("ERROR: Failed to stop recording");
            DebugPrint("BLE: ERROR - Failed to stop recording\r\n");
        }
    }
    else if (strcmp(cmd_lower, "status") == 0) {
        // Get comprehensive status including data logger
        char status_msg[200];
        char logger_status[150];

        if (DataLogger_GetStatusString(logger_status, sizeof(logger_status))) {
            snprintf(status_msg, sizeof(status_msg),
                    "BLE: %s, Connected: %s | Logger: %s",
                    data_transmission_enabled ? "Data mode" : "Recording mode",
                    BLE_IsConnected() ? "Yes" : "No",
                    logger_status);
        } else {
            snprintf(status_msg, sizeof(status_msg),
                    "BLE: %s, Connected: %s | Logger: Not available",
                    data_transmission_enabled ? "Data mode" : "Recording mode",
                    BLE_IsConnected() ? "Yes" : "No");
        }

        BLE_SendResponse(status_msg);
    }
    else if (strcmp(cmd_lower, "erase") == 0) {
        // Manually erase flash data
        if (DataLogger_EraseAll()) {
            BLE_SendResponse("Flash memory erased");
        } else {
            BLE_SendResponse("ERROR: Failed to erase flash");
        }
    }
    else if (strcmp(cmd_lower, "help") == 0) {
        // List available commands
        BLE_SendResponse("Commands: start, stop, status, erase, help");
    }
    else {
        // Check custom callback
        if (command_callback) {
            command_callback(command);
        } else {
            BLE_SendResponse("Unknown command - type 'help' for commands");
        }
    }
}

/**
 * @brief Check if BLE is ready
 */
bool BLE_IsReady(void) {
    return ble_initialized && (ble_status != BLE_STATUS_ERROR);
}

/**
 * @brief Get BLE status
 */
BLE_Status_t BLE_GetStatus(void) {
    return ble_status;
}

/**
 * @brief Check if BLE is connected
 */
bool BLE_IsConnected(void) {
    return ble_status == BLE_STATUS_CONNECTED;
}

/**
 * @brief Get BLE statistics
 */
bool BLE_GetStatistics(BLE_Statistics_t* stats) {
    if (!stats || !ble_initialized) {
        return false;
    }

    stats->total_bytes_received = total_bytes_received;
    stats->total_bytes_sent = total_bytes_sent;
    stats->last_activity_time = last_rx_time;
    stats->connection_count = connection_count;
    stats->uart_active = dma_active;  /* DMA active status (was uart_rx_active in interrupt mode) */
    stats->status = ble_status;

    return true;
}

/**
 * @brief Reset BLE module
 */
bool BLE_Reset(void) {
    DebugPrint("BLE: Resetting BLE module...\r\n");
    ResetBleModule();
    return BLE_Init();
}

/**
 * @brief Enhanced BLE configuration with better error handling
 */
bool BLE_Configure(void) {
    DebugPrint("BLE: Configuring module...\r\n");

    // Hardware reset with longer delays
    ResetBleModule();

    // Try to enter command mode
    if (!EnterCommandMode()) {
        DebugPrint("BLE: ERROR - Cannot enter command mode\r\n");
        return false;
    }

    bool success = true;
    int failed_commands = 0;

    // Configure device name
    if (!SendBleCommand("SN,STM32-SENSOR-GPS\r", "AOK", 2000)) failed_commands++;

    // Configure authentication
    if (!SendBleCommand("SA,0\r", "AOK", 2000)) failed_commands++;

    // Configure services
    if (!SendBleCommand("SS,C0\r", "AOK", 2000)) failed_commands++;

    // Configure output mode
    if (!SendBleCommand("SO,0\r", "AOK", 2000)) failed_commands++;

    // Reboot to apply settings
    if (!SendBleCommand("R,1\r", "Reboot", 2000)) {
        failed_commands++;
    } else {
        HAL_Delay(5000); // Wait for reboot
    }

    // Re-enter command mode for final configuration
    if (!EnterCommandMode()) {
        DebugPrint("BLE: ERROR - Cannot enter command mode after reboot\r\n");
        return false;
    }

    // Configure advertising
    if (!SendBleCommand("SGA,0\r", "AOK", 2000)) failed_commands++;

    // Enable advertising
    if (!SendBleCommand("A\r", "AOK", 2000)) failed_commands++;

    // Exit command mode
    if (!SendBleCommand("---\r", "END", 2000)) failed_commands++;

    // Consider it successful if most commands worked
    success = (failed_commands < 5);

    if (success) {
        DebugPrint("BLE: Configuration successful!\r\n");
    } else {
        DebugPrint("BLE: Configuration failed - too many command failures\r\n");
    }

    return success;
}

/**
 * @brief Get time since last activity
 */
uint32_t BLE_GetTimeSinceLastActivity(void) {
    return HAL_GetTick() - last_rx_time;
}

/**
 * @brief Clear the UART response buffer
 */
static void ClearResponseBuffer(void) {
    memset(uart_response_buffer, 0, sizeof(uart_response_buffer));
    uart_response_index = 0;
}

/**
 * @brief Enhanced command sending - uses DMA if active, otherwise blocking UART
 */
static bool SendBleCommand(const char* cmd, const char* expectedResponse, uint32_t timeout) {
    char debugMsg[300];

    ClearResponseBuffer();

    // DebugPrint("BLE: Sending command..."); // Commented out for quiet operation

    // Send command with error checking (TX can be blocking - doesn't conflict with RX)
    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
    if (tx_status != HAL_OK) {
        snprintf(debugMsg, sizeof(debugMsg), "BLE: ERROR - UART transmit failed (status=%d)\r\n", tx_status);
        DebugPrint(debugMsg);
        return false;
    }
    total_bytes_sent += strlen(cmd);

    // Wait for response - method depends on whether DMA is active
    uint32_t startTime = HAL_GetTick();
    bool responseFound = false;

    if (dma_active) {
        /* DMA Mode: Read from circular buffer (used during runtime) */
        while ((HAL_GetTick() - startTime) < timeout && !responseFound) {
            // Get current DMA write position (volatile, updated by ISR)
            uint16_t current_write_pos = last_dma_write_pos;

            // Read all available bytes from DMA circular buffer
            while (buffer_read_pos != current_write_pos) {
                uint8_t tempByte = ble_rx_dma_buffer[buffer_read_pos];
                buffer_read_pos = (buffer_read_pos + 1) % BLE_RX_BUFFER_SIZE;

                if (uart_response_index < sizeof(uart_response_buffer) - 1) {
                    uart_response_buffer[uart_response_index++] = tempByte;
                    uart_response_buffer[uart_response_index] = '\0';
                }

                // Check for expected response
                if (expectedResponse == NULL) {
                    responseFound = true;
                    break;
                } else if (strstr(uart_response_buffer, expectedResponse) != NULL) {
                    responseFound = true;
                    break;
                } else if (strstr(uart_response_buffer, "ERR") != NULL) {
                    snprintf(debugMsg, sizeof(debugMsg), "BLE: Got error response: %s\r\n", uart_response_buffer);
                    DebugPrint(debugMsg);
                    return false;
                }
            }

            // Small delay to avoid busy-waiting
            if (!responseFound) {
                HAL_Delay(1);
            }
        }
    } else {
        /* Blocking Mode: Use HAL UART receive (used during configuration) */
        while ((HAL_GetTick() - startTime) < timeout && !responseFound) {
            uint8_t tempByte;
            HAL_StatusTypeDef rx_status = HAL_UART_Receive(&huart1, &tempByte, 1, 1);

            if (rx_status == HAL_OK) {
                if (uart_response_index < sizeof(uart_response_buffer) - 1) {
                    uart_response_buffer[uart_response_index++] = tempByte;
                    uart_response_buffer[uart_response_index] = '\0';
                }

                // Check for expected response
                if (expectedResponse == NULL) {
                    responseFound = true;
                } else if (strstr(uart_response_buffer, expectedResponse) != NULL) {
                    responseFound = true;
                } else if (strstr(uart_response_buffer, "ERR") != NULL) {
                    snprintf(debugMsg, sizeof(debugMsg), "BLE: Got error response: %s\r\n", uart_response_buffer);
                    DebugPrint(debugMsg);
                    return false;
                }
            }
        }
    }

    if (responseFound) {
        return true;
    } else {
        // Only print timeout if it really failed
        // snprintf(debugMsg, sizeof(debugMsg), "BLE: TIMEOUT - Response: '%s'\r\n", uart_response_buffer);
        // DebugPrint(debugMsg);
        return false;
    }
}

/**
 * @brief Enhanced BLE module reset with longer delays
 */
static void ResetBleModule(void) {
    /* RN4871 pin mapping: CONFIG=P2_0, LPM=P1_6, RST_BT=RST_N */
    HAL_GPIO_WritePin(CONFIG_GPIO_Port, CONFIG_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LPM_GPIO_Port, LPM_Pin, GPIO_PIN_SET);

    HAL_Delay(10); // Brief delay for pins to stabilize

    /* Hardware reset sequence - RST_N is active LOW */
    HAL_GPIO_WritePin(RST_BT_GPIO_Port, RST_BT_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);  // Hold in reset for 100ms

    HAL_GPIO_WritePin(RST_BT_GPIO_Port, RST_BT_Pin, GPIO_PIN_SET);
    HAL_Delay(500);  // Wait for module to boot

    // Clear any pending UART data
    uint8_t dummy;
    while (HAL_UART_Receive(&huart1, &dummy, 1, 10) == HAL_OK) {
        // Drain buffer
    }
}

/**
 * @brief Process received byte
 */
static void BLE_ProcessReceivedByte(uint8_t byte) {
    // Removed GPIO Toggle to save power/noise
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (rx_index < sizeof(rx_buffer) - 1) {
        rx_buffer[rx_index++] = byte;
    }

    /* UPDATE: Track timing for both general activity and character timeout */
    uint32_t current_time = HAL_GetTick();
    last_rx_time = current_time;
    last_char_time = current_time;
    total_bytes_received++;

    /* Process complete messages immediately if properly terminated */
    if (byte == '\n') {  // Only process on newline
        rx_buffer[rx_index] = '\0';

        /* Remove trailing newlines/carriage returns */
        while (rx_index > 0 && (rx_buffer[rx_index-1] == '\r' || rx_buffer[rx_index-1] == '\n')) {
            rx_buffer[--rx_index] = '\0';
        }

        if (rx_index > 0 && !IsProtocolOverhead(rx_buffer, rx_index)) {
            BLE_ProcessCommand((char*)rx_buffer);
        }

        rx_index = 0; /* Reset for next message */
    }
}

/**
 * @brief Start DMA reception with circular buffer (same pattern as GPS)
 */
static bool BLE_StartDMA(void) {
    /* Clear UART errors */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);

    /* Stop any ongoing transfers */
    HAL_UART_AbortReceive(&huart1);

    /* Start DMA in CIRCULAR mode */
    if (HAL_UART_Receive_DMA(&huart1, ble_rx_dma_buffer, BLE_RX_BUFFER_SIZE) != HAL_OK) {
        dma_active = false;
        DebugPrint("BLE: DMA start failed\r\n");
        return false;
    }

    /* Disable half-transfer interrupt (not needed) */
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    /* ✅ ADD THIS LINE - Enable IDLE line interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    dma_active = true;
    buffer_read_pos = 0;
    last_dma_write_pos = 0;

    DebugPrint("BLE: DMA started with idle line detection\r\n");
    return true;
}


/**
 * @brief BLE UART RX Event handler - called from unified callback in gps_module.c
 */
void BLE_UART_RxEventCallback(void) {
    /* Just update position - DO NOT restart DMA! */
    last_dma_write_pos = BLE_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    last_rx_time = HAL_GetTick();
}

/**
 * @brief UART RX Complete Callback - handles DMA completion (timeout/buffer full)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        /* DIAGNOSTIC: Track DMA restarts */
        static uint32_t restart_count = 0;
        restart_count++;

        /* Update write position before restart */
        last_dma_write_pos = BLE_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        /* Restart DMA */
        HAL_StatusTypeDef restart_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ble_rx_dma_buffer, BLE_RX_BUFFER_SIZE);
        if (restart_status != HAL_OK) {
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                    "BLE: ERROR - DMA restart #%lu failed (status=%d)\r\n",
                    restart_count, restart_status);
            DebugPrint(debug_msg);
            dma_active = false;
        } else {
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

            /* Log successful restarts periodically */
            if (restart_count % 10 == 0) {
                char debug_msg[80];
                snprintf(debug_msg, sizeof(debug_msg),
                        "BLE: DMA restart #%lu successful\r\n", restart_count);
                DebugPrint(debug_msg);
            }
        }
    }
}

/* Legacy compatibility functions - deprecated with DMA implementation */
void HandleReceivedByte(uint8_t byte) { (void)byte; }
void StartUartReception(void) { }
bool ConfigureModule(void) { return BLE_Configure(); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        /* DIAGNOSTIC: Log UART errors */
        static uint32_t uart_error_count = 0;
        uart_error_count++;

        char debug_msg[150];
        snprintf(debug_msg, sizeof(debug_msg),
                "BLE: UART error #%lu, ErrorCode=0x%08lX\r\n",
                uart_error_count, huart->ErrorCode);
        DebugPrint(debug_msg);

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        extern DMA_HandleTypeDef hdma_usart1_rx;
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        HAL_StatusTypeDef restart_status = HAL_UART_Receive_DMA(&huart1, ble_rx_dma_buffer, BLE_RX_BUFFER_SIZE);
        if (restart_status != HAL_OK) {
            snprintf(debug_msg, sizeof(debug_msg),
                    "BLE: ERROR - Failed to restart DMA after error (status=%d)\r\n",
                    restart_status);
            DebugPrint(debug_msg);
            dma_active = false;
        }
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    }
}
