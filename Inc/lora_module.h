/* ============================================================================ */
/* CLEAN LORA MODULE - PRODUCTION READY - No External Variables */
/* ============================================================================ */

#ifndef __LORA_MODULE_H__
#define __LORA_MODULE_H__

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

/* LoRa Operating Modes */
typedef enum {
    LORA_MODE_NORMAL = 0,     // M0=0, M1=0 - Transparent transmission
    LORA_MODE_WAKEUP = 1,     // M0=1, M1=0 - Wake-up transmission
    LORA_MODE_POWERSAVE = 2,  // M0=0, M1=1 - Power saving
    LORA_MODE_SLEEP = 3       // M0=1, M1=1 - Sleep/configuration
} LoRa_Mode_t;

/* LoRa Status */
typedef enum {
    LORA_STATUS_READY,
    LORA_STATUS_TRANSMITTING,
    LORA_STATUS_ERROR,
    LORA_STATUS_TIMEOUT,
    LORA_STATUS_NOT_INITIALIZED
} LoRa_Status_t;

/* LoRa Statistics */
typedef struct {
    uint32_t total_transmissions;
    uint32_t successful_transmissions;
    uint32_t failed_transmissions;
    uint32_t timeout_transmissions;
    uint32_t last_transmission_time;
    uint32_t current_transmission_start;
    bool transmission_active;
} LoRa_Statistics_t;

/**
 * @brief LoRa Module API - Clean Interface (No External Variables)
 */

/**
 * @brief Initialize LoRa module
 * @return true if initialization successful
 */
bool LoRa_Init(void);

/**
 * @brief Send data via LoRa
 * @param data Pointer to data to send
 * @param length Length of data in bytes
 * @return true if transmission started successfully
 */
bool LoRa_SendData(uint8_t *data, uint16_t length);

/**
 * @brief Check if LoRa module is ready for new transmission
 * @return true if ready (not transmitting and hardware ready)
 */
bool LoRa_IsReady(void);

/**
 * @brief Check if LoRa is currently transmitting
 * @return true if transmission is in progress
 */
bool LoRa_IsTransmitting(void);

/**
 * @brief Get LoRa module status
 * @return Current LoRa status
 */
LoRa_Status_t LoRa_GetStatus(void);

/**
 * @brief Get total transmission count
 * @return Number of completed transmissions
 */
uint32_t LoRa_GetTransmissionCount(void);

/**
 * @brief Get age of current transmission
 * @return Time in milliseconds since current transmission started (0 if not transmitting)
 */
uint32_t LoRa_GetTransmissionAge(void);

/**
 * @brief Get LoRa statistics
 * @param stats Pointer to statistics structure to fill
 * @return true if statistics retrieved successfully
 */
bool LoRa_GetStatistics(LoRa_Statistics_t* stats);

/**
 * @brief Reset LoRa transmission state (for timeout recovery)
 * @return true if reset successful
 */
bool LoRa_Reset(void);

/**
 * @brief Set LoRa operating mode
 * @param mode Operating mode to set
 * @return true if mode set successfully
 */
bool LoRa_SetMode(LoRa_Mode_t mode);

/**
 * @brief Handle transmission completion (called from HAL callback)
 * @param success true if transmission completed successfully
 */
void LoRa_TransmissionComplete(bool success);

/**
 * @brief Update LoRa module - call regularly for timeout handling
 */
void LoRa_Update(void);

/**
 * @brief Check if packet buffer is busy (for rocket telemetry compatibility)
 * @return true if packet buffer is busy
 */
bool LoRa_IsPacketBufferBusy(void);

/**
 * @brief Set packet buffer busy state (for rocket telemetry compatibility)
 * @param busy true to set busy, false to clear
 */
void LoRa_SetPacketBufferBusy(bool busy);

#endif /* __LORA_MODULE_H__ */
