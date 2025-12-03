/**
  ******************************************************************************
  * @file    telemetry_manager.c
  * @brief   Telemetry Manager implementation - Phase 3 refactoring
  * @note    Extracted from rocket_telemetry.c - handles all data transmission
  ******************************************************************************
  */

#include "telemetry_manager.h"
#include "lora_module.h"
#include "battery_monitor.h"  // Battery voltage reading
#include "sensor_adapter.h"   // For SensorData_t (=NavSensorData_t)
#include "debug_utils.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "rocket_telemetry.h"

/* Transmission queue configuration */
#define TELEMETRY_QUEUE_SIZE 8
#define MAX_PACKET_SIZE 256

/* Rate limiting configuration */
#define DEFAULT_MAX_RATE_HZ 5.0f       // 5 packets per second default
#define EMERGENCY_TIMEOUT_MS 150       // Emergency transmission timeout
#define NORMAL_TIMEOUT_MS 200          // Normal transmission timeout

/* Auto telemetry configuration */
#define DEFAULT_AUTO_INTERVAL_MS 200   // 5Hz default rocket telemetry

static volatile bool transmission_completed = false;
static volatile bool last_transmission_success = false;

/**
 * @brief Internal transmission queue entry
 */
typedef struct {
    TelemetryType_t type;
    TelemetryPriority_t priority;
    uint8_t data[MAX_PACKET_SIZE];
    uint16_t data_length;
    uint32_t timestamp;
    uint32_t max_age_ms;
    bool requires_ack;
    bool valid;
} QueueEntry_t;

/**
 * @brief Telemetry Manager state
 */
typedef struct {
    bool initialized;
    TelemetryMode_t current_mode;

    // Transmission queue
    QueueEntry_t queue[TELEMETRY_QUEUE_SIZE];
    uint32_t queue_head;
    uint32_t queue_tail;
    uint32_t queue_count;

    // Rate limiting
    float max_rate_hz;
    uint32_t last_transmission_time;
    uint32_t min_interval_ms;

    // Transmission state
    bool transmission_active;
    uint32_t current_transmission_start;
    TelemetryType_t current_transmission_type;

    // Statistics
    uint32_t total_requests;
    uint32_t successful_transmissions;
    uint32_t failed_transmissions;
    uint32_t timeout_transmissions;
    uint32_t rate_limit_drops;
    uint32_t queue_overflows;

    // Auto telemetry
    bool auto_telemetry_enabled;
    uint32_t auto_interval_ms;
    uint32_t last_auto_transmission;

    // Health monitoring
    uint32_t consecutive_failures;
    bool lora_healthy;

} TelemetryManager_t;

/* Private instance */
static TelemetryManager_t telemetry_manager = {0};

/* Private function prototypes */
static bool AddToQueue(const TelemetryRequest_t* request);
static bool ProcessQueue(void);
static QueueEntry_t* GetNextQueueEntry(void);
static void RemoveQueueEntry(void);
static bool CreateRocketTelemetryPacket(const NavigationSolution_t* nav_solution,
                                       const SensorData_t* sensor_data,
                                       const GPS_Data_t* gps_data,
                                       uint8_t* buffer, uint16_t* length);
static bool CreateDebugPacket(const char* debug_string, uint16_t string_length,
                             uint8_t* buffer, uint16_t* length);
static bool CreateHealthPacket(const NavigationSolution_t* nav_solution,
                              const SensorData_t* sensor_data,
                              uint8_t* buffer, uint16_t* length);
static bool StartTransmission(QueueEntry_t* entry);
static void HandleTransmissionTimeout(void);
static void UpdateStatistics(void);
static uint16_t CalculateCRC16(const uint8_t* data, uint16_t length);

/**
 * @brief Initialize the Telemetry Manager
 */
bool TelemetryManager_Init(void) {
    DebugPrint("TelemetryManager: Starting initialization...\r\n");

    // Clear manager state
    memset(&telemetry_manager, 0, sizeof(telemetry_manager));

    // Initialize LoRa module
    if (!LoRa_Init()) {
        DebugPrint("TelemetryManager: ERROR - LoRa initialization failed\r\n");
        return false;
    }

    // Set default configuration
    telemetry_manager.current_mode = TELEMETRY_MODE_NORMAL;
    telemetry_manager.max_rate_hz = DEFAULT_MAX_RATE_HZ;
    telemetry_manager.min_interval_ms = (uint32_t)(1000.0f / DEFAULT_MAX_RATE_HZ);
    telemetry_manager.auto_telemetry_enabled = true;
    telemetry_manager.auto_interval_ms = DEFAULT_AUTO_INTERVAL_MS;
    telemetry_manager.lora_healthy = true;

    // Initialize queue
    telemetry_manager.queue_head = 0;
    telemetry_manager.queue_tail = 0;
    telemetry_manager.queue_count = 0;

    // Mark as initialized
    telemetry_manager.initialized = true;
    telemetry_manager.last_transmission_time = 0;
    telemetry_manager.last_auto_transmission = HAL_GetTick();

    DebugPrint("TelemetryManager: Initialization complete\r\n");
    return true;
}

/**
 * @brief Update telemetry system
 */
void TelemetryManager_Update(void) {
    if (!telemetry_manager.initialized) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    /* Handle completion flags set by interrupt callback */
    if (transmission_completed) {
        transmission_completed = false;
        telemetry_manager.transmission_active = false;
        telemetry_manager.last_transmission_time = current_time;

        if (last_transmission_success) {
            telemetry_manager.successful_transmissions++;
            telemetry_manager.consecutive_failures = 0;
        } else {
            telemetry_manager.failed_transmissions++;
            telemetry_manager.consecutive_failures++;
            DebugPrint("TelemetryManager: Transmission failed\r\n");
        }
    }

    /* Handle transmission timeout */
    if (telemetry_manager.transmission_active) {
        uint32_t transmission_duration = current_time - telemetry_manager.current_transmission_start;
        uint32_t timeout_limit = (telemetry_manager.current_transmission_type == TELEMETRY_TYPE_ROCKET) ?
                                EMERGENCY_TIMEOUT_MS : NORMAL_TIMEOUT_MS;

        if (transmission_duration > timeout_limit) {
            //DebugPrint("TelemetryManager: Transmission timeout\r\n");
            HandleTransmissionTimeout();
        }
    }

    /* Process transmission queue if not busy */
    if (!telemetry_manager.transmission_active && !LoRa_IsTransmitting()) {
        if (telemetry_manager.queue_count > 0) {
            ProcessQueue();
        }
    }

    /* Update statistics and health monitoring */
    UpdateStatistics();
    telemetry_manager.lora_healthy = LoRa_IsReady() && (telemetry_manager.consecutive_failures < 5);
}

/**
 * @brief Send rocket telemetry data
 */
bool TelemetryManager_SendRocketTelemetry(const NavigationSolution_t* nav_solution,
                                         const SensorData_t* sensor_data,
                                         const GPS_Data_t* gps_data) {
    if (!telemetry_manager.initialized) {
        /*DebugPrint("TelemetryManager: ERROR - Not initialized\r\n");*/
        return false;
    }

    if (telemetry_manager.current_mode == TELEMETRY_MODE_SILENT) {
        return false;
    }

    // Create transmission request
    TelemetryRequest_t request;
    request.type = TELEMETRY_TYPE_ROCKET;
    request.priority = TELEMETRY_PRIORITY_HIGH;
    request.max_age_ms = 500;
    request.timestamp = HAL_GetTick();
    request.requires_ack = false;

    // Create rocket telemetry packet
    if (!CreateRocketTelemetryPacket(nav_solution, sensor_data, gps_data,
                                    request.data, &request.data_length)) {
        DebugPrint("TelemetryManager: ERROR - Failed to create packet\r\n");
        return false;
    }

    return AddToQueue(&request);
}

/**
 * @brief Send debug data
 */
bool TelemetryManager_SendDebugData(const char* debug_string, uint16_t string_length) {
    if (!telemetry_manager.initialized ||
        telemetry_manager.current_mode == TELEMETRY_MODE_SILENT ||
        telemetry_manager.current_mode == TELEMETRY_MODE_EMERGENCY) {
        return false;
    }

    TelemetryRequest_t request;
    request.type = TELEMETRY_TYPE_DEBUG;
    request.priority = TELEMETRY_PRIORITY_NORMAL;
    request.max_age_ms = 2000;
    request.timestamp = HAL_GetTick();
    request.requires_ack = false;

    if (!CreateDebugPacket(debug_string, string_length,
                          request.data, &request.data_length)) {
        return false;
    }

    return AddToQueue(&request);
}

/**
 * @brief Send system health data
 */
bool TelemetryManager_SendHealthData(const NavigationSolution_t* nav_solution,
                                    const SensorData_t* sensor_data) {
    if (!telemetry_manager.initialized ||
        telemetry_manager.current_mode == TELEMETRY_MODE_SILENT) {
        return false;
    }

    TelemetryRequest_t request;
    request.type = TELEMETRY_TYPE_HEALTH;
    request.priority = TELEMETRY_PRIORITY_NORMAL;
    request.max_age_ms = 5000;
    request.timestamp = HAL_GetTick();
    request.requires_ack = false;

    if (!CreateHealthPacket(nav_solution, sensor_data,
                           request.data, &request.data_length)) {
        return false;
    }

    return AddToQueue(&request);
}

/**
 * @brief Add transmission request to queue
 */
static bool AddToQueue(const TelemetryRequest_t* request) {
    if (!request) {
        DebugPrint("TelemetryManager: ERROR - NULL request\r\n");
        return false;
    }

    if (telemetry_manager.queue_count >= TELEMETRY_QUEUE_SIZE) {
        telemetry_manager.queue_overflows++;
        return false;
    }

    if (request->priority != TELEMETRY_PRIORITY_EMERGENCY &&
        telemetry_manager.last_transmission_time > 0) {
        uint32_t time_since_last = HAL_GetTick() - telemetry_manager.last_transmission_time;
        if (time_since_last < telemetry_manager.min_interval_ms) {
            telemetry_manager.rate_limit_drops++;
            return false;
        }
    }

    // Add to queue
    QueueEntry_t* entry = &telemetry_manager.queue[telemetry_manager.queue_tail];
    entry->type = request->type;
    entry->priority = request->priority;
    entry->data_length = request->data_length;
    entry->timestamp = request->timestamp;
    entry->max_age_ms = request->max_age_ms;
    entry->requires_ack = request->requires_ack;
    entry->valid = true;

    memcpy(entry->data, request->data, request->data_length);

    telemetry_manager.queue_tail = (telemetry_manager.queue_tail + 1) % TELEMETRY_QUEUE_SIZE;
    telemetry_manager.queue_count++;
    telemetry_manager.total_requests++;

    return true;
}

/**
 * @brief Process transmission queue
 */
static bool ProcessQueue(void) {
    if (telemetry_manager.queue_count == 0) {
        return false;
    }

    QueueEntry_t* entry = GetNextQueueEntry();
    if (!entry) {
        return false;
    }

    // Check if data is still fresh
    uint32_t age = HAL_GetTick() - entry->timestamp;
    if (age > entry->max_age_ms) {
        RemoveQueueEntry();
        return false;
    }

    // Start transmission
    if (StartTransmission(entry)) {
        RemoveQueueEntry();
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Get next queue entry based on priority
 */
static QueueEntry_t* GetNextQueueEntry(void) {
    if (telemetry_manager.queue_count == 0) {
        return NULL;
    }

    return &telemetry_manager.queue[telemetry_manager.queue_head];
}

/**
 * @brief Remove entry from queue head
 */
static void RemoveQueueEntry(void) {
    if (telemetry_manager.queue_count > 0) {
        telemetry_manager.queue[telemetry_manager.queue_head].valid = false;
        telemetry_manager.queue_head = (telemetry_manager.queue_head + 1) % TELEMETRY_QUEUE_SIZE;
        telemetry_manager.queue_count--;
    }
}

/**
 * @brief Start transmission of queue entry
 */
static bool StartTransmission(QueueEntry_t* entry) {
    if (!LoRa_IsReady()) {
        return false;
    }

    if (LoRa_IsTransmitting()) {
        return false;
    }

    // Start LoRa transmission
    if (LoRa_SendData(entry->data, entry->data_length)) {
        telemetry_manager.transmission_active = true;
        telemetry_manager.current_transmission_start = HAL_GetTick();
        telemetry_manager.current_transmission_type = entry->type;
        telemetry_manager.last_transmission_time = HAL_GetTick();
        return true;
    } else {
        telemetry_manager.failed_transmissions++;
        telemetry_manager.consecutive_failures++;
        DebugPrint("TelemetryManager: ERROR - LoRa_SendData failed\r\n");
        return false;
    }
}

/**
 * @brief Handle transmission timeout
 */
static void HandleTransmissionTimeout(void) {
    telemetry_manager.transmission_active = false;
    telemetry_manager.timeout_transmissions++;
    telemetry_manager.consecutive_failures++;
    LoRa_Reset();
}

/**
 * @brief Create rocket telemetry packet
 */
static bool CreateRocketTelemetryPacket(const NavigationSolution_t* nav_solution,
                                       const SensorData_t* sensor_data,
                                       const GPS_Data_t* gps_data,
                                       uint8_t* buffer, uint16_t* length) {
    if (!buffer || !length || !sensor_data) {
        return false;
    }

    RobustTelemetryPacket_t* packet = (RobustTelemetryPacket_t*)buffer;
    memset(packet, 0, sizeof(RobustTelemetryPacket_t));

    // Set sync marker
    packet->sync_marker = TELEMETRY_SYNC_MARKER;
    packet->timestamp = HAL_GetTick();

    // Fill sensor data
    if (sensor_data->baro_valid) {
        packet->barometric_altitude = 44330.0f * (1.0f - powf(sensor_data->pressure / 101325.0f, 0.1903f));

        float temp_c = sensor_data->temperature;
        if (temp_c > 127.0f) temp_c = 127.0f;
        if (temp_c < -128.0f) temp_c = -128.0f;
        packet->temperature_c = (int8_t)temp_c;

        packet->acceleration_total = sqrtf(sensor_data->accel[0] * sensor_data->accel[0] +
                                          sensor_data->accel[1] * sensor_data->accel[1] +
                                          sensor_data->accel[2] * sensor_data->accel[2]);
    }

    // Fill navigation data
    if (nav_solution && nav_solution->navigation_valid) {
        packet->vertical_velocity = -nav_solution->velocity_ned[2];
        packet->horizontal_speed = (uint8_t)fminf(nav_solution->ground_speed, 255.0f);
    }

    // Fill AHRS data
    if (nav_solution && nav_solution->ahrs_valid) {
        packet->roll_angle_deg = (int16_t)(nav_solution->euler.roll * 180.0f / M_PI);
        packet->pitch_angle_deg = (int16_t)(nav_solution->euler.pitch * 180.0f / M_PI);
        packet->yaw_angle_deg = (int16_t)(nav_solution->euler.yaw * 180.0f / M_PI);
    }

    // Fill GPS data
    if (gps_data) {
        packet->gps_ground_speed = (gps_data->speed >= 0.0f && gps_data->speed <= 1000.0f) ?
                                  gps_data->speed : 0.0f;
        packet->gps_latitude = (float)gps_data->latitude;
        packet->gps_longitude = (float)gps_data->longitude;
        packet->gps_satellites = gps_data->satellites;
        packet->gps_fix_status = gps_data->fix_status;
    }

    // Fill system data
    packet->battery_voltage = ReadBatteryVoltage();
    packet->nav_quality = nav_solution ? (nav_solution->navigation_valid ? 100 : 50) : 0;

    // Calculate CRC and set end marker
    uint8_t* data_start = (uint8_t*)packet;
    uint16_t data_length = sizeof(RobustTelemetryPacket_t) - sizeof(packet->crc16) - sizeof(packet->end_marker);
    packet->crc16 = CalculateCRC16(data_start, data_length);
    packet->end_marker = TELEMETRY_END_MARKER;

    *length = sizeof(RobustTelemetryPacket_t);
    return true;
}

/**
 * @brief Create debug packet
 */
static bool CreateDebugPacket(const char* debug_string, uint16_t string_length,
                             uint8_t* buffer, uint16_t* length) {
    if (!debug_string || !buffer || !length || string_length > (MAX_PACKET_SIZE - 10)) {
        return false;
    }

    buffer[0] = 0xDB;
    buffer[1] = 0x6D;
    buffer[2] = (string_length >> 8) & 0xFF;
    buffer[3] = string_length & 0xFF;

    memcpy(&buffer[4], debug_string, string_length);

    uint16_t crc = CalculateCRC16(buffer, 4 + string_length);
    buffer[4 + string_length] = (crc >> 8) & 0xFF;
    buffer[4 + string_length + 1] = crc & 0xFF;

    *length = 4 + string_length + 2;
    return true;
}

/**
 * @brief Create health packet
 */
static bool CreateHealthPacket(const NavigationSolution_t* nav_solution,
                              const SensorData_t* sensor_data,
                              uint8_t* buffer, uint16_t* length) {
    if (!buffer || !length) {
        return false;
    }

    buffer[0] = 0x48;  // 'H'
    buffer[1] = 0x45;  // 'E'
    buffer[2] = sensor_data ? (sensor_data->valid ? 1 : 0) : 0;
    buffer[3] = nav_solution ? (nav_solution->ahrs_valid ? 1 : 0) : 0;
    buffer[4] = nav_solution ? (nav_solution->navigation_valid ? 1 : 0) : 0;
    buffer[5] = telemetry_manager.lora_healthy ? 1 : 0;

    uint16_t crc = CalculateCRC16(buffer, 6);
    buffer[6] = (crc >> 8) & 0xFF;
    buffer[7] = crc & 0xFF;

    *length = 8;
    return true;
}

/**
 * @brief Calculate CRC16
 */
static uint16_t CalculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Update statistics
 */
static void UpdateStatistics(void) {
    uint32_t total_attempts = telemetry_manager.successful_transmissions +
                             telemetry_manager.failed_transmissions +
                             telemetry_manager.timeout_transmissions;

    if (total_attempts > 0) {
        telemetry_manager.total_requests = telemetry_manager.successful_transmissions / (float)total_attempts * 100.0f;
    }
}

/**
 * @brief Transmission completion callback
 */
void TelemetryManager_TransmissionComplete(bool success) {
    if (!telemetry_manager.initialized || !telemetry_manager.transmission_active) {
        return;
    }

    last_transmission_success = success;
    transmission_completed = true;
}

/**
 * @brief Check if telemetry system is ready
 */
bool TelemetryManager_IsReady(void) {
    return telemetry_manager.initialized &&
           telemetry_manager.lora_healthy &&
           !telemetry_manager.transmission_active &&
           !LoRa_IsTransmitting();
}

/**
 * @brief Check if transmission is active
 */
bool TelemetryManager_IsTransmitting(void) {
    return telemetry_manager.transmission_active || LoRa_IsTransmitting();
}

/**
 * @brief Set telemetry mode
 */
void TelemetryManager_SetMode(TelemetryMode_t mode) {
    telemetry_manager.current_mode = mode;
}

/**
 * @brief Get current telemetry mode
 */
TelemetryMode_t TelemetryManager_GetMode(void) {
    return telemetry_manager.current_mode;
}

/**
 * @brief Set transmission rate limit
 */
void TelemetryManager_SetRateLimit(float max_rate_hz) {
    if (max_rate_hz > 0.0f && max_rate_hz <= 100.0f) {
        telemetry_manager.max_rate_hz = max_rate_hz;
        telemetry_manager.min_interval_ms = (uint32_t)(1000.0f / max_rate_hz);
    }
}

/**
 * @brief Get diagnostics
 */
void TelemetryManager_GetDiagnostics(TelemetryDiagnostics_t* diagnostics) {
    if (!diagnostics) {
        return;
    }

    diagnostics->lora_healthy = telemetry_manager.lora_healthy;
    diagnostics->transmission_active = telemetry_manager.transmission_active;
    diagnostics->total_requests = telemetry_manager.total_requests;
    diagnostics->successful_transmissions = telemetry_manager.successful_transmissions;
    diagnostics->failed_transmissions = telemetry_manager.failed_transmissions;
    diagnostics->timeout_transmissions = telemetry_manager.timeout_transmissions;
    diagnostics->last_transmission_time = telemetry_manager.last_transmission_time;
    diagnostics->queue_depth = telemetry_manager.queue_count;
    diagnostics->max_queue_depth = TELEMETRY_QUEUE_SIZE;
    diagnostics->rate_limit_drops = telemetry_manager.rate_limit_drops;

    uint32_t total_attempts = diagnostics->successful_transmissions +
                             diagnostics->failed_transmissions +
                             diagnostics->timeout_transmissions;

    if (total_attempts > 0) {
        diagnostics->success_rate_percent = ((float)diagnostics->successful_transmissions / total_attempts) * 100.0f;
    } else {
        diagnostics->success_rate_percent = 0.0f;
    }
}

/**
 * @brief Get diagnostics string
 */
void TelemetryManager_GetDiagnosticsString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return;
    }

    TelemetryDiagnostics_t diag;
    TelemetryManager_GetDiagnostics(&diag);

    snprintf(buffer, buffer_size,
        "TelemetryManager Diagnostics:\r\n"
        "  LoRa: %s, Mode: %d\r\n"
        "  Transmissions: %lu success, %lu failed, %lu timeout\r\n"
        "  Success rate: %.1f%%\r\n"
        "  Queue: %lu/%lu, Rate drops: %lu\r\n"
        "  Active: %s\r\n",
        diag.lora_healthy ? "HEALTHY" : "FAILED",
        telemetry_manager.current_mode,
        diag.successful_transmissions,
        diag.failed_transmissions,
        diag.timeout_transmissions,
        diag.success_rate_percent,
        diag.queue_depth,
        diag.max_queue_depth,
        diag.rate_limit_drops,
        diag.transmission_active ? "YES" : "NO"
    );
}

/**
 * @brief Check if system is healthy
 */
bool TelemetryManager_IsHealthy(void) {
    return telemetry_manager.lora_healthy &&
           telemetry_manager.consecutive_failures < 3;
}

/**
 * @brief Emergency transmission
 */
bool TelemetryManager_EmergencyTransmission(const uint8_t* data, uint16_t data_length) {
    if (!data || data_length == 0 || data_length > MAX_PACKET_SIZE) {
        return false;
    }

    if (LoRa_SendData((uint8_t*)data, data_length)) {
        telemetry_manager.transmission_active = true;
        telemetry_manager.current_transmission_start = HAL_GetTick();
        telemetry_manager.current_transmission_type = TELEMETRY_TYPE_ROCKET;
        return true;
    }

    return false;
}

/**
 * @brief Set auto telemetry
 */
void TelemetryManager_SetAutoTelemetry(bool enable, uint32_t interval_ms) {
    telemetry_manager.auto_telemetry_enabled = enable;
    if (interval_ms > 0) {
        telemetry_manager.auto_interval_ms = interval_ms;
    }
}

/* Additional utility functions */
void TelemetryManager_ClearQueue(void) {
    telemetry_manager.queue_count = 0;
    telemetry_manager.queue_head = 0;
    telemetry_manager.queue_tail = 0;
}

void TelemetryManager_ResetStatistics(void) {
    telemetry_manager.total_requests = 0;
    telemetry_manager.successful_transmissions = 0;
    telemetry_manager.failed_transmissions = 0;
    telemetry_manager.timeout_transmissions = 0;
    telemetry_manager.rate_limit_drops = 0;
    telemetry_manager.queue_overflows = 0;
    telemetry_manager.consecutive_failures = 0;
}
