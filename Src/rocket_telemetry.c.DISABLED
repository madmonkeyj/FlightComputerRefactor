/* ============================================================================ */
/* rocket_telemetry.c - Updated version without GPS altitude & flight phase */
/* ============================================================================ */

#include "rocket_telemetry.h"
#include "lora_module.h"
#include "sensor_manager.h"
#include "gps_module.h"
#include "battery_monitor.h"
#include "debug_utils.h"
#include <math.h>
#include <string.h>
#include "stdio.h"
#include "navigation_manager.h"
#include "sensor_system.h"


/* Private variables */
static uint32_t launch_time = 0;
static bool launch_detected = false;
static float ground_pressure = 101325.0f;
static float max_altitude = 0.0f;

// Global buffer with mutex protection
static RobustTelemetryPacket_t tx_packet_buffer;
volatile bool packet_buffer_busy = false;

/* Flight detection parameters */
#define LAUNCH_ACCEL_THRESHOLD    15.0f     // 15 m/s² above 1G
#define APOGEE_VELOCITY_THRESHOLD 2.0f      // 2 m/s vertical velocity
#define LANDING_VELOCITY_THRESHOLD 1.0f     // 1 m/s for landing

/**
 * @brief Calculate CRC16 checksum for data validation
 * @param data: Pointer to data buffer
 * @param length: Length of data in bytes
 * @return: 16-bit CRC value
 */
uint16_t calculate_crc16(uint8_t* data, uint16_t length) {
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
 * @brief Calculate altitude from pressure
 */
float RocketTelemetry_CalculateAltitude(float pressure_pa) {
    if (pressure_pa <= 0 || pressure_pa > 120000.0f) {
        return 0.0f;
    }

    // Barometric formula
    float altitude = 44330.0f * (1.0f - powf(pressure_pa / ground_pressure, 0.1903f));
    return altitude;
}

/**
 * @brief Calculate navigation quality from available data
 */
uint8_t CalculateNavQuality(void) {
    // Get current GPS data using clean interface
    GPS_Data_t current_gps_data;
    if (!GPS_GetCurrentData(&current_gps_data) || current_gps_data.fix_status != 'A') {
        return 0;  // No GPS fix
    }

    // Quality based on satellite count: 4 sats = 20%, 12+ sats = 100%
    uint8_t quality = 0;
    if (current_gps_data.satellites >= 12) {
        quality = 100;
    } else if (current_gps_data.satellites >= 4) {
        quality = 20 + ((current_gps_data.satellites - 4) * 10);  // Linear scale 20-100%
    }
    return quality;
}

/**
 * @brief Calculate horizontal speed from available data - FIXED VERSION
 */
uint8_t CalculateHorizontalSpeed(void) {
    float vel_north, vel_east, vel_down;

    if (!GetNavigationVelocityNED(&vel_north, &vel_east, &vel_down)) {
        // Fallback: Use GPS ground speed if available
        GPS_Data_t current_gps_data;
        if (GPS_GetCurrentData(&current_gps_data) &&
            current_gps_data.speed >= 0.0f && current_gps_data.speed <= 255.0f) {
            return (uint8_t)current_gps_data.speed;
        }
        return 0;  // No navigation data available
    }

    // Calculate horizontal speed magnitude
    float horizontal_speed = sqrtf(vel_north * vel_north + vel_east * vel_east);

    // Clamp to 0-255 m/s range
    if (horizontal_speed > 255.0f) {
        horizontal_speed = 255.0f;
    }

    return (uint8_t)horizontal_speed;
}

/**
 * @brief Initialize rocket telemetry
 */
bool RocketTelemetry_Init(void) {
    launch_time = 0;
    max_altitude = 0.0f;

    // Initialize LoRa module (automatically configures for 4.8kbps, 915MHz)
    if (!LoRa_Init()) {
        DebugPrint("ERROR: LoRa initialization failed\r\n");
        return false;
    }

    DebugPrint("Rocket telemetry system initialized\r\n");
    return true;
}

/**
 * @brief Send rocket telemetry packet - FIXED VERSION
 */
bool RocketTelemetry_SendData(void) {
    // Don't clear buffer if transmission in progress
    if (packet_buffer_busy) {
        DebugPrint("Packet buffer busy\r\n");
        return false;
    }

    packet_buffer_busy = true;
    memset(&tx_packet_buffer, 0, sizeof(tx_packet_buffer));

    // Get current sensor data
    SensorData_t current_sensors;
    bool sensors_valid = SensorSystem_Read(&current_sensors);

    // Get current navigation solution
    NavigationSolution_t nav_solution;
    bool nav_valid = NavigationManager_GetSolution(&nav_solution);

    // Get current GPS data using clean interface
    GPS_Data_t current_gps_data;
    bool gps_valid = GPS_GetCurrentData(&current_gps_data);

    // ALWAYS set sync marker first
    tx_packet_buffer.sync_marker = TELEMETRY_SYNC_MARKER;

    // Fill rest of packet
    if (launch_detected) {
        tx_packet_buffer.timestamp = HAL_GetTick() - launch_time;
    } else {
        tx_packet_buffer.timestamp = HAL_GetTick();
    }

    // Barometric altitude - use current sensor data
    if (sensors_valid && current_sensors.baro_valid) {
        tx_packet_buffer.barometric_altitude = RocketTelemetry_CalculateAltitude(current_sensors.pressure);

        // Temperature
        float temp_c = current_sensors.temperature;
        if (temp_c > 127.0f) temp_c = 127.0f;
        if (temp_c < -128.0f) temp_c = -128.0f;
        tx_packet_buffer.temperature_c = (int8_t)temp_c;

        // Total acceleration magnitude
        tx_packet_buffer.acceleration_total = sqrtf(current_sensors.accel[0] * current_sensors.accel[0] +
                                                   current_sensors.accel[1] * current_sensors.accel[1] +
                                                   current_sensors.accel[2] * current_sensors.accel[2]);
    } else {
        tx_packet_buffer.barometric_altitude = 0.0f;
        tx_packet_buffer.temperature_c = 0;
        tx_packet_buffer.acceleration_total = 0.0f;
    }

    // Vertical velocity from EKF
    if (nav_valid && nav_solution.navigation_valid) {
        tx_packet_buffer.vertical_velocity = -nav_solution.velocity_ned[2];  // Convert to positive-up
        tx_packet_buffer.horizontal_speed = (uint8_t)fminf(nav_solution.ground_speed, 255.0f);
    } else {
        tx_packet_buffer.vertical_velocity = 0.0f;
        tx_packet_buffer.horizontal_speed = 0;
    }

    // GPS data with validation - FIXED TO USE CLEAN INTERFACE
    if (gps_valid) {
        tx_packet_buffer.gps_ground_speed = (current_gps_data.speed >= 0.0f && current_gps_data.speed <= 1000.0f) ?
                                           current_gps_data.speed : 0.0f;

        tx_packet_buffer.gps_latitude = (current_gps_data.latitude >= -90.0f && current_gps_data.latitude <= 90.0f) ?
                                       (float)current_gps_data.latitude : 0.0f;

        tx_packet_buffer.gps_longitude = (current_gps_data.longitude >= -180.0f && current_gps_data.longitude <= 180.0f) ?
                                        (float)current_gps_data.longitude : 0.0f;

        tx_packet_buffer.gps_satellites = (current_gps_data.satellites >= 0 && current_gps_data.satellites <= 50) ?
                                         current_gps_data.satellites : 0;

        tx_packet_buffer.gps_fix_status = (current_gps_data.fix_status == 'A') ? 'A' : 'V';
    } else {
        tx_packet_buffer.gps_ground_speed = 0.0f;
        tx_packet_buffer.gps_latitude = 0.0f;
        tx_packet_buffer.gps_longitude = 0.0f;
        tx_packet_buffer.gps_satellites = 0;
        tx_packet_buffer.gps_fix_status = 'V';
    }

    tx_packet_buffer.battery_voltage = ReadBatteryVoltage();

    // AHRS orientation data
    if (nav_valid && nav_solution.ahrs_valid) {
        tx_packet_buffer.roll_angle_deg = (int16_t)(nav_solution.euler.roll * 180.0f / M_PI);
        tx_packet_buffer.pitch_angle_deg = (int16_t)(nav_solution.euler.pitch * 180.0f / M_PI);
        tx_packet_buffer.yaw_angle_deg = (int16_t)(nav_solution.euler.yaw * 180.0f / M_PI);
    } else {
        tx_packet_buffer.roll_angle_deg = 0;
        tx_packet_buffer.pitch_angle_deg = 0;
        tx_packet_buffer.yaw_angle_deg = 0;
    }

    // Navigation quality
    tx_packet_buffer.nav_quality = CalculateNavQuality();

    // Calculate CRC and set end marker
    uint8_t* data_start = (uint8_t*)&tx_packet_buffer;
    uint16_t data_length = sizeof(tx_packet_buffer) - sizeof(tx_packet_buffer.crc16) - sizeof(tx_packet_buffer.end_marker);
    tx_packet_buffer.crc16 = calculate_crc16(data_start, data_length);
    tx_packet_buffer.end_marker = TELEMETRY_END_MARKER;

    // Send via LoRa
    bool success = LoRa_SendData((uint8_t*)&tx_packet_buffer, sizeof(tx_packet_buffer));

    if (!success) {
        packet_buffer_busy = false;  // Release buffer on failure
    }

    // On success, mutex released by interrupt callback
    return success;
}

/**
 * @brief Create rocket telemetry packet - FIXED VERSION
 */
bool RocketTelemetry_CreatePacket(RobustTelemetryPacket_t* packet) {
    if (!packet) return false;

    memset(packet, 0, sizeof(*packet));

    // Get current sensor data
    SensorData_t current_sensors;
    bool sensors_valid = SensorSystem_Read(&current_sensors);

    // Get current navigation solution
    NavigationSolution_t nav_solution;
    bool nav_valid = NavigationManager_GetSolution(&nav_solution);

    // Get current GPS data using clean interface
    GPS_Data_t current_gps_data;
    bool gps_valid = GPS_GetCurrentData(&current_gps_data);

    // Fill packet with available data
    packet->sync_marker = TELEMETRY_SYNC_MARKER;
    packet->timestamp = HAL_GetTick();

    // Barometric altitude - use current sensor data
    if (sensors_valid && current_sensors.baro_valid) {
        packet->barometric_altitude = RocketTelemetry_CalculateAltitude(current_sensors.pressure);

        // Temperature
        float temp_c = current_sensors.temperature;
        if (temp_c > 127.0f) temp_c = 127.0f;
        if (temp_c < -128.0f) temp_c = -128.0f;
        packet->temperature_c = (int8_t)temp_c;

        // Total acceleration
        packet->acceleration_total = sqrtf(current_sensors.accel[0] * current_sensors.accel[0] +
                                          current_sensors.accel[1] * current_sensors.accel[1] +
                                          current_sensors.accel[2] * current_sensors.accel[2]);
    } else {
        packet->barometric_altitude = 0.0f;
        packet->temperature_c = 0;
        packet->acceleration_total = 0.0f;
    }

    // Vertical velocity from navigation
    if (nav_valid && nav_solution.navigation_valid) {
        packet->vertical_velocity = -nav_solution.velocity_ned[2];  // Convert to positive-up
        packet->horizontal_speed = (uint8_t)fminf(nav_solution.ground_speed, 255.0f);
    } else {
        packet->vertical_velocity = 0.0f;
        packet->horizontal_speed = 0;
    }

    // GPS data - FIXED TO USE CLEAN INTERFACE
    if (gps_valid) {
        packet->gps_ground_speed = (current_gps_data.speed >= 0.0f && current_gps_data.speed <= 1000.0f) ?
                                  current_gps_data.speed : 0.0f;
        packet->gps_latitude = (current_gps_data.latitude >= -90.0f && current_gps_data.latitude <= 90.0f) ?
                              (float)current_gps_data.latitude : 0.0f;
        packet->gps_longitude = (current_gps_data.longitude >= -180.0f && current_gps_data.longitude <= 180.0f) ?
                               (float)current_gps_data.longitude : 0.0f;
        packet->gps_satellites = (current_gps_data.satellites >= 0 && current_gps_data.satellites <= 50) ?
                                current_gps_data.satellites : 0;
        packet->gps_fix_status = (current_gps_data.fix_status == 'A') ? 'A' : 'V';
    } else {
        packet->gps_ground_speed = 0.0f;
        packet->gps_latitude = 0.0f;
        packet->gps_longitude = 0.0f;
        packet->gps_satellites = 0;
        packet->gps_fix_status = 'V';
    }

    packet->battery_voltage = ReadBatteryVoltage();

    // AHRS orientation data
    if (nav_valid && nav_solution.ahrs_valid) {
        packet->roll_angle_deg = (int16_t)(nav_solution.euler.roll * 180.0f / M_PI);
        packet->pitch_angle_deg = (int16_t)(nav_solution.euler.pitch * 180.0f / M_PI);
        packet->yaw_angle_deg = (int16_t)(nav_solution.euler.yaw * 180.0f / M_PI);
    } else {
        packet->roll_angle_deg = 0;
        packet->pitch_angle_deg = 0;
        packet->yaw_angle_deg = 0;
    }

    // Navigation quality
    packet->nav_quality = CalculateNavQuality();

    // Calculate CRC and set end marker
    uint8_t* data_start = (uint8_t*)packet;
    uint16_t data_length = sizeof(*packet) - sizeof(packet->crc16) - sizeof(packet->end_marker);
    packet->crc16 = calculate_crc16(data_start, data_length);
    packet->end_marker = TELEMETRY_END_MARKER;

    return true;
}
