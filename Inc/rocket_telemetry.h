// ADD this at the top of rocket_telemetry.h:
#ifndef ROCKET_TELEMETRY_H_
#define ROCKET_TELEMETRY_H_

// Keep the packet structure for TelemetryManager
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_SYNC_MARKER 0xCAFEBABE
#define TELEMETRY_END_MARKER  0xDEADBEEF

typedef struct __attribute__((packed)) {
    uint32_t sync_marker;        // 4 bytes - Start marker
    uint32_t timestamp;          // 4 bytes
    float barometric_altitude;   // 4 bytes
    float vertical_velocity;     // 4 bytes
    float acceleration_total;    // 4 bytes
    float gps_ground_speed;      // 4 bytes
    float gps_latitude;          // 4 bytes
    float gps_longitude;         // 4 bytes
    float battery_voltage;       // 4 bytes
    uint8_t gps_satellites;      // 1 byte
    uint8_t gps_fix_status;      // 1 byte
    int16_t roll_angle_deg;      // 2 bytes
    int16_t pitch_angle_deg;     // 2 bytes
    int16_t yaw_angle_deg;       // 2 bytes
    int8_t temperature_c;        // 1 byte
    uint8_t nav_quality;         // 1 byte
    uint8_t horizontal_speed;    // 1 byte
    uint16_t crc16;              // 2 bytes
    uint32_t end_marker;         // 4 bytes
} RobustTelemetryPacket_t;       // Total: 53 bytes

// Deprecated functions - use TelemetryManager instead
bool RocketTelemetry_Init(void);
bool RocketTelemetry_SendData(void);

#endif
