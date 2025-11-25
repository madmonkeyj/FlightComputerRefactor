# FlightComputer Integration Plan
## Complete System Integration Guide

**Date:** 2025-11-25
**Purpose:** Integrate new navigation and telemetry modules with existing refactored sensor infrastructure

---

## Executive Summary

The user has added 11 new modules totaling 6,346 lines of code to complete the flight controller software. This document provides a comprehensive integration plan that:

1. **Identifies which modules to use** (sensor hardware mismatch resolved)
2. **Maps data flow** between all subsystems
3. **Provides step-by-step integration instructions**
4. **Shows exact main.c modifications needed**
5. **Documents all integration points and callbacks**

---

## 1. Module Analysis & Usage Decision

### 1.1 Sensor Layer - HARDWARE MISMATCH IDENTIFIED

**Issue**: The new `sensor_system` module uses **different sensors** than your hardware:

| Component | New Modules | Your Hardware |
|-----------|-------------|---------------|
| IMU | BMI088 (I2C bit-bang) | ICM42688 (SPI + DMA) |
| Magnetometer | MLX90393 | MMC5983MA (I2C DMA arbiter) |
| Barometer | BMP390 | BMP581 (I2C DMA arbiter) |
| High-G Accel | Not present | H3LIS331DL (I2C DMA arbiter) |
| Communication | I2C bit-bang | I2C DMA arbiter (priority-based) |

**Decision**: ❌ **DO NOT USE** `sensor_system` module
**Resolution**: ✅ **KEEP EXISTING** `sensor_manager` with all your refactored code

The new modules came from a different hardware revision of the flight computer. The navigation and telemetry layers are hardware-agnostic and will work with your existing sensors.

### 1.2 Navigation Layer - INTEGRATE ALL

✅ **USE THESE MODULES**:
- `navigation_ekf.h/.c` (1247 lines) - 6-state Extended Kalman Filter
- `navigation_manager.h/.c` (675 lines) - AHRS + EKF coordinator
- `motion_detector.h/.c` (362 lines) - Motion state detection
- `coord_transform.h/.c` (286 lines) - Body ↔ NED transformations
- `nav_matrix.h/.c` (79 lines) - EKF matrix operations
- `nav_config.h/.c` (45 lines) - Configuration parameters

**Why**: These modules provide advanced navigation capabilities:
- Fuses IMU + GPS + Barometer for position/velocity estimation
- Adaptive Kalman filtering based on motion state
- Zero-velocity updates (ZUPT) for stationary detection
- Proper coordinate frame handling
- Works with your existing `mahony_filter` for attitude

### 1.3 Telemetry Layer - INTEGRATE ALL

✅ **USE THESE MODULES**:
- `telemetry_manager.h/.c` (725 lines) - Telemetry coordination
- `lora_module.h/.c` (372 lines) - LoRa wireless driver
- `rocket_telemetry.h/.c` (346 lines) - Telemetry packet format

**Why**: Provides complete wireless telemetry:
- Priority-based transmission queue
- Rate limiting and health monitoring
- LoRa hardware management
- 53-byte robust packet format with CRC16

### 1.4 System Layer - INTEGRATE

✅ **USE THIS MODULE**:
- `flight_manager.h/.c` (428 lines) - Top-level orchestrator

**Why**: Coordinates all subsystems with proper timing:
- Sensors: 500Hz
- Navigation EKF: 50Hz (decimated from AHRS)
- Telemetry: 5Hz
- GPS: 0.2Hz (as available)

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        FLIGHT MANAGER                            │
│  (Top-level orchestrator - timing and coordination)             │
└──────────────┬────────────────────────────────┬─────────────────┘
               │                                │
               ▼                                ▼
┌──────────────────────────────┐  ┌────────────────────────────────┐
│    EXISTING SENSOR LAYER     │  │   TELEMETRY MANAGER            │
│  (Keep all refactored code)  │  │  (Priority queue + rate limit) │
│                              │  │                                │
│  • sensor_manager            │  │  • lora_module                 │
│  • i2c_dma_arbiter          │  │  • rocket_telemetry            │
│  • icm42688 (SPI)           │  └───────────┬────────────────────┘
│  • mmc5983ma (I2C)          │              │
│  • h3lis331dl (I2C)         │              │ LoRa UART
│  • bmp581 (I2C)             │              ▼
│  • gps_module (UART)        │      [Wireless Transmission]
│  • ble_module (UART)        │
└──────────┬───────────────────┘
           │
           │ SensorManager_RawData_t
           ▼
┌────────────────────────────────────────────────────────────────┐
│                  NAVIGATION MANAGER                            │
│  (AHRS 500Hz + EKF 50Hz coordination)                         │
│                                                                │
│  ┌──────────────────┐    ┌──────────────────────────────────┐ │
│  │ Mahony Filter    │    │  Navigation EKF (6-state)        │ │
│  │ (existing)       │    │  • motion_detector               │ │
│  │ 500Hz AHRS       │◄───┤  • coord_transform               │ │
│  │ Quaternion NED   │    │  • nav_matrix                    │ │
│  └──────────────────┘    │  • nav_config                    │ │
│                          │  50Hz Position/Velocity           │ │
│                          └──────────────────────────────────┘ │
└───────────────────────────┬────────────────────────────────────┘
                            │
                            │ NavigationSolution_t
                            ▼
┌────────────────────────────────────────────────────────────────┐
│                      DATA LOGGER                               │
│  (Existing - with NavigationProvider_t interface)             │
│  • Uses NavigationProvider callbacks                           │
│  • Records to QSPI flash (192-byte records)                   │
└────────────────────────────────────────────────────────────────┘
```

---

## 3. Critical Integration Points

### 3.1 Sensor Manager ↔ Navigation Manager Adapter

The new `navigation_manager` expects `SensorData_t` from the incompatible `sensor_system`. We need an **adapter** to convert your existing `SensorManager_RawData_t` to the format the navigation manager needs.

**File**: Create `Inc/sensor_adapter.h` and `Src/sensor_adapter.c`

```c
// sensor_adapter.h
#ifndef SENSOR_ADAPTER_H
#define SENSOR_ADAPTER_H

#include "sensor_manager.h"
#include "navigation_manager.h"  // This expects different SensorData_t

/**
 * @brief Convert SensorManager data to format expected by navigation_manager
 * @param raw_data Your existing sensor_manager output
 * @param nav_data Output for navigation_manager (different structure name conflict)
 * @return true if conversion successful
 * @note Handles the sensor hardware abstraction mismatch
 */
bool SensorAdapter_ConvertToNavFormat(const SensorManager_RawData_t* raw_data,
                                      void* nav_data);  // Use void* to avoid name conflict

#endif
```

**OR SIMPLER APPROACH**: Modify `navigation_manager.c` to directly use `SensorManager_RawData_t` instead of the incompatible `SensorData_t`. This requires editing the navigation_manager implementation.

### 3.2 Navigation Manager → Data Logger Integration

The existing `data_logger` has a `NavigationProvider_t` interface that needs callbacks from `navigation_manager`.

**File**: Modify `Src/main.c` to add adapter functions:

```c
/* Navigation Provider Callbacks for Data Logger */

bool NavProvider_GetQuaternion(float quat[4]) {
    Quaternion_t q;
    if (NavigationManager_GetAttitude(&q)) {
        quat[0] = q.q0;
        quat[1] = q.q1;
        quat[2] = q.q2;
        quat[3] = q.q3;
        return true;
    }
    return false;
}

bool NavProvider_GetPositionNED(float pos[3]) {
    return NavigationManager_GetPositionNED(&pos[0], &pos[1], &pos[2]);
}

bool NavProvider_GetVelocityNED(float vel[3]) {
    return NavigationManager_GetVelocityNED(&vel[0], &vel[1], &vel[2]);
}

bool NavProvider_IsValid(void) {
    return NavigationManager_IsHealthy();
}

static const NavigationProvider_t nav_provider = {
    .get_quaternion = NavProvider_GetQuaternion,
    .get_position_ned = NavProvider_GetPositionNED,
    .get_velocity_ned = NavProvider_GetVelocityNED,
    .is_valid = NavProvider_IsValid
};
```

### 3.3 GPS Module Integration

Both your existing code and new modules use `gps_module`, so this should integrate cleanly. The `navigation_manager` will call:

```c
NavigationManager_UpdateGPS(const GPS_Data_t* gps_data);
```

This feeds GPS position/velocity into the EKF.

### 3.4 LoRa Module Hardware Setup

The `lora_module` expects:
- **UART**: Needs assignment (USART2? Check your hardware)
- **GPIO Control**: M0, M1 pins for mode control
- **Callbacks**: UART TX complete interrupt

**Required in main.c**:
```c
// In HAL UART callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {  // Assuming USART2 for LoRa
        LoRa_TransmissionComplete(true);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        LoRa_TransmissionComplete(false);
    }
}
```

### 3.5 Flight Manager as Main Orchestrator

Option A: **Use FlightManager_Update()** as your main loop (recommended)
Option B: **Manual timing** - call subsystems directly with your own timing

**Option A Example (Recommended)**:
```c
int main(void) {
    // ... HAL init, peripherals init ...

    // Initialize all systems
    SensorManager_Init(&sensor_config);
    NavigationManager_Init();
    TelemetryManager_Init();
    BLE_Init();
    GPS_Init();
    DataLogger_Init();

    // Register navigation provider for data logger
    DataLogger_RegisterNavProvider(&nav_provider);

    // Initialize flight manager LAST (coordinates all above)
    FlightManager_Init();

    // Main loop - single call!
    while (1) {
        FlightManager_Update();  // Handles all timing internally

        // Critical refactored integration points
        I2C_DMA_Arbiter_Watchdog();
        DataLogger_Update();

        HAL_Delay(1);  // Optional rate limiting
    }
}
```

---

## 4. Required Changes to main.c

### 4.1 Add Includes

```c
/* USER CODE BEGIN Includes */
#include "debug_utils.h"
#include "icm42688.h"
#include "mmc5983ma.h"
#include "h3lis331dl.h"
#include "bmp581.h"
#include "i2c_dma_arbiter.h"
#include "sensor_manager.h"
#include "mahony_filter.h"
#include "gps_module.h"
#include "ble_module.h"
#include "data_logger.h"
#include "usb_commands.h"

/* NEW MODULE INCLUDES */
#include "navigation_manager.h"   // AHRS + EKF coordinator
#include "navigation_ekf.h"       // Extended Kalman Filter
#include "motion_detector.h"      // Motion state detection
#include "coord_transform.h"      // Frame transformations
#include "telemetry_manager.h"    // Telemetry coordination
#include "lora_module.h"          // LoRa wireless
#include "flight_manager.h"       // Top-level orchestrator

#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */
```

### 4.2 Remove Global mahony_filter (Now Managed by navigation_manager)

**BEFORE**:
```c
/* USER CODE BEGIN PV */
/* Mahony Filter - Global instance for data_logger access */
MahonyFilter_t mahony_filter;  // REMOVE THIS

/* BLE Test Variables */
static uint32_t ble_test_counter = 0;
static bool ble_initialized = false;
/* USER CODE END PV */
```

**AFTER**:
```c
/* USER CODE BEGIN PV */
/* BLE Test Variables */
static uint32_t ble_test_counter = 0;
static bool ble_initialized = false;
/* USER CODE END PV */
```

### 4.3 Add Navigation Provider Callbacks

```c
/* USER CODE BEGIN PFP */
/* BLE Command Callback */
static void BLE_CommandCallback(const char* command);

/* Navigation Provider Callbacks for Data Logger */
static bool NavProvider_GetQuaternion(float quat[4]);
static bool NavProvider_GetPositionNED(float pos[3]);
static bool NavProvider_GetVelocityNED(float vel[3]);
static bool NavProvider_IsValid(void);
/* USER CODE END PFP */
```

### 4.4 Implement Navigation Provider Callbacks

```c
/* USER CODE BEGIN 0 */

/**
 * @brief Navigation provider callback implementations
 * @note These adapt NavigationManager to DataLogger's NavigationProvider_t interface
 */
static bool NavProvider_GetQuaternion(float quat[4]) {
    Quaternion_t q;
    if (NavigationManager_GetAttitude(&q)) {
        quat[0] = q.q0;
        quat[1] = q.q1;
        quat[2] = q.q2;
        quat[3] = q.q3;
        return true;
    }
    return false;
}

static bool NavProvider_GetPositionNED(float pos[3]) {
    return NavigationManager_GetPositionNED(&pos[0], &pos[1], &pos[2]);
}

static bool NavProvider_GetVelocityNED(float vel[3]) {
    return NavigationManager_GetVelocityNED(&vel[0], &vel[1], &vel[2]);
}

static bool NavProvider_IsValid(void) {
    return NavigationManager_IsHealthy();
}

/* Navigation provider structure */
static const NavigationProvider_t nav_provider = {
    .get_quaternion = NavProvider_GetQuaternion,
    .get_position_ned = NavProvider_GetPositionNED,
    .get_velocity_ned = NavProvider_GetVelocityNED,
    .is_valid = NavProvider_IsValid
};

/* ... BLE_CommandCallback remains unchanged ... */

/* USER CODE END 0 */
```

### 4.5 Update Initialization Section

```c
/* USER CODE BEGIN 2 */

// Wait for USB CDC to be ready
HAL_Delay(2000);

// ===== SENSOR MANAGER INITIALIZATION =====
SensorManager_Config_t sensor_config = {
    .imu_odr_hz = 1000,
    .mag_odr_hz = 1000,
    .baro_odr_hz = 100,
    .highg_odr_hz = 50,
    .main_loop_hz = 500,
    .imu_accel_fs = 0,
    .imu_gyro_fs = 0,
    .use_mag = true,
    .use_high_g = false,
    .use_baro = true
};

if (SensorManager_Init(&sensor_config) == HAL_OK) {
    DebugPrint("Sensor Manager initialized\r\n");
} else {
    DebugPrint("ERROR: Sensor Manager init failed\r\n");
}

// ===== NAVIGATION MANAGER INITIALIZATION =====
// This internally initializes:
//   - Mahony filter (500Hz AHRS)
//   - Navigation EKF (50Hz position/velocity)
//   - Motion detector
//   - Coordinate transform
if (NavigationManager_Init()) {
    DebugPrint("Navigation Manager initialized (AHRS + EKF)\r\n");
} else {
    DebugPrint("ERROR: Navigation Manager init failed\r\n");
}

// ===== GPS MODULE INITIALIZATION =====
if (GPS_Init()) {
    DebugPrint("GPS module initialized\r\n");
} else {
    DebugPrint("ERROR: GPS init failed\r\n");
}

// ===== BLE MODULE INITIALIZATION =====
if (BLE_Init()) {
    ble_initialized = true;
    DebugPrint("BLE Module initialized successfully\r\n");
    BLE_RegisterCommandCallback(BLE_CommandCallback);
    BLE_SetDataTransmissionEnabled(true);
    BLE_SendResponse("=== FlightComputer - Full Navigation ===\r\n");
} else {
    DebugPrint("ERROR: BLE Module initialization failed!\r\n");
}

// ===== LORA MODULE INITIALIZATION =====
if (LoRa_Init()) {
    DebugPrint("LoRa module initialized\r\n");
} else {
    DebugPrint("ERROR: LoRa init failed\r\n");
}

// ===== TELEMETRY MANAGER INITIALIZATION =====
if (TelemetryManager_Init()) {
    DebugPrint("Telemetry Manager initialized\r\n");
    TelemetryManager_SetRateLimit(5.0f);  // 5Hz telemetry
} else {
    DebugPrint("ERROR: Telemetry Manager init failed\r\n");
}

// ===== DATA LOGGER INITIALIZATION =====
if (DataLogger_Init()) {
    DebugPrint("Data Logger initialized successfully\r\n");

    // Register navigation provider for EKF data logging
    DataLogger_RegisterNavProvider(&nav_provider);
    DebugPrint("Navigation provider registered (EKF integration)\r\n");

    // Display flash status
    char status_buffer[200];
    if (DataLogger_GetStatusString(status_buffer, sizeof(status_buffer))) {
        DebugPrint(status_buffer);
        DebugPrint("\r\n");
    }
} else {
    DebugPrint("ERROR: Data Logger initialization failed!\r\n");
}

// ===== USB COMMANDS INITIALIZATION =====
if (USBCommands_Init()) {
    DebugPrint("USB Commands initialized successfully\r\n");
} else {
    DebugPrint("ERROR: USB Commands initialization failed!\r\n");
}

// ===== FLIGHT MANAGER INITIALIZATION (LAST) =====
if (FlightManager_Init()) {
    DebugPrint("Flight Manager initialized - system ready\r\n");
} else {
    DebugPrint("ERROR: Flight Manager init failed\r\n");
}

DebugPrint("=== All systems initialized - entering main loop ===\r\n");

/* USER CODE END 2 */
```

### 4.6 Update Main Loop

```c
/* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    // ===== FLIGHT MANAGER UPDATE =====
    // This handles all subsystem timing internally:
    //   - Sensor reads at 500Hz
    //   - Navigation updates (AHRS 500Hz, EKF 50Hz)
    //   - GPS integration as available
    //   - Telemetry at 5Hz
    FlightManager_Update();

    // ===== CRITICAL REFACTORED INTEGRATION POINTS =====

    // Task 1: I2C arbiter watchdog (prevent deadlocks)
    if (I2C_DMA_Arbiter_Watchdog()) {
        DebugPrint("WARNING: I2C arbiter timeout recovery\r\n");
    }

    // Task 2: Data logger periodic update (metadata saves every 10s)
    DataLogger_Update();

    // ===== BLE MODULE UPDATE =====
    if (ble_initialized) {
        BLE_Update();
    }

    // ===== USB COMMANDS UPDATE =====
    USBCommands_Update();

    // Small delay to prevent CPU saturation
    HAL_Delay(1);
}
/* USER CODE END 3 */
```

### 4.7 Add QSPI DMA Callbacks (Critical for Data Logger)

```c
/* USER CODE BEGIN 4 */

/**
 * @brief QSPI Tx Transfer completed callback
 * @note Required for Task 2 - Data Logger DMA safety
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) {
    DataLogger_QSPI_WriteComplete();
}

/**
 * @brief QSPI error callback
 * @note Required for Task 2 - Data Logger DMA safety
 */
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi) {
    DataLogger_QSPI_WriteError();
}

/**
 * @brief UART Tx Transfer completed callback
 * @note Required for LoRa module transmission tracking
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Assuming USART2 is used for LoRa (verify your hardware!)
    if (huart->Instance == USART2) {
        LoRa_TransmissionComplete(true);
    }
}

/**
 * @brief UART error callback
 * @note Required for LoRa module error handling
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        LoRa_TransmissionComplete(false);
    }
}

/* USER CODE END 4 */
```

---

## 5. Compilation Issues to Resolve

### 5.1 SensorData_t Name Conflict

**Problem**: Both `sensor_system.h` and `navigation_manager.h` define `SensorData_t` but with different structures.

**Solution Options**:

**Option A - Rename in navigation_manager (RECOMMENDED)**:
Edit `navigation_manager.h` and `navigation_manager.c`:
- Change `SensorData_t` → `NavSensorData_t` throughout
- Update all function signatures

**Option B - Don't include sensor_system.h**:
Since we're not using `sensor_system`, we can modify `navigation_manager.c` to not include it and instead use your `sensor_manager` directly.

**Option C - Use namespace qualification**:
```c
typedef struct {
    // ... fields from sensor_system
} SensorSystem_SensorData_t;

typedef struct {
    // ... fields from navigation
} Navigation_SensorData_t;
```

### 5.2 Missing Sensor Driver Includes

The new modules reference `BMI088a.h`, `BMI088g.h`, `BMP390.h`, `MLX90393.h` which don't exist in your project.

**Solution**: Since we're NOT using `sensor_system`, we need to either:
1. Remove `sensor_system.c` from the build entirely
2. Or comment out the includes and sensor references in `sensor_system.c`

---

## 6. Build Configuration

### 6.1 Add New Source Files to Build

Ensure these files are included in your build:

**Navigation Layer**:
- `Src/navigation_ekf.c`
- `Src/navigation_manager.c`
- `Src/motion_detector.c`
- `Src/coord_transform.c`
- `Src/nav_matrix.c`
- `Src/nav_config.c`

**Telemetry Layer**:
- `Src/telemetry_manager.c`
- `Src/lora_module.c`
- `Src/rocket_telemetry.c`

**System Layer**:
- `Src/flight_manager.c`

**EXCLUDE** (sensor hardware mismatch):
- ~~`Src/sensor_system.c`~~ - DO NOT BUILD THIS FILE

### 6.2 Update Include Paths

Verify `-I` flags include:
- `-IInc`
- All necessary paths for your existing headers

---

## 7. Testing Strategy

### 7.1 Phase 1: Compilation Test

```bash
# Remove sensor_system from build if it causes issues
# Verify all new modules compile with existing code
```

### 7.2 Phase 2: Initialization Test

**Expected Debug Output**:
```
Sensor Manager initialized
Navigation Manager initialized (AHRS + EKF)
GPS module initialized
BLE Module initialized successfully
LoRa module initialized
Telemetry Manager initialized
Data Logger initialized successfully
Navigation provider registered (EKF integration)
Flight Manager initialized - system ready
=== All systems initialized - entering main loop ===
```

### 7.3 Phase 3: Navigation Verification

**Test Steps**:
1. Let system run for 60 seconds stationary
2. Check debug output for:
   - Motion state = STATIONARY
   - ZUPT (Zero Velocity Update) applied
   - GPS reference frame established
3. Move the system and verify:
   - Motion state changes to SLOW/FAST
   - Position/velocity NED values change
   - EKF updates successfully

### 7.4 Phase 4: Telemetry Test

**Test Steps**:
1. Connect LoRa receiver
2. Start data logger recording via BLE
3. Verify telemetry packets received at ~5Hz
4. Check packet CRC16 validity
5. Verify quaternion, position, velocity data populated

### 7.5 Phase 5: Data Logger Integration Test

**Test Steps**:
1. Record flight data for 30 seconds
2. Stop recording
3. Download via USB
4. Verify data records contain:
   - Valid quaternion from NavigationManager (not all zeros)
   - Position NED from EKF
   - Velocity NED from EKF
   - EKF debug data (innovation, Kalman gains)

---

## 8. Known Issues and Workarounds

### 8.1 Sensor System Module Incompatibility

**Issue**: `sensor_system` references missing sensor drivers
**Workaround**: Exclude from build, use existing `sensor_manager`
**Long-term**: Could adapt sensor_system to work with your hardware

### 8.2 SensorData_t Type Conflict

**Issue**: Name collision between modules
**Workaround**: Rename in `navigation_manager` or use namespace qualification
**Long-term**: Standardize on one sensor data structure

### 8.3 LoRa UART Assignment

**Issue**: Need to verify which UART is connected to LoRa module
**Workaround**: Check hardware schematic, update callbacks accordingly
**Current assumption**: USART2 for LoRa

### 8.4 Flight Manager Timing

**Issue**: `flight_manager` assumes specific sensor timing
**Workaround**: May need to adjust timing constants in `flight_manager.h`:
```c
#define FLIGHT_SENSOR_INTERVAL_MS    2      /* 500Hz sensor updates */
#define FLIGHT_TELEMETRY_INTERVAL_MS 200    /* 5Hz telemetry */
```

---

## 9. Critical Integration Checklist

Before testing, verify:

- [ ] `sensor_system.c` excluded from build (hardware mismatch)
- [ ] All new navigation modules added to build
- [ ] `SensorData_t` conflict resolved (rename or namespace)
- [ ] Navigation provider callbacks implemented in `main.c`
- [ ] `DataLogger_RegisterNavProvider()` called
- [ ] `I2C_DMA_Arbiter_Watchdog()` in main loop (Task 1)
- [ ] `DataLogger_Update()` in main loop (Task 2)
- [ ] `HAL_QSPI_TxCpltCallback()` implemented (Task 2)
- [ ] `HAL_QSPI_ErrorCallback()` implemented (Task 2)
- [ ] `HAL_UART_TxCpltCallback()` for LoRa implemented
- [ ] LoRa UART peripheral configured (verify USART2)
- [ ] Flight manager initialized LAST in initialization sequence
- [ ] All module Init() functions return success

---

## 10. Data Flow Summary

```
[Sensors] (Your existing hardware: ICM42688, MMC5983MA, BMP581, GPS)
    ↓
[SensorManager_ReadRaw()] (500Hz)
    ↓
[FlightManager_Update()] ← Orchestrates timing
    ├─→ [NavigationManager_UpdateAHRS()] (500Hz)
    │       ↓
    │   [Mahony Filter] → Attitude Quaternion
    │
    ├─→ [NavigationManager_UpdateNavigation()] (50Hz decimated)
    │       ↓
    │   [MotionDetector_Update()] → Motion state
    │       ↓
    │   [CoordTransform_BodyToNED()] → Accel NED
    │       ↓
    │   [NavEKF_Predict()] → Position/Velocity NED
    │
    ├─→ [NavigationManager_UpdateGPS()] (as available)
    │       ↓
    │   [NavEKF_UpdateGPS()] → GPS measurement update
    │   [NavEKF_UpdateGPSVelocity()] → Velocity measurement update
    │
    ├─→ [TelemetryManager_Update()] (5Hz)
    │       ↓
    │   [TelemetryManager_SendRocketTelemetry()]
    │       ↓
    │   [LoRa_SendData()] → Wireless transmission
    │
    └─→ [DataLogger_RecordData()] (100Hz when recording)
            ↓
        [NavigationProvider callbacks] → Get attitude/position/velocity
            ↓
        [QSPI Flash Write] → 192-byte records
```

---

## 11. Next Steps

1. **Resolve sensor_system conflict** - Exclude from build or adapt to your hardware
2. **Fix SensorData_t name collision** - Rename in navigation_manager
3. **Verify LoRa UART assignment** - Confirm hardware connection
4. **Update main.c** - Follow Section 4 exactly
5. **Build and test** - Follow Section 7 testing strategy
6. **Commit changes** - Document integration in commit message

---

## 12. Support and Troubleshooting

### Common Build Errors

**Error**: `undefined reference to BMI088a_init`
**Fix**: Exclude `sensor_system.c` from build

**Error**: `SensorData_t redefined`
**Fix**: Rename in navigation_manager or use namespace qualification

**Error**: `undefined reference to LoRa_Init`
**Fix**: Verify `lora_module.c` is in build, check USART configuration

### Runtime Issues

**Issue**: Navigation not updating
**Check**: Debug output for `NavigationManager_Init()` success

**Issue**: Telemetry not transmitting
**Check**: LoRa UART callbacks configured, `TelemetryManager_IsReady()` returns true

**Issue**: Data logger quaternion all zeros
**Check**: Navigation provider registered, `NavProvider_IsValid()` returns true

---

**Document Version**: 1.0
**Last Updated**: 2025-11-25
**Author**: Claude Code Integration Assistant
