# CLAUDE.md - FlightComputerRefactor AI Assistant Guide

**Last Updated:** 2025-12-03
**Repository:** FlightComputerRefactor
**Project Type:** Embedded Flight Controller for Model Rockets (STM32G4xx)

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Repository Structure](#repository-structure)
3. [Architecture & Design Patterns](#architecture--design-patterns)
4. [Key Modules & Dependencies](#key-modules--dependencies)
5. [Hardware Platform](#hardware-platform)
6. [Development Conventions](#development-conventions)
7. [Error Handling Patterns](#error-handling-patterns)
8. [Testing & Debugging](#testing--debugging)
9. [Current System State](#current-system-state)
10. [Working with This Codebase](#working-with-this-codebase)

---

## Project Overview

### What This Is

A production-grade embedded avionics system for high-altitude model rockets featuring:
- Multi-sensor fusion (IMU, Magnetometer, Barometer, GPS, High-G Accelerometer)
- Real-time attitude estimation (500 Hz AHRS using Mahony filter)
- Advanced navigation (6-state Extended Kalman Filter)
- Hardware-accelerated computation (CORDIC unit for trigonometry)
- Wireless telemetry (BLE, planned LoRa)
- High-speed data logging (QSPI Flash)

### Technology Stack

- **Language:** C (C99 standard)
- **Microcontroller:** STM32G4xx (ARM Cortex-M4 with FPU)
- **HAL:** STM32 HAL (Hardware Abstraction Layer)
- **Lines of Code:** ~17,333 lines across 78 files
- **Build System:** STM32CubeMX + IDE (Keil/STM32CubeIDE)

---

## Repository Structure

```
FlightComputerRefactor/
├── Inc/                      # Header files (41 files)
│   ├── Hardware Drivers
│   │   ├── icm42688.h       # Primary IMU (SPI)
│   │   ├── mmc5983ma.h      # Magnetometer (I2C)
│   │   ├── bmp581.h         # Barometer (I2C)
│   │   ├── h3lis331dl.h     # High-G Accelerometer (I2C)
│   │   └── gps_module.h     # GPS (UART)
│   │
│   ├── Navigation & Filtering
│   │   ├── mahony_filter.h          # AHRS orientation filter
│   │   ├── navigation_ekf.h         # 6-state Kalman filter
│   │   ├── navigation_manager.h     # Navigation coordinator
│   │   ├── motion_detector.h        # Motion classification
│   │   └── coord_transform.h        # Frame transformations
│   │
│   ├── Sensor Management
│   │   ├── sensor_manager.h         # Unified sensor interface
│   │   ├── sensor_adapter.h         # Hardware-to-nav adapter
│   │   └── i2c_dma_arbiter.h       # I2C DMA scheduler
│   │
│   ├── Communication
│   │   ├── ble_module.h            # Bluetooth (RN4871)
│   │   ├── lora_module.h           # LoRa (disabled - UART conflict)
│   │   └── usb_commands.h          # USB debugging interface
│   │
│   ├── Data Management
│   │   ├── data_logger.h           # QSPI Flash logging
│   │   └── rocket_telemetry.h      # Telemetry structures
│   │
│   ├── Utilities
│   │   ├── error_codes.h           # Error handling patterns
│   │   ├── cordic_math.h           # Hardware math acceleration
│   │   ├── nav_matrix.h            # Matrix operations
│   │   └── debug_utils.h           # Debug helpers
│   │
│   └── STM32 Configuration
│       ├── main.h
│       ├── stm32g4xx_hal_conf.h
│       └── stm32g4xx_it.h
│
└── Src/                      # Implementation files (37 files)
    ├── main.c                # System initialization & main loop
    └── [corresponding .c files for headers above]
```

### Key File Sizes (by complexity)

| File | Size | Purpose |
|------|------|---------|
| `navigation_ekf.c` | 45 KB | Extended Kalman Filter implementation |
| `navigation_manager.c` | 33 KB | AHRS + EKF coordinator |
| `ble_module.c` | 27 KB | Bluetooth communication protocol |
| `gps_module.c` | 26 KB | GPS UBX protocol parsing |
| `data_logger.c` | 23 KB | Flash data logging system |
| `sensor_manager.c` | 22 KB | Multi-sensor acquisition |

---

## Architecture & Design Patterns

### Layered Architecture

```
┌─────────────────────────────────────────┐
│  Application Layer                      │
│  (Data Logger, BLE Commands, USB)       │
├─────────────────────────────────────────┤
│  Coordination Layer                     │
│  (FlightManager, TelemetryManager)      │  ← Currently disabled
├─────────────────────────────────────────┤
│  Navigation Processing Layer            │
│  (NavigationManager, AHRS, EKF)         │
├─────────────────────────────────────────┤
│  Sensor Integration Layer               │
│  (SensorAdapter, SensorManager)         │
├─────────────────────────────────────────┤
│  Hardware Driver Layer                  │
│  (IMU, Mag, Baro, GPS)                  │
├─────────────────────────────────────────┤
│  Communication HAL Layer                │
│  (I2C DMA Arbiter, UART DMA, SPI DMA)   │
├─────────────────────────────────────────┤
│  STM32 Hardware Abstraction             │
│  (CORDIC, FMAC, DMA, GPIO)              │
└─────────────────────────────────────────┘
```

### Core Design Patterns

#### 1. **Adapter Pattern** - Sensor Integration
```c
// Hardware layer provides raw data
SensorManager_RawData_t → SensorAdapter_Convert() → NavSensorData_t
// Navigation layer works with standardized format
```

**Purpose:** Decouples sensor hardware from navigation algorithms

#### 2. **Provider Pattern** - Navigation Callbacks
```c
// Navigation data exposed via callback functions
typedef bool (*NavProvider_GetQuaternion_t)(float quat[4]);
typedef bool (*NavProvider_GetPositionNED_t)(float pos[3]);
// Used by DataLogger to record navigation state
```

**Purpose:** Allows modules to access navigation data without tight coupling

#### 3. **DMA-First Communication**
```c
// All peripherals use DMA with polling
HAL_I2C_Mem_Read_DMA() → Poll busy flag → Callback on complete
```

**Purpose:** Non-blocking I/O for real-time operation

#### 4. **Multi-Filter Fusion Architecture**

```
Raw Sensors (1 kHz IMU, 240 Hz Baro, 1 Hz GPS)
     ↓
Mahony AHRS (500 Hz)  →  Attitude Quaternion
     ↓
Extended Kalman Filter (50 Hz decimated)  →  Position & Velocity
     ↓
Data Logger (500 Hz sensor, 50 Hz nav state)
```

**Purpose:** Hierarchical sensor fusion with frequency decimation

#### 5. **FCFS Resource Scheduler** - I2C DMA Arbiter
```c
// Three I2C sensors share one DMA channel
Request → Queue → First-Come-First-Served → Execute → Release
```

**Purpose:** Prevents DMA collisions on shared I2C bus

---

## Key Modules & Dependencies

### Critical Data Flow

```
┌──────────────┐
│ ICM42688 IMU │ (SPI, 1 kHz)
└──────┬───────┘
       │
┌──────▼────────┐    ┌─────────────────┐
│ MMC5983MA Mag │───▶│ SensorManager   │
└───────────────┘    │ (Raw Data)      │
                     └────────┬────────┘
┌──────────────┐             │
│ BMP581 Baro  │────────────▶│
└──────────────┘             │
                             ▼
                     ┌───────────────┐
                     │SensorAdapter  │ (Unit conversion)
                     └───────┬───────┘
                             │
                     ┌───────▼──────────────┐
                     │ MahonyFilter (AHRS)  │ @ 500 Hz
                     │ Output: Quaternion   │
                     └───────┬──────────────┘
                             │
                     ┌───────▼──────────────┐
      ┌─────────────│ NavigationEKF        │ @ 50 Hz
      │             │ State: [Pn,Pe,Pd,    │
┌─────▼─────┐       │        Vn,Ve,Vd]     │
│GPS Module │──────▶│ Measurements: GPS    │
└───────────┘       └───────┬──────────────┘
 (UART DMA)                 │
                            │
                    ┌───────▼────────┐
                    │ DataLogger     │
                    │ (QSPI Flash)   │
                    └────────────────┘
```

### Module Dependencies (Must Understand)

| Module | Depends On | Provides To |
|--------|-----------|-------------|
| `SensorManager` | Hardware drivers, I2C Arbiter | `SensorAdapter` |
| `SensorAdapter` | `SensorManager` | `NavigationManager` |
| `MahonyFilter` | `SensorAdapter`, CORDIC | `NavigationManager`, `NavigationEKF` |
| `NavigationEKF` | `MahonyFilter`, `GPS` | `NavigationManager` |
| `NavigationManager` | All navigation modules | `DataLogger`, Application |
| `DataLogger` | `NavigationManager` (via callbacks) | Flash storage |

### Initialization Order (CRITICAL)

```c
// From main.c - must follow this sequence:
1. HAL_Init() + SystemClock_Config()
2. Peripheral Init (GPIO, DMA, I2C, SPI, UART, CORDIC, FMAC, QSPI)
3. I2C_DMA_Arbiter_Init()
4. SensorManager_Init()  // Initialize all sensors
5. GPS_Init()
6. NavigationManager_Init()  // Initializes AHRS + EKF internally
7. DataLogger_Init()  // Register navigation callbacks
8. BLE_Init()
9. Main loop starts
```

**NEVER** change this order without understanding dependencies!

---

## Hardware Platform

### STM32G4xx Microcontroller

- **Core:** ARM Cortex-M4 with FPU (Floating Point Unit)
- **Clock:** Configured via `SystemClock_Config()`
- **Special Hardware Used:**
  - **CORDIC:** Hardware trigonometry (atan2, sin, cos, sqrt, asin)
  - **FMAC:** Floating-point MAC (initialized but not heavily used)
  - **DMA:** Extensive use for I2C, SPI, UART, QSPI

### Sensors

| Sensor | Interface | Purpose | Update Rate | Driver File |
|--------|-----------|---------|-------------|-------------|
| ICM-42688-P | SPI + DMA | 6-DOF IMU (accel + gyro) | 1 kHz | `icm42688.c` |
| MMC5983MA | I2C + DMA | 3-axis magnetometer | 1 kHz | `mmc5983ma.c` |
| BMP581 | I2C + DMA | Barometer (pressure + temp) | 240 Hz | `bmp581.c` |
| H3LIS331DL | I2C + DMA | High-G accelerometer (±400g) | 400 Hz | `h3lis331dl.c` |
| u-blox GPS | UART3 + DMA | Position + Velocity | 1 Hz | `gps_module.c` |

### Communication Peripherals

| Peripheral | Purpose | Status | Driver File |
|------------|---------|--------|-------------|
| USART1 + DMA | BLE (RN4871) | Active | `ble_module.c` |
| USART3 + DMA | GPS | Active | `gps_module.c` |
| USB CDC | Debug commands | Active | `usb_commands.c` |
| UART1 | LoRa telemetry | Disabled (conflict) | `lora_module.h` |

### Data Storage

- **QSPI Flash:** 4 MB external flash for flight data logging
  - Record size: 192 bytes (optimized for QSPI page writes)
  - Logging rate: 500 Hz sensor data + 50 Hz navigation state
  - Metadata stored in last 4 KB sector

---

## Development Conventions

### Naming Conventions

```c
// Module prefix followed by function name (PascalCase)
HAL_StatusTypeDef SensorManager_Init(void);
bool NavigationManager_Update(uint32_t timestamp_ms);
DataLogger_ErrorCode_t DataLogger_WriteRecord(const LogRecord_t* record);

// Private/static functions use lowercase with underscores
static void calculate_covariance_matrix(void);
static bool validate_sensor_data(void);

// Constants: UPPERCASE with underscores
#define MAHONY_KP_DEFAULT 2.0f
#define FLIGHT_SENSOR_INTERVAL_MS 2

// Typedefs: End with _t
typedef struct { ... } SensorData_t;
typedef enum { ... } FlightPhase_t;
```

### File Organization

Every `.c` file follows this structure:
```c
/* Header comment with @file, @brief, @note */

/* Includes */
#include "header.h"
#include <stdint.h>

/* Private defines */
#define TIMEOUT_MS 100

/* Private typedefs */
typedef struct { ... } PrivateState_t;

/* Private variables */
static PrivateState_t state;

/* Private function prototypes */
static void helper_function(void);

/* Public function implementations */
HAL_StatusTypeDef Module_PublicFunction(void) { ... }

/* Private function implementations */
static void helper_function(void) { ... }
```

### Documentation Standards

**ALWAYS include for public functions:**
```c
/**
 * @brief Clear, concise description
 * @param input_name Description with units (e.g., "Angle in radians")
 * @retval HAL_OK on success
 * @retval HAL_ERROR on failure
 * @note Important usage notes
 * @warning Critical warnings about misuse
 */
```

**Units MUST be specified:**
- Angles: Always clarify degrees vs radians
- Acceleration: m/s² or g
- Angular velocity: rad/s or °/s
- Magnetic field: µT or gauss
- Pressure: Pa or hPa
- Temperature: °C or K

### Code Style

```c
// Braces: K&R style (opening brace on same line)
if (condition) {
    do_something();
}

// Function definitions: Opening brace on same line
void Module_Function(void) {
    // implementation
}

// Pointer declarations: Asterisk with type
uint8_t* buffer;    // Preferred
SensorData_t* data; // Preferred

// Boolean logic: Use explicit comparisons for clarity
if (sensor_valid == true) { ... }   // Clear intent
if (error_code != HAL_OK) { ... }   // Explicit error check

// Use stdint types
uint8_t, uint16_t, uint32_t, int8_t, etc.
// NOT: unsigned char, unsigned int, etc.
```

---

## Error Handling Patterns

### Standard Return Types

1. **HAL Functions:** Return `HAL_StatusTypeDef`
   ```c
   HAL_StatusTypeDef Module_Init(void) {
       if (hardware_error) return HAL_ERROR;
       if (resource_busy) return HAL_BUSY;
       if (timeout) return HAL_TIMEOUT;
       return HAL_OK;
   }
   ```

2. **Boolean Functions:** Return `bool`
   ```c
   bool Module_IsDataValid(void) {
       return (data_timestamp + TIMEOUT_MS) > HAL_GetTick();
   }
   ```

3. **Data Retrieval:** Use output pointer + status return
   ```c
   bool Module_GetData(DataStruct_t* output) {
       if (output == NULL) return false;
       if (!data_valid) return false;
       *output = internal_data;
       return true;
   }
   ```

4. **Module-Specific Errors:** Use enums (see `error_codes.h`)
   ```c
   DataLogger_ErrorCode_t DataLogger_WriteRecord(...) {
       if (!initialized) return LOGGER_NOT_INITIALIZED;
       if (flash_full) return LOGGER_FLASH_FULL;
       return LOGGER_OK;
   }
   ```

### Critical Error Patterns

#### Pattern 1: DMA Busy Protection
```c
// ALWAYS check busy flag before starting DMA
uint32_t start = HAL_GetTick();
while (dma_busy_flag) {
    if ((HAL_GetTick() - start) > TIMEOUT_MS) {
        return HAL_TIMEOUT;
    }
}
dma_busy_flag = true;
HAL_I2C_Mem_Read_DMA(...);
```

#### Pattern 2: Shared Resource Protection
```c
// Use interrupt disable for atomic operations
__disable_irq();
if (resource_busy) {
    __enable_irq();
    return HAL_BUSY;
}
resource_busy = true;
__enable_irq();
```

#### Pattern 3: NULL Pointer Checks
```c
// ALWAYS validate input pointers
HAL_StatusTypeDef Module_Process(const Data_t* input, Result_t* output) {
    if (input == NULL || output == NULL) {
        return HAL_ERROR;
    }
    // process...
}
```

#### Pattern 4: Validity Flags
```c
// Check validity before using sensor data
SensorManager_RawData_t raw_data;
SensorManager_GetRawData(&raw_data);

if (raw_data.imu_valid) {
    process_imu_data(&raw_data.imu);
} else {
    // Handle missing IMU data gracefully
}
```

### Error Recovery Philosophy

This codebase follows **graceful degradation**:
- System continues operation even if sensors fail
- Missing data marked as invalid, not fatal error
- Critical errors logged but don't crash system
- Timeouts prevent infinite loops

---

## Testing & Debugging

### No Automated Testing Infrastructure

**Current State:**
- No unit test framework
- No integration test suite
- Testing relies on hardware integration

### Manual Testing Interfaces

#### 1. BLE Command Interface
```c
// Send commands via Bluetooth (USART1)
// Examples:
"STATUS"     - Get system status
"SENSORS"    - Sensor diagnostic dump
"NAV"        - Navigation state
"FLASH"      - Flash memory info
"GPS"        - GPS status
```

#### 2. USB Command Interface
```c
// Send commands via USB CDC
// Same command set as BLE
// Implemented in usb_commands.c
```

#### 3. Debug Utilities
```c
// Use debug_utils.h for printf-style debugging
#include "debug_utils.h"
DebugPrint("Sensor value: %f\n", value);
```

### Diagnostic Functions

Every major module provides diagnostics:
```c
// Get human-readable status strings
char buffer[256];
NavigationManager_GetDiagnosticsString(buffer, sizeof(buffer));
TelemetryManager_GetDiagnosticsString(buffer, sizeof(buffer));
SensorManager_GetStatus(&status);

// Get statistics
MotionState_t state;
uint32_t update_count, transition_count;
MotionDetector_GetStats(&detector, &update_count, &transition_count);
```

### Adding New Debug Output

```c
// 1. Include debug utilities
#include "debug_utils.h"

// 2. Use DebugPrint for USB output
void Module_DebugDump(void) {
    DebugPrint("Module State: %d\n", state);
    DebugPrint("Error Count: %lu\n", error_count);
}

// 3. Respond to BLE/USB commands
static void BLE_CommandCallback(const char* command) {
    if (strcmp(command, "MYMODULE") == 0) {
        Module_DebugDump();
    }
}
```

---

## Current System State

### Active Features (Fully Operational)

✅ **Sensor Acquisition**
- ICM-42688 IMU (1 kHz)
- MMC5983MA Magnetometer (1 kHz)
- BMP581 Barometer (240 Hz)
- H3LIS331DL High-G Accelerometer (400 Hz)
- GPS (1 Hz)

✅ **Navigation**
- Mahony AHRS filter (500 Hz)
- 6-state Extended Kalman Filter (50 Hz)
- Motion detection with ZUPT
- Coordinate transformations
- Gravity compensation

✅ **Data Logging**
- QSPI Flash data logging (500 Hz)
- 192-byte optimized record structure
- Metadata management
- Circular buffer implementation

✅ **Communication**
- BLE wireless (RN4871)
- USB command interface
- GPS UBX protocol parsing

### Disabled Features (In Development)

❌ **LoRa Telemetry**
- Reason: UART1 hardware conflict with BLE
- File: `lora_module.h` (header exists, not integrated)
- Status: Commented out in `main.c:41`

❌ **TelemetryManager**
- Reason: Depends on LoRa module
- File: `telemetry_manager.h`
- Status: Commented out in `main.c:40`

❌ **FlightManager**
- Reason: Top-level orchestrator requires telemetry
- File: `flight_manager.h`
- Status: Commented out in `main.c:42`

### Known Limitations

1. **UART Resource Conflict:**
   - BLE and LoRa cannot operate simultaneously
   - Need hardware rework or UART multiplexing solution

2. **No Build System:**
   - No Makefile or CMake configuration
   - Relies on STM32CubeMX project file (not in repo)

3. **Single Commit History:**
   - Repository uploaded as bulk "Add files via upload"
   - No incremental commit history for reference

4. **Limited Testing:**
   - No automated test infrastructure
   - Manual hardware testing only

---

## Working with This Codebase

### Before Making Changes

1. **Read `error_codes.h` first** - Understand error handling patterns
2. **Check initialization order** in `main.c` - Dependencies are strict
3. **Understand DMA busy flags** - Most peripheral errors are DMA-related
4. **Review module's header file** - All public APIs documented there
5. **Check for hardware dependencies** - CORDIC, DMA, peripheral availability

### Common Tasks

#### Adding a New Sensor

1. Create driver files: `Inc/new_sensor.h`, `Src/new_sensor.c`
2. Add to `SensorManager`:
   - Include in `sensor_manager.h`
   - Initialize in `SensorManager_Init()`
   - Add to acquisition loop in `SensorManager_Update()`
   - Add validity flag to `SensorManager_RawData_t`
3. Update `SensorAdapter` if needed for navigation integration
4. Add error codes to `error_codes.h`

#### Adding a New Navigation Algorithm

1. Create module files: `Inc/algorithm.h`, `Src/algorithm.c`
2. Follow existing patterns (see `mahony_filter.c` as example):
   - Init function
   - Update function (takes sensor data, returns status)
   - Getter functions for output state
   - Diagnostics function
3. Integrate into `NavigationManager`
4. Add to data logger record if needed

#### Debugging Sensor Issues

1. Check DMA busy flags:
   ```c
   // I2C sensors
   I2C_DMA_Arbiter_IsBusy(&result);

   // SPI sensors
   if (spi_dma_busy_flag) { /* still busy */ }
   ```

2. Verify initialization order in `main.c`

3. Use BLE commands to check sensor status:
   ```
   "SENSORS" - Full sensor diagnostic
   "STATUS"  - System-wide status
   ```

4. Check validity flags:
   ```c
   if (!raw_data.imu_valid) {
       // IMU data not available
   }
   ```

#### Modifying the Main Loop

**CRITICAL:** The main loop timing is carefully tuned:

```c
// main.c main loop structure
while (1) {
    current_time = HAL_GetTick();

    // 500 Hz sensor update
    if ((current_time - last_sensor_update) >= 2) {
        SensorManager_Update();
        NavigationManager_Update(current_time);
        DataLogger_LogCurrentState();
        last_sensor_update = current_time;
    }

    // 5 Hz GPS update
    if ((current_time - last_gps_update) >= 200) {
        GPS_Process();
        last_gps_update = current_time;
    }

    // 1 Hz BLE status update
    if ((current_time - last_ble_update) >= 1000) {
        BLE_SendStatus();
        last_ble_update = current_time;
    }
}
```

**Do not add blocking operations in the main loop!**

### Performance Considerations

1. **CPU-Intensive Operations:**
   - Use CORDIC for trig functions (faster than software math library)
   - Prefer fixed-point math for simple scaling
   - Avoid `printf()` in high-frequency loops

2. **Memory:**
   - Stack: Limited in embedded systems
   - Heap: Avoid `malloc()`/`free()` in flight code
   - Use static allocation for large buffers

3. **DMA:**
   - Always check busy flags before starting new transfer
   - Use appropriate timeout values (typically 100ms)
   - Be aware of DMA channel conflicts

### Best Practices for AI Assistants

#### DO:
- ✅ Read the module's header file before modifying implementation
- ✅ Follow existing naming conventions exactly
- ✅ Add Doxygen-style documentation to all new functions
- ✅ Use module-specific error codes from `error_codes.h`
- ✅ Test initialization order if adding new modules
- ✅ Add validity flags for new data sources
- ✅ Use `_Static_assert()` for compile-time checks
- ✅ Specify units in all comments (radians, m/s, etc.)
- ✅ Check for NULL pointers before dereferencing
- ✅ Use DMA busy protection patterns

#### DON'T:
- ❌ Add `malloc()`/`free()` calls
- ❌ Create blocking operations in main loop
- ❌ Modify initialization order without understanding dependencies
- ❌ Use floating-point printf formats (use integer scaling)
- ❌ Add external library dependencies
- ❌ Create new error handling patterns (use existing ones)
- ❌ Assume sensor data is always valid (check flags)
- ❌ Start DMA transfers without checking busy flags
- ❌ Use degrees when functions expect radians (or vice versa)
- ❌ Break the layered architecture (bypass abstraction layers)

### Code Review Checklist

Before committing changes:

- [ ] All public functions have Doxygen documentation
- [ ] Units specified for all physical quantities
- [ ] NULL pointer checks on all input pointers
- [ ] DMA busy flags checked before transfers
- [ ] Error codes follow module conventions
- [ ] Validity flags added for new data sources
- [ ] No blocking operations in high-frequency paths
- [ ] Initialization order preserved
- [ ] No memory leaks (no dynamic allocation)
- [ ] Hardware dependencies documented

---

## Quick Reference

### File Location Guide

| Need to... | Look in... |
|------------|-----------|
| Add sensor driver | `Inc/` + `Src/` + integrate into `sensor_manager.c` |
| Modify navigation | `navigation_manager.c`, `mahony_filter.c`, `navigation_ekf.c` |
| Change logging format | `data_logger.h` (update `LogRecord_t`) |
| Add BLE command | `ble_module.c` (`BLE_ProcessCommand()`) |
| Add USB command | `usb_commands.c` |
| Fix initialization | `main.c` (`main()` function) |
| Update error codes | `error_codes.h` |
| Hardware math (trig) | `cordic_math.c` |
| Coordinate transforms | `coord_transform.c` |

### Key Constants

```c
// Timing
#define FLIGHT_SENSOR_INTERVAL_MS 2       // 500 Hz sensor loop
#define FLIGHT_TELEMETRY_INTERVAL_MS 200  // 5 Hz telemetry
#define FLIGHT_GPS_INTERVAL_MS 5000       // 0.2 Hz GPS (if needed)

// AHRS
#define MAHONY_SAMPLE_FREQ_DEFAULT 500.0f // Hz
#define MAHONY_KP_DEFAULT 2.0f            // Proportional gain
#define MAHONY_KI_DEFAULT 0.01f           // Integral gain

// EKF
#define EKF_UPDATE_RATE_HZ 50             // Decimated from 500 Hz AHRS

// Data Logger
#define RECORD_SIZE 192                   // Bytes per record
#define METADATA_SECTOR_SIZE 4096         // Last 4KB of flash
```

### Critical Function References

```c
// Initialization (must call in this order)
HAL_Init();
I2C_DMA_Arbiter_Init();
SensorManager_Init();
NavigationManager_Init();  // Inits AHRS + EKF internally
DataLogger_Init();

// Main loop (500 Hz)
SensorManager_Update();
NavigationManager_Update(timestamp_ms);
DataLogger_LogCurrentState();

// Data retrieval
bool NavigationManager_GetQuaternion(float quat[4]);
bool NavigationManager_GetPositionNED(float pos[3]);
bool NavigationManager_GetVelocityNED(float vel[3]);

// Diagnostics
NavigationManager_GetDiagnosticsString(buffer, size);
SensorManager_GetStatus(&status);
```

---

## Version History

| Date | Changes |
|------|---------|
| 2025-12-03 | Initial CLAUDE.md creation based on repository analysis |

---

## Additional Resources

- **STM32G4 Reference Manual:** Details on CORDIC, FMAC, DMA
- **ICM-42688-P Datasheet:** Primary IMU specifications
- **UBX Protocol Specification:** GPS message format
- **RN4871 Bluetooth Module:** BLE command reference

---

**Note for AI Assistants:** This document represents the current state as of 2025-12-03. Always verify critical details by reading the actual source files, as the codebase may evolve. When in doubt, check `error_codes.h` for error handling patterns and `main.c` for initialization order.
