# FlightComputerRefactor - AI Assistant Guide

## Project Overview

**Project Type:** Embedded systems firmware for model rocket flight computer
**Target Hardware:** STM32G4 microcontroller
**Primary Purpose:** Real-time sensor data acquisition, attitude estimation, GPS tracking, and data logging for model rocketry applications
**Programming Language:** C (embedded C for STM32)
**Total Lines of Code:** ~11,265 lines

## Hardware Architecture

### Microcontroller
- **MCU:** STM32G4xx series (ARM Cortex-M4 with FPU)
- **HAL:** STM32 HAL (Hardware Abstraction Layer)
- **Key Hardware Features:**
  - CORDIC hardware accelerator (for trigonometric calculations)
  - FMAC (Filter Math Accelerator)
  - Multi-channel DMA
  - Multiple I2C, SPI, USART peripherals
  - Quad SPI interface
  - USB device interface
  - 12-bit ADC

### Sensors and Peripherals

**IMU (Inertial Measurement Unit):**
- **ICM42688** (SPI) - Primary 6-axis accelerometer + gyroscope
  - Configured for high-speed operation (500Hz+ for Mahony filter)
  - Interrupt-driven data ready signal (INT_IMU on PC14)

**Magnetometer:**
- **MMC5983MA** (I2C1) - 3-axis magnetometer
  - 1kHz update rate for Mahony filter fusion
  - Highest priority in I2C DMA arbiter
  - Interrupt on INT_MAG (PB7)

**High-G Accelerometer:**
- **H3LIS331DL** (I2C1) - High-G shock detection sensor
  - 400Hz update rate
  - Lowest priority in I2C DMA arbiter
  - Interrupt on INT_ACC (PA13)

**Barometer:**
- **BMP581** (I2C1) - High precision pressure/temperature sensor
  - 100Hz decimated update rate
  - Medium priority in I2C DMA arbiter
  - Interrupt on INT_BMP (PB6)

**GPS Module:**
- **u-blox GPS** (USART3) - Enhanced UBX protocol
  - Provides position, velocity, DOP values, covariance data
  - DMA circular buffer with idle line detection
  - Time pulse on TIMEPULSE (PC15)

**Bluetooth Module:**
- **RN4871 BLE** (USART1) @ 115200 baud
  - DMA circular buffer with idle line detection
  - Control pins: RST_BT (PC6), CONFIG (PB15), LPM (PA8)
  - Command-based interface for remote control

**Flash Storage:**
- **Quad SPI Flash** - Data logging storage
  - 192-byte optimized record structure
  - Metadata stored in last 4KB sector
  - Supports ~21K records (4MB flash)

**USB Interface:**
- **USB Device** - Command and control interface
  - CDC (Communications Device Class)
  - Command processing for start/stop/status/download

**LEDs:**
- LED1 (PA4), LED2 (PA5), LED3 (PC4) - Status indicators

## Software Architecture

### Directory Structure

```
FlightComputerRefactor/
├── Inc/          # Header files (.h)
├── Src/          # Source files (.c)
└── .git/         # Git repository
```

**Note:** This is a refactored codebase. Original implementations may have used polling, but current architecture emphasizes DMA-based operations for efficiency.

### Core Modules

#### 1. Sensor Management Layer

**sensor_manager.{c,h}**
- Unified sensor data acquisition orchestrator
- Coordinates all sensor reads via DMA
- Provides raw and scaled data structures
- Optimized for 500Hz+ Mahony filter operation
- Key structures:
  - `SensorManager_RawData_t` - Raw sensor readings with validity flags
  - `SensorManager_ScaledData_t` - Converted physical units
  - `SensorManager_Config_t` - ODR and full-scale configurations

**sensor_driver_common.{c,h}**
- Common utilities for I2C sensor drivers
- Eliminates code duplication
- Provides `I2C_Sensor_ReadRegisters_DMA()` with retry logic
- Handles DMA busy states and timeouts

**i2c_dma_arbiter.{c,h}**
- **Critical Component:** Priority-based I2C bus arbitration
- Manages multiple I2C sensors sharing single bus (I2C1)
- Priority levels (high to low):
  1. MAG (magnetometer) - 1kHz for Mahony filter
  2. BARO (barometer) - 100Hz decimated
  3. HIGHG (high-G accelerometer) - 400Hz shock detection
- Prevents bus conflicts and ensures deterministic behavior
- Tracks statistics for debugging

#### 2. Sensor Drivers

Each sensor has dedicated driver:

**icm42688.{c,h}** - Primary IMU (SPI-based)
- Direct SPI access, no arbiter needed
- Highest bandwidth sensor
- Accel + Gyro data

**mmc5983ma.{c,h}** - Magnetometer (I2C via arbiter)
- Uses I2C DMA arbiter with highest priority
- Critical for heading estimation

**h3lis331dl.{c,h}** - High-G accelerometer (I2C via arbiter)
- Lowest priority, designed for shock detection
- ±400G full scale range

**bmp581.{c,h}** - Barometer (I2C via arbiter)
- Medium priority
- Provides pressure/temperature for altitude estimation

**gps_module.{c,h}** - GPS receiver (UART with DMA)
- Enhanced UBX protocol support
- Comprehensive data extraction:
  - Position (lat/lon/alt)
  - Velocity components (NED frame)
  - Accuracy estimates (hAcc, vAcc, sAcc)
  - Covariance matrices
  - DOP values (PDOP, HDOP, VDOP, GDOP, etc.)
  - Spoofing/jamming detection
- DMA circular buffer for efficient data reception
- Self-managing with automatic parsing

#### 3. Attitude Estimation

**mahony_filter.{c,h}** - AHRS (Attitude and Heading Reference System)
- Mahony complementary filter algorithm
- Fuses gyro + accel + magnetometer data
- Outputs quaternion orientation in NED (North-East-Down) frame
- Hardware-accelerated using STM32G4 CORDIC
- Key features:
  - 500Hz+ update rate
  - Configurable gains (Kp, Ki)
  - Launch pad initialization (X-axis up orientation)
  - Coordinate transformations (sensor frame ↔ NED frame)
- Functions:
  - `Mahony_Update()` - Full 9-DOF update (gyro + accel + mag)
  - `Mahony_UpdateIMU()` - 6-DOF update (gyro + accel only)
  - `Mahony_GetQuaternion()` - Get attitude quaternion
  - `Mahony_GetEulerAngles()` - Get roll/pitch/yaw

**cordic_math.{c,h}** - Hardware CORDIC acceleration
- STM32G4 CORDIC peripheral wrapper
- Fast trigonometric functions (sin, cos, atan2)
- Used by Mahony filter for performance

#### 4. Data Logging

**data_logger.{c,h}** - Flash-based data recording
- **Record Size:** Exactly 192 bytes (compile-time verified)
- Optimized structure including:
  - Sensor data (accel, gyro, raw readings)
  - GPS data (position, velocity, NED components)
  - Navigation data (quaternion, position/velocity NED)
  - EKF diagnostics (innovation, Kalman gains, uncertainties)
  - Motion detection state
- Flash layout:
  - Data area: 0 to (4MB - 4KB)
  - Metadata sector: Last 4KB
- Metadata versioning and checksums
- Status tracking: IDLE, RECORDING, DOWNLOADING, ERROR
- Integration with SensorManager, Mahony filter, GPS

#### 5. Communication Interfaces

**ble_module.{c,h}** - Bluetooth Low Energy (RN4871)
- DMA circular buffer with idle line detection
- Command-based interface
- Built-in commands: start, stop, status, help, erase
- Custom command callback support
- Self-managing with 2s command timeout
- Hardware control (reset, config, low-power mode)

**usb_commands.{c,h}** - USB CDC interface
- Command processing for PC connectivity
- Built-in commands similar to BLE
- Custom command callback support
- Response buffering

**debug_utils.{c,h}** - Debug output utilities
- Likely uses UART or USB for printf-style debugging

#### 6. Peripheral Drivers (HAL-generated)

These are STM32CubeMX-generated initialization code:
- **i2c.{c,h}** - I2C peripheral initialization
- **spi.{c,h}** - SPI peripheral initialization
- **usart.{c,h}** - USART/UART initialization
- **dma.{c,h}** - DMA controller initialization
- **gpio.{c,h}** - GPIO pin configuration
- **adc.{c,h}** - ADC initialization
- **quadspi.{c,h}** - Quad SPI flash interface
- **cordic.{c,h}** - CORDIC peripheral initialization
- **fmac.{c,h}** - FMAC peripheral initialization

**stm32g4xx_hal_msp.{c}** - HAL MSP (MCU Support Package) callbacks
**stm32g4xx_it.{c,h}** - Interrupt handlers
**system_stm32g4xx.c** - System initialization
**syscalls.c**, **sysmem.c** - Standard library support

#### 7. Utilities

**i2c_scanner.{c,h}** - I2C bus device scanner (debugging tool)

### Main Application

**main.{c,h}** - Application entry point
- System initialization sequence
- Main loop structure:
  1. Sensor updates (via SensorManager)
  2. Mahony filter update
  3. GPS update
  4. BLE update
  5. USB command update
  6. Data logging
- Interrupt callbacks routing

## Key Design Patterns and Conventions

### 1. DMA-First Architecture

**All high-bandwidth sensors use DMA:**
- I2C sensors use DMA via arbiter
- SPI sensor (ICM42688) uses DMA
- UART (GPS, BLE) uses DMA with circular buffers
- **Benefits:** Reduces CPU load, enables high sample rates

### 2. Interrupt-Driven Data Ready

**Sensors signal data ready via GPIO interrupts:**
- IMU: INT_IMU (PC14) → EXTI15_10_IRQn
- Magnetometer: INT_MAG (PB7) → EXTI9_5_IRQn
- Accelerometer: INT_ACC (PA13) → EXTI15_10_IRQn
- Barometer: INT_BMP (PB6) → EXTI9_5_IRQn

**Pattern:**
1. Sensor signals data ready via interrupt
2. ISR sets flag or triggers DMA read
3. Main loop processes when DMA completes

### 3. Arbiter Pattern for Shared Resources

**I2C DMA Arbiter is critical:**
- Multiple sensors on one I2C bus
- Priority-based scheduling prevents conflicts
- High-priority sensors (MAG) can retry if bus busy
- Low-priority sensors (HIGHG, BARO) fail gracefully

### 4. Circular Buffers for Streaming Data

**GPS and BLE use DMA circular buffers:**
- UART DMA in circular mode
- Idle line detection for message boundaries
- Parser processes complete messages
- No data loss at high rates

### 5. Coordinate Frame Conventions

**NED Frame (North-East-Down):**
- Standard aerospace convention
- Mahony filter outputs quaternion in NED
- GPS velocity in NED components
- Coordinate transformations available:
  - `Mahony_TransformIMUToNED()`
  - `Mahony_TransformMagToNED()`

**Sensor Frames:**
- Each sensor has its own body frame
- Transformations handle alignment
- Magnetometer has specific alignment compensation

### 6. Data Structures with Validity Flags

**Example from SensorManager_RawData_t:**
```c
uint8_t imu_valid    : 1;
uint8_t mag_valid    : 1;
uint8_t high_g_valid : 1;
uint8_t baro_valid   : 1;
```
- Always check validity before using data
- Enables graceful degradation if sensor fails

### 7. Compile-Time Assertions

**Example from data_logger.h:**
```c
_Static_assert(sizeof(DataRecord_t) == RECORD_SIZE, "DataRecord_t must be exactly 192 bytes");
```
- Ensures structure sizes are correct
- Catches alignment issues at compile time

### 8. Pragma Pack for Binary Data

**Used in data_logger.h:**
```c
#pragma pack(push, 1)
typedef struct {
    // ... fields ...
} DataRecord_t;
#pragma pack(pop)
```
- Forces byte alignment for flash storage
- Ensures cross-platform compatibility

### 9. HAL Status Return Convention

**All HAL functions return HAL_StatusTypeDef:**
- `HAL_OK` - Success
- `HAL_ERROR` - Generic error
- `HAL_BUSY` - Resource busy (critical for DMA)
- `HAL_TIMEOUT` - Operation timeout

**Always check return values**, especially for DMA operations.

### 10. USER CODE Sections

**STM32CubeMX code generation:**
```c
/* USER CODE BEGIN section_name */
// Your code here - preserved during regeneration
/* USER CODE END section_name */
```
- Code outside these sections gets overwritten
- **ALWAYS** place custom code in USER CODE sections in HAL-generated files

### 11. Navigation Provider Interface Pattern

**Decouples data logger from specific navigation implementations:**

```c
// In data_logger.h
typedef struct {
    bool (*get_quaternion)(float quat[4]);
    bool (*get_position_ned)(float pos[3]);
    bool (*get_velocity_ned)(float vel[3]);
    bool (*is_valid)(void);
} NavigationProvider_t;

void DataLogger_RegisterNavProvider(const NavigationProvider_t* provider);
```

**Benefits:**
- Data logger works with Mahony filter, EKF, or any navigation system
- No extern dependencies between modules
- Easy to swap navigation algorithms
- NULL provider allowed for testing
- **See:** data_logger.h for implementation

**Usage Example:**
```c
// Wrapper functions for Mahony filter
bool My_Mahony_GetQuaternion(float quat[4]) {
    Quaternion_t q;
    if (Mahony_GetQuaternion(&mahony_filter, &q) == HAL_OK) {
        quat[0] = q.q0;
        quat[1] = q.q1;
        quat[2] = q.q2;
        quat[3] = q.q3;
        return true;
    }
    return false;
}

// Register provider
NavigationProvider_t mahony_provider = {
    .get_quaternion = My_Mahony_GetQuaternion,
    // ... other callbacks
};
DataLogger_RegisterNavProvider(&mahony_provider);
```

### 12. Sensor Calibration Framework

**Optional calibration support for improved accuracy:**

```c
// In sensor_manager.h
typedef struct {
    float gyro_bias[3];          // rad/s
    bool gyro_bias_valid;

    float accel_offset[3];       // g
    float accel_scale[3];        // scale factor
    bool accel_cal_valid;

    float mag_offset[3];         // Gauss (hard iron)
    float mag_scale[3][3];       // 3x3 matrix (soft iron)
    bool mag_cal_valid;
} SensorCalibration_t;

void SensorManager_SetCalibration(const SensorCalibration_t* cal);
bool SensorManager_GetCalibration(SensorCalibration_t* cal);
```

**Features:**
- Disabled by default - system works without calibration
- Pass NULL to `SensorManager_SetCalibration()` to disable
- Each sensor type has independent validity flag
- Applied automatically in `SensorManager_GetMahonyData()`
- Zero overhead when disabled
- **See:** sensor_manager.h for full API

**Calibration Types:**
1. **Gyro:** Bias correction (subtract bias vector)
2. **Accel:** Offset removal + scale factor
3. **Mag:** Hard iron (offset) + soft iron (3x3 matrix) compensation

### 13. Critical Sections for Race Prevention

**Use atomic operations for shared state:**

```c
// Example from i2c_dma_arbiter.c
__disable_irq();
if (arbiter_state.busy) {
    __enable_irq();
    return HAL_BUSY;
}
arbiter_state.busy = true;
arbiter_state.current_device = device;
__enable_irq();
```

**When to use:**
- Checking and setting flags in interrupt context
- Modifying shared state from multiple contexts
- Arbiter/scheduler implementations
- **Critical:** Keep critical sections as short as possible
- **See:** i2c_dma_arbiter.c:183-195

### 14. Overflow Tracking for Extended Timers

**GetMicros() with overflow detection:**

```c
static uint32_t us_high_word = 0;
static uint32_t last_cyccnt = 0;
static uint32_t cycles_per_us = 0;

static inline uint32_t GetMicros(void) {
    uint32_t cyccnt = DWT->CYCCNT;

    /* Detect overflow */
    if (cyccnt < last_cyccnt) {
        us_high_word += (0xFFFFFFFFU / cycles_per_us) + 1;
    }

    last_cyccnt = cyccnt;
    return us_high_word + (cyccnt / cycles_per_us);
}
```

**Benefits:**
- Extends range from ~25 seconds to ~71 minutes at 170MHz
- Still uses unsigned arithmetic for correct delta calculations
- No performance impact (inline function)
- **See:** sensor_manager.c:97-109

### 15. Metadata Dirty Flags for Flash Wear

**Reduce flash erase cycles with dirty flag pattern:**

```c
static bool metadata_dirty = false;
static uint32_t last_metadata_save_time = 0;
#define METADATA_SAVE_INTERVAL_MS  10000  // 10 seconds

// Mark dirty instead of immediate save
metadata_dirty = true;

// In periodic update function
void DataLogger_Update(void) {
    if (metadata_dirty && (HAL_GetTick() - last_metadata_save_time) > METADATA_SAVE_INTERVAL_MS) {
        if (Metadata_Save()) {
            metadata_dirty = false;
            last_metadata_save_time = HAL_GetTick();
        }
    }
}
```

**Impact:**
- Reduced metadata saves from 180/flight to 3/flight
- Flash lifespan increased from 555 to 33,000 flights (60x improvement)
- **See:** data_logger.c:42-44, 288-296

## Error Handling Standards

**Standard error codes and patterns are documented in `Inc/error_codes.h`**

### Key Principles

1. **Always validate pointer parameters** before dereferencing
2. **Always use timeouts** for blocking operations (never infinite wait)
3. **Always check HAL return values**, especially `HAL_BUSY` for DMA
4. **Always check validity flags** before using sensor data
5. **Always implement graceful degradation** on sensor failures
6. **Always track error counts** for diagnostics

### Module-Specific Error Codes

Each module has error codes in 0x1000-offset ranges:
- **I2C Arbiter:** 0x1000-0x1FFF
- **Data Logger:** 0x2000-0x2FFF
- **Sensor Manager:** 0x3000-0x3FFF
- **GPS Module:** 0x4000-0x4FFF
- **BLE Module:** 0x5000-0x5FFF
- **Mahony Filter:** 0x6000-0x6FFF

**See:** `Inc/error_codes.h` for complete documentation of error handling patterns

## Known Limitations and Constraints

### 1. GetMicros Wraparound (~71 minutes)

**Issue:** Microsecond timer wraps at 2^32 µs ≈ 71.6 minutes

**Mitigation:**
- Unsigned arithmetic handles deltas correctly for intervals < 71 min
- Most operations complete well within this timeframe
- Sufficient for individual flight sessions
- Consider using HAL_GetTick() (milliseconds) for longer intervals

**Code:** sensor_manager.c:97-109

### 2. I2C DMA Arbiter Timeout (100ms)

**Issue:** Arbiter releases bus after 100ms to prevent permanent locks

**Mitigation:**
- Watchdog function `I2C_DMA_Arbiter_Watchdog()` must be called periodically
- Call from main loop every 10-50ms
- Failed transfers increment error counters
- System continues operation after timeout

**Code:** i2c_dma_arbiter.c:224-240

### 3. Flash Metadata Erase Cycles

**Issue:** Flash sectors have limited erase cycles (typically 100K)

**Mitigation:**
- Dirty flag pattern reduces saves from 180 to 3 per 30-min flight
- Estimated 33,000 flights before wear-out
- Metadata sector at end of flash (easily replaceable)

**Code:** data_logger.c:42-44, 288-296

### 4. DMA Completion Wait (100ms timeout)

**Issue:** Data logger waits for DMA completion before next write

**Mitigation:**
- 100ms timeout prevents infinite blocking
- Timeout triggers error state
- Previous record skipped on timeout
- Error counters track failures

**Code:** data_logger.c:260-270

### 5. Navigation Provider Interface Overhead

**Issue:** Function pointer calls have slight overhead vs direct calls

**Mitigation:**
- Negligible impact on 100Hz data logging
- Benefits of modularity outweigh performance cost
- Can be optimized later if needed

**Code:** data_logger.c:96-135

## Critical Implementation Details

### Timing and Real-Time Constraints

1. **Mahony Filter:** Target 500Hz+ update rate
2. **Magnetometer:** 1kHz to feed Mahony filter
3. **IMU (ICM42688):** Highest rate possible via SPI
4. **GPS:** Typically 5-10Hz (UBX protocol)
5. **BLE/USB:** Asynchronous command processing
6. **Data Logger:** Records at main loop rate (typically 100Hz decimated)

### DMA Busy Handling

**Critical pattern in sensor drivers:**
```c
// Wait for previous DMA to complete
uint32_t start = HAL_GetTick();
while (*driver->dma_busy_flag) {
    if ((HAL_GetTick() - start) > driver->timeout_ms) {
        return HAL_TIMEOUT;
    }
}
```
- Always implement timeouts
- Check busy flags before starting new transfer
- High-priority sensors retry if arbiter busy

### I2C DMA Arbiter Priority

**Retry configuration:**
- MAG: `max_retries > 0` (will retry if bus busy)
- BARO: `max_retries = 0` (fails gracefully)
- HIGHG: `max_retries = 0` (fails gracefully)

This ensures magnetometer (critical for Mahony) gets preference.

### Flash Memory Management

**Metadata sector protection:**
- Metadata in last 4KB (address: 4MB - 4KB)
- Never write data to metadata sector
- Always validate metadata checksum before use
- Erase operations must handle both areas separately

### GPS Data Validity

**Always check multiple flags:**
```c
if (gps_data.data_valid && gps_data.fix_status == 'A' && gps_data.satellites >= 4) {
    // GPS data is trustworthy
}
```
- `data_valid` - Overall UBX validity
- `fix_status` - 'A' = valid, 'V' = invalid
- Satellite count - More satellites = better accuracy
- Check `last_update` timestamp for staleness

### Mahony Filter Initialization

**Two initialization modes:**

1. **Standard (horizontal):**
```c
Mahony_Init(&filter, 2.0f, 0.01f, 500.0f);
```

2. **Launch pad (X-axis up):**
```c
Mahony_InitLaunchPad(&filter, 2.0f, 0.01f, 500.0f);
```

Use launch pad mode for rocket applications where vehicle starts vertical.

### Error Recovery Strategies

1. **Sensor Init Failures:**
   - Continue without failed sensor
   - Set corresponding `_valid` flag to 0
   - System degrades gracefully

2. **DMA Timeouts:**
   - Increment error counters
   - Return error to caller
   - Next update attempt will retry

3. **Flash Errors:**
   - Set logger status to ERROR
   - Preserve existing data
   - Allow re-initialization

4. **GPS Loss:**
   - Continue with IMU-only navigation
   - Mahony filter works without GPS
   - Check timestamp staleness

## Common Development Tasks

### Adding a New Sensor

1. Create driver files: `new_sensor.{c,h}` in `Src/` and `Inc/`
2. Determine communication interface (I2C, SPI, UART)
3. If I2C: integrate with `i2c_dma_arbiter` and assign priority
4. If SPI: use DMA where possible
5. Add to `sensor_manager.c` if part of main sensor suite
6. Configure interrupt GPIO if data-ready available
7. Add to main loop in `main.c`
8. Update `SensorManager_RawData_t` if needed
9. Add validity flag for new sensor

### Modifying Data Logger Structure

1. Edit `DataRecord_t` in `data_logger.h`
2. **CRITICAL:** Ensure size remains 192 bytes (or change `RECORD_SIZE`)
3. Verify `_Static_assert` passes at compile time
4. Increment `METADATA_VERSION` if incompatible change
5. Update `DataLogger_RecordData()` in `data_logger.c` to populate new fields
6. Consider flash capacity (current: ~21K records at 192 bytes)

### Adding BLE/USB Commands

**For BLE:**
```c
// In main.c or custom module
void My_BLE_CommandCallback(const char* command) {
    if (strcmp(command, "mycommand") == 0) {
        // Handle command
        BLE_SendResponse("Response\r\n");
    }
}

// In main()
BLE_RegisterCommandCallback(My_BLE_CommandCallback);
```

**For USB:**
```c
USBCommandStatus_t My_USB_CommandCallback(const char* command,
                                           char* response_buffer,
                                           size_t buffer_size) {
    if (strcmp(command, "mycommand") == 0) {
        snprintf(response_buffer, buffer_size, "Response\r\n");
        return USB_CMD_OK;
    }
    return USB_CMD_INVALID;
}

// In main()
USBCommands_RegisterCallback(My_USB_CommandCallback);
```

### Tuning Mahony Filter

**Gains adjustment:**
- `Kp` (proportional): Faster convergence, higher noise sensitivity
  - Increase for faster attitude correction
  - Decrease for smoother output
- `Ki` (integral): Gyro bias compensation
  - Increase if gyro drift is significant
  - Too high causes slow oscillations

**Typical ranges:**
- Kp: 0.5 to 5.0 (default: 2.0)
- Ki: 0.0 to 0.1 (default: 0.01)

**Testing procedure:**
1. Start with defaults
2. Apply known rotations
3. Check convergence speed and stability
4. Adjust Kp first, then Ki
5. Test in actual flight conditions

### Debugging Sensor Issues

**I2C Scanner:**
```c
// In main.c
#include "i2c_scanner.h"
I2C_ScanBus(&hi2c1);  // Scan for devices on I2C1
```

**Check arbiter statistics:**
```c
I2C_DMA_Arbiter_Stats_t stats;
I2C_DMA_Arbiter_GetStats(&stats);
// Examine conflict counts
```

**SensorManager status:**
```c
SensorManager_Status_t status;
SensorManager_GetStatus(&status);
// Check error counts, actual rates
```

**Common issues:**
- Wrong I2C address (check datasheet)
- Missing pull-ups on I2C lines
- DMA channel conflicts
- Interrupt priority misconfiguration
- Arbiter priority wrong for application needs

### Testing in STM32CubeIDE

1. Build project: `Project → Build All`
2. Flash device: `Run → Debug` or `Run → Run`
3. Use breakpoints in sensor callbacks
4. Watch expressions for data structures
5. Use SWV (Serial Wire Viewer) for printf debugging
6. Logic analyzer on I2C/SPI for protocol debugging

## Build System and Tools

**Expected Build Environment:**
- STM32CubeIDE or compatible GCC ARM toolchain
- STM32CubeMX for peripheral initialization (likely .ioc file exists in full project)
- Make-based build system
- ST-Link or compatible debugger/programmer

**Note:** This repository appears to contain only source files. Full project likely includes:
- `.ioc` file (STM32CubeMX configuration)
- Linker script (`.ld`)
- Makefile or IDE project files
- Startup assembly (`.s`)
- HAL library files

## Dependencies

**External Libraries:**
- STM32G4xx HAL Driver (ST Microelectronics)
- USB Device Library (ST Microelectronics)
- ARM CMSIS (Cortex Microcontroller Software Interface Standard)

**Hardware-Specific:**
- All code assumes STM32G4 peripherals
- CORDIC and FMAC accelerators are STM32G4-specific
- Not easily portable to other MCU families without modification

## Memory Considerations

**RAM Usage:**
- Sensor data buffers (DMA)
- Mahony filter state (~100 bytes)
- GPS circular buffer (typically 256-512 bytes)
- BLE circular buffer (typically 256 bytes)
- Data logger record buffer (192 bytes)
- Stack for calculations (watch for overflow with FPU operations)

**Flash Usage:**
- Program flash: Typical embedded firmware size
- Data flash (Quad SPI): 4MB for data logging

**Optimization Tips:**
- Use `-O2` or `-Os` optimization
- Enable hardware FPU flags
- Minimize stack usage in ISRs
- Use DMA to reduce CPU load

## Testing and Validation

**Critical Tests:**
1. **I2C Arbiter:** Stress test with all sensors active
2. **Mahony Filter:** Static orientation tests, rotation tests
3. **GPS Parsing:** Test with real GPS data streams
4. **Flash Logging:** Fill flash completely, verify no corruption
5. **DMA Conflicts:** Run all peripherals simultaneously
6. **Interrupt Latency:** Measure worst-case response times
7. **BLE/USB Commands:** Stress test with rapid commands

**Validation Checklist:**
- [ ] All sensors initialize successfully
- [ ] I2C arbiter prevents bus conflicts
- [ ] Mahony filter converges to correct orientation
- [ ] GPS data parses correctly (position, velocity, covariance)
- [ ] Data logger writes and reads back correctly
- [ ] BLE/USB commands execute reliably
- [ ] No DMA overwrites or data corruption
- [ ] System runs continuously without crashes
- [ ] Flight profile data captures correctly

## Version Control and Development Workflow

**Branch Strategy:**
- Main/master: Stable releases
- Development branches: Feature additions
- Claude Code uses auto-generated branch names: `claude/claude-md-*`

**Commit Guidelines:**
- Clear, descriptive messages
- Focus on "why" rather than "what"
- Reference sensor or module affected
- Example: "Fix I2C arbiter priority for magnetometer timing"

**When Making Changes:**
1. Read existing code first (understand before modifying)
2. Maintain existing patterns (DMA, arbiter, validity flags)
3. Add compile-time assertions for critical sizes
4. Update documentation if interfaces change
5. Test with hardware if available
6. Consider backwards compatibility for data logger structures

## Important Notes for AI Assistants

### Code Style Consistency
- Follow existing function naming: `ModuleName_FunctionName()`
- Use `typedef struct` with `_t` suffix
- Document functions with Doxygen-style comments
- Use `/* */` for block comments, `//` for inline

### Safety Critical Considerations
- This is flight control software
- Sensor failures should degrade gracefully
- Always validate data before use
- Timeouts prevent infinite waits
- Error counters help diagnose issues

### Performance Critical Paths
- Mahony filter update (500Hz+ target)
- Sensor DMA reads (minimize latency)
- I2C arbiter decisions (deterministic)
- Interrupt handlers (keep short)

### Do Not Break
- I2C DMA arbiter priority order
- 192-byte data logger structure size
- NED frame coordinate conventions
- USER CODE section boundaries in HAL files
- DMA busy flag patterns

### When in Doubt
- Check sensor datasheets for register addresses
- Verify I2C addresses (7-bit vs 8-bit shifted)
- Consult STM32G4 reference manual for peripheral details
- Test DMA operations thoroughly
- Validate with hardware if possible

## Resources and References

**STM32G4 Documentation:**
- STM32G4 Reference Manual (RM0440)
- STM32G4 HAL User Manual
- STM32CubeMX User Manual

**Sensor Datasheets:**
- ICM-42688-P (TDK InvenSense)
- MMC5983MA (MEMSIC)
- H3LIS331DL (STMicroelectronics)
- BMP581 (Bosch Sensortec)
- RN4871 (Microchip)
- u-blox GPS receiver datasheet (model-specific)

**Algorithms:**
- Mahony AHRS Filter (Sebastian Madgwick's documentation)
- Coordinate frame transformations (aerospace conventions)
- Kalman filtering (for future EKF integration based on data logger structure)

**Tools:**
- STM32CubeIDE
- STM32CubeMX
- ST-Link Utility
- Logic analyzer (for I2C/SPI debugging)

## Glossary

- **AHRS:** Attitude and Heading Reference System
- **DMA:** Direct Memory Access
- **DOP:** Dilution of Precision (GPS accuracy metric)
- **EXTI:** External Interrupt
- **HAL:** Hardware Abstraction Layer
- **I2C:** Inter-Integrated Circuit (serial communication)
- **IMU:** Inertial Measurement Unit (accelerometer + gyroscope)
- **ISR:** Interrupt Service Routine
- **MAG:** Magnetometer
- **MSP:** MCU Support Package
- **NED:** North-East-Down (coordinate frame)
- **ODR:** Output Data Rate
- **SPI:** Serial Peripheral Interface
- **UART/USART:** Universal (Asynchronous) Synchronous Receiver-Transmitter
- **UBX:** u-blox binary protocol (GPS)
- **USB CDC:** USB Communications Device Class

---

**Document Version:** 1.0
**Last Updated:** 2025-11-25
**Maintained For:** AI Assistant comprehension of FlightComputerRefactor codebase
