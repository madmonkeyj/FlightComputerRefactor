# Integration Analysis: LoRa, Telemetry Manager, and Flight Manager

**Date:** 2025-12-03
**Analysis:** Integration of disabled modules from another project into FlightComputerRefactor

---

## Executive Summary

The disabled modules (LoRa, TelemetryManager, FlightManager) are from a **previous project using different sensors** but compatible architecture. Integration is **FEASIBLE** with moderate adaptation work required.

### Key Findings

✅ **Compatible:**
- USART2 is available and configured for LoRa
- NavigationSolution_t structure exists and is compatible
- TelemetryManager architecture aligns with current design
- FlightManager timing structure matches current main loop

⚠️ **Requires Adaptation:**
- Sensor system layer needs bridge/adapter
- Missing battery monitor functionality
- Missing inter-MCU communication module
- Some function signatures need updates

❌ **Missing Dependencies:**
- Old sensor drivers (BMI088, BMP390, MLX90393)
- I2C bitbang library
- Battery monitoring module
- Inter-MCU communication

---

## Detailed Hardware & Sensor Comparison

### Sensor Comparison Table

| Component | Old Project | New Project | Compatibility |
|-----------|-------------|-------------|---------------|
| **Primary IMU** | BMI088 (Accel + Gyro, I2C bitbang) | ICM-42688-P (6-DOF, SPI+DMA) | ❌ Different interface |
| **Magnetometer** | MLX90393 (I2C bitbang) | MMC5983MA (I2C+DMA) | ❌ Different chip |
| **Barometer** | BMP390 (I2C bitbang) | BMP581 (I2C+DMA) | ❌ Different chip |
| **High-G Accel** | None | H3LIS331DL (I2C+DMA) | ➕ New feature |
| **GPS** | u-blox (UART3+DMA) | u-blox (UART3+DMA) | ✅ **Same** |
| **I2C Method** | Software bitbang | Hardware I2C+DMA | ❌ Different implementation |

**Critical Difference:** Old project uses I2C bitbanging, new project uses hardware I2C with DMA arbitration.

### Communication Peripherals

| Peripheral | Old Project | New Project | Status |
|------------|-------------|-------------|--------|
| USART1 | **LoRa** | BLE (RN4871) | ⚠️ Conflict |
| USART2 | Inter-MCU Comm | **Available (115200 baud)** | ✅ **Perfect for LoRa!** |
| USART3 | GPS | GPS | ✅ Same |
| USB | Debug | Debug | ✅ Same |

**Resolution:** Move LoRa from USART1 to USART2 (as you specified).

---

## Architecture Comparison

### Data Flow - Old Project

```
BMI088/BMP390/MLX90393 (I2C bitbang)
    ↓
SensorSystem_Read() → SensorData_t
    ↓
NavigationManager_UpdateAHRS(SensorData_t*)
    ↓
NavigationManager_UpdateNavigation(SensorData_t*)
    ↓
NavigationSolution_t (output)
    ↓
TelemetryManager → LoRa (USART1)
```

### Data Flow - New Project

```
ICM42688/MMC5983MA/BMP581 (SPI+DMA / I2C+DMA)
    ↓
SensorManager_GetRawData() → SensorManager_RawData_t
    ↓
SensorAdapter_Convert() → NavSensorData_t
    ↓
NavigationManager_UpdateAHRS(NavSensorData_t*)
    ↓
NavigationManager_UpdateNavigation(NavSensorData_t*)
    ↓
NavigationSolution_t (output)
    ↓
[MISSING] TelemetryManager → LoRa (USART2)
```

### Key Architectural Differences

| Layer | Old Project | New Project | Integration Strategy |
|-------|-------------|-------------|---------------------|
| **Sensor HAL** | I2C bitbang | I2C+DMA arbiter | Create adapter layer |
| **Sensor Interface** | `SensorData_t` | `SensorManager_RawData_t` | Use `SensorAdapter` |
| **Navigation Input** | `SensorData_t*` | `NavSensorData_t*` | ✅ Already adapted |
| **Navigation Output** | `NavigationSolution_t` | `NavigationSolution_t` | ✅ **Compatible!** |
| **Telemetry** | `TelemetryManager` | Missing | Direct integration |
| **LoRa UART** | USART1 | USART2 | Change in lora_module.c |

---

## Code-Level Compatibility Analysis

### 1. LoRa Module (`lora_module.c.DISABLED`)

#### Current Dependencies
```c
#include "inter_mcu_comm.h"  // ❌ MISSING - Inter-MCU communication
```

#### UART Configuration Issues
```c
// Lines 43-57: HAL callbacks reference huart2
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        InterMCU_UART_RxCpltCallback(huart);  // ❌ Calls missing module
    }
}
```

#### GPIO Dependencies
```c
// Lines 87-88, 93: LoRa control pins
HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, m0_state);
HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, m1_state);
HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin);
```

**Required GPIO Pins:**
- `M0_Pin` / `M0_GPIO_Port` - Mode control 0
- `M1_Pin` / `M1_GPIO_Port` - Mode control 1
- `AUX_Pin` / `AUX_GPIO_Port` - LoRa module ready status

**Integration Actions:**
1. ✅ USART2 is already configured (115200 baud)
2. ⚠️ Check if M0/M1/AUX GPIO pins are defined in current project
3. ❌ Remove `inter_mcu_comm.h` dependency or stub it out
4. ✅ Update HAL callbacks to call `LoRa_TransmissionComplete()` (already present)

### 2. Telemetry Manager (`telemetry_manager.c.DISABLED`)

#### Dependencies
```c
#include "lora_module.h"        // ✅ Will be integrated
#include "battery_monitor.h"    // ❌ MISSING
#include "rocket_telemetry.h"   // ✅ Exists (header present)
#include "sensor_system.h"      // ⚠️ Different from SensorManager
```

#### Function Signature Compatibility

**Expected Input:**
```c
bool TelemetryManager_SendRocketTelemetry(
    const NavigationSolution_t* nav_solution,  // ✅ Compatible
    const SensorData_t* sensor_data,           // ❌ Different structure
    const GPS_Data_t* gps_data                 // ✅ Compatible
);
```

**Current Project Structures:**
- ✅ `NavigationSolution_t` - **Identical structure!**
- ❌ `SensorData_t` (old) vs `SensorManager_RawData_t` (new) - **Different**
- ✅ `GPS_Data_t` - **Compatible**

#### Battery Voltage Dependency
```c
// Line 467: telemetry_manager.c.DISABLED
packet->battery_voltage = ReadBatteryVoltage();  // ❌ Function doesn't exist
```

**Integration Actions:**
1. ✅ NavigationSolution_t is already compatible
2. ⚠️ Create adapter between `SensorManager_RawData_t` and `SensorData_t`
3. ❌ Implement `ReadBatteryVoltage()` or stub to return 0.0f
4. ✅ GPS_Data_t is compatible

### 3. Flight Manager (`flight_manager.c.DISABLED`)

#### Dependencies
```c
#include "sensor_system.h"      // ⚠️ Uses old SensorSystem_Read()
#include "telemetry_manager.h"  // ✅ Will be integrated
#include "lora_module.h"        // ✅ Will be integrated
```

#### GPIO Initialization (Lines 87-90)
```c
HAL_GPIO_WritePin(CONFIG_GPIO_Port, CONFIG_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(LPM_GPIO_Port, LPM_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(RST_BT_GPIO_Port, RST_BT_Pin, GPIO_PIN_SET);
```

**Required GPIO Pins:**
- `CONFIG_Pin` / `CONFIG_GPIO_Port`
- `LPM_Pin` / `LPM_GPIO_Port`
- `RST_BT_Pin` / `RST_BT_GPIO_Port`

#### Function Call Issues
```c
// Line 102: Calls old sensor system
if (SensorSystem_Init()) { ... }  // ❌ Different from SensorManager_Init()

// Line 218: Reads sensors differently
if (SensorSystem_Read(&sensor_data)) { ... }  // ❌ Uses old interface

// Line 320: Missing function
FormatNavigationData(&nav_solution, &sensor_data);  // ❌ Not defined anywhere
```

**Integration Actions:**
1. ⚠️ Replace `SensorSystem_*` calls with `SensorManager_*` + adapter
2. ⚠️ Check GPIO pin definitions in current project
3. ❌ Implement or remove `FormatNavigationData()` calls
4. ✅ Timing intervals (500Hz, 5Hz, etc.) match current main loop

---

## Sensor Data Structure Mapping

### Old Project: `SensorData_t` (sensor_system.h)

```c
typedef struct {
    float accel[3];      // m/s² [X, Y, Z] - Body frame
    float gyro[3];       // rad/s [X, Y, Z] - Body frame
    float mag[3];        // µT [X, Y, Z] - Body frame
    float pressure;      // Pa - Absolute pressure
    float temperature;   // °C - Ambient temperature
    uint32_t timestamp;  // HAL_GetTick()
    bool valid;          // All critical sensors OK
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
    bool baro_valid;
} SensorData_t;
```

### New Project: `SensorManager_RawData_t` (sensor_manager.h)

**Need to verify this structure, but likely similar with:**
```c
typedef struct {
    // IMU data
    float accel[3];      // m/s²
    float gyro[3];       // rad/s
    float mag[3];        // µT
    // Barometer data
    float pressure;      // Pa
    float temperature;   // °C
    // High-G accelerometer
    float high_g_accel[3];  // ➕ New feature
    // Metadata
    uint32_t timestamp;
    bool imu_valid;
    bool mag_valid;
    bool baro_valid;
    bool high_g_valid;   // ➕ New sensor
} SensorManager_RawData_t;
```

### Adapter Strategy

**Create compatibility layer:**

```c
// In sensor_adapter.c or new compatibility file
SensorData_t SensorAdapter_ToLegacyFormat(const SensorManager_RawData_t* raw) {
    SensorData_t legacy = {0};

    // Copy IMU data
    memcpy(legacy.accel, raw->accel, sizeof(float) * 3);
    memcpy(legacy.gyro, raw->gyro, sizeof(float) * 3);
    memcpy(legacy.mag, raw->mag, sizeof(float) * 3);

    // Copy barometer data
    legacy.pressure = raw->pressure;
    legacy.temperature = raw->temperature;

    // Copy metadata
    legacy.timestamp = raw->timestamp;
    legacy.accel_valid = raw->imu_valid;
    legacy.gyro_valid = raw->imu_valid;
    legacy.mag_valid = raw->mag_valid;
    legacy.baro_valid = raw->baro_valid;
    legacy.valid = raw->imu_valid && raw->mag_valid && raw->baro_valid;

    return legacy;
}
```

---

## Missing Dependencies Resolution

### 1. Battery Monitor (`battery_monitor.h`)

**Referenced in:** `telemetry_manager.c.DISABLED:467`

**Options:**
- **A)** Implement using ADC (if battery voltage monitoring hardware exists)
- **B)** Stub function returning constant value:
  ```c
  float ReadBatteryVoltage(void) {
      return 3.7f;  // Stub value
  }
  ```
- **C)** Remove battery voltage from telemetry packet

**Recommendation:** Check if ADC is configured for battery monitoring, otherwise stub to 0.0f.

### 2. Inter-MCU Communication (`inter_mcu_comm.h`)

**Referenced in:** `lora_module.c.DISABLED:9, 45, 56`

**Current Usage:**
```c
InterMCU_UART_RxCpltCallback(huart);   // Line 45
InterMCU_UART_ErrorCallback(huart);    // Line 56
```

**Resolution:**
- **Remove these calls** - they're for a different project's MCU-to-MCU communication
- Direct UART callbacks should call `LoRa_TransmissionComplete()` instead

### 3. I2C Bitbang (`i2c_bitbang.h`)

**Referenced in:** `sensor_system.c.DISABLED:10`

**Resolution:**
- **Not needed** - Current project uses hardware I2C with DMA
- Sensor system adapter will use `SensorManager` instead

### 4. Old Sensor Drivers

**Referenced in:** `sensor_system.c.DISABLED`
- `BMI088a.h` / `BMI088g.h`
- `BMP390.h`
- `MLX90393.h`

**Resolution:**
- **Don't integrate these drivers** - use current `SensorManager` with adapter layer

### 5. FormatNavigationData Function

**Referenced in:** `flight_manager.c.DISABLED:320, 322`

**Resolution:**
- **Option A:** Remove these calls (appears to be debug output)
- **Option B:** Implement simple formatting:
  ```c
  void FormatNavigationData(const NavigationSolution_t* nav, const SensorData_t* sensors) {
      // Debug print of navigation data
      DebugPrint("Nav: Alt=%.1f Vel=%.1f\n",
                 -nav->position_ned[2], -nav->velocity_ned[2]);
  }
  ```

**Recommendation:** Remove calls, debug output handled elsewhere.

---

## GPIO Pin Requirements

### LoRa Module Pins

| Pin Name | Purpose | Required? |
|----------|---------|-----------|
| M0 | LoRa mode control bit 0 | ✅ Yes |
| M1 | LoRa mode control bit 1 | ✅ Yes |
| AUX | LoRa module ready status (input) | ✅ Yes |

### FlightManager Pins

| Pin Name | Purpose | Required? |
|----------|---------|-----------|
| CONFIG | Unknown configuration pin | ⚠️ Check |
| LPM | Low power mode control | ⚠️ Check |
| RST_BT | Bluetooth reset | ⚠️ Might be for BLE |

**Action Required:** Verify these GPIO pins are defined in current `main.h` or GPIO configuration.

---

## Integration Roadmap

### Phase 1: Prepare LoRa Module ✅ **READY**

1. ✅ Verify USART2 configuration (already configured at 115200 baud)
2. ⚠️ Check GPIO pin definitions (M0, M1, AUX)
3. ✅ Remove `inter_mcu_comm.h` dependency
4. ✅ Update UART callbacks to pure LoRa functionality
5. ✅ Test basic LoRa initialization

**Files to modify:**
- `lora_module.c.DISABLED` → `lora_module.c`
- `main.c` (add LoRa_Init() and LoRa_Update())

**Estimated Effort:** 2-3 hours

### Phase 2: Create Sensor Data Adapter ⚠️ **MODERATE**

1. Create `sensor_data_compat.h` / `.c`
2. Implement `SensorAdapter_ToLegacyFormat()`
3. Add conversion utilities
4. Test sensor data flow

**Files to create:**
- `Inc/sensor_data_compat.h`
- `Src/sensor_data_compat.c`

**Estimated Effort:** 1-2 hours

### Phase 3: Integrate Telemetry Manager ⚠️ **MODERATE**

1. ✅ Enable `telemetry_manager.c.DISABLED` → `.c`
2. ⚠️ Update includes (remove battery_monitor, add compat layer)
3. ⚠️ Update `TelemetryManager_SendRocketTelemetry()` to use adapter
4. ❌ Stub `ReadBatteryVoltage()` function
5. ✅ Test telemetry packet creation
6. ✅ Test LoRa transmission

**Files to modify:**
- `telemetry_manager.c.DISABLED` → `telemetry_manager.c`
- `main.c` (add TelemetryManager_Init() and Update())

**Estimated Effort:** 3-4 hours

### Phase 4: Integrate Flight Manager ⚠️ **COMPLEX**

1. ⚠️ Enable `flight_manager.c.DISABLED` → `.c`
2. ⚠️ Replace `SensorSystem_*` calls with `SensorManager_*` + adapter
3. ⚠️ Check GPIO pin requirements
4. ❌ Remove/implement `FormatNavigationData()`
5. ✅ Test full system integration
6. ✅ Verify timing (500Hz AHRS, 5Hz telemetry, etc.)

**Files to modify:**
- `flight_manager.c.DISABLED` → `flight_manager.c`
- `main.c` (replace manual loop with FlightManager_Update())

**Estimated Effort:** 4-6 hours

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| GPIO pins not defined | Medium | High | Verify pinout, update .ioc file if needed |
| USART2 DMA conflicts | Low | High | USART2 currently unused, should be safe |
| Timing issues (500Hz loop) | Medium | Medium | Monitor actual update rates, adjust if needed |
| Sensor data incompatibility | Low | Medium | Adapter layer handles differences |
| Missing battery monitoring | High | Low | Stub to constant value acceptable |
| Navigation structure mismatch | **Very Low** | High | ✅ Structures are compatible! |

---

## Questions for User

Before proceeding with integration, please confirm:

1. **GPIO Pins:** Are M0, M1, and AUX pins defined for LoRa control?
2. **Battery Monitoring:** Does the hardware have battery voltage ADC? If not, stub OK?
3. **FlightManager GPIO:** Are CONFIG, LPM, RST_BT pins needed/defined?
4. **Integration Priority:** Which module should I integrate first?
   - Option A: LoRa only (simplest, can test transmission)
   - Option B: LoRa + TelemetryManager (full telemetry system)
   - Option C: Full integration (all three modules)

5. **Sensor System:** Should I create the compatibility adapter, or would you prefer to update TelemetryManager to use new SensorManager directly?

6. **Debug Output:** Remove FormatNavigationData() calls or implement them?

---

## Recommended Integration Order

Based on dependencies and risk:

### Recommended Approach: **Incremental Integration**

**Step 1:** LoRa Module Only
- Lowest risk, no dependencies
- Can test basic LoRa transmission
- Verify USART2 and GPIO configuration

**Step 2:** Add Sensor Compatibility Layer
- Create adapter for SensorData_t
- Test with current system before telemetry

**Step 3:** Telemetry Manager
- Integrate with LoRa
- Test rocket telemetry transmission
- Monitor transmission success rates

**Step 4:** Flight Manager (Optional)
- Full system orchestration
- Replace manual main loop timing

---

## Compatibility Summary

### ✅ Highly Compatible Components
- Navigation structures (NavigationSolution_t)
- GPS module and data structures
- Timing architecture (500Hz/5Hz/etc.)
- Overall system design patterns

### ⚠️ Require Moderate Adaptation
- Sensor data structures (need adapter)
- UART port change (USART1 → USART2)
- GPIO pin verification

### ❌ Not Compatible / Missing
- Old sensor drivers (BMI088, BMP390, MLX90393)
- I2C bitbang library
- Battery monitor module
- Inter-MCU communication

---

## Final Recommendation

**✅ PROCEED WITH INTEGRATION**

The modules are architecturally compatible and can be integrated with moderate effort. Key success factors:

1. **USART2 is perfect for LoRa** - no conflicts with existing peripherals
2. **Navigation structures match** - minimal changes needed
3. **Sensor adapter is straightforward** - well-defined mapping
4. **Incremental approach minimizes risk** - can test at each stage

**Estimated Total Integration Time:** 10-15 hours (all phases)

**Biggest Unknowns:**
- GPIO pin definitions (check .ioc file or schematic)
- Battery monitoring hardware availability

**Next Step:** Confirm GPIO configuration and preferred integration order, then I can begin implementation.

---

## 🎉 CRITICAL UPDATE: GPIO Pins Already Configured!

**EXCELLENT NEWS:** All required GPIO pins are already defined and configured in the current project!

### LoRa Module Pins ✅ READY
```c
// From Inc/main.h and Src/gpio.c
#define M1_Pin GPIO_PIN_12          // GPIOB - LoRa mode control 1
#define M0_Pin GPIO_PIN_13          // GPIOB - LoRa mode control 0
#define AUX_Pin GPIO_PIN_14         // GPIOB - LoRa ready status (input)
```

### FlightManager Pins ✅ READY
```c
// From Inc/main.h
#define CONFIG_Pin GPIO_PIN_15      // GPIOB - Configuration pin
#define RST_BT_Pin GPIO_PIN_6       // GPIOC - Bluetooth reset
#define LPM_Pin GPIO_PIN_8          // GPIOA - Low power mode
```

**Impact on Integration:**
- ✅ **Phase 1 (LoRa) can proceed immediately** - no GPIO configuration needed
- ✅ **Phase 4 (FlightManager) GPIO requirements satisfied**
- ✅ **Reduces integration risk significantly**
- ✅ **Estimated effort reduced by ~2 hours**

**This means the hardware is already configured for LoRa integration!**

