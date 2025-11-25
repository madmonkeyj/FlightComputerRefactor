# Compilation Fixes Required

## Summary

The integration is now complete, but a few compilation issues need to be resolved before the code will build successfully. This document provides step-by-step fixes.

---

## Fix 1: Disable Incompatible sensor_system Module (ALREADY DONE)

**Issue**: The `sensor_system.c` module references sensor drivers that don't exist in this hardware revision:
- BMI088a.h (accel)
- BMI088g.h (gyro)
- BMP390.h (barometer)
- MLX90393.h (magnetometer)
- i2c_bitbang.h

Your hardware uses different sensors (ICM42688, MMC5983MA, BMP581, etc.)

**Fix Applied**: Renamed `Src/sensor_system.c` → `Src/sensor_system.c.DISABLED`

**Result**: The build system will not compile this file.

**Note**: We don't need `sensor_system` because your existing `sensor_manager` handles all sensor I/O.

---

## Fix 2: LoRa UART Assignment (VERIFY HARDWARE)

**Issue**: The code assumes `USART2` is connected to the LoRa module.

**Files Affected**:
- `Src/main.c` lines 422-437 (HAL_UART_TxCpltCallback and HAL_UART_ErrorCallback)

**Action Required**:
1. Check your hardware schematic
2. Identify which UART is connected to LoRa module
3. If not USART2, update the callbacks in `main.c`:

```c
// Change this:
if (huart->Instance == USART2) {

// To your actual UART:
if (huart->Instance == USARTx) {  // Replace x with your UART number
```

**Current UART Usage** (from existing code):
- USART1: BLE module (RN4871)
- USART3: GPS module
- USART2: **Assumed for LoRa** (VERIFY THIS!)

---

## Fix 3: Check for Multiple Definition Errors

**Potential Issue**: `HAL_UART_TxCpltCallback` and `HAL_UART_ErrorCallback` might already be defined elsewhere.

**If you get compilation error**:
```
multiple definition of `HAL_UART_TxCpltCallback'
```

**Solution**: Find the existing callback and merge the LoRa check into it:

```c
// In the existing callback:
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // ... existing code for BLE, GPS, etc ...

    // Add this for LoRa:
    if (huart->Instance == USART2) {  // Or your LoRa UART
        LoRa_TransmissionComplete(true);
    }
}
```

---

## Fix 4: Verify Peripheral Initialization

**Action Required**: Ensure these peripherals are initialized by STM32CubeMX:

**Required for existing sensors**:
- [x] SPI3 - ICM42688 (IMU)
- [x] I2C1 - MMC5983MA (mag), BMP581 (baro), H3LIS331DL (high-G)
- [x] USART1 - BLE module
- [x] USART3 - GPS module
- [x] QUADSPI1 - Flash memory
- [x] USB Device - Data download

**NEW requirement for LoRa**:
- [ ] **USART2** (or whichever UART is connected to LoRa)

**How to check**:
1. Open STM32CubeIDE
2. Open your `.ioc` file
3. Verify USART2 (or LoRa UART) is enabled
4. Verify it's configured with:
   - Mode: Asynchronous
   - Baud Rate: Check LoRa module datasheet (typically 115200)
   - DMA enabled for TX (recommended for LoRa_SendData)

**If missing**: Enable the UART in CubeMX and regenerate code

---

## Fix 5: Add LoRa Control GPIO Pins

**Action Required**: The LoRa module (E32/E22 series) requires GPIO control pins for mode switching.

**Required GPIO Pins**:
- `M0` - Mode control 0
- `M1` - Mode control 1
- `AUX` - Auxiliary status (optional)

**Configuration in CubeMX**:
1. Assign 2-3 GPIO pins for LoRa control
2. Configure as GPIO_Output (M0, M1) and GPIO_Input (AUX)
3. Define in `lora_module.c` or a config header

**Example GPIO mapping** (adjust to your hardware):
```c
#define LORA_M0_PIN     GPIO_PIN_6
#define LORA_M0_PORT    GPIOC
#define LORA_M1_PIN     GPIO_PIN_7
#define LORA_M1_PORT    GPIOC
```

**If not configured**: LoRa_Init() will fail during initialization

---

## Fix 6: Build Configuration in STM32CubeIDE

**Action Required**: Exclude disabled files from build

**Steps**:
1. Open STM32CubeIDE
2. Right-click on `Src/sensor_system.c.DISABLED` in Project Explorer
3. Select **Resource Configurations** → **Exclude from Build**
4. Check both Debug and Release configurations
5. Click OK

**Or**: The `.DISABLED` extension already prevents most build systems from compiling it

---

## Expected Compilation Warnings (Safe to Ignore)

After fixes, you may see these warnings (they are safe):

1. **Unused variable warnings** in new modules during initial testing
2. **Implicit function declaration** if header includes are missing (fix by adding includes)
3. **Comparison of float to zero** warnings (cosmetic, not critical)

---

## Compilation Success Checklist

Before attempting to build:

- [ ] `Src/sensor_system.c` disabled (renamed to `.DISABLED`)
- [ ] LoRa UART verified and configured (USART2 or correct assignment)
- [ ] LoRa GPIO pins defined (M0, M1 at minimum)
- [ ] No duplicate HAL callback definitions
- [ ] All new module source files added to build:
  - [ ] navigation_ekf.c
  - [ ] navigation_manager.c
  - [ ] motion_detector.c
  - [ ] coord_transform.c
  - [ ] nav_matrix.c
  - [ ] nav_config.c
  - [ ] telemetry_manager.c
  - [ ] lora_module.c
  - [ ] rocket_telemetry.c
  - [ ] flight_manager.c

---

## Build Command (STM32CubeIDE)

1. **Clean Project**: `Project` → `Clean...` → Select project → Click `Clean`
2. **Build Project**: `Project` → `Build Project` (Ctrl+B)
3. **Check Console** for errors

---

## Common Build Errors and Fixes

### Error: `undefined reference to 'LoRa_Init'`
**Cause**: `lora_module.c` not in build
**Fix**: Right-click `Src/lora_module.c` → **Resource Configurations** → Ensure NOT excluded

### Error: `undefined reference to 'NavigationManager_Init'`
**Cause**: `navigation_manager.c` not in build
**Fix**: Add all new `.c` files to build (see checklist above)

### Error: `unknown type name 'SensorData_t'`
**Cause**: Type conflict between `sensor_system.h` and navigation modules
**Fix**: Verify `sensor_system.c` is disabled. The header `sensor_system.h` can remain but shouldn't cause issues if the `.c` file is excluded.

### Error: `BMI088a.h: No such file or directory`
**Cause**: `sensor_system.c` is still being compiled
**Fix**: Ensure `sensor_system.c.DISABLED` is excluded from build

### Error: Multiple definition of `HAL_UART_TxCpltCallback`
**Cause**: Callback already defined elsewhere
**Fix**: Merge LoRa check into existing callback (see Fix 3 above)

---

## Next Steps After Successful Compilation

1. **Flash the firmware** to your STM32G4
2. **Connect via USB CDC** to monitor debug output
3. **Verify initialization sequence**:
   ```
   === FlightComputer Full Navigation System ===
   Sensor Manager initialized
   Navigation Manager initialized (AHRS + EKF)
   GPS module initialized
   BLE Module initialized successfully
   LoRa module initialized  <-- Should see this!
   Telemetry Manager initialized
   Data Logger initialized successfully
   Navigation provider registered (EKF integration)
   Flight Manager initialized - system ready
   === All systems initialized - entering main loop ===
   ```

4. **If LoRa init fails**: Check UART assignment and GPIO pins
5. **Test telemetry transmission**: Use BLE commands to start recording
6. **Verify EKF integration**: Check data logger records contain quaternion/position/velocity

---

## Support Resources

- **Integration Plan**: See `INTEGRATION_PLAN.md` for architecture details
- **Testing Checklist**: See `TESTING_CHECKLIST.md` for validation tests
- **Refactoring Documentation**: See `CLAUDE.md` for system overview

---

**Document Version**: 1.0
**Last Updated**: 2025-11-25
**Status**: Ready for compilation testing
