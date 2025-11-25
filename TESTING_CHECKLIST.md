# FlightComputer Refactoring - Testing Checklist

This checklist validates all refactoring work completed in Tasks 1-9.

## ✅ Success Criteria

- [ ] No compiler warnings
- [ ] All sensors initialize successfully
- [ ] Data logger records 1000+ records without error
- [ ] Metadata persists across power cycle
- [ ] BLE commands respond correctly
- [ ] I2C arbiter handles conflicts without hangs
- [ ] No race conditions detected in testing

---

## Unit Tests (Module-Level Validation)

### Task 1: I2C DMA Arbiter

#### Race Condition Fix
- [ ] **Test 1.1:** Multiple rapid requests from different devices
  - **Method:** Trigger MAG, BARO, HIGHG requests in quick succession
  - **Expected:** No bus collisions, arbiter handles FCFS correctly
  - **Code:** i2c_dma_arbiter.c:183-195 (critical section)

- [ ] **Test 1.2:** Simultaneous request from interrupt and main loop
  - **Method:** Request transfer from main loop, trigger interrupt request
  - **Expected:** One succeeds with HAL_OK, other gets HAL_BUSY
  - **Code:** i2c_dma_arbiter.c:183-195

#### Timeout/Watchdog
- [ ] **Test 1.3:** Watchdog timeout recovery
  - **Method:** Simulate stuck DMA (disconnect sensor), call watchdog after 100ms
  - **Expected:** Arbiter releases after timeout, returns true from watchdog
  - **Code:** i2c_dma_arbiter.c:224-240

- [ ] **Test 1.4:** Normal operation doesn't trigger watchdog
  - **Method:** Normal sensor reads with watchdog running
  - **Expected:** Watchdog always returns false during normal operation
  - **Code:** i2c_dma_arbiter.c:224-240

### Task 2: Data Logger DMA and Flash Wear

#### DMA Race Condition Fix
- [ ] **Test 2.1:** Rapid record writes
  - **Method:** Call DataLogger_RecordData() at maximum rate
  - **Expected:** No corruption, each record increments address correctly
  - **Code:** data_logger.c:260-280

- [ ] **Test 2.2:** DMA completion wait
  - **Method:** Monitor qspi_write_busy flag during record writes
  - **Expected:** Flag cleared before address increment
  - **Code:** data_logger.c:260-270

- [ ] **Test 2.3:** DMA timeout handling
  - **Method:** Simulate QSPI failure, verify timeout triggers error
  - **Expected:** Logger enters ERROR state after 100ms timeout
  - **Code:** data_logger.c:266-269

#### Flash Wear Reduction
- [ ] **Test 2.4:** Metadata dirty flag behavior
  - **Method:** Write 100 records, count metadata saves
  - **Expected:** ~3 saves max (one every 10 seconds) vs 100 immediate saves
  - **Code:** data_logger.c:288-296

- [ ] **Test 2.5:** Metadata save on stop
  - **Method:** Record data, stop logging, power cycle, verify metadata
  - **Expected:** Metadata accurately reflects last state
  - **Code:** data_logger.c:197-211

### Task 3: Sensor Manager Static Variables

- [ ] **Test 3.1:** SensorManager_ResetState() clears all state
  - **Method:** Read sensors, call reset, verify all counters/data cleared
  - **Expected:** All decimation counters = 0, last_valid data cleared
  - **Code:** sensor_manager.c:521-537

- [ ] **Test 3.2:** Decimated sensor data persistence
  - **Method:** Read decimated sensors (BARO, HIGHG) multiple times
  - **Expected:** Last valid data returned when decimation skips update
  - **Code:** sensor_manager.c:35-36 (file-scope variables)

### Task 4: Data Logger Decoupling

- [ ] **Test 4.1:** NULL navigation provider
  - **Method:** Register NULL provider, record data
  - **Expected:** Navigation fields zeroed, no crash
  - **Code:** data_logger.c:123-135

- [ ] **Test 4.2:** Navigation provider callbacks
  - **Method:** Register provider with valid callbacks, record data
  - **Expected:** Callbacks invoked, quaternion/position/velocity populated
  - **Code:** data_logger.c:96-122

- [ ] **Test 4.3:** Provider with partial callbacks
  - **Method:** Provider with get_quaternion only, others NULL
  - **Expected:** Quaternion populated, position/velocity zeroed
  - **Code:** data_logger.c:100-115

### Task 5: BLE Module Cleanup

- [ ] **Test 5.1:** BLE commands still work
  - **Method:** Send start, stop, status, erase, help commands
  - **Expected:** All commands respond correctly
  - **Code:** ble_module.c:368-425

- [ ] **Test 5.2:** No debug variables referenced
  - **Method:** Search codebase for command_received_flag, response_sent_flag
  - **Expected:** No references found (removed)
  - **Code:** ble_module.c (removed lines 51-54)

### Task 6: Error Handling Standards

- [ ] **Test 6.1:** Error codes documentation complete
  - **Method:** Review Inc/error_codes.h
  - **Expected:** All modules have error code ranges defined
  - **Code:** Inc/error_codes.h

- [ ] **Test 6.2:** Error handling patterns documented
  - **Method:** Verify all 10 patterns have code examples
  - **Expected:** Each pattern has example implementation
  - **Code:** Inc/error_codes.h

### Task 7: GetMicros Overflow Handling

- [ ] **Test 7.1:** Overflow detection
  - **Method:** Manually increment DWT->CYCCNT to near overflow, call GetMicros()
  - **Expected:** us_high_word increments on overflow
  - **Code:** sensor_manager.c:97-109

- [ ] **Test 7.2:** Delta calculations across overflow
  - **Method:** Compute delta time across simulated overflow
  - **Expected:** Delta still computed correctly (unsigned arithmetic)
  - **Code:** sensor_manager.c:311-318

- [ ] **Test 7.3:** Extended range validation
  - **Method:** Monitor GetMicros() for >25 seconds
  - **Expected:** No wraparound at 25 seconds, continues to 71 min
  - **Code:** sensor_manager.c:97-109

### Task 8: Sensor Calibration Framework

- [ ] **Test 8.1:** Calibration disabled by default
  - **Method:** Initialize sensor manager without calling SetCalibration
  - **Expected:** Raw sensor data unchanged in GetMahonyData
  - **Code:** sensor_manager.c:479-521

- [ ] **Test 8.2:** Gyro bias correction
  - **Method:** Set gyro_bias = {0.1, 0.2, 0.3}, read data
  - **Expected:** Output gyro values reduced by bias amounts
  - **Code:** sensor_manager.c:478-483

- [ ] **Test 8.3:** Accel calibration
  - **Method:** Set accel offset/scale, read data
  - **Expected:** Accel values = (raw - offset) * scale
  - **Code:** sensor_manager.c:490-495

- [ ] **Test 8.4:** Mag hard/soft iron compensation
  - **Method:** Set mag offset and scale matrix, read data
  - **Expected:** Mag values corrected by offset and matrix multiply
  - **Code:** sensor_manager.c:502-521

- [ ] **Test 8.5:** NULL disables calibration
  - **Method:** Set calibration, then call SetCalibration(NULL)
  - **Expected:** Calibration disabled, raw data returned
  - **Code:** sensor_manager.c:493-500

- [ ] **Test 8.6:** GetCalibration retrieves settings
  - **Method:** Set calibration, call GetCalibration, compare
  - **Expected:** Retrieved calibration matches what was set
  - **Code:** sensor_manager.c:508-515

---

## Integration Tests (System-Level Validation)

### Sensor System Integration

- [ ] **Test I.1:** All sensors reading at target rates
  - **Method:** Monitor sensor_status.actual_rate_hz for each sensor
  - **Expected:**
    - IMU: 1000Hz
    - MAG: 1000Hz
    - BARO: 100Hz (decimated)
    - HIGHG: 50Hz (decimated)
  - **Duration:** 60 seconds continuous

- [ ] **Test I.2:** I2C arbiter handles conflicts gracefully
  - **Method:** All I2C sensors active simultaneously
  - **Expected:** No hangs, MAG gets priority, BARO/HIGHG retry or skip
  - **Monitor:** Arbiter statistics (conflict counts)

- [ ] **Test I.3:** DMA transfers don't interfere
  - **Method:** IMU (SPI DMA), MAG (I2C DMA), GPS (UART DMA) all active
  - **Expected:** No data corruption, all sensors reading correctly
  - **Duration:** 5 minutes

### Data Logger Integration

- [ ] **Test I.4:** Data logger records 1000+ records without corruption
  - **Method:** Start recording, run for 1000 records, stop, verify
  - **Expected:**
    - All records exactly 192 bytes
    - Timestamps monotonically increasing
    - No corrupted data
    - Metadata correct

- [ ] **Test I.5:** Metadata survives power cycle
  - **Method:** Record data, stop, power cycle, check metadata
  - **Expected:**
    - Records count correct
    - Write address correct
    - Session ID incremented
    - Checksum valid

- [ ] **Test I.6:** Flash full scenario (graceful stop)
  - **Method:** Fill flash completely
  - **Expected:**
    - Logger stops at DATA_AREA_SIZE
    - Status = IDLE
    - No crash
    - Data intact

- [ ] **Test I.7:** QSPI writes complete before next write starts
  - **Method:** Monitor qspi_write_busy flag during rapid writes
  - **Expected:** Flag always cleared before next write begins
  - **Monitor:** Use oscilloscope on QSPI signals if available

### BLE/USB Command Integration

- [ ] **Test I.8:** BLE commands still work after cleanup
  - **Method:** Send all commands (start, stop, status, erase, help)
  - **Expected:** All commands respond correctly
  - **Verify:** No debug code interfering

- [ ] **Test I.9:** Start/stop recording via BLE
  - **Method:** BLE start command, verify recording, BLE stop command
  - **Expected:**
    - Start command triggers recording
    - Data logged to flash
    - Stop command ends recording
    - Metadata saved

- [ ] **Test I.10:** Status command shows logger state
  - **Method:** Send status command during various states
  - **Expected:**
    - Shows IDLE/RECORDING/ERROR correctly
    - Shows record count
    - Shows flash usage percentage

### Navigation Provider Integration

- [ ] **Test I.11:** Mahony provider integration
  - **Method:** Register Mahony provider, log data
  - **Expected:**
    - Quaternion logged correctly
    - Position NED from provider
    - Velocity NED from provider
    - nav_valid flag set correctly

- [ ] **Test I.12:** Provider swap without restart
  - **Method:** Register provider A, record, switch to provider B, record
  - **Expected:**
    - No crashes
    - Data reflects current provider
    - Clean transition

---

## Stress Tests (Robustness Validation)

### Long Duration Tests

- [ ] **Test S.1:** 30-minute continuous recording
  - **Method:** Start recording, run for 30 minutes, stop
  - **Expected:**
    - ~21K records (at configured rate)
    - No errors
    - Metadata saves ~3 times (dirty flag working)
    - Flash usage ~90%
  - **Monitor:**
    - Error counters remain 0
    - No memory leaks
    - No watchdog resets

- [ ] **Test S.2:** GetMicros extended range (60+ minutes)
  - **Method:** Run system continuously for >60 minutes
  - **Expected:**
    - GetMicros() continues working
    - Sensor rate calculations still accurate
    - No timing glitches

### Error Recovery Tests

- [ ] **Test S.3:** I2C arbiter timeout recovery
  - **Method:** Disconnect MAG mid-operation, verify watchdog recovery
  - **Expected:**
    - Arbiter releases after 100ms
    - System continues with remaining sensors
    - Error counters increment

- [ ] **Test S.4:** QSPI DMA timeout recovery
  - **Method:** Simulate QSPI failure during write
  - **Expected:**
    - Timeout after 100ms
    - Logger enters ERROR state
    - Previous data intact
    - Can restart after recovery

- [ ] **Test S.5:** Sensor initialization failure recovery
  - **Method:** Disconnect one sensor, initialize system
  - **Expected:**
    - System initializes other sensors
    - Fails gracefully for disconnected sensor
    - Continues operation
    - validity flags set correctly

### High-Rate Operation Tests

- [ ] **Test S.6:** Multiple rapid BLE commands
  - **Method:** Send 100 commands rapidly (< 1 second)
  - **Expected:**
    - All commands processed correctly
    - No buffer overflows
    - Responses for all commands
    - No crashes

- [ ] **Test S.7:** Maximum sensor data rate
  - **Method:** All sensors at maximum ODR simultaneously
  - **Expected:**
    - No dropped data
    - I2C arbiter handles load
    - DMA transfers complete successfully
    - CPU usage acceptable

---

## Hardware-Specific Tests (If Available)

### Oscilloscope Verification

- [ ] **Test H.1:** QSPI write timing
  - **Method:** Probe QSPI CLK and CS lines during write
  - **Expected:** No glitches, proper CS timing, DMA completion verified

- [ ] **Test H.2:** I2C bus arbitration
  - **Method:** Probe I2C SDA/SCL during multi-sensor operation
  - **Expected:** No bus collisions, proper START/STOP conditions

### Power Cycle Tests

- [ ] **Test H.3:** Metadata persistence after unexpected power loss
  - **Method:** Record data, pull power during recording (various times)
  - **Expected:** Last metadata save recoverable, data up to last save intact

- [ ] **Test H.4:** Flash data integrity after 100 power cycles
  - **Method:** Write test pattern, power cycle 100 times
  - **Expected:** Data still valid, no corruption

---

## Regression Tests

### Existing Functionality Preserved

- [ ] **Test R.1:** Mahony filter still converges correctly
  - **Method:** Static orientation test, rotation test
  - **Expected:** Quaternion converges to correct values within seconds

- [ ] **Test R.2:** GPS data parsing unchanged
  - **Method:** Connect GPS, verify position/velocity/DOP parsing
  - **Expected:** All UBX fields parsed correctly

- [ ] **Test R.3:** Sensor scaling factors unchanged
  - **Method:** Compare sensor readings before/after refactoring
  - **Expected:** Identical scaling (within floating-point precision)

---

## Performance Benchmarks

### Timing Measurements

- [ ] **Bench P.1:** Main loop rate
  - **Target:** ≥500Hz
  - **Method:** Measure HAL_GetTick() delta in main loop
  - **Expected:** Consistent 500Hz+ with all sensors active

- [ ] **Bench P.2:** DataLogger_RecordData() execution time
  - **Target:** <2ms
  - **Method:** Measure time from entry to exit
  - **Expected:** Completes within 2ms (DMA wait included)

- [ ] **Bench P.3:** I2C_DMA_Arbiter_RequestTransfer() latency
  - **Target:** <100µs for acquire
  - **Method:** Measure time to acquire arbiter when free
  - **Expected:** Minimal overhead, <100µs

- [ ] **Bench P.4:** SensorManager_GetMahonyData() with calibration
  - **Target:** <50µs
  - **Method:** Measure with calibration enabled vs disabled
  - **Expected:** <10% overhead when calibration enabled

### Memory Usage

- [ ] **Bench M.1:** RAM usage
  - **Method:** Check linker map, measure stack/heap usage
  - **Expected:**
    - Calibration: +72 bytes (SensorCalibration_t)
    - Overflow tracking: +12 bytes
    - Nav provider pointer: +4 bytes
    - Total increase: <100 bytes

- [ ] **Bench M.2:** Code size increase
  - **Method:** Compare binary size before/after refactoring
  - **Expected:** <2KB increase (inline functions, calibration code)

---

## Code Quality Checks

### Static Analysis

- [ ] **Code Q.1:** No compiler warnings (-Wall -Wextra)
  - **Method:** Clean build with all warnings enabled
  - **Expected:** Zero warnings

- [ ] **Code Q.2:** No unused variables
  - **Method:** Compiler analysis + manual review
  - **Expected:** All variables either used or explicitly marked unused

- [ ] **Code Q.3:** All TODOs resolved
  - **Method:** Search for "TODO" comments
  - **Expected:** No outstanding TODOs from refactoring

### Code Review

- [ ] **Code Q.4:** Consistent style
  - **Method:** Manual review of new/modified code
  - **Expected:**
    - Function naming: ModuleName_FunctionName()
    - Typedefs end with _t
    - Doxygen comments for public functions

- [ ] **Code Q.5:** No magic numbers
  - **Method:** Review constants
  - **Expected:** All values #defined with meaningful names

---

## Documentation Verification

- [ ] **Doc V.1:** CLAUDE.md updated with all patterns
  - **Verify:**
    - Navigation provider pattern (Pattern #11)
    - Calibration framework (Pattern #12)
    - Critical sections (Pattern #13)
    - Overflow tracking (Pattern #14)
    - Dirty flags (Pattern #15)

- [ ] **Doc V.2:** error_codes.h complete
  - **Verify:**
    - All modules have error code ranges
    - All 10 error patterns documented
    - Common scenarios documented

- [ ] **Doc V.3:** Code comments match implementation
  - **Method:** Review function headers vs actual behavior
  - **Expected:** No stale comments from old implementations

---

## Final Acceptance Criteria

### All Systems Go ✅

- [ ] All unit tests pass
- [ ] All integration tests pass
- [ ] All stress tests pass
- [ ] No memory leaks detected
- [ ] Performance benchmarks met
- [ ] Code quality checks pass
- [ ] Documentation complete and accurate

### Ready for EKF/LoRa Integration ✅

- [ ] Navigation provider interface tested and working
- [ ] Calibration framework ready for EKF use
- [ ] Error handling patterns documented
- [ ] No race conditions in shared resources
- [ ] Flash management reliable for long flights

---

## Notes

**Testing Priority:**
1. **Critical (Must Pass):** Unit tests for race conditions, DMA handling
2. **High (Should Pass):** Integration tests for system stability
3. **Medium (Nice to Have):** Stress tests for edge cases
4. **Low (Optional):** Hardware-specific tests (if equipment available)

**Estimated Testing Time:**
- Unit Tests: 2-3 hours
- Integration Tests: 3-4 hours
- Stress Tests: 4-6 hours (includes long-duration tests)
- **Total:** 10-13 hours

**Testing Environment:**
- STM32G4 development board
- All sensors connected (IMU, MAG, BARO, HIGHG, GPS, BLE)
- QSPI flash memory
- USB connection for monitoring
- Logic analyzer (optional, for hardware tests)

**Pass/Fail Criteria:**
- **Pass:** All critical and high-priority tests pass
- **Conditional Pass:** Some medium tests fail with documented workarounds
- **Fail:** Any critical test fails - requires debugging before EKF integration
