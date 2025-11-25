/**
  ******************************************************************************
  * @file    INTEGRATION_EXAMPLE_main.c
  * @brief   Integration example showing how to wire up refactored modules
  * @note    This is an EXAMPLE - adapt to your actual main.c
  ******************************************************************************
  */

#include "main.h"
#include "sensor_manager.h"
#include "data_logger.h"
#include "mahony_filter.h"
#include "gps_module.h"
#include "ble_module.h"
#include "i2c_dma_arbiter.h"
#include "debug_utils.h"

/* Private variables */
MahonyFilter_t mahony_filter;

/* ============================================================================
 * TASK 4: NAVIGATION PROVIDER INTEGRATION (Optional)
 * ============================================================================
 * These wrapper functions adapt Mahony filter to NavigationProvider_t interface
 */

bool NavProvider_GetQuaternion(float quat[4]) {
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

bool NavProvider_GetPositionNED(float pos[3]) {
    // If you don't have position estimation yet, return false
    // This will leave position fields zeroed in data logger
    return false;
}

bool NavProvider_GetVelocityNED(float vel[3]) {
    // If you don't have velocity estimation yet, return false
    // This will leave velocity fields zeroed in data logger
    return false;
}

bool NavProvider_IsValid(void) {
    // Return true if Mahony filter has converged
    // For now, just return true if initialized
    return true;
}

/* Navigation provider structure */
static const NavigationProvider_t mahony_nav_provider = {
    .get_quaternion = NavProvider_GetQuaternion,
    .get_position_ned = NavProvider_GetPositionNED,
    .get_velocity_ned = NavProvider_GetVelocityNED,
    .is_valid = NavProvider_IsValid
};

/* ============================================================================
 * SYSTEM INITIALIZATION
 * ============================================================================ */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_QUADSPI_Init(void);

int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();  // BLE
    MX_USART3_UART_Init();  // GPS
    MX_QUADSPI_Init();

    DebugPrint("=== FlightComputer Refactored - Starting ===\r\n");

    /* ========================================================================
     * SENSOR MANAGER INITIALIZATION
     * ======================================================================== */
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
        .use_baro = false
    };

    if (SensorManager_Init(&sensor_config) == HAL_OK) {
        DebugPrint("Sensor Manager initialized\r\n");
    } else {
        DebugPrint("ERROR: Sensor Manager init failed\r\n");
    }

    /* ========================================================================
     * MAHONY FILTER INITIALIZATION
     * ======================================================================== */
    // Initialize for launch pad (X-axis up) or standard (horizontal)
    Mahony_InitLaunchPad(&mahony_filter, 2.0f, 0.01f, 500.0f);
    DebugPrint("Mahony filter initialized (launch pad mode)\r\n");

    /* ========================================================================
     * GPS MODULE INITIALIZATION
     * ======================================================================== */
    if (GPS_Init()) {
        DebugPrint("GPS module initialized\r\n");
    } else {
        DebugPrint("ERROR: GPS init failed\r\n");
    }

    /* ========================================================================
     * BLE MODULE INITIALIZATION
     * ======================================================================== */
    if (BLE_Init()) {
        DebugPrint("BLE module initialized\r\n");
    } else {
        DebugPrint("ERROR: BLE init failed\r\n");
    }

    /* ========================================================================
     * DATA LOGGER INITIALIZATION
     * ======================================================================== */
    if (DataLogger_Init()) {
        DebugPrint("Data logger initialized\r\n");

        /* TASK 4: Register navigation provider (optional) */
        DataLogger_RegisterNavProvider(&mahony_nav_provider);
        DebugPrint("Navigation provider registered\r\n");
    } else {
        DebugPrint("ERROR: Data logger init failed\r\n");
    }

    /* ========================================================================
     * OPTIONAL: SENSOR CALIBRATION (TASK 8)
     * ======================================================================== */
    #if 0  // Enable if you have calibration data
    SensorCalibration_t calibration = {
        .gyro_bias = {0.001f, -0.002f, 0.0005f},
        .gyro_bias_valid = true,

        .accel_offset = {0.0f, 0.0f, 0.0f},
        .accel_scale = {1.0f, 1.0f, 1.0f},
        .accel_cal_valid = false,

        .mag_offset = {0.0f, 0.0f, 0.0f},
        .mag_scale = {{1,0,0}, {0,1,0}, {0,0,1}},
        .mag_cal_valid = false
    };
    SensorManager_SetCalibration(&calibration);
    DebugPrint("Sensor calibration applied\r\n");
    #endif

    DebugPrint("=== Initialization complete, entering main loop ===\r\n");

    /* ========================================================================
     * MAIN LOOP
     * ======================================================================== */
    uint32_t loop_count = 0;
    uint32_t last_status_print = 0;

    while (1)
    {
        /* ====================================================================
         * TASK 1: I2C ARBITER WATCHDOG (CRITICAL - MUST BE CALLED EVERY LOOP)
         * ==================================================================== */
        if (I2C_DMA_Arbiter_Watchdog()) {
            // Watchdog triggered - arbiter was stuck for >100ms
            DebugPrint("WARNING: I2C arbiter timeout recovery\r\n");
        }

        /* ====================================================================
         * TASK 2: DATA LOGGER PERIODIC UPDATE (CRITICAL - METADATA SAVES)
         * ==================================================================== */
        DataLogger_Update();  // Handles periodic metadata saves (every 10s)

        /* ====================================================================
         * SENSOR MANAGER UPDATE
         * ==================================================================== */
        SensorManager_RawData_t sensor_data;
        if (SensorManager_ReadRaw(&sensor_data) == HAL_OK) {

            /* Update Mahony filter if IMU and MAG data valid */
            if (sensor_data.imu_valid && sensor_data.mag_valid) {
                float gx, gy, gz, ax, ay, az, mx, my, mz;

                SensorManager_GetMahonyData(&sensor_data,
                    &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);

                // Full 9-DOF update (gyro + accel + mag)
                Mahony_Update(&mahony_filter, gx, gy, gz, ax, ay, az, mx, my, mz);
            }
            else if (sensor_data.imu_valid) {
                float gx, gy, gz, ax, ay, az, mx, my, mz;

                SensorManager_GetMahonyData(&sensor_data,
                    &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);

                // 6-DOF update (gyro + accel only)
                Mahony_UpdateIMU(&mahony_filter, gx, gy, gz, ax, ay, az);
            }
        }

        /* ====================================================================
         * GPS UPDATE
         * ==================================================================== */
        GPS_Update();

        /* ====================================================================
         * BLE UPDATE (TASK 5 - Cleaned up, no debug overhead)
         * ==================================================================== */
        BLE_Update();

        /* ====================================================================
         * DATA LOGGER - RECORD DATA IF ACTIVE
         * ==================================================================== */
        if (DataLogger_IsRecording()) {
            if (!DataLogger_RecordData()) {
                // Record failed - logger might be full or in error state
                LoggerStats_t stats;
                if (DataLogger_GetStats(&stats)) {
                    if (stats.status == LOGGER_ERROR) {
                        DebugPrint("ERROR: Data logger in error state\r\n");
                    }
                }
            }
        }

        /* ====================================================================
         * PERIODIC STATUS PRINT (every 5 seconds)
         * ==================================================================== */
        if ((HAL_GetTick() - last_status_print) > 5000) {
            last_status_print = HAL_GetTick();

            // Get sensor status
            SensorManager_Status_t status;
            SensorManager_GetStatus(&status);

            // Get logger status
            LoggerStats_t logger_stats;
            DataLogger_GetStats(&logger_stats);

            DebugPrint("Loop: %lu | Sensors: %.1f Hz | Logger: %lu records\r\n",
                loop_count, status.actual_rate_hz, logger_stats.records_written);
        }

        loop_count++;

        /* Small delay to achieve target loop rate (adjust as needed) */
        // HAL_Delay(1);  // Uncomment if loop runs too fast
    }
}

/* ============================================================================
 * TASK 2: QSPI DMA CALLBACKS (CRITICAL FOR FLASH WRITES)
 * ============================================================================
 * Add these to your HAL callback file or here in USER CODE section
 */

/**
 * @brief QSPI Tx Transfer completed callback
 * @note This must be called when QSPI DMA write completes
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    DataLogger_QSPI_WriteComplete();
}

/**
 * @brief QSPI error callback
 * @note This must be called when QSPI DMA write fails
 */
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
    DataLogger_QSPI_WriteError();
}

/* ============================================================================
 * PERIPHERAL INITIALIZATION FUNCTIONS (CUBEMX GENERATED)
 * ============================================================================
 * These would be generated by STM32CubeMX - shown here for completeness
 */

void SystemClock_Config(void)
{
    // Your clock configuration here (170 MHz for STM32G4)
}

static void MX_GPIO_Init(void)
{
    // Your GPIO initialization here
}

static void MX_I2C1_Init(void)
{
    // Your I2C1 initialization here (for MAG, BARO, HIGHG)
}

static void MX_SPI1_Init(void)
{
    // Your SPI1 initialization here (for IMU)
}

static void MX_USART1_UART_Init(void)
{
    // Your USART1 initialization here (for BLE)
}

static void MX_USART3_UART_Init(void)
{
    // Your USART3 initialization here (for GPS)
}

static void MX_QUADSPI_Init(void)
{
    // Your QSPI initialization here (for flash)
}

/**
 * ============================================================================
 * TESTING THE REFACTORED CODE
 * ============================================================================
 *
 * 1. BASIC FUNCTIONALITY TEST:
 *    - Compile and flash
 *    - Monitor debug output via USB CDC
 *    - Check "Initialization complete" message
 *    - Verify sensors reading (check status prints every 5s)
 *
 * 2. BLE COMMANDS TEST (Task 5 - cleaned up debug code):
 *    - Connect via BLE
 *    - Send: "help" -> should list commands
 *    - Send: "status" -> should show logger and BLE status
 *    - Send: "start" -> should start recording
 *    - Send: "status" -> should show records increasing
 *    - Send: "stop" -> should stop recording
 *
 * 3. DATA LOGGER TEST (Task 2 - DMA race fix + flash wear):
 *    - Start recording via BLE
 *    - Let run for 30+ seconds
 *    - Watch debug prints - should see records increasing
 *    - Check that metadata saves happen every 10 seconds (not every record!)
 *    - Stop recording
 *    - Power cycle board
 *    - Check that metadata survived (session ID should increment)
 *
 * 4. I2C ARBITER TEST (Task 1 - race condition + timeout):
 *    - All sensors running simultaneously
 *    - Watch for "I2C arbiter timeout recovery" messages
 *    - Should be rare/never in normal operation
 *    - If frequent, investigate sensor connections
 *
 * 5. NAVIGATION PROVIDER TEST (Task 4 - decoupling):
 *    - Record data with mahony_nav_provider registered
 *    - Download flash data after recording
 *    - Verify quaternion fields populated (not all zeros)
 *    - Position/velocity will be zero if callbacks return false
 *
 * 6. GETMICROS OVERFLOW TEST (Task 7 - extended range):
 *    - Run continuously for >25 seconds
 *    - Check sensor rate calculations still accurate
 *    - Should not glitch at 25-second mark anymore
 *
 * 7. CALIBRATION TEST (Task 8 - optional):
 *    - Enable calibration code block (set #if 1)
 *    - Verify sensor readings change by calibration amounts
 *    - Compare GetMahonyData output with/without calibration
 *
 * ============================================================================
 * CRITICAL INTEGRATION POINTS SUMMARY
 * ============================================================================
 *
 * REQUIRED (must add to work correctly):
 *   1. I2C_DMA_Arbiter_Watchdog() in main loop (Task 1)
 *   2. DataLogger_Update() in main loop (Task 2)
 *   3. HAL_QSPI_TxCpltCallback() calls DataLogger_QSPI_WriteComplete() (Task 2)
 *   4. HAL_QSPI_ErrorCallback() calls DataLogger_QSPI_WriteError() (Task 2)
 *
 * OPTIONAL (for enhanced functionality):
 *   5. DataLogger_RegisterNavProvider() for attitude logging (Task 4)
 *   6. SensorManager_SetCalibration() for improved accuracy (Task 8)
 *
 * ============================================================================
 */
