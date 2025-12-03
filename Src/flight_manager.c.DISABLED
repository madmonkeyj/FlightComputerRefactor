/**
  ******************************************************************************
  * @file    flight_manager.c
  * @brief   Flight Manager implementation - Updated to use TelemetryManager
  ******************************************************************************
  */

#include "flight_manager.h"
#include "debug_utils.h"
#include "sensor_manager.h"
#include "telemetry_manager.h"
#include "gps_module.h"
#include "ble_module.h"
#include <stdio.h>
#include <string.h>
#include "lora_module.h"

/* Private variables for timing control */
static bool flight_manager_initialized = false;
static uint32_t system_start_time = 0;

/* Timing control variables */
static uint32_t last_sensor_time = 0;
static uint32_t last_telemetry_time = 0;
static uint32_t last_gps_time = 0;
static uint32_t last_debug_time = 0;

/* System status tracking */
static FlightSystemStatus_t system_status = {0};

/* Update counters */
static uint32_t total_update_calls = 0;
static uint32_t total_errors = 0;

/* Private function prototypes */
static bool InitializeModules(void);
static void UpdateSensors(void);
static void UpdateTelemetry(void);
static void UpdateGPS(void);
static void UpdateDebugOutput(void);
static void UpdateSystemStatus(void);

/**
 * @brief Initialize flight management system
 */
bool FlightManager_Init(void) {
    DebugPrint("FlightManager: Starting system initialization...\r\n");

    /* Record system start time */
    system_start_time = HAL_GetTick();

    /* Clear system status */
    memset(&system_status, 0, sizeof(system_status));

    /* Initialize timing variables */
    uint32_t current_time = HAL_GetTick();
    last_sensor_time = current_time;
    last_telemetry_time = current_time;
    last_gps_time = current_time;
    last_debug_time = current_time;

    /* Reset counters */
    total_update_calls = 0;
    total_errors = 0;

    /* Initialize all modules */
    if (!InitializeModules()) {
        DebugPrint("FlightManager: ERROR - Module initialization failed\r\n");
        return false;
    }

    /* System ready */
    flight_manager_initialized = true;
    system_status.system_uptime = 0;

    DebugPrint("FlightManager: System initialization complete\r\n");
    return true;
}

/**
 * @brief Initialize all flight modules
 */
static bool InitializeModules(void) {
    bool all_success = true;

    /* Setup GPIO pins for modules */
    HAL_GPIO_WritePin(CONFIG_GPIO_Port, CONFIG_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LPM_GPIO_Port, LPM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RST_BT_GPIO_Port, RST_BT_Pin, GPIO_PIN_SET);
    HAL_Delay(500);

    /* Initialize BLE module */
    if (BLE_Init()) {
        system_status.ble_healthy = true;
        DebugPrint("FlightManager: ✓ BLE module initialized\r\n");
    } else {
        system_status.ble_healthy = false;
        DebugPrint("FlightManager: ✗ BLE module initialization failed\r\n");
    }

    /* Initialize sensor system */
    if (SensorSystem_Init()) {
        system_status.sensors_healthy = true;
        DebugPrint("FlightManager: ✓ Sensor system initialized\r\n");
    } else {
        system_status.sensors_healthy = false;
        DebugPrint("FlightManager: ✗ Sensor system initialization failed\r\n");
        all_success = false;
    }

    /* Initialize navigation manager */
    if (NavigationManager_Init()) {
        system_status.navigation_healthy = true;
        DebugPrint("FlightManager: ✓ Navigation manager initialized\r\n");
    } else {
        system_status.navigation_healthy = false;
        DebugPrint("FlightManager: ✗ Navigation manager initialization failed\r\n");
        all_success = false;
    }

    /* Initialize GPS */
    if (GPS_Init()) {
        system_status.gps_healthy = true;
        DebugPrint("FlightManager: ✓ GPS initialized\r\n");
    } else {
        system_status.gps_healthy = false;
        DebugPrint("FlightManager: ✗ GPS initialization failed\r\n");
    }

    /* Initialize telemetry manager */
    if (TelemetryManager_Init()) {
        system_status.telemetry_healthy = true;
        DebugPrint("FlightManager: ✓ Telemetry manager initialized\r\n");

        /* Configure telemetry for rocket operations */
        TelemetryManager_SetMode(TELEMETRY_MODE_NORMAL);
        TelemetryManager_SetRateLimit(5.0f);
        TelemetryManager_SetAutoTelemetry(true, 200);
    } else {
        system_status.telemetry_healthy = false;
        DebugPrint("FlightManager: ✗ Telemetry manager initialization failed\r\n");
    }

    return all_success;
}

/**
 * @brief Update flight management system - main coordination function
 */
void FlightManager_Update(void) {
    if (!flight_manager_initialized) {
        return;
    }

    total_update_calls++;
    uint32_t current_time = HAL_GetTick();

    /* Update modules */
    BLE_Update();
    LoRa_Update();
    TelemetryManager_Update();

    /* Critical path: Sensors + AHRS (500Hz) */
    if (current_time - last_sensor_time >= FLIGHT_SENSOR_INTERVAL_MS) {
        UpdateSensors();
        last_sensor_time = current_time;
    }

    /* Medium priority: Telemetry (5Hz) */
    if (current_time - last_telemetry_time >= FLIGHT_TELEMETRY_INTERVAL_MS) {
        UpdateTelemetry();
        last_telemetry_time = current_time;
    }

    /* Low priority: GPS (0.2Hz) */
    if (current_time - last_gps_time >= FLIGHT_GPS_INTERVAL_MS) {
        UpdateGPS();
        last_gps_time = current_time;
    }

    /* Debug output (100Hz) */
    if (current_time - last_debug_time >= FLIGHT_DEBUG_INTERVAL_MS) {
        UpdateDebugOutput();
        last_debug_time = current_time;
    }

    /* Update system status */
    UpdateSystemStatus();
}

/**
 * @brief Update sensors and navigation
 */
static void UpdateSensors(void) {
    SensorData_t sensor_data;

    /* SIMPLE RATE MEASUREMENT */
    static uint32_t sensor_count = 0;
    static uint32_t last_print = 0;

    sensor_count++;

    uint32_t now = HAL_GetTick();
    if (now - last_print >= 1000) { // Every 1 second
        char rate_msg[100];
        sprintf(rate_msg, "*** Sensor Rate: %lu Hz ***\r\n", sensor_count);
        DebugPrint(rate_msg);
        sensor_count = 0;
        last_print = now;
    }
    /* END RATE MEASUREMENT */

    // ===== ADD THESE LINES =====
    uint32_t t1 = HAL_GetTick();
    // ===========================

    /* Read sensors at 500Hz */
    if (SensorSystem_Read(&sensor_data)) {
        system_status.sensor_update_count++;

        // ===== ADD THESE LINES =====
        uint32_t t2 = HAL_GetTick();
        // ===========================

        /* Update AHRS at 500Hz */
        if (NavigationManager_UpdateAHRS(&sensor_data)) {

            // ===== ADD THESE LINES =====
            uint32_t t3 = HAL_GetTick();
            // ===========================

            /* AHRS update successful - update navigation at decimated rate */
            NavigationManager_UpdateNavigation(&sensor_data);

            // ===== ADD THESE LINES =====
            uint32_t t4 = HAL_GetTick();

            static uint32_t last_timing = 0;
            if (HAL_GetTick() - last_timing >= 1000) {
                char msg[100];
                sprintf(msg, "UpdateSensors timing: Read=%lu, AHRS=%lu, Nav=%lu ms\r\n",
                        t2-t1, t3-t2, t4-t3);
                DebugPrint(msg);
                last_timing = HAL_GetTick();
            }
            // ===========================

        } else {
            total_errors++;
        }
    } else {
        total_errors++;
    }
}

/**
 * @brief Update telemetry transmission
 */
static void UpdateTelemetry(void) {
    /* Get current data for telemetry */
    NavigationSolution_t nav_solution;
    SensorData_t sensor_data;
    GPS_Data_t gps_data;

    bool nav_ok = NavigationManager_GetSolution(&nav_solution);
    bool sensor_ok = SensorSystem_Read(&sensor_data);
    bool gps_ok = GPS_GetCurrentData(&gps_data);

    /* Send telemetry using TelemetryManager */
    if (sensor_ok) {
        bool result = TelemetryManager_SendRocketTelemetry(
            nav_ok ? &nav_solution : NULL,
            &sensor_data,
            gps_ok ? &gps_data : NULL
        );

        if (result) {
            system_status.telemetry_update_count++;
        } else {
            total_errors++;
        }
    } else {
        total_errors++;
    }
}

/**
 * @brief Update GPS data
 */
static void UpdateGPS(void) {
    /* Read GPS data at low frequency to avoid blocking */
    GPS_Update();
    system_status.gps_update_count++;

    /* Update navigation with GPS data if valid */
    if (GPS_IsDataValid()) {
        GPS_Data_t current_gps_data;
        if (GPS_GetCurrentData(&current_gps_data)) {
            NavigationManager_UpdateGPS(&current_gps_data);
        }
    }
}

/**
 * @brief Update debug output
 */
static void UpdateDebugOutput(void) {
    /* Get current navigation solution for debug output */
    NavigationSolution_t nav_solution;
    SensorData_t sensor_data;
    GPS_Data_t gps_data;

    bool nav_ok = NavigationManager_GetSolution(&nav_solution);
    bool sensor_ok = SensorSystem_Read(&sensor_data);
    bool gps_ok = GPS_GetCurrentData(&gps_data);

    if (nav_ok && sensor_ok) {
        /* Format and send navigation data */
        if (gps_ok) {
            FormatNavigationData(&nav_solution, &sensor_data);
        } else {
            FormatNavigationData(&nav_solution, &sensor_data);
        }
    }
}

/**
 * @brief Update system status tracking
 */
static void UpdateSystemStatus(void) {
    uint32_t current_time = HAL_GetTick();

    /* Update uptime */
    system_status.system_uptime = current_time - system_start_time;

    /* Update error count */
    system_status.error_count = total_errors;

    /* Check module health */
    system_status.sensors_healthy = SensorSystem_IsHealthy();
    system_status.navigation_healthy = NavigationManager_IsHealthy();
    system_status.gps_healthy = GPS_IsDataValid();
    system_status.telemetry_healthy = TelemetryManager_IsReady() && TelemetryManager_IsHealthy();
    system_status.ble_healthy = BLE_IsReady();
}

/**
 * @brief Get current system status
 */
FlightSystemStatus_t FlightManager_GetStatus(void) {
    return system_status;
}

/**
 * @brief Check if system is ready for operation
 */
bool FlightManager_IsReady(void) {
    if (!flight_manager_initialized) {
        return false;
    }

    /* Critical systems: sensors and navigation */
    bool critical_systems_ok = system_status.sensors_healthy &&
                              system_status.navigation_healthy;

    return critical_systems_ok;
}

/**
 * @brief Get system uptime
 */
uint32_t FlightManager_GetUptime(void) {
    if (!flight_manager_initialized) {
        return 0;
    }

    return HAL_GetTick() - system_start_time;
}

/**
 * @brief Force telemetry transmission
 */
bool FlightManager_ForceTelemetry(void) {
    if (!flight_manager_initialized) {
        return false;
    }

    UpdateTelemetry();
    return true;
}

/**
 * @brief Get formatted status string
 */
void FlightManager_GetStatusString(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return;
    }

    FlightSystemStatus_t status = FlightManager_GetStatus();

    /* Get TelemetryManager diagnostics */
    TelemetryDiagnostics_t diag;
    TelemetryManager_GetDiagnostics(&diag);

    snprintf(buffer, buffer_size,
        "FlightManager Status:\r\n"
        "  Uptime: %lu ms\r\n"
        "  Sensors: %s (%lu updates)\r\n"
        "  Navigation: %s\r\n"
        "  GPS: %s (%lu updates)\r\n"
        "  Telemetry: %s (%lu updates)\r\n"
        "  TelemetryMgr: %s (Success: %lu, Failed: %lu, Rate: %.1f%%)\r\n"
        "  BLE: %s\r\n"
        "  Total Errors: %lu\r\n"
        "  System Ready: %s\r\n",
        status.system_uptime,
        status.sensors_healthy ? "HEALTHY" : "FAILED", status.sensor_update_count,
        status.navigation_healthy ? "HEALTHY" : "FAILED",
        status.gps_healthy ? "HEALTHY" : "NO FIX", status.gps_update_count,
        status.telemetry_healthy ? "HEALTHY" : "FAILED", status.telemetry_update_count,
        TelemetryManager_IsReady() ? "READY" : "NOT_READY",
        diag.successful_transmissions, diag.failed_transmissions, diag.success_rate_percent,
        status.ble_healthy ? "HEALTHY" : "FAILED",
        status.error_count,
        FlightManager_IsReady() ? "YES" : "NO"
    );
}
