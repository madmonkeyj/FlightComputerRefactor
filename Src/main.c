/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - With GPS Integration
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
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

/* NEW MODULE INCLUDES - Navigation and Telemetry */
#include "sensor_adapter.h"       // Adapter: sensor_manager → navigation format
#include "navigation_manager.h"   // AHRS + EKF coordinator (NOW WITH ADAPTER!)
// #include "telemetry_manager.h"    // Telemetry coordination (disabled - requires LoRa)
// #include "lora_module.h"          // LoRa wireless (disabled - UART conflict with BLE)
// #include "flight_manager.h"       // Top-level orchestrator (disabled - requires telemetry)

#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* BLE Test Variables */
static uint32_t ble_test_counter = 0;
static bool ble_initialized = false;

/* NOTE: Mahony filter now managed internally by navigation_manager */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* BLE Command Callback */
static void BLE_CommandCallback(const char* command);

/* Navigation Provider Callbacks for Data Logger */
static bool NavProvider_GetQuaternion(float quat[4]);
static bool NavProvider_GetPositionNED(float pos[3]);
static bool NavProvider_GetVelocityNED(float vel[3]);
static bool NavProvider_IsValid(void);
static bool NavProvider_GetPositionUncertainty(float uncertainty[3]);
static bool NavProvider_GetVelocityUncertainty(float uncertainty[3]);
static bool NavProvider_GetInnovationPosition(float innovation[3]);
static bool NavProvider_GetInnovationVelocity(float innovation[3]);
static bool NavProvider_GetKalmanGainPosition(float gain[3]);
static bool NavProvider_GetKalmanGainVelocity(float gain[3]);
static bool NavProvider_GetAccelNED(float accel_ned[3]);
static bool NavProvider_GetRejectionFlags(uint8_t* gps_pos_rejected, uint8_t* gps_vel_rejected, uint8_t* zupt_applied);
static bool NavProvider_GetMotionState(uint8_t* motion_state, uint8_t* gps_velocity_suspect);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Navigation provider callback implementations
 * @note Using navigation_manager with full EKF integration!
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

static bool NavProvider_GetPositionUncertainty(float uncertainty[3]) {
    return NavigationManager_GetPositionUncertainty(uncertainty);
}

static bool NavProvider_GetVelocityUncertainty(float uncertainty[3]) {
    return NavigationManager_GetVelocityUncertainty(uncertainty);
}

static bool NavProvider_GetInnovationPosition(float innovation[3]) {
    return NavigationManager_GetInnovationPosition(innovation);
}

static bool NavProvider_GetInnovationVelocity(float innovation[3]) {
    if (!innovation) return false;

    // Mark that we got here – harmless dummy values
    innovation[0] = 0.0f;
    innovation[1] = 0.0f;
    innovation[2] = 0.0f;

    // IMPORTANT: return false so the logger still fills NANs,
    // i.e. we don’t change any logged data format/meaning yet.
    return false;
}


static bool NavProvider_GetKalmanGainPosition(float gain[3]) {
    return NavigationManager_GetKalmanGainPosition(gain);
}

static bool NavProvider_GetKalmanGainVelocity(float gain[3]) {
    return NavigationManager_GetKalmanGainVelocity(gain);
}

static bool NavProvider_GetAccelNED(float accel_ned[3]) {
    return NavigationManager_GetAccelNED(accel_ned);
}

static bool NavProvider_GetRejectionFlags(uint8_t* gps_pos_rejected,
                                           uint8_t* gps_vel_rejected,
                                           uint8_t* zupt_applied) {
    return NavigationManager_GetRejectionFlags(gps_pos_rejected, gps_vel_rejected, zupt_applied);
}

static bool NavProvider_GetMotionState(uint8_t* motion_state, uint8_t* gps_velocity_suspect) {
    return NavigationManager_GetMotionState(motion_state, gps_velocity_suspect);
}

/* Navigation provider structure with full EKF diagnostics */
static const NavigationProvider_t nav_provider = {
    .get_quaternion = NavProvider_GetQuaternion,
    .get_position_ned = NavProvider_GetPositionNED,
    .get_velocity_ned = NavProvider_GetVelocityNED,
    .is_valid = NavProvider_IsValid,

    .get_position_uncertainty = NavProvider_GetPositionUncertainty,
    .get_velocity_uncertainty = NavProvider_GetVelocityUncertainty,
    .get_innovation_position = NavProvider_GetInnovationPosition,

    // We will *not* call this from the logger:
    .get_innovation_velocity = NavProvider_GetInnovationVelocity,

    .get_kalman_gain_position = NavProvider_GetKalmanGainPosition,
    .get_kalman_gain_velocity = NavProvider_GetKalmanGainVelocity,
    .get_accel_ned = NavProvider_GetAccelNED,
    .get_rejection_flags = NavProvider_GetRejectionFlags,
    .get_motion_state = NavProvider_GetMotionState
};

/**
 * @brief BLE command callback - handles custom commands
 * @param command The received command string
 * @note Built-in commands (start, stop, status, help, erase) are handled by BLE module
 * @note This callback is for additional custom commands
 */
static void BLE_CommandCallback(const char* command) {
    char response[256];

    // Navigation diagnostics command
    if (strcmp(command, "nav") == 0) {
        NavigationManager_GetDiagnosticsString(response, sizeof(response));
        BLE_SendResponse(response);
    }
    // GPS status command
    else if (strcmp(command, "gps") == 0) {
        GPS_Data_t gps;
        if (GPS_GetCurrentData(&gps)) {
            snprintf(response, sizeof(response),
                    "GPS: Fix=%c, Sats=%d, Spd=%.1fm/s\r\nLat=%.6f, Lon=%.6f, Alt=%.1fm\r\n",
                    gps.fix_status, gps.satellites, gps.speed,
                    gps.latitude, gps.longitude, gps.altitude);
        } else {
            snprintf(response, sizeof(response), "GPS: No valid data\r\n");
        }
        BLE_SendResponse(response);
    }
    // Example: custom "test" command
    else if (strcmp(command, "test") == 0) {
        snprintf(response, sizeof(response), "BLE Test OK! Counter=%lu\r\n", ble_test_counter);
        BLE_SendResponse(response);
    }
    // Example: get system info
    else if (strcmp(command, "info") == 0) {
        snprintf(response, sizeof(response),
                "FlightComputer v1.0\r\nBLE Module Active\r\nUptime: %lu ms\r\n",
                HAL_GetTick());
        BLE_SendResponse(response);
    }
    // Unknown command
    else {
        snprintf(response, sizeof(response), "Unknown command: %s\r\nTry: nav, gps, info, test\r\n", command);
        BLE_SendResponse(response);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  // Wait for USB CDC to be ready
  HAL_Delay(2000);

  DebugPrint("=== FlightComputer Full Navigation System ===\r\n");

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
  // Full AHRS + EKF with sensor adapter for actual hardware
  if (NavigationManager_Init()) {
      DebugPrint("Navigation Manager initialized (AHRS 500Hz + EKF 50Hz)\r\n");
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

      // Register command callback for custom commands
      BLE_RegisterCommandCallback(BLE_CommandCallback);

      // Enable data transmission
      BLE_SetDataTransmissionEnabled(true);

      // Send welcome message
      BLE_SendResponse("=== FlightComputer Full Navigation ===\r\n");
      BLE_SendResponse("Built-in commands: start, stop, status, help, erase\r\n");
      BLE_SendResponse("Custom commands: test, info\r\n");
      BLE_SendResponse("Ready!\r\n");
  } else {
      DebugPrint("ERROR: BLE Module initialization failed!\r\n");
  }

  // ===== LORA MODULE INITIALIZATION =====
  // DISABLED: LoRa on USART1 conflicts with BLE
  // To enable: resolve UART conflict (move BLE to USART2 or use USART2 for LoRa)
  // GPIO pins already configured: M0=PB13, M1=PB12, AUX=PB14
  /*
  if (LoRa_Init()) {
      DebugPrint("LoRa module initialized\r\n");
  } else {
      DebugPrint("ERROR: LoRa init failed\r\n");
  }
  */

  // ===== TELEMETRY MANAGER INITIALIZATION =====
  // DISABLED: Requires LoRa module for transmission
  /*
  if (TelemetryManager_Init()) {
      DebugPrint("Telemetry Manager initialized\r\n");
      TelemetryManager_SetRateLimit(5.0f);  // 5Hz telemetry
  } else {
      DebugPrint("ERROR: Telemetry Manager init failed\r\n");
  }
  */

  // ===== DATA LOGGER INITIALIZATION =====
  if (DataLogger_Init()) {
      DebugPrint("Data Logger initialized successfully\r\n");

      // Register navigation provider (AHRS + EKF integration)
      DataLogger_RegisterNavProvider(&nav_provider);
      DebugPrint("Navigation provider registered (quaternion + position + velocity)\r\n");

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
  // DISABLED: Requires telemetry_manager and lora_module
  // Manual subsystem coordination used instead
  /*
  if (FlightManager_Init()) {
      DebugPrint("Flight Manager initialized - system ready\r\n");
  } else {
      DebugPrint("ERROR: Flight Manager init failed\r\n");
  }
  */

  DebugPrint("=== All systems initialized - entering main loop ===\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // ===== LED HEARTBEAT (visible debugging) =====
      static uint32_t led_counter = 0;
      if (++led_counter >= 500) {
          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          led_counter = 0;
      }

      // ===== NAVIGATION UPDATE =====
      NavSensorData_t nav_sensor_data;
      if (SensorAdapter_Read(&nav_sensor_data)) {
          NavigationManager_UpdateAHRS(&nav_sensor_data);
          // REMOVED: NavigationManager_UpdateNavigation(&nav_sensor_data);
          // Reason: UpdateAHRS already calls UpdateNavigation internally!
      }

      // ===== GPS UPDATE =====
      GPS_Update();
      GPS_Data_t gps_data;
      if (GPS_GetCurrentData(&gps_data)) {
          NavigationManager_UpdateGPS(&gps_data);  // Now only handles init
      }

      // ===== I2C WATCHDOG =====
      I2C_DMA_Arbiter_Watchdog();

      // ===== DATA LOGGER =====
      DataLogger_RecordData();
      DataLogger_Update();

      // ===== BLE MODULE =====
      if (ble_initialized) {
          BLE_Update();
      }

      // ===== USB COMMANDS =====
      USBCommands_Update();

      // Small delay
      HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief QSPI TX complete callback - for quadspi.c polling operations
 * @note Overrides __weak callback in quadspi.c
 */
extern volatile uint8_t qspi_tx_complete;  // From quadspi.c
extern volatile uint8_t qspi_error;  // Ensure this is declared extern

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) {
    // For quadspi.c polling operations
    qspi_tx_complete = 1;

    // Note: data_logger now uses blocking writes, no callback needed
}

/**
 * @brief QSPI error callback - for quadspi.c polling operations
 * @note Overrides __weak callback in quadspi.c
 */
extern volatile uint8_t qspi_error;  // From quadspi.c

void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi) {
    // For quadspi.c polling operations
    qspi_error = 1;
    qspi_tx_complete = 1; // Set complete flag to break any blocking wait loops!

    // Note: data_logger now uses blocking writes, no callback needed
}

/**
 * @note Other HAL callbacks:
 * @note - UART callbacks: defined in ble_module.c (for BLE communication)
 * @note - For LoRa support: add UART routing in ble_module.c or create runtime selection
 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
