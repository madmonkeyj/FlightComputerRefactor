/**
  ******************************************************************************
  * @file    battery_monitor.c
  * @brief   Battery voltage monitoring implementation
  * @note    Uses ADC1 with internal VBAT channel
  ******************************************************************************
  */

#include "battery_monitor.h"
#include "adc.h"

/**
 * @brief Read battery voltage using internal VBAT ADC channel
 */
float ReadBatteryVoltage(void) {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion to complete (max 100ms timeout)
    HAL_ADC_PollForConversion(&hadc1, 100);

    // Get ADC value (12-bit: 0-4095)
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    // STM32G4 VBAT channel:
    // - Internal voltage divider by 3
    // - 12-bit ADC (4095 max)
    // - 3.3V reference voltage
    // Formula: V_BAT = (ADC_value * 3.3V * 3) / 4095
    float voltage = (adc_value * 3.3f * 3.0f) / 4095.0f;

    return voltage;
}
