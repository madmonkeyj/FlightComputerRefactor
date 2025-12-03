/**
  ******************************************************************************
  * @file    battery_monitor.h
  * @brief   Simple battery voltage monitoring via internal ADC
  * @note    Uses STM32 internal VBAT channel (ADC_CHANNEL_VBAT)
  ******************************************************************************
  */

#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include "main.h"
#include <stdint.h>

/**
 * @brief Read battery voltage using internal VBAT ADC channel
 * @return Battery voltage in volts
 * @note STM32 VBAT is internally divided by 3, this function compensates
 * @note ADC must be initialized before calling this function
 */
float ReadBatteryVoltage(void);

#endif /* BATTERY_MONITOR_H_ */
