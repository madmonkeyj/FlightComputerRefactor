/**
  ******************************************************************************
  * @file    debug_utils.h
  * @brief   Debug and utility functions with printf support
  ******************************************************************************
  */

#ifndef DEBUG_UTILS_H_
#define DEBUG_UTILS_H_

#include "main.h"
#include <stdarg.h>

/* Function prototypes */
void DebugPrint(const char *format, ...);
void DebugPrintSimple(const char *str);
uint32_t DebugGetDroppedCount(void);

#endif /* DEBUG_UTILS_H_ */
