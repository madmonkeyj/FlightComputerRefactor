/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.h
  * @brief   Modular QSPI Flash Driver for STM32G4xx
  * @version 2.3
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __QUADSPI_H__
#define __QUADSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================================================================
   PUBLIC CONSTANTS - Memory Parameters
   ========================================================================= */
#define MEMORY_FLASH_SIZE    0x400000  /* 32Mbit = 4MB */
#define MEMORY_SECTOR_SIZE   0x1000    /* 4KB sectors */
#define MEMORY_PAGE_SIZE     0x100     /* 256 byte pages */

/* ============================================================================
   PUBLIC API - Initialization
   ========================================================================= */

uint8_t QSPI_Simple_Init(void);
uint8_t QSPI_Read_ID(uint8_t *id_buffer);

/* ============================================================================
   PUBLIC API - Erase Operations
   ========================================================================= */

uint8_t QSPI_Simple_Erase(uint32_t address);
uint8_t CSP_QSPI_Erase_Chip(void);

/* ============================================================================
   PUBLIC API - Write Operations
   ========================================================================= */

/* FIX: Added missing prototype for Blocking Write */
uint8_t QSPI_Quad_Write(uint8_t *buffer, uint32_t address, uint32_t size);

/* DMA Write */
uint8_t QSPI_Quad_Write_DMA(uint8_t *buffer, uint32_t address, uint32_t size);

/* ============================================================================
   PUBLIC API - Read Operations
   ========================================================================= */

uint8_t QSPI_Quad_Read(uint8_t *buffer, uint32_t address, uint32_t size);
uint8_t QSPI_Read_Std(uint8_t *buffer, uint32_t address, uint32_t size);

/* ============================================================================
   INTERNAL FUNCTIONS
   ========================================================================= */

extern QSPI_HandleTypeDef hqspi1;

void MX_QUADSPI1_Init(void);
uint8_t QSPI_Timing_Workaround(void);
uint8_t QSPI_AutoPollingMemReady(uint32_t timeout_ms);
uint8_t QSPI_Fast_AutoPolling(void);
uint8_t QSPI_AutoPoll_IT(void);

/* ============================================================================
   PRIVATE DEFINES
   ========================================================================= */
#define CHIP_ERASE_CMD           0xC7
#define READ_STATUS_REG_CMD      0x05
#define WRITE_ENABLE_CMD         0x06
#define SECTOR_ERASE_CMD         0x20
#define QUAD_IN_FAST_PROG_CMD    0x32
#define QUAD_OUT_FAST_READ_CMD   0x6B
#define DUMMY_CLOCK_CYCLES_READ_QUAD 8

#ifdef __cplusplus
}
#endif

#endif /* __QUADSPI_H__ */
