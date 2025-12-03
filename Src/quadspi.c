/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.c
  * @brief   Modular QSPI Flash Driver - Fixed Init Sequence
  ******************************************************************************
  */
/* USER CODE END Header */
#include "quadspi.h"

/* --- COMMAND DEFINITIONS --- */
#define WRITE_ENABLE_CMD      0x06
#define READ_STATUS_REG2_CMD  0x35  // Winbond specific
#define WRITE_STATUS_REG2_CMD 0x31  // Winbond specific

/* USER CODE BEGIN 0 */
static void QSPI_ResetMemory(void);
static uint8_t QSPI_WriteEnable(void);
static uint8_t QSPI_EnterQuadMode(void);

volatile uint8_t qspi_tx_complete = 0;
volatile uint8_t qspi_error = 0;
/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi1;
DMA_HandleTypeDef hdma_quadspi;

/* QUADSPI1 init function */
void MX_QUADSPI1_Init(void)
{
  hqspi1.Instance = QUADSPI;
  hqspi1.Init.ClockPrescaler = 2; // ~56MHz (Safe)
  hqspi1.Init.FifoThreshold = 1;
  hqspi1.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi1.Init.FlashSize = 21;     // 4MB
  hqspi1.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi1.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi1.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi1.Init.DualFlash = QSPI_DUALFLASH_DISABLE;

  if (HAL_QSPI_Init(&hqspi1) != HAL_OK) Error_Handler();
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInit.QspiClockSelection = RCC_QSPICLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

    __HAL_RCC_QSPI_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PA6, PA7
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB0, PB1, PB10, PB11
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // DMA
    hdma_quadspi.Instance = DMA1_Channel7;
    hdma_quadspi.Init.Request = DMA_REQUEST_QUADSPI;
    hdma_quadspi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_quadspi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_quadspi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_quadspi.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_quadspi.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_quadspi.Init.Mode = DMA_NORMAL;
    hdma_quadspi.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_quadspi) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(qspiHandle,hdma,hdma_quadspi);

    HAL_NVIC_SetPriority(QUADSPI_IRQn, 0, 0);  // Match DMA priority to prevent starvation
    HAL_NVIC_EnableIRQ(QUADSPI_IRQn);
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{
  if(qspiHandle->Instance==QUADSPI)
  {
    __HAL_RCC_QSPI_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11);
    HAL_DMA_DeInit(qspiHandle->hdma);
    HAL_NVIC_DisableIRQ(QUADSPI_IRQn);
  }
}

/* ============================================================================
   PUBLIC API - INITIALIZATION
   ========================================================================= */

uint8_t QSPI_Simple_Init(void)
{
  /* 1. Basic HAL Init */
  MX_QUADSPI1_Init();

  /* 2. Apply Timing Workaround FIRST (STM32G4 Errata 2.6.2) */
  /* If we don't do this first, subsequent commands might fail! */
  CLEAR_BIT(QUADSPI->CR, QUADSPI_CR_EN);
  while(QUADSPI->SR & QUADSPI_SR_BUSY) {}

  QUADSPI->CR = 0xFF000001;  /* Max prescaler + enable */
  QUADSPI->CCR = 0x20000000; /* Free-running clock */
  QUADSPI->CCR = 0x20000000; /* Repeated per errata */

  CLEAR_BIT(QUADSPI->CR, QUADSPI_CR_EN);
  while(QUADSPI->SR & QUADSPI_SR_BUSY) {}

  SET_BIT(QUADSPI->CR, QUADSPI_CR_EN);

  /* 3. Reset Memory (Wakes from Quad Mode hangs) */
  QSPI_ResetMemory();

  /* 4. Enable Quad Mode (QE Bit) */
  /* Must happen after Reset and Workaround */
  if (QSPI_EnterQuadMode() != HAL_OK) {
      // If this fails, we return error, otherwise writes will fail silently
      return HAL_ERROR;
  }

  return HAL_OK;
}

uint8_t QSPI_Read_ID(uint8_t *id_buffer)
{
  QSPI_CommandTypeDef sCommand = {0};
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x9F;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.NbData = 3;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
  if (HAL_QSPI_Receive(&hqspi1, id_buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

/* ============================================================================
   PUBLIC API - ERASE OPERATIONS
   ========================================================================= */

uint8_t QSPI_Simple_Erase(uint32_t address)
{
  QSPI_CommandTypeDef sCommand = {0};
  if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = SECTOR_ERASE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = address;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
  return QSPI_AutoPollingMemReady(5000);  /* 5 second timeout for sector erase */
}

uint8_t CSP_QSPI_Erase_Chip(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = CHIP_ERASE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
  return QSPI_AutoPollingMemReady(60000);  /* 60 second timeout for chip erase (20-40s typical) */
}

/* ============================================================================
   PUBLIC API - WRITE OPERATIONS
   ========================================================================= */

uint8_t QSPI_Quad_Write(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};
  uint32_t current_addr = address;
  uint32_t remaining = size;
  uint32_t page_offset, write_size;

  while (remaining > 0) {
    page_offset = current_addr % MEMORY_PAGE_SIZE;
    write_size = MEMORY_PAGE_SIZE - page_offset;
    if (write_size > remaining) write_size = remaining;
    if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;

    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
    sCommand.Address = current_addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;
    sCommand.DummyCycles = 0;
    sCommand.NbData = write_size;
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Transmit(&hqspi1, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (QSPI_AutoPollingMemReady(1000) != HAL_OK) return HAL_ERROR;  /* 1 second timeout for page write */

    buffer += write_size;
    current_addr += write_size;
    remaining -= write_size;
  }
  return HAL_OK;
}

uint8_t QSPI_Quad_Write_DMA(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};
  uint32_t current_addr = address;
  uint32_t remaining = size;
  uint32_t page_offset, write_size;

  while (remaining > 0) {
    page_offset = current_addr % MEMORY_PAGE_SIZE;
    write_size = MEMORY_PAGE_SIZE - page_offset;
    if (write_size > remaining) write_size = remaining;
    if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;

    sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
    sCommand.Address = current_addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode = QSPI_DATA_4_LINES;
    sCommand.DummyCycles = 0;
    sCommand.NbData = write_size;
    sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    qspi_tx_complete = 0;
    qspi_error = 0;
    if (HAL_QSPI_Transmit_DMA(&hqspi1, buffer) != HAL_OK) return HAL_ERROR;

    uint32_t timeout = HAL_GetTick() + 1000;
    while (!qspi_tx_complete && !qspi_error && HAL_GetTick() < timeout) __NOP();
    if (qspi_error || !qspi_tx_complete) return HAL_ERROR;
    if (QSPI_AutoPoll_IT() != HAL_OK) return HAL_ERROR;

    buffer += write_size;
    current_addr += write_size;
    remaining -= write_size;
  }
  return HAL_OK;
}

uint8_t QSPI_Quad_Read(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = address;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_4_LINES;
  sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
  sCommand.NbData = size;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
  if (HAL_QSPI_Receive(&hqspi1, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

/* ============================================================================
   HELPER FUNCTIONS
   ========================================================================= */

static void QSPI_ResetMemory(void)
{
    QSPI_CommandTypeDef sCommand = {0};
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = 0x66; // Reset Enable
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.DummyCycles       = 0;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);

    sCommand.Instruction = 0x99; // Reset
    HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    HAL_Delay(20);
}

static uint8_t QSPI_EnterQuadMode(void)
{
    QSPI_CommandTypeDef sCommand = {0};
    uint8_t reg_val = 0;

    /* 1. Read Status Register 2 */
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = READ_STATUS_REG2_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.DummyCycles       = 0;
    sCommand.NbData            = 1;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Receive(&hqspi1, &reg_val, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

    /* If QE bit (bit 1) is already set, we are good */
    if (reg_val & 0x02) return HAL_OK;

    /* 2. Set QE Bit */
    reg_val |= 0x02;

    /* 3. Write Enable */
    if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;

    /* 4. Write Status Register 2 */
    sCommand.Instruction = WRITE_STATUS_REG2_CMD;
    sCommand.DataMode    = QSPI_DATA_1_LINE;
    sCommand.NbData      = 1;

    if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Transmit(&hqspi1, &reg_val, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

    /* 5. Wait for write to complete */
    return QSPI_AutoPollingMemReady(1000);  /* 1 second timeout for status register write */
}

static uint8_t QSPI_WriteEnable(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};

  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = WRITE_ENABLE_CMD;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

  sConfig.Match = 0x02;
  sConfig.Mask = 0x02;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
  sCommand.Instruction = READ_STATUS_REG_CMD; // SR1 for Write Enable Latch
  sCommand.DataMode = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

uint8_t QSPI_AutoPollingMemReady(uint32_t timeout_ms)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x05; // Read Status Reg 1
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match = 0x00;
  sConfig.Mask = 0x01;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
  /* FIX: Use configurable timeout instead of HAL_MAX_DELAY to prevent infinite hang */
  if (HAL_QSPI_AutoPolling(&hqspi1, &sCommand, &sConfig, timeout_ms) != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

uint8_t QSPI_AutoPoll_IT(void)
{
  QSPI_CommandTypeDef sCommand = {0};
  QSPI_AutoPollingTypeDef sConfig = {0};
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x05;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
  sConfig.Match = 0x00;
  sConfig.Mask = 0x01;
  sConfig.MatchMode = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval = 0x10;
  sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
  qspi_tx_complete = 0;
  if (HAL_QSPI_AutoPolling_IT(&hqspi1, &sCommand, &sConfig) != HAL_OK) return HAL_ERROR;
  uint32_t timeout = HAL_GetTick() + 1000;
  while (!qspi_tx_complete && HAL_GetTick() < timeout) __NOP();
  return qspi_tx_complete ? HAL_OK : HAL_TIMEOUT;
}

__weak void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi) {
    qspi_tx_complete = 1;
}

__weak void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi) {
    qspi_error = 1;
}

__weak void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi) {
    qspi_tx_complete = 1;
}

uint8_t QSPI_Read_Std(uint8_t *buffer, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef sCommand = {0};

  /* Standard Read (0x03) - 1 Line, No Dummy Cycles, Lowest Speed Requirement */
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction = 0x03;
  sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = address;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0;
  sCommand.NbData = size;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi1, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  if (HAL_QSPI_Receive(&hqspi1, buffer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}
