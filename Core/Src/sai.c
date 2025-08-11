/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : SAI.c
  * Description        : This file provides code for the configuration
  *                      of the SAI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sai.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SAI_HandleTypeDef hsai_BlockA3;
SAI_HandleTypeDef hsai_BlockB3;
DMA_HandleTypeDef hdma_sai3_a;
DMA_HandleTypeDef hdma_sai3_b;

/* SAI3 init function */
void MX_SAI3_Init(void)
{

  /* USER CODE BEGIN SAI3_Init 0 */

  /* USER CODE END SAI3_Init 0 */

  /* USER CODE BEGIN SAI3_Init 1 */

  /* USER CODE END SAI3_Init 1 */

  /* USER CODE BEGIN SAI3_Init 1 */

  /* USER CODE END SAI3_Init 1 */

  hsai_BlockA3.Instance = SAI3_Block_A;
  hsai_BlockA3.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA3.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA3.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA3.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA3.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA3.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA3.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA3.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA3.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA3.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA3, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB3.Instance = SAI3_Block_B;
  hsai_BlockB3.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB3.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB3.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB3.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB3.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB3.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB3.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB3.Init.TriState = SAI_OUTPUT_RELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB3, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI3_Init 2 */

  /* USER CODE END SAI3_Init 2 */

}
static uint32_t SAI3_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* saiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
/* SAI3 */
    if(saiHandle->Instance==SAI3_Block_A)
    {
    /* SAI3 clock enable */
    if (SAI3_client == 0)
    {
       __HAL_RCC_SAI3_CLK_ENABLE();
    }
    SAI3_client ++;

    /**SAI3_A_Block_A GPIO Configuration
    PD15     ------> SAI3_MCLK_A
    PD0     ------> SAI3_SCK_A
    PD1     ------> SAI3_SD_A
    PD4     ------> SAI3_FS_A
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_sai3_a.Instance = DMA1_Stream0;
    hdma_sai3_a.Init.Request = DMA_REQUEST_SAI3_A;
    hdma_sai3_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai3_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai3_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai3_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai3_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai3_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai3_a.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_sai3_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai3_a) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(saiHandle,hdmarx,hdma_sai3_a);
    __HAL_LINKDMA(saiHandle,hdmatx,hdma_sai3_a);
    }
    if(saiHandle->Instance==SAI3_Block_B)
    {
      /* SAI3 clock enable */
      if (SAI3_client == 0)
      {
       __HAL_RCC_SAI3_CLK_ENABLE();
      }
    SAI3_client ++;

    /**SAI3_B_Block_B GPIO Configuration
    PD9     ------> SAI3_SD_B
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_sai3_b.Instance = DMA1_Stream1;
    hdma_sai3_b.Init.Request = DMA_REQUEST_SAI3_B;
    hdma_sai3_b.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sai3_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai3_b.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai3_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai3_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai3_b.Init.Mode = DMA_CIRCULAR;
    hdma_sai3_b.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_sai3_b.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_sai3_b) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(saiHandle,hdmarx,hdma_sai3_b);
    __HAL_LINKDMA(saiHandle,hdmatx,hdma_sai3_b);
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* saiHandle)
{

/* SAI3 */
    if(saiHandle->Instance==SAI3_Block_A)
    {
    SAI3_client --;
    if (SAI3_client == 0)
      {
      /* Peripheral clock disable */
       __HAL_RCC_SAI3_CLK_DISABLE();
      }

    /**SAI3_A_Block_A GPIO Configuration
    PD15     ------> SAI3_MCLK_A
    PD0     ------> SAI3_SCK_A
    PD1     ------> SAI3_SD_A
    PD4     ------> SAI3_FS_A
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4);

    HAL_DMA_DeInit(saiHandle->hdmarx);
    HAL_DMA_DeInit(saiHandle->hdmatx);
    }
    if(saiHandle->Instance==SAI3_Block_B)
    {
    SAI3_client --;
      if (SAI3_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI3_CLK_DISABLE();
      }

    /**SAI3_B_Block_B GPIO Configuration
    PD9     ------> SAI3_SD_B
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9);

    HAL_DMA_DeInit(saiHandle->hdmarx);
    HAL_DMA_DeInit(saiHandle->hdmatx);
    }
}

/**
  * @}
  */

/**
  * @}
  */
