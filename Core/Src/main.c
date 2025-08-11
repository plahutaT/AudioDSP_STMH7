/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "rng.h"
#include "sai.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"
#include "cli.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "tlv320aic3204.h"
#include "string.h"
#include "filter_fisrtorder.h"
#include "firfilter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * TODO play around to get the correct buffer size
 * 		buffer size is related to the delay
 * 		bigger buffer allows for more processing time
 * 		but it brings larger delays
 * */


/*
 * TODO  CHange the C24 and C32 values from 10n to 10u
 * 		 with the value 10n the -3dB for the HP is around 2 kHz witch is criminal
 * 		 with the value 10u the -3dB for the HP is around 200 Hz
 *
 * */


#define BUFFER_SIZE 128
#define INT16TOFLOAT (1.0f)/(32768.0f)
#define FLOATTOINT16 32768.0f

#define SAMPLE_RATE 48000  // Codec's sample rate (48kHz)
#define SINE_FREQ   1000    // Frequency of the sine wave (e.g., A4 = 440Hz)
#define PI          3.14159265358979323846f

#define SAMPLES_PER_WAVE (SAMPLE_RATE / SINE_FREQ)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cmd_usb_buffer[64];
uint8_t cmd_length;

int16_t adc_data[BUFFER_SIZE];
int16_t dac_data[BUFFER_SIZE];
uint8_t data_ready_f = 0;


/* pInBuff and pOutBuff point to the half of the buffer where the data is ready to be processed or sent  */
static volatile int16_t *pInBuff;
static volatile int16_t *pOutBuff = &dac_data[0];


int16_t sine_wave_buffer[2 * SAMPLES_PER_WAVE];  // Buffer to hold one cycle of the sine wave
int16_t sine_wave_recieve_buffer[2 * SAMPLES_PER_WAVE];

/* FX Processing */

LowPass_FirstOrder_t LowPass_Filt_L;
LowPass_FirstOrder_t LowPass_Filt_R;

FirFilter_t LowPass_FIRFilt_L;
FirFilter_t LowPass_FIRFilt_R;


/* Codec driver  */


void play_sine_wave(void);
void processData(void);
void processSineWave(void);


/* We are generating sine wave in the full scale of int16_t range  */

void generate_sine_wave(void)
{
    for (int i = 0; i < SAMPLES_PER_WAVE; i++)
    {
        // Generate a sine wave scaled to 16-bit signed integer range (-32768 to 32767)
        sine_wave_buffer[i * 2] = (int16_t)(32767 * sinf(2 * PI * i / SAMPLES_PER_WAVE)); // Left channel
        sine_wave_buffer[i * 2 + 1] = sine_wave_buffer[i * 2]; // Right channel (copying the same value for mono)
    }
}

void fill_sine_wave_buffer_with_zero(uint16_t *buff) {
    for (int i = 0; i < 2 * SAMPLES_PER_WAVE; i++) {
        buff[i] = 0;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_SAI3_Init();
  //MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  //uint32_t rng;
  uint8_t loop_itertation = 0;

  HAL_Delay(10);

  /*    HARDWARE RESET   */
  //tlv320aic3204_hardwareReset(nCodecRST_GPIO_Port, nCodecRST_Pin);
  tlv320aic3204_Init();

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //SCB_DisableDCache();


  //setbuf(stdout, NULL);

  generate_sine_wave();


  LowPass_FirstOrder_Init(&LowPass_Filt_L, 1000.0f, SAMPLE_RATE);
  LowPass_FirstOrder_Init(&LowPass_Filt_R, 1000.0f, SAMPLE_RATE);

  FirFilter_Init(&LowPass_FIRFilt_L);
  FirFilter_Init(&LowPass_FIRFilt_R);
  //fill_sine_wave_buffer_with_zero((uint16_t *)sine_wave_buffer);

  HAL_GPIO_WritePin(PE3_GPIO_Port, PE3_Pin, GPIO_PIN_SET);


  //processSineWave();

  SCB_CleanDCache_by_Addr((uint32_t*) adc_data, sizeof(adc_data));
  SCB_CleanDCache_by_Addr((uint32_t*) dac_data, sizeof(dac_data));


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (loop_itertation == 0)
	  {
		  if (HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_SET)
		  {


			  play_sine_wave();

			  /*
			  if (HAL_SAI_Receive_DMA(&hsai_BlockB3, (uint8_t *) adc_data, BUFFER_SIZE) != HAL_OK)
			  {
				  Error_Handler();
			  }
			  if (HAL_SAI_Transmit_DMA(&hsai_BlockA3, (uint8_t *) dac_data, BUFFER_SIZE) != HAL_OK)
			  {
				  Error_Handler();
			  }
			  */

			  HAL_GPIO_TogglePin(PE3_GPIO_Port, PE3_Pin);
			  loop_itertation = 1;

		  }
	  }


	  //HAL_Delay(1000);

	  /*
	   * When data is ready to be processed we apply our effects
	   *
	   * */
	  if (data_ready_f)
	  {
		  processData();
	  }



	  // Check if data has been received over USB VCP
	  if (cmd_length > 0) {
		  cli_processCommand(cmd_usb_buffer);  // Process the command
		  cmd_length = 0;  // Reset the buffer after processing
	  }





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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI3;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 24;
  PeriphClkInitStruct.PLL2.PLL2P = 25;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 4719;
  PeriphClkInitStruct.Sai23ClockSelection = RCC_SAI23CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
 *  The DMA has sent over the first part of the buffer so we can
 *  process the first part of the buffer and then send it
 * */

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	SCB_CleanDCache_by_Addr((uint32_t*) (dac_data + (BUFFER_SIZE / 2)) , sizeof(int16_t) * (BUFFER_SIZE / 2));
	pOutBuff = &dac_data[0];    // pOutBuff points to the first part of the buffer where we just processed data

}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	SCB_InvalidateDCache_by_Addr((uint32_t*) (adc_data), sizeof(int16_t) * (BUFFER_SIZE / 2));
	pInBuff = &adc_data[0];      // pInBuff points to the first part of the buffer where we just received data
	data_ready_f = 1;
}


/*
 *  The DMA has sent over the last part of the buffer so we can
 *  process the last part of the buffer and then send it
 * */



void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	SCB_CleanDCache_by_Addr((uint32_t*) (dac_data) , sizeof(int16_t) * (BUFFER_SIZE / 2));
	pOutBuff = &dac_data[BUFFER_SIZE/2];    // pOutBuff points to the second part of the buffer where we just processed data
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	SCB_InvalidateDCache_by_Addr((uint32_t*) (adc_data + (BUFFER_SIZE / 2)), sizeof(int16_t) * (BUFFER_SIZE / 2));
	pInBuff = &adc_data[BUFFER_SIZE/2];    // pInBuff points to the first part of the buffer where we just received data
	data_ready_f = 1;
}


// Function to start sending the sine wave over I2S
void play_sine_wave(void)
{
    // The sine_wave_buffer contains one cycle of the sine wave. To play the sine wave continuously,
    // you can use HAL_SAI_Transmit_DMA to send the buffer over I2S in a loop.
	SCB_CleanDCache_by_Addr((uint32_t*)sine_wave_buffer, sizeof(sine_wave_buffer));
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA3, (uint8_t *)sine_wave_buffer, 2 * SAMPLES_PER_WAVE) != HAL_OK)
    {
        // Transmission Error
        Error_Handler();
    }
}


void processData(void)
{

	static float inLeft, inRight;
	static float outLeft, outRight;

	for (uint8_t n = 0; n < (BUFFER_SIZE/2) - 1; n += 2)
	{

		// Left Channel

		inLeft = pInBuff[n];

		inLeft = INT16TOFLOAT * pInBuff[n];

		if (inLeft > 1.0f)
		{
			inLeft -= 2.0f;
		}

		// Data processing outLeft = f(inLeft)

		outLeft = LowPass_FirstOrder_Update(&LowPass_Filt_L, inLeft);
		//outLeft = FirFilter_Update(&LowPass_FIRFilt_L, inLeft);
//		outLeft = inLeft;

		pOutBuff[n] = (int16_t)(FLOATTOINT16 * outLeft);


		// Right Channel

		inRight = pInBuff[n + 1];

		inRight = INT16TOFLOAT * pInBuff[n + 1];

				if (inRight > 1.0f)
				{
					inRight -= 2.0f;
				}

		// Data processing outRight = f(inRight)

		outRight = LowPass_FirstOrder_Update(&LowPass_Filt_R, inRight);
		//outRight = FirFilter_Update(&LowPass_FIRFilt_R, inRight);
		pOutBuff[n + 1] = (int16_t)(FLOATTOINT16 * outRight);

		/*   I am only using the left channel because the guitar is mono
		 * 	 we are the doing the mono to stereo conversion
		 *
		 */
//		outLeft = inLeft;
//		pOutBuff[n] = (int16_t)(FLOATTOINT16 * outLeft);
//
//		outRight = inRight;
//		pOutBuff[n + 1] = (int16_t)(FLOATTOINT16 * outRight);


	}

	data_ready_f = 0;

}


void processSineWave(void)
{
    static float inLeft, inRight;
    static float outLeft, outRight;

    for (uint8_t n = 0; n < SAMPLES_PER_WAVE - 1; n += 2)
    {
        // Left Channel
        inLeft = INT16TOFLOAT * sine_wave_buffer[n];

        if (inLeft > 1.0f)
        {
            inLeft -= 2.0f;
        }

        // Data processing outLeft = f(inLeft)
        outLeft = LowPass_FirstOrder_Update(&LowPass_Filt_L, inLeft);
        // Uncomment below if using FIR filter
        // outLeft = FirFilter_Update(&LowPass_FIRFilt_L, inLeft);
        sine_wave_buffer[n] = (int16_t)(FLOATTOINT16 * outLeft);

        // Right Channel
        inRight = INT16TOFLOAT * sine_wave_buffer[n + 1];

        if (inRight > 1.0f)
        {
            inRight -= 2.0f;
        }

        // Data processing outRight = f(inRight)
        outRight = LowPass_FirstOrder_Update(&LowPass_Filt_R, inRight);
        // Uncomment below if using FIR filter
        // outRight = FirFilter_Update(&LowPass_FIRFilt_R, inRight);
        sine_wave_buffer[n + 1] = (int16_t)(FLOATTOINT16 * outRight);
    }


}




/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

	  HAL_GPIO_TogglePin(PE3_GPIO_Port, PE3_Pin);
	  HAL_Delay(500);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
