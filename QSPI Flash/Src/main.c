/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stm32l4s5i_iot01_qspi.h"

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_OCTOSPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin);
// void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim);

uint8_t arrayOne[33000]; // 2kHZ 22
float arrayOneRad = 0;

uint8_t arrayTwo[33000]; // 1kHz 44
float arrayTwoRad = 0;

uint8_t arrayThree[33000]; // 1.5 kHz 30
float arrayThreeRad = 0;

uint8_t arrayFour[33000]; // 3kHZ 15
float arrayFourRad = 0;


uint8_t arrayFive[33000]; // 5kHz 9
float arrayFiveRad = 0;



int callbackCounter = 0;

int memoryOffset = 65000;


uint32_t flash_address = 0x08020800;
uint8_t writeData[8];
uint8_t readArrayOne[33000];
uint8_t readArrayTwo[33000];
uint8_t readArrayThree[33000];
uint8_t readArrayFour[33000];
uint8_t readArrayFive[33000];

uint8_t* readBuffer;


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

  //HAL_GPIO_EXTI_Callback (BLUE_Pin);			// WORKS WITHOUT CALLING

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */

  BSP_QSPI_Init();


  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);


  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)arrayOne, 8, DAC_ALIGN_8B_R);





  //HAL_TIM_Base_Start(&htim2);
  //HAL_TIM_Base_Start_DMA(&htim2, (uint32_t*)arrayOne, 32);
  //HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)arrayOne, 22, DAC_ALIGN_12B_R);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {

	if(GPIO_Pin == BLUE_Pin) {

		for (int i = 0; i < 33000 ; i++){

			if(arrayOneRad > 6.284) {
				arrayOneRad = 0;
			}
			uint8_t conv = (uint8_t)((arm_sin_f32(arrayOneRad) * 127) + 127);
			arrayOne[i] = conv;
			arrayOneRad += 0.286;
			//indexCounterOne++;
		}
		if(BSP_QSPI_Erase_Block(0) == QSPI_OK) {		// 0 0xD8 65000 130000 195000 260000 work
			if(BSP_QSPI_Write(&arrayOne, 0, 33000) != QSPI_OK) {
				Error_Handler();
			}
			int i = 0;
		} else {
			Error_Handler();
		}

		for (int i = 0; i < 33000 ; i++){
			if(arrayTwoRad > 6.284) {
				arrayTwoRad = 0;
			}
			uint8_t conv = (uint8_t)((arm_sin_f32(arrayTwoRad) * 127) + 127);
			arrayTwo[i] = conv;
			arrayTwoRad += 0.143;
			//indexCounterOne++;
		}
		if(BSP_QSPI_Erase_Block(65537) == QSPI_OK) {		// 0 0xD8 65000 130000 195000 260000 work
			if(BSP_QSPI_Write(&arrayTwo, 65537, 33000) != QSPI_OK) {
				Error_Handler();
			}
		} else {
			Error_Handler();
		}

		for (int i = 0; i < 33000 ; i++){
			if(arrayThreeRad > 6.284) {
				arrayThreeRad = 0;
			}
			uint8_t conv = (uint8_t)((arm_sin_f32(arrayThreeRad) * 127) + 127);
			arrayThree[i] = conv;
			arrayThreeRad += 0.209;
			//indexCounterOne++;
		}
		if(BSP_QSPI_Erase_Block(131074) == QSPI_OK) {		// 0 0xD8 65000 130000 195000 260000 work
			if(BSP_QSPI_Write(&arrayThree, 131074, 33000) != QSPI_OK) {
				Error_Handler();
			}
		} else {
			Error_Handler();
		}

		for (int i = 0; i < 33000 ; i++){
			if(arrayFourRad > 6.284) {
				arrayFourRad = 0;
			}
			uint8_t conv = (uint8_t)((arm_sin_f32(arrayFourRad) * 127) + 127);
			arrayFour[i] = conv;
			arrayFourRad += 0.418;
			//indexCounterOne++;
		}
		if(BSP_QSPI_Erase_Block(196610) == QSPI_OK) {		// 0 0xD8 65000 130000 195000 260000 work
			if(BSP_QSPI_Write(&arrayFour, 196610, 33000) != QSPI_OK) {
				Error_Handler();
			}
		} else {
			Error_Handler();
		}

		for (int i = 0; i < 33000 ; i++){
			if(arrayFiveRad > 6.284) {
				arrayFiveRad = 0;
			}
			uint8_t conv = (uint8_t)((arm_sin_f32(arrayFiveRad) * 127) + 127);
			arrayFive[i] = conv;
			arrayFiveRad += 0.698;
			//indexCounterOne++;
		}
		if(BSP_QSPI_Erase_Block(262146) == QSPI_OK) {		// 0 0xD8 65000 130000 195000 260000 work
			if(BSP_QSPI_Write(&arrayFive, 262146, 33000) != QSPI_OK) {
				Error_Handler();
			}
		} else {
			Error_Handler();
		}

		// LIGHTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTUPPPPPPPPPPPPPPPPPPP
		if(HAL_GPIO_ReadPin(GPIOB, LED1_Pin) == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
		}



		if(BSP_QSPI_Read(&readArrayOne, 0, 33000) == QSPI_OK) {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readArrayOne, 33000, DAC_ALIGN_12B_R);
			int i = 0;
			callbackCounter = 0;
		} else {
			Error_Handler();
		}
	}


}

void HAL_DAC_ConvCpltCallbackCh1 (DAC_HandleTypeDef * hdac){
	callbackCounter++;
	if(callbackCounter > 4) {
		callbackCounter = 0;
	}



	if(callbackCounter == 0) {
		readBuffer = &readArrayOne;
		memoryOffset = 0;
		if(BSP_QSPI_Read(&readArrayOne, memoryOffset, 33000) == QSPI_OK) {
			//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readArrayOne, 33000, DAC_ALIGN_12B_R);
			int i = 0;
		} else {
			Error_Handler();
		}
	}
	if(callbackCounter == 1) {
		readBuffer = &readArrayTwo;
		memoryOffset = 65537;

		if(BSP_QSPI_Read(&readArrayTwo, memoryOffset, 33000) == QSPI_OK) {
			//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readArrayTwo, 33000, DAC_ALIGN_12B_R);
			int i = 0;
		} else {
			Error_Handler();
		}
	}
	if(callbackCounter == 2) {
		readBuffer = &readArrayThree;
		memoryOffset = 131074;

		if(BSP_QSPI_Read(&readArrayThree, memoryOffset, 33000) == QSPI_OK) {
			//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readArrayThree, 33000, DAC_ALIGN_12B_R);
			int i = 0;
		} else {
			Error_Handler();
		}
	}
	if(callbackCounter == 3) {
		readBuffer = &readArrayFour;
		memoryOffset = 196610;

		if(BSP_QSPI_Read(&readArrayFour, memoryOffset, 33000) == QSPI_OK) {
			//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readArrayFour, 33000, DAC_ALIGN_12B_R);
			int i = 0;
		} else {
			Error_Handler();
		}
	}
	if(callbackCounter == 4) {
		readBuffer = &readArrayFive;
		memoryOffset = 262146;

		if(BSP_QSPI_Read(&readArrayFive, memoryOffset, 33000) == QSPI_OK) {
			//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readArrayFive, 33000, DAC_ALIGN_12B_R);
			int i = 0;
		} else {
			Error_Handler();
		}
	}


//	if(BSP_QSPI_Read(readBuffer, memoryOffset, 33000) == QSPI_OK) {
//		//HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, readBuffer, 33000, DAC_ALIGN_12B_R);
//		int i = 0;
//	} else {
//		Error_Handler();
//	}




}















//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
//	if(callbackCounter > 21) {
//		callbackCounter = 0;
//	}
//	uint8_t tone = (uint8_t) arrayOne[callbackCounter];
//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, tone);
//	callbackCounter++;
//
//
//}

/* USER CODE END 4 */

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
  }
  HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
  __BKPT();
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
