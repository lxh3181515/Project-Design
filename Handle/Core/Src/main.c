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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
// button
uint8_t botton_LU, botton_LD, botton_LL, botton_LR;
uint8_t botton_RU, botton_RD, botton_RL, botton_RR;
uint8_t botton_top_LT, botton_top_LB, botton_top_RT, botton_top_RB;
uint8_t botton_A, botton_B, botton_center;
// rocker 0-1700-3400
// rocker[0] right-Y
// rocker[1] right-X
// rocker[2] left-Y
// rocker[3] left-X
uint16_t rocker_raw[16];
uint16_t rocker[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void getBottonValue(void);
void getRockerValue(void);
void sendMsg(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		getRockerValue();
		getBottonValue();
		sendMsg();
		HAL_Delay(50);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)rocker_raw, 16);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC6 PC7 PC8
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB11 PB14
                           PB15 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief Get Button Value Function
  * @param None
  * @retval None
  */
void getBottonValue()
{
	static uint8_t last_botton_LU = 0x01, last_botton_LD = 0x01, last_botton_LL = 0x01, last_botton_LR = 0x01;
	static uint8_t last_botton_RU = 0x01, last_botton_RD = 0x01, last_botton_RL = 0x01, last_botton_RR = 0x01;
	static uint8_t last_botton_top_LT = 0x01, last_botton_top_LB = 0x01, last_botton_top_RT = 0x01, last_botton_top_RB = 0x01;
	static uint8_t last_botton_A = 0x01, last_botton_B = 0x01, last_botton_center = 0x01;
	
	// botton_LU
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0x00) {
		if (last_botton_LU == 0x00) {
			botton_LU = 0x00;
		}
		last_botton_LU = 0x00;
	} else {
		botton_LU = 0x01;
		last_botton_LU = 0x01;
	}
	// botton_LD
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 0x00) {
		if (last_botton_LD == 0x00) {
			botton_LD = 0x00;
		}
		last_botton_LD = 0x00;
	} else {
		botton_LD = 0x01;
		last_botton_LD = 0x01;
	}
	// botton_LL
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0x00) {
		if (last_botton_LL == 0x00) {
			botton_LL = 0x00;
		}
		last_botton_LL = 0x00;
	} else {
		botton_LL = 0x01;
		last_botton_LL = 0x01;
	}
	// botton_LR
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0x00) {
		if (last_botton_LR == 0x00) {
			botton_LR = 0x00;
		}
		last_botton_LR = 0x00;
	} else {
		botton_LR = 0x01;
		last_botton_LR = 0x01;
	}
	// botton_RU
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0x00) {
		if (last_botton_RU == 0x00) {
			botton_RU = 0x00;
		}
		last_botton_RU = 0x00;
	} else {
		botton_RU = 0x01;
		last_botton_RU = 0x01;
	}
	// botton_RD
	if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0x00) {
		if (last_botton_RD == 0x00) {
			botton_RD = 0x00;
		}
		last_botton_RD = 0x00;
	} else {
		botton_RD = 0x01;
		last_botton_RD = 0x01;
	}
	// botton_RL
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0x00) {
		if (last_botton_RL == 0x00) {
			botton_RL = 0x00;
		}
		last_botton_RL = 0x00;
	} else {
		botton_RL = 0x01;
		last_botton_RL = 0x01;
	}
	// botton_RR
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 0x00) {
		if (last_botton_RR == 0x00) {
			botton_RR = 0x00;
		}
		last_botton_RR = 0x00;
	} else {
		botton_RR = 0x01;
		last_botton_RR = 0x01;
	}
	// botton_top_LT
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 0x00) {
		if (last_botton_top_LT == 0x00) {
			botton_top_LT = 0x00;
		}
		last_botton_top_LT = 0x00;
	} else {
		botton_top_LT = 0x01;
		last_botton_top_LT = 0x01;
	}
	// botton_top_LB
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0x00) {
		if (last_botton_top_LB == 0x00) {
			botton_top_LB = 0x00;
		}
		last_botton_top_LB = 0x00;
	} else {
		botton_top_LB = 0x01;
		last_botton_top_LB = 0x01;
	}
	// botton_top_RT
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0x00) {
		if (last_botton_top_RT == 0x00) {
			botton_top_RT = 0x00;
		}
		last_botton_top_RT = 0x00;
	} else {
		botton_top_RT = 0x01;
		last_botton_top_RT = 0x01;
	}
	// botton_top_RB
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == 0x00) {
		if (last_botton_top_RB == 0x00) {
			botton_top_RB = 0x00;
		}
		last_botton_top_RB = 0x00;
	} else {
		botton_top_RB = 0x01;
		last_botton_top_RB = 0x01;
	}
	// botton_A
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0x00) {
		if (last_botton_A == 0x00) {
			botton_A = 0x00;
		}
		last_botton_A = 0x00;
	} else {
		botton_A = 0x01;
		last_botton_A = 0x01;
	}
	// botton_B
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0x00) {
		if (last_botton_B == 0x00) {
			botton_B = 0x00;
		}
		last_botton_B = 0x00;
	} else {
		botton_B = 0x01;
		last_botton_B = 0x01;
	}
	// botton_center
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0x00) {
		if (last_botton_center == 0x00) {
			botton_center = 0x00;
		}
		last_botton_center = 0x00;
	} else {
		botton_center = 0x01;
		last_botton_center = 0x01;
	}
}

/**
  * @brief Get Rocker Value Function
  * @param None
  * @retval None
  */
void getRockerValue()
{
	uint8_t i;
	uint32_t rocker_tmp[4];
	
	for (i = 0; i < 4; i++)
	{
		rocker_tmp[i] = 0;
	}
	for (i = 0; i < 4; i++)
	{
		rocker_tmp[0] += rocker_raw[i * 4 + 0];
		rocker_tmp[1] += rocker_raw[i * 4 + 1];
		rocker_tmp[2] += rocker_raw[i * 4 + 2];
		rocker_tmp[3] += rocker_raw[i * 4 + 3];
	}
	for (i = 0; i < 4; i++)
	{
		rocker[i] = rocker_tmp[i] / 4;
	}
}

/**
  * @brief Send Message
  * @param None
  * @retval None
  */
void sendMsg()
{
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox; 

	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;           
	TxHeader.TransmitGlobalTime = DISABLE;
	
	TxHeader.StdId = 0x201;
	TxHeader.DLC = 2;
	TxData[0] = botton_LU | (botton_LD << 1) | (botton_LL << 2) | (botton_LR << 3) | (botton_RU << 4) | (botton_RD << 5) | (botton_RL << 6) | (botton_RR << 7);
	TxData[1] = botton_top_LT | (botton_top_LB << 1) | (botton_top_RT << 2) | (botton_top_RB << 3) | (botton_A << 4) | (botton_B << 5) | (botton_center << 6);
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		 /* Transmission request Error */
		 Error_Handler();
	}
	
	TxHeader.StdId = 0x202;
	TxHeader.DLC = 8;
	TxData[0] = rocker[0] >> 8;
	TxData[1] = rocker[0];
	TxData[2] = rocker[1] >> 8;
	TxData[3] = rocker[1];
	TxData[4] = rocker[2] >> 8;
	TxData[5] = rocker[2];
	TxData[6] = rocker[3] >> 8;
	TxData[7] = rocker[3];
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		 /* Transmission request Error */
		 Error_Handler();
	}
}
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
