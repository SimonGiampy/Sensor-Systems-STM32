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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stdlib.h>
#include <string.h>

int column;

int matrixColumn = 0;
char symbol = '0';

struct character {
	uint8_t row;
	uint8_t col;
};

struct character ZERO[] = { { 62, 1 }, { 65, 2 }, { 65, 4 }, { 65, 8 },
		{ 62, 16 } };

struct character ONE[] =
		{ { 0, 1 }, { 1, 2 }, { 127, 4 }, { 33, 8 }, { 0, 16 } };

struct character TWO[] = { { 49, 1 }, { 73, 2 }, { 69, 4 }, { 67, 8 },
		{ 49, 16 } };

struct character THREE[] = { { 54, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 }, { 65,
		16 } };

struct character FOUR[] = { { 4, 1 }, { 127, 2 }, { 36, 4 }, { 20, 8 },
		{ 12, 16 } };

struct character FIVE[] = { { 70, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 }, { 121,
		16 } };

struct character SIX[] =
		{ { 6, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 }, { 62, 16 } };

struct character SEVEN[] = { { 96, 1 }, { 80, 2 }, { 79, 4 }, { 64, 8 }, { 64,
		16 } };

struct character EIGHT[] = { { 54, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 }, { 54,
		16 } };

struct character NINE[] = { { 62, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 },
		{ 48, 16 } };

struct character A[] =
		{ { 31, 1 }, { 36, 2 }, { 68, 4 }, { 36, 8 }, { 31, 16 } };

struct character B[] =
		{ { 62, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 }, { 127, 16 } };

struct character C[] =
		{ { 65, 1 }, { 65, 2 }, { 65, 4 }, { 65, 8 }, { 62, 16 } };
struct character D[] =
		{ { 62, 1 }, { 65, 2 }, { 65, 4 }, { 65, 8 }, { 127, 16 } };
struct character E[] =
		{ { 73, 1 }, { 73, 2 }, { 73, 4 }, { 73, 8 }, { 127, 16 } };
struct character F[] =
		{ { 72, 1 }, { 72, 2 }, { 72, 4 }, { 72, 8 }, { 127, 16 } };

char receivedByte;

int elapsedBaud = 0;

int column_index = 0;
int column_flag = 1;

//keyboard mapping buttons to a character
char map[16] = "FB73EA62D951C840";

int key_press[16];
int ack[16];

void IRByteTransmit(char byte) {
	elapsedBaud = 1;
	HAL_TIM_Base_Start_IT(&htim3);
	while (elapsedBaud == 0) {

	}

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	elapsedBaud = 0;
	while (elapsedBaud == 0) {

	}

	for (int i = 0; i < 8; i++) {
		if ((byte & (0x01 << i)) == 0) {
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		} else {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		}
		elapsedBaud = 0;
		while (elapsedBaud == 0) {

		}
	}

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	while (elapsedBaud == 0) {

	}

	HAL_TIM_Base_Stop_IT(&htim3);
	elapsedBaud = 0;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {

		HAL_UART_Receive_IT(&huart1, (uint8_t*) &receivedByte, 1);

		char RX = receivedByte;

		symbol = RX;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim3) {
		elapsedBaud = 1;
	}
	if (htim == &htim10) {
		struct character *chosen;
		switch (symbol) {
		case '0':
			chosen = ZERO;
			break;
		case '1':
			chosen = ONE;
			break;
		case '2':
			chosen = TWO;
			break;
		case '3':
			chosen = THREE;
			break;
		case '4':
			chosen = FOUR;
			break;
		case '5':
			chosen = FIVE;
			break;
		case '6':
			chosen = SIX;
			break;
		case '7':
			chosen = SEVEN;
			break;
		case '8':
			chosen = EIGHT;
			break;
		case '9':
			chosen = NINE;
			break;
		case 'A':
			chosen = A;
			break;
		case 'B':
			chosen = B;
			break;
		case 'C':
			chosen = C;
			break;
		case 'D':
			chosen = D;
			break;
		case 'E':
			chosen = E;
			break;
		case 'F':
			chosen = F;
			break;

		}
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*) (chosen + matrixColumn), 2);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

		matrixColumn = (matrixColumn + 1) % 5;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);

		HAL_TIM_Base_Start_IT(&htim10);

	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim10);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &receivedByte, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, column_index == 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, column_index == 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, column_index == 2);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, column_index == 3);

		HAL_Delay(5); // waits for the column activation to be completed

		column_index = (column_index + 1) % 4; // iterates from 0 to 4 in loop

		// reads every button press
		key_press[4 * column_index] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		key_press[4 * column_index + 1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		key_press[4 * column_index + 2] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		key_press[4 * column_index + 3] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);

		for (int i = 0; i < 16; i++) {
			if (key_press[i]) { // if the corresponding button was pressed
				if (!ack[i]) {
					// prints string containing the pressed character on terminal
					IRByteTransmit(map[i]);
					ack[i] = 1;
				}
			} else {
				ack[i] = 0;
			}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2211 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 1105;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 35000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 84 - 1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 4000;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 2400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC2 PC3 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC8 PC9 PC10 PC11 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
