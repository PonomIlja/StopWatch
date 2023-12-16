/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const uint8_t tm1637_digits[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
const uint8_t tm1637_all_zero[TM1637_SYMBOLS] = {0x3f, 0xBf, 0x3f, 0x3f};
const uint8_t tm1637_dot = 0x80;

tm1637_t D_left, D_right;

uint8_t fRun = 0, fPause = 0, DebounceStart[2] = {0, 0};
int Timer = 0, test = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void nopdelay (uint32_t times) {
  for (uint32_t i = 0; i < times; i++) {
	  asm("nop");
  }
}

void tm1637_start(tm1637_t *tm1637)
{
  HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_RESET);
  nopdelay(TM1637_DELAY);
}

void tm1637_stop(tm1637_t *tm1637)
{
  HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_SET);
  nopdelay(TM1637_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_SET);
  nopdelay(TM1637_DELAY);
}

uint8_t tm1637_write_byte(tm1637_t *tm1637, uint8_t data)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_RESET);
    if (data & 0x01)
      HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_SET);
    nopdelay(TM1637_DELAY);
    data = data >> 1;
  }

  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_RESET);
  nopdelay(TM1637_DELAY);
  HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_SET);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_SET);
  nopdelay(TM1637_DELAY);
  uint8_t ack = HAL_GPIO_ReadPin(tm1637->gpio_dio, tm1637->pin_dio);
  if (ack == 0)
    HAL_GPIO_WritePin(tm1637->gpio_dio, tm1637->pin_dio, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(tm1637->gpio_clk, tm1637->pin_clk, GPIO_PIN_RESET);
  return ack;
}

void tm1637_brightness(tm1637_t *tm1637, uint8_t bright)    // from 0 to 7
{
  tm1637->brightness = (bright & 0x7) | 0x08;
}

void tm1637_write(tm1637_t *tm1637, const uint8_t *raw, uint8_t length, uint8_t pos)
{
  if (pos > TM1637_SYMBOLS-1)
    return;
  if (length > TM1637_SYMBOLS)
    length = TM1637_SYMBOLS;

  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM1);
  tm1637_stop(tm1637);

  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM2 + (pos & 0x03));

  for (uint8_t k=0; k < length; k++)
    tm1637_write_byte(tm1637, raw[k]);
  tm1637_stop(tm1637);

  tm1637_start(tm1637);
  tm1637_write_byte(tm1637, TM1637_COMM3 + tm1637->brightness);
  tm1637_stop(tm1637);
}

void tm1637_init(tm1637_t *tm1637, GPIO_TypeDef *gpio_clk, uint16_t pin_clk, GPIO_TypeDef *gpio_dio, uint16_t pin_dio)
{
  tm1637_brightness(tm1637, TM1637_BR_MED);
  tm1637->gpio_clk = gpio_clk;
  tm1637->pin_clk = pin_clk;
  tm1637->gpio_dio = gpio_dio;
  tm1637->pin_dio = pin_dio;
  tm1637_write(tm1637, tm1637_all_zero, TM1637_SYMBOLS, TM1637_STARTPOS);
}

void DiplayUpdate(int data) {
    uint8_t buffer[TM1637_SYMBOLS] = {0};
	int hour, min, sec, dms;

	dms = data % 100;
	sec = (int)((data - dms) / 100);
	hour = (int)(sec / 3600);
	sec = sec % 3600;
	min = (int)(sec / 60);
	sec = sec % 60;

    buffer[0] = tm1637_digits[(int)(sec / 10)];
    buffer[1] = tm1637_digits[sec % 10] | tm1637_dot;
    buffer[2] = tm1637_digits[(int)(dms / 10)];
    buffer[3] = tm1637_digits[dms % 10];
	tm1637_write(&D_right, buffer, TM1637_SYMBOLS, TM1637_STARTPOS);

    buffer[0] = tm1637_digits[(int)(hour / 10)];
    buffer[1] = tm1637_digits[hour % 10] | tm1637_dot;
    buffer[2] = tm1637_digits[(int)(min / 10)];
    buffer[3] = tm1637_digits[min % 10];
	tm1637_write(&D_left, buffer, TM1637_SYMBOLS, TM1637_STARTPOS);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		HAL_NVIC_DisableIRQ(B1_EXTI_IRQn);
		__HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
		DebounceStart[0] = 1;
		HAL_TIM_Base_Start_IT(&htim4);
		if (fPause) {
 		    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
			Timer = 0;
			DiplayUpdate(Timer);
   		    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
   		    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}

	}
	if (GPIO_Pin == B2_Pin) {
		HAL_NVIC_DisableIRQ(B2_EXTI_IRQn);
		__HAL_GPIO_EXTI_CLEAR_IT(B2_Pin);
		DebounceStart[1] = 1;
		HAL_TIM_Base_Start_IT(&htim4);
		if (fRun) {
  		    HAL_TIM_Base_Stop_IT(&htim2);
  		    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  		    NVIC_ClearPendingIRQ(TIM2_IRQn);
			fRun = 0;
  		    fPause = 1;
   		    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
   		    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			DiplayUpdate(Timer);
		}
		else {
			fRun = 1;
			fPause = 0;
   		    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
   		    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
   		    HAL_TIM_Base_Start_IT(&htim2);
		}
	}
}

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	static uint8_t DebounceCounter[2] = {0, 0};
	if (htim->Instance == TIM2) {		// Stopwatch timer
		Timer += 1;						// 10x milliseconds
		DiplayUpdate(Timer);
	}
	if (htim->Instance == TIM4) {		// Debounce timer
		if (DebounceStart[0]) {
			if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == (GPIO_PIN_RESET)) {	    // button not pressed
				DebounceCounter[0] += 1;
				if (DebounceCounter[0] > 4) {
					if (!DebounceStart[1]) {
						HAL_TIM_Base_Stop_IT(&htim4);
					}
					DebounceStart[0] = 0;
					DebounceCounter[0] = 0;
					__HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
					NVIC_ClearPendingIRQ(B1_EXTI_IRQn);
					HAL_NVIC_EnableIRQ(B1_EXTI_IRQn);
				}
			}
			else {
				DebounceCounter[0] = 0;
			}
		}
		if (DebounceStart[1]) {
			if (HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == (GPIO_PIN_SET)) {    // button not pressed
				DebounceCounter[1] += 1;
				if (DebounceCounter[1] > 4) {
					if (!DebounceStart[0]) {
						HAL_TIM_Base_Stop_IT(&htim4);
					}
					DebounceStart[1] = 0;
					DebounceCounter[1] = 0;
					__HAL_GPIO_EXTI_CLEAR_IT(B2_Pin);
					NVIC_ClearPendingIRQ(B2_EXTI_IRQn);
					HAL_NVIC_EnableIRQ(B2_EXTI_IRQn);
				}
			}
			else {
				DebounceCounter[1] = 0;
			}
		}
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  tm1637_init(&D_left, D1_CLK_GPIO_Port, D1_CLK_Pin, D1_DIO_GPIO_Port, D1_DIO_Pin);
  tm1637_init(&D_right, D2_CLK_GPIO_Port, D2_CLK_Pin, D2_DIO_GPIO_Port, D2_DIO_Pin);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim2.Init.Period = 840053;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D2_DIO_Pin|D2_CLK_Pin|D1_DIO_Pin|D1_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D2_DIO_Pin D2_CLK_Pin D1_DIO_Pin D1_CLK_Pin */
  GPIO_InitStruct.Pin = D2_DIO_Pin|D2_CLK_Pin|D1_DIO_Pin|D1_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);

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
