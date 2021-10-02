/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "MAX31865.h"
#include "stdlib.h"
#include <string.h>
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

/* USER CODE BEGIN PV */
Max31865_t pt100;
bool pt100isOK;
float pt100Temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM1_IC_INIT(void) {
	/* TIM1 CONFIGURATION */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;	//ENABLE TIMER CLOCK
	//RCC->CFGR3 |= RCC_CFGR3_TIM1SW;		//PLLCLK*2 AS CLOCK SOURCE
	/* CH1 */
	TIM1->CCMR1 |= 1 << TIM_CCMR1_CC1S_Pos;
	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);	//RISING EDGE POLARITY
	TIM1->CCMR1 |= 3 << TIM_CCMR1_IC1PSC_Pos;	//INPUT PRESCALER SET TO 8

	TIM1->CCER |= TIM_CCER_CC1E;				//ENABLE CAPTURE
	TIM1->CR1 |= TIM_CR1_CEN; // START TIMER
}
void clean_data(uint32_t *a, float *dest) {
	int count = 0, maxValue = 0, maxCount = 0, i, j, num = 0;
	float avg = 0;
	for (i = 0; i < 100; i++) {
		count = 0;

		for (j = 0; j < 100; j++) {
			if (abs(a[j] - a[i]) < 5)
				count++;
		}

		if (count > maxCount) {
			maxCount = count;
			maxValue = a[i];
		}
	}
	for (i = 0; i < 100; i++) {
		if (abs(maxValue - a[i]) < 5) {
			num++;
			avg = avg + a[i];
		}
	}
	avg = avg / num;
	*dest = avg;

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
	MX_USART2_UART_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	Max31865_init(&pt100, &hspi3, MAX_CS_GPIO_Port, MAX_CS_Pin, 4, 50);

	uint32_t rising_1 = 0, rising_2 = 0;
	int data_count = 0;
	uint32_t tp_array[100];
	float frequency = 0, time_period_cnt = 0;
	float capacitance = 0;
	TIM1_IC_INIT();
	SystemCoreClockUpdate();
	float t;
	USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
	//printf("program entry");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		pt100isOK = Max31865_readTempC(&pt100, &t);
		pt100Temp = Max31865_Filter(t, pt100Temp, 0.1); //  << For Smoothing data
		data_count = 0;
		while (data_count < 100) {
			TIM1->SR &= ~(TIM_SR_CC1IF);
			while (!(TIM1->SR & TIM_SR_CC1IF))
				;
			rising_1 = TIM1->CCR1;

			while (!(TIM1->SR & TIM_SR_CC1IF))
				;
			rising_2 = TIM1->CCR1;

			if (rising_2 > rising_1) {
				tp_array[data_count] = rising_2 - rising_1;
				data_count++;
			}
		}
		clean_data(tp_array, &time_period_cnt);
		//frequency = (8 * 2 * SystemCoreClock) / time_period_cnt;
		frequency = (8 * 127992192) / time_period_cnt;
		capacitance = (144 * 10000000000) / (2 * 26970 * frequency);

		printf("T%f\n", pt100Temp);
		HAL_Delay(1500);
		/*printf("F%f\n", frequency);
		HAL_Delay(2000);*/
		printf("C%f\n",capacitance);
		HAL_Delay(1500);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
