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
//#include "ble_commands.h"
#include "ble.h"
#include "i2c.h"
#include "timer.h"
#include "leds.h"
#include "lsm6dsl.h"

/* Include memory map of our MCU */
#include <stm32l475xx.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_ACCELERATION_RANGE 18000
#define MIN_ACCELERATION_RANGE 15000
#define TIMER_PERIOD 50
#define LOST_COUNT_START_BLINK 1200

// declare it as volatile to avoid compiler optimization
static volatile int count = 0, lost_count = 0;
static volatile int32_t magnitude = 0;

static volatile char LED_FLASH_PATTERN[33] = "10011001001001001010010100000000"; // REAL ID is 9381
// 10, 01, 10, 01, 00, 10, 01, 00, 10, 10, 01, 01, (00, 00, 00, 00), quoted is min

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len)
{
	int i = 0;
	for (i = 0; i < len; i++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

// helper function
static uint8_t get_Led(char c1, char c2)
{
	if (c1 == '0' && c2 == '1')
		return 1;
	if (c1 == '1' && c2 == '0')
		return 2;
	if (c1 == '1' && c2 == '1')
		return 3;
	return 0;
}

static void light_LED_pattern()
{
	// map the count to indices, for example 0 -> 0, 1 -> 2, 2->4, 3->6
	// since the interrupt will increment count first, then we light, we must subtract count by 1 to get the correct LED
	int idx = (count - 1) * 2;
	// get the current count and map to the pattern to flash
	char c1 = LED_FLASH_PATTERN[idx];
	char c2 = LED_FLASH_PATTERN[idx + 1];
	uint8_t led = get_Led(c1, c2);
	// light LED
	leds_set(led);
}

void TIM2_IRQHandler()
{
	// only doing count update in the interrupt handler

	// if it's within the range, then it's not moving, increment the lost_count by 1
	if (magnitude <= MAX_ACCELERATION_RANGE && magnitude >= MIN_ACCELERATION_RANGE)
		lost_count++;
	else
		lost_count = 0;

	// only start counting when it's lost for over 60 seconds
	if (lost_count >= LOST_COUNT_START_BLINK)
	{
		// increment the counter if it's not out of bound, else reset it to 0
		if (count < 16)
			count++;
		else
			count = 0;
	}

	// clearing the interrupt bit
	if (TIM2->SR & TIM_SR_UIF)
		TIM2->SR &= ~TIM_SR_UIF;
}

// Fast Square Root Algorithm, it will compute the square root of x
static uint32_t fast_sqrt(uint32_t x)
{
	uint32_t res = 0;
	uint32_t bit_mask = 1 << 30;

	while (bit_mask > x)
		bit_mask >>= 2;

	while (bit_mask != 0)
	{
		if (x >= res + bit_mask)
		{
			x -= res + bit_mask;
			res += bit_mask << 1;
		}
		res >>= 1;
		bit_mask >>= 2;
	}
	return res;
}

// helper to update the minute as byte sequence into the pattern string
static void update_min_string(int flag)
{
	const static int offset = 24;
	// flag = 1 for lost mode minute update, and 0 for moving(not lost) reset

	// set the string last 8 char to be minute
	if (flag == 1)
	{
		// calculate the minute

		int minute = lost_count / LOST_COUNT_START_BLINK;
		// convert int to binary string
		for (int i = 7; i >= 0; --i)
		{
			if (minute & (1 << i))
				LED_FLASH_PATTERN[offset + 7 - i] = '1';
			else
				LED_FLASH_PATTERN[offset + 7 - i] = '0';
		}
	}
	// reset the string last 8 char to be 0
	else
	{
		for (int i = 0; i <= 7; ++i)
		{
			LED_FLASH_PATTERN[offset + i] = '0';
		}
	}
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI3_Init();

	//RESET BLE MODULE
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

	ble_init();
	HAL_Delay(10);
	uint8_t nonDiscoverable = 0;

	// init LED, timer, i2c, and lsm6dsl
	leds_init();
	timer_init(TIM2);
	lsm6dsl_init();

	// set the period in ms
	timer_set_ms(TIM2, TIMER_PERIOD);
	int16_t x = 0, y = 0, z = 0;
	long long xs = 0, ys = 0, zs = 0, sqrt_sum = 0;
	printf("If\n");

	while (1)
	{
		// read the raw x y z acceleration value, and compute its magnitude
		lsm6dsl_read_xyz(&x, &y, &z);
		xs = x * x;
		ys = y * y;
		zs = z * z;
		sqrt_sum = xs + ys + zs;
		magnitude = fast_sqrt(sqrt_sum);

		// the lost detection check is in the tim2 interrupt handler
		// if it's over 60 seconds, it's lost. Each timer tick is 50ms, to be 60s, it has to tick 1200 times
		if (lost_count >= LOST_COUNT_START_BLINK)
		{
			update_min_string(1);
			light_LED_pattern();
		}
		// it's not lost, turn off LED
		else
		{
			update_min_string(0);
			leds_set(0);
		}

//		if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin))
//		{
//			printf("If\n");
//			catchBLE();
//		}
//		else
//		{
//			printf("Else\n");
//			HAL_Delay(1000);
//			// Send a string to the NORDIC UART service, remember to not include the newline
//			unsigned char test_str[] = "CSE190 too hard";
//			updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str) - 1, test_str);
//		}
		// Wait for interrupt, only uncomment if low power is needed
		//__WFI();
	}
}

/**
 * @brief System Clock Configuration
 * @attention This changes the System clock frequency, make sure you reflect that change in your timer
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

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
	// This lines changes system clock frequency
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : BLE_INT_Pin */
	GPIO_InitStruct.Pin = BLE_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
	GPIO_InitStruct.Pin = GPIO_LED1_Pin | BLE_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BLE_CS_Pin */
	GPIO_InitStruct.Pin = BLE_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
