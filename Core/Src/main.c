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
#include <string.h>
#include <stdint.h>
#include <stdio.h>

// low power mode will use LPTIM1 instead of TIM2
#define LOWPOWER

// utils for formatting message that's sending to phone
#define PTAG20_FORMAT "PTAG20 LOST %d\'s"
#define TEST_ENV

#define MAX_ACCELERATION_RANGE 18000
#define MIN_ACCELERATION_RANGE 15000
// change timer_period as needed, unit is ms
#define TIMER_PERIOD 10000
#define BUFFER_SZ 20

// Calculation: TEN_SECONDS_DIV = 10000 / TIMER_PERIOD
#define TEN_SECONDS_DIV 1

// adjust to 20s for lost mode in TEST ENV
// Calculation: for 60s, LOST_COUNT_START = 60000 / TIMER_PERIOD
// 				for 20s, LOST_COUNT_START = 20000 / TIMER_PERIOD
#ifdef TEST_ENV
#define LOST_COUNT_START 2
#else
#define LOST_COUNT_START 6
#endif

// declare it as volatile to avoid compiler optimization
static volatile int count = 0, lost_count = 0, ten_seconds_count = 0, ten_seconds_flag = 0;
static volatile int sec_lost = 0, magnitude = 0, discoverable_flag = 1, is_lost = 0;
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

void TIM2_IRQHandler()
{

	// only doing count update in the interrupt handler

	// only start counting when it's lost for over 60 seconds
	if (lost_count >= LOST_COUNT_START)
	{
		// every 10S, turn the flag to 1
		ten_seconds_count++;
		ten_seconds_flag = 1;
		count++;
		is_lost = 1;
	}

	// increment count no matter what, if it's stationary we can detect it in the main loop and reset it to 0
	lost_count++;

	// clearing the interrupt bit
	if (TIM2->SR & TIM_SR_UIF)
		TIM2->SR &= ~TIM_SR_UIF;
}

void LPTIM1_IRQHandler()
{
	// only doing count update in the interrupt handler

	// increment count no matter what, if it's stationary we can detect it in the main loop and reset it to 0
	lost_count++;

	// only start counting when it's lost for over 60 seconds
	if (lost_count >= LOST_COUNT_START)
	{
		// every 10S, turn the flag to 1
		ten_seconds_count++;
		ten_seconds_flag = 1;
		count++;
		is_lost = 1;
	}

	// clear interrupt bit
	if (LPTIM1->ISR & LPTIM_ISR_ARRM)
		LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
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

static void pre_sleep()
{
	// put into sleep mode
	printf("Entering Sleep mode...\r\n");
//		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

// Disable SPI3 clock
	RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;
	__disable_irq();
	HAL_SuspendTick();
}

static void post_sleep()
{
	HAL_ResumeTick();
	__enable_irq();
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
//		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	printf("Woke up from Sleep mode!\r\n");
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	// turn off all unused RCC
	RCC->APB1ENR1 = 0;
	RCC->APB1ENR2 = 0;
	RCC->AHB2ENR = 0;
	RCC->AHB3ENR = 0;
	RCC->APB2ENR = 0;

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

	// init LED, timer/low power timer, i2c, and lsm6dsl
	leds_init();
	lsm6dsl_init();

	// set the timer
#ifdef LOWPOWER
	LPTIM1_lowpower_timer_init();
#else
	timer_init(TIM2);
	timer_set_ms(TIM2, TIMER_PERIOD);
	timer_reset(TIM2);
#endif

	// for accelerometer variables
	int16_t x = 0, y = 0, z = 0;
	long long xs = 0, ys = 0, zs = 0, sqrt_sum = 0;

	// for storing message
	unsigned char buffer[BUFFER_SZ];

	// setting it to be low power run
	PWR->CR1 |= PWR_CR1_LPR;

	// turn off all unused RCC. Disable the SYSCFG, COMP, and VREFBUF clocks
	RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
	RCC->CR &= ~RCC_CR_HSEON;
	RCC->CR &= ~RCC_CR_PLLON;

	while (1)
	{
		// read the raw x y z acceleration value, and compute its magnitude
		lsm6dsl_read_xyz(&x, &y, &z);
		xs = x * x;
		ys = y * y;
		zs = z * z;
		sqrt_sum = xs + ys + zs;
		magnitude = fast_sqrt(sqrt_sum);
		// if it's not within the range, then it's moving, reset the lost_count to 0, and the is_lost flag to 0
		if (!(magnitude <= MAX_ACCELERATION_RANGE && magnitude >= MIN_ACCELERATION_RANGE))
		{
			lost_count = 0;
			is_lost = 0;
		}

		// the lost detection check is in the tim2 interrupt handler
		// is_lost flag is set in the interrupt handler, and is reset in the magnitude check above
		if (is_lost)
		{
			// set it to be discoverable
			if (!discoverable_flag)
			{
				setDiscoverability(1);
				discoverable_flag = 1;
			}

			// capture connection handler
			if (HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin))
				catchBLE();

			// every 10s, send a message
			if (ten_seconds_flag)
			{
				sec_lost = (ten_seconds_count - 1) * 10;
				snprintf((char*) buffer, sizeof(buffer), PTAG20_FORMAT, sec_lost);
				updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen((char*) buffer), buffer);
				ten_seconds_flag = 0;
			}
			// if lost light up LED1 to indicate so
			leds_set(1);
		}
		// it's not lost
		else
		{
			// keep resetting the counter if it's not lost
			count = 0;
			ten_seconds_count = 0;
			ten_seconds_flag = 0;

			// disconnect then set it to be non discoverable
			if (discoverable_flag)
			{
				disconnectBLE();
				setDiscoverability(0);
				discoverable_flag = 0;

				// Put BLE into standby mode
				standbyBle();
			}
			// if not lost turn off LED
			leds_set(0);
		}
		// Wait for interrupt, only uncomment if low power is needed
		pre_sleep();
		__WFI();
		post_sleep();
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
