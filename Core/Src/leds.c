/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *  Author: schulman
 */

/* Include memory map of our MCU */
#include <stm32l475xx.h>
#include "stm32l4xx_hal.h"
#include "leds.h"

void leds_init()
{

	// PA5 is LED1, PB14 is LED2

	// Set RCC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	/* Configure PA5 as an output by clearing all bits and setting the mode */
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5;

	/* Configure the GPIO to use low speed mode */
	GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

	/* Configure PB14 as an output by clearing all bits and setting the mode */
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= GPIO_MODER_MODE14_0;

	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14;

	/* Configure the GPIO to use low speed mode */
	GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

	/* Turn off the LEDs */
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	GPIOB->ODR &= ~GPIO_ODR_OD14;
}

void leds_set(uint8_t led)
{
	// TODO implement this

	// Turn on RCC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// put into output mode
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= GPIO_MODER_MODE14_0;

	// clear first
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	GPIOB->ODR &= ~GPIO_ODR_OD14;

	// if second bit is 1, turn on LED2, if first bit is 1, turn on LED1
	if (led & 0b10)
		GPIOB->ODR |= GPIO_ODR_OD14;
	if (led & 0b01)
		GPIOA->ODR |= GPIO_ODR_OD5;

	// Put into analog mode to save power
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE5_1);
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= (GPIO_MODER_MODE14_0 | GPIO_MODER_MODE14_1);

	// Turn off RCC
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
}

void gpio_set_analog()
{
	// Turn on RCC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Put into analog mode to save power
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE5_1);
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= (GPIO_MODER_MODE14_0 | GPIO_MODER_MODE14_1);

	// Turn off RCC
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
}
