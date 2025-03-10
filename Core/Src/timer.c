/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

void timer_init(TIM_TypeDef *timer)
{

	// setup RCC for TIM2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// stop the timer
	timer->CR1 &= ~TIM_CR1_CEN;

	// setup auto reload(how many ticks)
	timer->ARR = 49;

	// initialize counter to 0
	timer->CNT = 0;

	// clear the timer state
	timer->SR = 0;

	// setup prescaler(for frequency)
	timer->PSC = 7999;

	// set up interrupt with priority level to 0
	NVIC_SetPriority(TIM2_IRQn, 0);

	// enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);

	// enable the interrupt internally
	timer->DIER |= TIM_DIER_UIE;

	// enable the timer
	timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef *timer)
{
	// TODO implement this

	// reset counter to 0
	timer->CNT = 0;

	// clearing the interrupt bit if it's 1
	if (TIM2->SR & TIM_SR_UIF)
		TIM2->SR &= ~TIM_SR_UIF;
}

void timer_set_ms(TIM_TypeDef *timer, uint16_t period_ms)
{
	// TODO implement this

	// MSI Clock is 8M Hz
	uint32_t default_frequency = 8000; // 8000000 ticks/s -> 8000 ticks/ms

	// ARR calculation = (default_frequency / freqTimer) / (timer->PSC + 1) - 1;
	timer->ARR = (default_frequency * period_ms) / (timer->PSC + 1) - 1;

	// reset counter to 0
	timer->CNT = 0;
}

//void low_power_timer_init()
//{
//	// Enable the LPTIM1 clock
//	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
//
//	// Select LSE (32.768 kHz) as clock source
//	RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
//	RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;
//
//	// Disable LPTIM1
//	LPTIM1->CR &= ~LPTIM_CR_ENABLE;
//
//	// Set ARR (Auto Reload Register) for desired timeout period
//	LPTIM1->ARR = 32768 - 1; // 1-second period if LSE (32.768 kHz) is used
//
//	// Initialize counter to 0
//	LPTIM1->CNT = 0;
//
//	// Enable interrupt
//	NVIC_SetPriority(LPTIM1_IRQn, 0);
//	NVIC_EnableIRQ(LPTIM1_IRQn);
//	LPTIM1->IER |= LPTIM_IER_ARRMIE;
//
//	// enable the timer
//	LPTIM1->CR |= LPTIM_CR_ENABLE;
//
//	// Start the timer in continuous mode
//	LPTIM1->CR |= LPTIM_CR_CNTSTRT;
//}

