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
