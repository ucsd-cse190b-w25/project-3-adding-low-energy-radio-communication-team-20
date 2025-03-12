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

	// Clear status register to remove any pending interrupts
	TIM2->SR &= 0;
	NVIC_ClearPendingIRQ(TIM2_IRQn);

	// setup auto reload(how many ticks), for 10S
	timer->ARR = 9999;

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

	// stop the timer
	timer->CR1 &= ~TIM_CR1_CEN;

	// MSI Clock is 8M Hz
	uint32_t default_frequency = 8000; // 8000000 ticks/s -> 8000 ticks/ms

	// ARR calculation = (default_frequency / freqTimer) / (timer->PSC + 1) - 1;
	timer->ARR = (default_frequency * period_ms) / (timer->PSC + 1) - 1;

	// reset counter to 0
	timer->CNT = 0;

	// Clear the timer's status register before enabling it
	TIM2->SR &= 0;
	NVIC_ClearPendingIRQ(TIM2_IRQn);

	// enable the timer
	timer->CR1 |= TIM_CR1_CEN;
}

void LPTIM1_lowpower_timer_init()
{
	// Enable LSI
	RCC->CSR |= RCC_CSR_LSION;

	// Reset LPTIM1
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;
	// Clear reset
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;

	while (!(RCC->CSR & RCC_CSR_LSIRDY))
	{
	}

	// Enable LPTIM1 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

	// Reset LPTIM1
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;

	// Set LPTIM1 clock source to LSI
	RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
	RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;  // Ensure LSI is selected

	// Disable LPTIM1 before configuring
	LPTIM1->CR &= ~LPTIM_CR_ENABLE;
	while (LPTIM1->CR & LPTIM_CR_ENABLE)
	{
	}
	// Reset CNT
	LPTIM1->CNT = 0;

	// Clear configuration
	LPTIM1->CFGR &= ~(LPTIM_CFGR_PRESC | LPTIM_CFGR_CKSEL | LPTIM_CFGR_COUNTMODE);

	// Software trigger (TRIGEN = 00)
	LPTIM1->CFGR &= ~LPTIM_CFGR_TRIGEN;

	// Set prescaler to 1/64
	LPTIM1->CFGR |= (LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2);

	// Enable interrupt
	LPTIM1->IER |= LPTIM_IER_ARRMIE;

	NVIC_SetPriority(LPTIM1_IRQn, 0);
	NVIC_EnableIRQ(LPTIM1_IRQn);

	// Clear Auto-Reload Match flag
	LPTIM1->ICR |= LPTIM_ICR_ARRMCF;

	// Enable and start LPTIM1
	LPTIM1->CR |= LPTIM_CR_ENABLE;
	// Wait until LPTIM is enabled
	while (!(LPTIM1->CR & LPTIM_CR_ENABLE))
	{
	}

	// Set the ARR after enable, else we have error
	LPTIM1->ARR = 4999;
	// Send start
	LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}

