/*
 * i2c.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Yu-Heng Lin, Linfeng Zhang
 */
#include "i2c.h"

void i2c_init()
{

	// Configure RCC for I2C2 and GPIOB
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Enable 8M HZ clock source
	RCC->CCIPR &= ~RCC_CCIPR_I2C2SEL;
	RCC->CCIPR |= RCC_CCIPR_I2C2SEL_0;

	// Disable I2C2
	I2C2->CR1 &= ~I2C_CR1_PE;

	// Configure PB10 as an alternative function mode by clearing all bits and setting the mode
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= GPIO_MODER_MODE10_1;

	// Set PB10 as open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT10;

	// Set PB10 as pull-up
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;

	// Configure AFRH for PB10
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2;

	// Configure PB11 as an alternative function mode by clearing all bits and setting the mode
	GPIOB->MODER &= ~GPIO_MODER_MODE11;
	GPIOB->MODER |= GPIO_MODER_MODE11_1;

	// Set PB11 as open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT11;

	// Set PB11 as pull-up
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD11;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;

	// Configure AFRH for PB11
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL11_2;

	// I2C 10K HZ baud rate, with clock source being system clock 8M HZ, and example in page 1304
	I2C2->TIMINGR = 0;
	// Setting SCLL, SCLH, SDADEL, SCLDEL, PRESC bits
	I2C2->TIMINGR = 0xC7 | (0xC3 << 8) | (0x2 << 16) | (0x4 << 20) | (0x1 << 28);
	// Enable I2C2
	I2C2->CR1 |= I2C_CR1_PE;

}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t *data, uint8_t len)
{

	// turn on I2C RCC
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Enable 7-bit addressing mode
	I2C2->CR2 &= ~I2C_CR2_ADD10;

	// Clear previous address, and set the new slave address
	I2C2->CR2 &= ~I2C_CR2_SADD;
	I2C2->CR2 |= (address << 1);
	// 0 for writing, 1 for reading
	if (dir == 1)
		I2C2->CR2 |= I2C_CR2_RD_WRN;
	else
		I2C2->CR2 &= ~I2C_CR2_RD_WRN;
	// Set the number of bytes we need to send/receive, which is passed in as len
	I2C2->CR2 &= ~I2C_CR2_NBYTES;
	I2C2->CR2 |= (len << 16);

	// If the transaction failed, it auto ends
	I2C2->CR2 |= I2C_CR2_AUTOEND;

	// Start the transfer
	I2C2->CR2 |= I2C_CR2_START;

	// If writing
	if (dir == 0)
	{
		for (uint8_t i = 0; i < len; ++i)
		{
			// wait till the bit is ready
			while (!(I2C2->ISR & I2C_ISR_TXIS))
			{
			}
			// Send the bit
			I2C2->TXDR = data[i];
		}
	}
	// If reading
	else
	{
		for (uint8_t i = 0; i < len; ++i)
		{
			// Wait till the bit is ready
			while (!(I2C2->ISR & I2C_ISR_RXNE))
			{
			}
			// Received the bit
			data[i] = I2C2->RXDR;
		}
	}
	// turn off I2C RCC
	RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
	return 0;
}
