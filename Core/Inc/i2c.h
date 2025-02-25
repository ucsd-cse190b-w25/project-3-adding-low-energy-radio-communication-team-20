/*
 * i2c.h
 *
 *  Created on: Feb 2, 2025
 *      Author: Yu-Heng Lin, Linfeng Zhang
 */

#ifndef I2C_H_
#define I2C_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>
#include <stdint.h>

void i2c_init();
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t *data, uint8_t len);

#endif /* I2C_H_ */
