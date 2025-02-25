/*
 * lsm6dsl.h
 *
 *  Created on: Feb 5, 2025
 *      Author: Yu-Heng Lin, Linfeng Zhang
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

// macros for addresses
#define LSM6DSL_ADDRESS 0x6A
#define CTRL1_XL_ADDRESS 0x10
#define INT1_CTRL_ADDRESS 0x0D
#define STATUS_REG_ADDRESS 0x1E
#define OUTX_L_XL_ADDRESS 0x28
#define OUTX_H_XL_ADDRESS 0x29
#define OUTY_L_XL_ADDRESS 0x2A
#define OUTY_H_XL_ADDRESS 0x2B
#define OUTZ_L_XL_ADDRESS 0x2C
#define OUTZ_H_XL_ADDRESS 0x2D

#include "i2c.h"
#include <stm32l475xx.h>
#include <stdint.h>

void lsm6dsl_init();
void lsm6dsl_read_xyz(int16_t *x, int16_t *y, int16_t *z);

#endif /* LSM6DSL_H_ */
