/*
 * lsm6dsl.c
 *
 *  Created on: Feb 5, 2025
 *      Author: Yu-Heng Lin, Linfeng Zhang
 */
#include "lsm6dsl.h"

void lsm6dsl_init()
{
	// initialize i2c
	i2c_init();

	// set up ctrl1_xl data
	uint8_t ctrl1_xl_data[2];
	ctrl1_xl_data[0] = CTRL1_XL_ADDRESS;
	ctrl1_xl_data[1] = 0x60;

	// set up int1_ctrl data
	uint8_t int1_ctrl_data[2];
	int1_ctrl_data[0] = INT1_CTRL_ADDRESS;
	int1_ctrl_data[1] = 0x01;

	// write to ctrll_xl and int1_ctrl to initialize
	i2c_transaction(LSM6DSL_ADDRESS, 0x00, ctrl1_xl_data, 0x02);
	i2c_transaction(LSM6DSL_ADDRESS, 0x00, int1_ctrl_data, 0x02);
}

void lsm6dsl_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
	// Tell lsm6dsl where we want to read the data from, write->read

	// addresses to write to
	uint8_t outx_l_xl_reg[1] =
	{ OUTX_L_XL_ADDRESS };

	uint8_t outx_h_xl_reg[1] =
	{ OUTX_H_XL_ADDRESS };

	uint8_t outy_l_xl_reg[1] =
	{ OUTY_L_XL_ADDRESS };

	uint8_t outy_h_xl_reg[1] =
	{ OUTY_H_XL_ADDRESS };

	uint8_t outz_l_xl_reg[1] =
	{ OUTZ_L_XL_ADDRESS };

	uint8_t outz_h_xl_reg[1] =
	{ OUTZ_H_XL_ADDRESS };

	// array to store the read data
	uint8_t outx_l_xl_data[1];
	uint8_t outx_h_xl_data[1];
	uint8_t outy_l_xl_data[1];
	uint8_t outy_h_xl_data[1];
	uint8_t outz_l_xl_data[1];
	uint8_t outz_h_xl_data[1];

	// x low
	i2c_transaction(LSM6DSL_ADDRESS, 0, outx_l_xl_reg, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, outx_l_xl_data, 1);
	// x high
	i2c_transaction(LSM6DSL_ADDRESS, 0, outx_h_xl_reg, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, outx_h_xl_data, 1);
	// find raw x
	*x = (int16_t) ((outx_h_xl_data[0] << 8) | outx_l_xl_data[0]);

	// y low
	i2c_transaction(LSM6DSL_ADDRESS, 0, outy_l_xl_reg, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, outy_l_xl_data, 1);
	// y high
	i2c_transaction(LSM6DSL_ADDRESS, 0, outy_h_xl_reg, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, outy_h_xl_data, 1);
	*y = (int16_t) ((outy_h_xl_data[0] << 8) | outy_l_xl_data[0]);

	// z low
	i2c_transaction(LSM6DSL_ADDRESS, 0, outz_l_xl_reg, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, outz_l_xl_data, 1);
	// z high
	i2c_transaction(LSM6DSL_ADDRESS, 0, outz_h_xl_reg, 1);
	i2c_transaction(LSM6DSL_ADDRESS, 1, outz_h_xl_data, 1);
	*z = (int16_t) ((outz_h_xl_data[0] << 8) | outz_l_xl_data[0]);
}
