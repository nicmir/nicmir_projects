/*
 * imu.c
 *
 *  Created on: 28 sept. 2019
 *      Author: Patrick
 */

#include "main.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>

// HW : https://www.pololu.com/product/2738
// DS : https://www.pololu.com/file/download/LSM6DS33.pdf?file_id=0J1087
// AN : https://www.pololu.com/file/0J1088/LSM6DS33-AN4682.pdf

// device I2C address (L3GD20H)
#define GYRO_I2C_ADDRESS 	0x6b

// device internal register addresses (LSM6DS33)
#define WHO_AM_I_ADDRESS 	0x0F
#define CTRL1 				0x10
#define CTRL2 				0x11
#define CTRL3 				0x12
#define CTRL4 				0x13
#define CTRL5 				0x14
#define CTRL6 				0x15
#define CTRL7 				0x16
#define CTRL8 				0x17
#define CTRL9 				0x18
#define CTRL10 				0x19

//#define LOW_ODR 			0x39
#define STATUS				0x1E
#define OUT_X_L 			0x22
#define OUT_Z_L 			0x26
#define OUT_Z_H 			0x27

// register default value (LSM6DS33)
#define WHO_AM_I_VALUE 0x69

// register configuration values (LSM6DS33)
#define CTRL10_value_init 	0x20 // GYR: Z-only enabled
#define CTRL7_value_init 	0x43 // cut off 16Hz
#define CTRL3_value_init 	0x40 // BDU Enable, BLE Data LSB @ lower address
#define CTRL2_value_init 	0x30 // GYR: 52 Hz Full scale : 245 dps

#define INIT_GYRO_BIAS 						0.0F	// unit : dps
#define GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
#define GYRO_SENSITIVITY_CORRECTION 		0.98F // unit : %

 // GYRO return codes
#define GYRO_OK 0
#define GYRO_NOT_DETECTED 1
#define GYRO_NOT_IDENTIFIED 2
#define GYRO_SETUP_FAILURE 2

// constants
#define ANGULAR_RATE_SENSITIVITY_245 0.00875 // factory sensitivity (p.15 datasheet)
#define ANGULAR_RATE_SENSITIVITY_500 0.0175 // factory sensitivity (p.15 datasheet)
#define ANGULAR_RATE_SENSITIVITY_1000 0.0350 // factory sensitivity (p.15 datasheet)

// globals
extern I2C_HandleTypeDef hi2c1;
#ifdef IMU_TRACE
	static HAL_Serial_Handler ai_com;
#endif

// private data ///////////////////////////////////////////////////////////////

typedef struct {
	int16_t raw_value; // 12-bit measure
	float rate; //dps
	float bias; // dps
	float heading; //degres
	uint32_t locked;
} ctx_gyro;

static ctx_gyro ctx;

// private functions //////////////////////////////////////////////////////////

// read helper for I2C operation
// input : device (7bit, not shifted) and register (8bit) addresses
// output : register value (8bit)
uint8_t gyro_read_8bit_register(
		uint8_t device_address,
		uint8_t register_address,
		HAL_StatusTypeDef * res
	)
{
	// send the register address to I2C device
	*res = HAL_I2C_Master_Transmit(&hi2c1, device_address << 1, &register_address , 1, 10);
	if(*res==HAL_OK)
	{
		uint8_t data = 0;
		// read the register value from I2C device
		*res = HAL_I2C_Master_Receive(&hi2c1, device_address << 1, &data, 1, 10);
		if(*res==HAL_OK)
		{
			// return the register value
			return data;
		}
		else
		{
			return 0xFF;
		}
	}
	else
	{
		return 0xFF;
	}
}

// write helper for I2C operation
// input : device (7bit, not shifted) and register (8bit) addresses, register value (8bit)
void gyro_write_8bit_register(
		uint8_t device_address,
		uint8_t register_address,
		uint8_t data,
		HAL_StatusTypeDef * res
	)
{
	// send the register address and data to I2C device
	uint8_t data_buf[]= {register_address, data};
	*res = HAL_I2C_Master_Transmit(&hi2c1, device_address << 1, data_buf , 2, 10);
}

// public functions ///////////////////////////////////////////////////////////

uint32_t gyro_init()
{
	uint8_t res_read;
	HAL_StatusTypeDef result;
	uint8_t who_am_i;

	ctx.raw_value = 0;
	ctx.rate = 0.0;
	ctx.bias = INIT_GYRO_BIAS;
	ctx.heading = 0.0f;
	ctx.locked = 0;

	who_am_i = gyro_read_8bit_register(GYRO_I2C_ADDRESS,WHO_AM_I_ADDRESS,&result);
	if(result != HAL_OK)
	{
		return GYRO_NOT_DETECTED;
	}
	if(who_am_i != WHO_AM_I_VALUE)
	{
		return GYRO_NOT_IDENTIFIED;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL2, CTRL2_value_init, &result);
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL2, &result);
	if(res_read!=CTRL2_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL3, CTRL3_value_init, &result);
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL3, &result);
	if(res_read!=CTRL3_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL7, CTRL7_value_init, &result);
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL7, &result);
	if(res_read!=CTRL7_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL10, CTRL10_value_init, &result);
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL10, &result);
	if(res_read!=CTRL10_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	return GYRO_OK;
}

void gyro_update(float duration_s)
{
	HAL_StatusTypeDef result;
	// TODO : burst read (16bits)
	uint8_t res_read_H = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUT_Z_H, &result);
	uint8_t res_read_L = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUT_Z_L, &result);
	ctx.raw_value = ((uint16_t)(res_read_H) << 8) + (uint16_t) res_read_L;
	ctx.rate = (float)(ctx.raw_value*ANGULAR_RATE_SENSITIVITY_245*GYRO_SENSITIVITY_CORRECTION);
	ctx.heading += (ctx.rate - ctx.bias)*duration_s;
}

float gyro_get_dps()
{
	return ctx.rate- ctx.bias;
}

void gyro_reset_heading()
{
	ctx.heading = 0.0F;
}

float gyro_get_heading()
{
	return ctx.heading;
}

bool gyro_is_calibrated()
{
	return ctx.locked >= 128;
}

float mean = 0.0;
float variance = 0.0;
float alpha_mean_update = 0.01;
float alpha_variance_update = 0.05;
float alpha_bias_update = 0.01;

void gyro_auto_calibrate(float duration_s)
{
	gyro_update(duration_s);
	// update mean and variance
	mean = alpha_mean_update *ctx.rate + (1.0-alpha_mean_update) * mean;
	variance = alpha_variance_update * pow( ctx.rate-mean,2)  + (1.0-alpha_variance_update) * variance;
	// if mean stable, update bias
	if(variance<GYRO_AUTOCAL_VARIANCE_THRESHOLD)
	{
		ctx.bias = alpha_bias_update*mean + (1.0-alpha_bias_update)* ctx.bias;
		if(ctx.locked<1024)
			++ctx.locked;
	}
#ifdef IMU_TRACE
		if((time%500)==0)
			HAL_Serial_Print(&ai_com,"dps=%d m=%d v=%d b=%d h=%d rate=%d\r\n",
				(int32_t)(ctx.rate*1000.0),
				(int32_t)(mean*1000.0),
				(int32_t)(variance*1000.0),
				(int32_t)(ctx.bias*1000.0),
				(int32_t)(ctx.heading),
				(int32_t)(gyro_get_dps()*1000.0)
								  );
#endif
}
