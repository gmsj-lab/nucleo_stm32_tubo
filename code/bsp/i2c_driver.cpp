/*
 * i2c_driver.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "i2c_driver.h"

#define I2C_ONE_BYTE  		  1
#define READ_TIMEOUT	     10
#define WRITE_TIMEOUT	    100

I2cDriver::I2cDriver ( I2C_HandleTypeDef * handle ) {
	this->handle = handle ;
}

I2cDriver::~I2cDriver () {
}

void I2cDriver::begin ( void ) {
}

void I2cDriver::write ( uint8_t  devAddress, uint8_t  memAddress, uint8_t data ) {
	if ( HAL_I2C_Mem_Write ( handle, devAddress, memAddress , I2C_MEMADD_SIZE_8BIT, & data, I2C_ONE_BYTE , WRITE_TIMEOUT ) != HAL_OK ) {
		app_error ( I2C_WRITE ) ;
	}
}

uint8_t I2cDriver::read ( uint8_t devAddress, uint8_t memAddress ) {
	uint8_t data ;
	if ( HAL_I2C_Mem_Read ( handle, devAddress, memAddress , I2C_MEMADD_SIZE_8BIT, & data, I2C_ONE_BYTE , READ_TIMEOUT ) != HAL_OK ) {
		app_error ( I2C_READ ) ;
	}
	return data ;
}

void I2cDriver::read  ( uint8_t devAddress, uint8_t memAddress , uint8_t * data , uint8_t length ) {

	if ( HAL_I2C_Mem_Read ( handle, devAddress, memAddress , I2C_MEMADD_SIZE_8BIT, data, length , READ_TIMEOUT ) != HAL_OK ) {
		app_error ( I2C_MULTI_READ ) ;
	}
}
