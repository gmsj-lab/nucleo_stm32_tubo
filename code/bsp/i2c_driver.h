/*
 * i2c_driver.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_BSP_I2C_DRIVER_H_
#define CODE_BSP_I2C_DRIVER_H_


#include "utils.h"
#include "stm32f7xx_hal.h"

class I2cDriver {
private:
	I2C_HandleTypeDef *handle ;
public:
	I2cDriver ( I2C_HandleTypeDef * handle ) ;
	virtual ~I2cDriver () ;

	void 	begin ( void ) ;
	void    write ( uint8_t devAddress, uint8_t memAddress, uint8_t data ) ;

	uint8_t read  ( uint8_t devAddress, uint8_t memAddress ) ;
	void    read  ( uint8_t devAddress, uint8_t memAddress , uint8_t * data , uint8_t length ) ;
} ;
#endif /* CODE_BSP_I2C_DRIVER_H_ */
