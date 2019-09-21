/*
 * gpio_input.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_BSP_GPIO_INPUT_H_
#define CODE_BSP_GPIO_INPUT_H_


#include <utils.h>
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"

class GpioInput {

private:
	uint32_t 	  	gpioPin ;
	GPIO_TypeDef* 	gpioPort ;
	Callback		gpioInterruptCallback ;

public:
					GpioInput 		( uint32_t gpioPin , GPIO_TypeDef* gpioPort ) ;
	virtual 		~GpioInput 		() {} ;

	void 			setCallback		( Callback gpioInterruptCallback ) ;
	GPIO_PinState 	read			( void ) ;
	void 			gpioInterrupt 	( void ) ;
} ;

#endif /* CODE_BSP_GPIO_INPUT_H_ */
