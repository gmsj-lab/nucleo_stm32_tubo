/*
 * gpio_input.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include <utils.h>
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "gpio_input.h"

GpioInput::GpioInput ( uint32_t gpioPin , GPIO_TypeDef* gpioPort ) {
	this->gpioPin  				= gpioPin ;
	this->gpioPort 				= gpioPort ;
	this->gpioInterruptCallback = NULL ;
}

void GpioInput::setCallback ( Callback gpioInterruptCallback ) {
	this->gpioInterruptCallback = gpioInterruptCallback ;
}

GPIO_PinState GpioInput::read ( void ) {
	return ( HAL_GPIO_ReadPin ( gpioPort , gpioPin ) ) ;
}

void GpioInput::gpioInterrupt  ( void ) {
	if ( gpioInterruptCallback != NULL ) gpioInterruptCallback () ;
}
