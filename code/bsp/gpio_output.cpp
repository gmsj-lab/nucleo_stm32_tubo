/*
 * gpio_output.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "gpio_output.h"

GpioOutput::GpioOutput ( uint32_t gpioPin , GPIO_TypeDef* gpioPort , bool inverted ) {
	this->gpioPin  	= gpioPin ;
	this->gpioPort 	= gpioPort ;

	if ( inverted == true ) {
		onState  = GPIO_PIN_RESET ;
		offState = GPIO_PIN_SET ;
	}
	else {
		onState  = GPIO_PIN_SET ;
		offState = GPIO_PIN_RESET ;
	}
}
