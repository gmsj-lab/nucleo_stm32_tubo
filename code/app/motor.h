/*
 * motor.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_MOTOR_H_
#define CODE_APP_MOTOR_H_


#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "gpio_output.h"
#include "pwm_out.h"

#define MAX_THROTTLE 		1000

class Motor {

private:

	PwmOut 		* pwm ;
	GpioOutput 	* forwardInput ;
	GpioOutput 	* backwardInput ;
	int           gearPlayDirection ;

	void setForward  ( uint32_t throttle ) ;
	void setBackward ( uint32_t throttle ) ;

public:

	Motor () ;
	virtual ~Motor () {} ;

	void init			( PwmOut & pwm , GpioOutput & forwardInput , GpioOutput & backwardInput ) ;
	void setThrottle 	( int16_t  throttle ) ; 	//  From -1000  to 1000
	void brake 		 	( uint32_t brake ) ;		//  From 0 to 100 %, max brake at 100 %
} ;
#endif /* CODE_APP_MOTOR_H_ */
