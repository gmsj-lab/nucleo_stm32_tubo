/*
 * motor.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "motor.h"
#include <utils.h>

// TIM1 used at 19,63 KHz ( 216 MHz / 11 (prescaler = 10) / 1000 (autoreload = 999)
#define TIM_AUTORELOADREG 		   999  // valeur du registre Auto Reload Register du Timers 1
#define BREAK_THRESHOLD		  	  	0   // value. max : 1000
#define FORWARD_GEAR_PLAY 			0
#define BACKWARD_GEAR_PLAY 			1
#define CATCH_UP_GEAR_PLAY_TIME    30	// in ms, time to operate full speed to catch up gear play

Motor::Motor ( void ) {
	pwm 			= NULL ;
	forwardInput	= NULL ;
	backwardInput	= NULL ;
	gearPlayDirection = 0 ;
}
void Motor::init ( PwmOut & pwm , GpioOutput & forwardInput , GpioOutput & backwardInput ) {

	this->pwm 			= & pwm ;
	this->forwardInput	= & forwardInput ;
	this->backwardInput	= & backwardInput ;

	setThrottle ( 0 ) ;
	pwm.start 	() ;
}

void Motor::setThrottle ( int16_t  throttle ) {

	uint32_t throttleValue = (uint32_t) ABS( throttle ) ;

	if ( throttleValue < BREAK_THRESHOLD ) {
		brake ( throttleValue ) ;
	}
	else if ( throttle < 0 ) {
		if ( gearPlayDirection == FORWARD_GEAR_PLAY) {
			// Catch up the play in gears by going full speed backward for limited time before command
			setBackward ( TIM_AUTORELOADREG ) ;
			delay_ms ( CATCH_UP_GEAR_PLAY_TIME ) ;
			gearPlayDirection = BACKWARD_GEAR_PLAY ;
		}
		setBackward ( throttleValue ) ;
	}
	else {
		if ( gearPlayDirection == BACKWARD_GEAR_PLAY) {
			// Catch up the play in gears by going full speed forward for limited time before command
			setForward ( TIM_AUTORELOADREG ) ;
			delay_ms ( CATCH_UP_GEAR_PLAY_TIME ) ;
			gearPlayDirection = FORWARD_GEAR_PLAY ;
		}
		setForward  ( throttleValue ) ;
	}
}
void Motor::setForward ( uint32_t throttle ) {
	pwm->set ( CONSTRAIN ( throttle , 0 , TIM_AUTORELOADREG ) ) ;
	forwardInput ->on  () ;
	backwardInput->off () ;
}
void Motor::setBackward ( uint32_t throttle ) {
	pwm->set ( CONSTRAIN ( throttle , 0 , TIM_AUTORELOADREG ) ) ;
	forwardInput ->off () ;
	backwardInput->on  () ;
}
void Motor::brake ( uint32_t brake ) {
	pwm->set ( CONSTRAIN ( brake , 0 , TIM_AUTORELOADREG ) ) ;
	forwardInput ->on () ;
	backwardInput->on () ;
}


