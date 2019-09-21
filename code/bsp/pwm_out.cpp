/*
 * pwm_out.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "pwm_out.h"

PwmOut::PwmOut ( TIM_HandleTypeDef & htim , uint32_t channel ) {
	this->htim 		= & htim ;
	this->channel 	= channel ;
	this->timerCCR  = NULL ;
}

void PwmOut::start ( void ) {
	HAL_TIM_PWM_Start ( htim , channel ) ;

	if      ( channel == TIM_CHANNEL_1  ) timerCCR = & htim->Instance->CCR1 ;
	else if ( channel == TIM_CHANNEL_2  ) timerCCR = & htim->Instance->CCR2 ;
}
