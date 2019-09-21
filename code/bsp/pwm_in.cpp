/*
 * pwm_in.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "pwm_in.h"

PwmIn::PwmIn ( TIM_HandleTypeDef & htim , uint32_t channel ) {
	this->htim = & htim ;

	if ( channel == TIM_CHANNEL_1  ) {
		this->activeChannel = TIM_CHANNEL_1 ;
		this->otherChannel  = TIM_CHANNEL_2 ;
	}
	else if ( channel == TIM_CHANNEL_2  ) {
		this->activeChannel = TIM_CHANNEL_2 ;
		this->otherChannel  = TIM_CHANNEL_1 ;
	}
}

void PwmIn::start ( void ) {
	HAL_TIM_IC_Start ( htim , activeChannel ) ;
	HAL_TIM_IC_Start ( htim , otherChannel  ) ;
}

uint32_t PwmIn::getRaw ( void ) {
	HAL_TIM_ReadCapturedValue ( htim, activeChannel ) ;
	return HAL_TIM_ReadCapturedValue ( htim, otherChannel  ) ;
}
//uint32_t PwmIn::getDutyCycle ( void ) {
//	uint32_t dutyCycle 	= 0 ;
//	uint32_t active  	= HAL_TIM_ReadCapturedValue ( htim, activeChannel ) ;
//	uint32_t low	 	= HAL_TIM_ReadCapturedValue ( htim, otherChannel  ) ;
//
//	if ( active > low ) {
//		// something must be inverted..
//		uint32_t tmp = active ;
//		active 		 = low ;
//		low			 = tmp ;
//	}
//
//	if ( low != 0 ) dutyCycle = ( active * 100.0 ) / (double) low ;
//	return dutyCycle ;
//}
uint32_t PwmIn::getFrequency ( void ) {
    return (HAL_RCC_GetHCLKFreq()) /2 / HAL_TIM_ReadCapturedValue ( htim, activeChannel ) ;
}
