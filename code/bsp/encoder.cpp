/*
 * encoder.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#include "encoder.h"

Encoder::Encoder ( TIM_HandleTypeDef & htim ) {
	this->htim 		= & htim ;
	this->lastCount = 0 ;
}
void Encoder::setup ( void ) {
    HAL_TIM_Encoder_Start ( htim ,TIM_CHANNEL_ALL ) ;
}
void Encoder::stop ( void ) {
	HAL_TIM_Encoder_Stop ( htim ,TIM_CHANNEL_ALL ) ;
}



