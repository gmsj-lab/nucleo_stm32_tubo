/*
 * timer.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "timer.h"

Timer::Timer ( TIM_HandleTypeDef & htim ) {
	timerCallback = NULL ;
	this->htim = & htim ;
}

void Timer::setCallback ( Callback callback ) {
	timerCallback = callback ;
}

void Timer::init ( void ) {
	HAL_TIM_Base_Start_IT ( htim ) ;
}

int Timer::getPeriod ( void ) {
	// timer period in ms  = 1000ms * (timer prescaler+1) * (timer counter+1) / timer frequency
	return (htim->Init.Prescaler + 1 ) * ( htim->Init.Period + 1 ) / 108000.0 ;
}

int Timer::getFrequency ( void ) {
	return 1000  / getPeriod () ;
}

void Timer::stop ( void ) {
	HAL_TIM_Base_Stop_IT ( htim ) ;
}
