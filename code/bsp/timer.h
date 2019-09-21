/*
 * timer.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_BSP_TIMER_H_
#define CODE_BSP_TIMER_H_


#include <utils.h>
#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

class Timer {
public:
	Timer ( TIM_HandleTypeDef & htim ) ;
	virtual ~Timer () {}

	TIM_HandleTypeDef * htim ;
	Callback 			timerCallback ;

	void setCallback 	( Callback callback ) ;
	void init 		 	( void ) ;
	int	 getPeriod 	 	( void ) ;  // Period in ms
	int	 getFrequency	( void ) ;  // Frequency in Hz
	void stop 		 	( void ) ;

};

#endif /* CODE_BSP_TIMER_H_ */
