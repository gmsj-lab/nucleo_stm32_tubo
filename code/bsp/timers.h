/*
 * timers.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_BSP_TIMERS_H_
#define CODE_BSP_TIMERS_H_


#include <utils.h>
#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

typedef enum
{
	 _1000_HZ_TIMER = 1000 ,
	  _500_HZ_TIMER =  500 ,
	  _400_HZ_TIMER =  400 ,
      _200_HZ_TIMER =  200 ,
       _50_HZ_TIMER =   50 ,
	   _25_HZ_TIMER =   25 ,
       _10_HZ_TIMER =   10 ,
		_1_HZ_TIMER = 	 1

} TimersFrequency ;

class Timers {
private:
	uint32_t tickCount ;
public:
	Timers ( TIM_HandleTypeDef & htim ) ;
	virtual ~Timers () {}

	TIM_HandleTypeDef * htim ;
	Callback 			   _1msCallback ;
	Callback 			   _2msCallback ;
	Callback 			_2_5_msCallback ;
	Callback 			   _5msCallback ;
	Callback 			  _20msCallback ;
	Callback 			  _40msCallback ;
	Callback 			 _100msCallback ;
	Callback 			_1000msCallback ;

	void setCallback 	( TimersFrequency frequency, Callback callback ) ;
	void init 		 	( void ) ;
	void stop 		 	( void ) ;
	void timersCallback ( void ) ;
};
#endif /* CODE_BSP_TIMERS_H_ */
