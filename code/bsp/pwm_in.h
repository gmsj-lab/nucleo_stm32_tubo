/*
 * pwm_in.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_BSP_PWM_IN_H_
#define CODE_BSP_PWM_IN_H_


#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

class PwmIn {
private:
	TIM_HandleTypeDef * htim ;
	uint32_t 			activeChannel ;
	uint32_t 			otherChannel ;

public:
	PwmIn ( TIM_HandleTypeDef & htim , uint32_t channel ) ;
	virtual ~PwmIn () {} ;

	void 		start 			( void ) ;
	uint32_t	getFrequency  	( void ) ;
	uint32_t	getRaw  		( void ) ;
};

#endif /* CODE_BSP_PWM_IN_H_ */
