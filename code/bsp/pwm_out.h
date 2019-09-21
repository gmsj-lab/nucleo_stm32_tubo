/*
 * pwm_out.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_BSP_PWM_OUT_H_
#define CODE_BSP_PWM_OUT_H_


#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

class PwmOut {
private:
	TIM_HandleTypeDef * htim ;
	uint32_t 			channel ;
	volatile uint32_t *	timerCCR ;

public:
	PwmOut ( TIM_HandleTypeDef & htim , uint32_t channel ) ;
	virtual ~PwmOut () {}

	void 		start 	( void ) ;
	inline void set  	( uint32_t value ) { * timerCCR = value ; }
};
#endif /* CODE_BSP_PWM_OUT_H_ */
