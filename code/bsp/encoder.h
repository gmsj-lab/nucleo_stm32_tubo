/*
 * encoder.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_BSP_ENCODER_H_
#define CODE_BSP_ENCODER_H_


#include "stm32f7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

class Encoder {

private:
	TIM_HandleTypeDef * htim ;
	int32_t				lastCount ;

	Encoder ( void ) ;
public:
	Encoder ( TIM_HandleTypeDef & htim ) ;
	virtual ~Encoder () {}

	void setup	( void ) ;
	void stop	( void ) ;

	inline int32_t getDelta ( void ) {
		int32_t  currentCount	=  (int32_t) __HAL_TIM_GET_COUNTER ( htim ) ;
		int32_t  delta 			= currentCount - lastCount ;
		lastCount 				= currentCount ;
		return ( delta ) ;
	}

	inline int32_t get ( void ) {
		return (int32_t) __HAL_TIM_GET_COUNTER ( htim ) ;
	}
	inline void reset ( void ) {
		__HAL_TIM_SET_COUNTER ( htim , 0 ) ;
		lastCount = 0 ;
	}
} ;

#ifdef __cplusplus
}
#endif
#endif /* CODE_BSP_ENCODER_H_ */
