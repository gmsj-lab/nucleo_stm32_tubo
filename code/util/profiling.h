/*
 * profiling.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_PROFILING_H_
#define CODE_UTIL_PROFILING_H_

#include "main_user.h"
#if USE_PROFILING == true

#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stdint.h"
#include "util.h"
#include "stack.h"
#include "target_attribute.h"

#define 		MAX_LEVEL	6
#define 		MASK_IT		HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn) // __disable_irq () ;
#define 		UNMASK_IT	HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn)  // __enable_irq () ;

class Profiling {

private:
	uint8_t 	currentLevel ;
	Stack 		stack ;
	uint32_t 	processingTime [ MAX_LEVEL ] ;
	uint32_t	lastTime ;
	TT_bool		on ;
	TT_uint32	defaultLevel ;
	TT_uint32	level1 ;
	TT_uint32	level2 ;
	TT_uint32	level3 ;
	TT_uint32	level4 ;
	TT_uint32	level5 ;

public:

	Profiling ( const char * group = "PROFILING" ,
				const char * name1 = "Level1" 	 ,
				const char * name2 = "Level2" 	 ,
				const char * name3 = "Level3" 	 ,
				const char * name4 = "Level4" 	 ,
				const char * name5 = "Level5" 	 ) ;

	~Profiling () {} ;

	inline void clear ( void ) {
		defaultLevel.set ( 0 ) ;
		level1		.set ( 0 ) ;
		level2		.set ( 0 ) ;
		level3		.set ( 0 ) ;
		level4		.set ( 0 ) ;
		level5		.set ( 0 ) ;
	}

	 void begin ( uint8_t level ) {
		MASK_IT ;
		// Previous level is interrupted, record the time spent on this level so far
		record ( stack.top () ) ;
		// Time start to be counted for this level
		stack.push ( (int)level ) ;

		UNMASK_IT ;
	}
	 void end ( void ) {
		MASK_IT ;
		// Record the time spent on this level so far and switch back to previous level
		if ( ! stack.isEmpty() )
			record ( (int) stack.pop () ) ;
		UNMASK_IT ;
	}
	 void record ( uint8_t level ) {
		uint32_t currentTime  = micros () ;
		if ( level > 5 ) {
			app_error ( MAIN_ERROR ) ;
		}
		processingTime [ level ] += ( currentTime - lastTime ) ;
		lastTime = currentTime ;
	}
	inline void show ( void ) {
		if ( on.get () == true ) {
			defaultLevel.set ( processingTime [ 0 ] ) ;
			level1		.set ( processingTime [ 1 ] ) ;
			level2		.set ( processingTime [ 2 ] ) ;
			level3		.set ( processingTime [ 3 ] ) ;
			level4		.set ( processingTime [ 4 ] ) ;
			level5		.set ( processingTime [ 5 ] ) ;
		}
		// Reset
		for( uint8_t level = 0 ; level < MAX_LEVEL ; level++ ) {
			processingTime [ level ] = 0 ;
		}
	}
} ;
#endif /* USE_PROFILING == true */

#endif /* CODE_UTIL_PROFILING_H_ */
