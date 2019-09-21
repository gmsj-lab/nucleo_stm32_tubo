/*
 * gpio_output.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_BSP_GPIO_OUTPUT_H_
#define CODE_BSP_GPIO_OUTPUT_H_

#include "utils.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"

class GpioOutput {

private:
	GPIO_PinState	onState ;
	GPIO_PinState	offState ;
	uint32_t 	  	gpioPin ;
	GPIO_TypeDef* 	gpioPort ;

public:
			GpioOutput 	( uint32_t gpioPin , GPIO_TypeDef* gpioPort , bool inverted = false) ;
	virtual ~GpioOutput () {} ;

	inline void write	( GPIO_PinState pinState )  { HAL_GPIO_WritePin  ( gpioPort , gpioPin , pinState ) ;							}
	inline void set		( bool state ) 				{ HAL_GPIO_WritePin  ( gpioPort , gpioPin , (state == true) ? onState : offState ) ;}
	inline void on		( void ) 					{ HAL_GPIO_WritePin  ( gpioPort , gpioPin , onState ) ;								}
	inline void off		( void ) 					{ HAL_GPIO_WritePin  ( gpioPort , gpioPin , offState ) ;							}
	inline void toggle	( void ) 					{ HAL_GPIO_TogglePin ( gpioPort , gpioPin ) ; 										}
} ;

#endif /* CODE_BSP_GPIO_OUTPUT_H_ */
