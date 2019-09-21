/*
 * adc.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_BSP_ADC_H_
#define CODE_BSP_ADC_H_

#include "utils.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32f7xx_hal_adc.h"

typedef union _uint16_32_t {
	uint16_t u16 ;
	uint32_t u32 ;
} uint16_32_t ;

class Adc {
private:
	ADC_HandleTypeDef * hadc ;
	__IO uint16_32_t convertedValue ;
	double			 adcRatio ;
	bool 			 active ;
	Adc(const Adc&) ;
public:
				 Adc ( ADC_HandleTypeDef & hadc , double conversionRatio ) ;
	virtual 	~Adc () {} ;

			void	 	init 	( void ) ;
			void	 	stop 	( void ) ;
	inline  uint16_t 	read 	( void ) { return (uint16_t) convertedValue.u16 * adcRatio  ; }	// Read DMA (readings in mV)
#if 0

	uint16_t 	readADC ( void ) ;	// Read without DMA, unused, kept for archives (Do not call init() with this version)
#endif
};

#endif /* CODE_BSP_ADC_H_ */
