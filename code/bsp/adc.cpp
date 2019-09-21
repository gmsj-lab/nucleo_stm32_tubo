/*
 * adc.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include <adc_driver.h>

#define ADC_MAX_DIGITAL_VALUE	4095.0  // ADC 16 Bits
#define ADC_MAX_ANALOG_VALUE	3300.0 	// Max Value is given for VCC voltage (in mV)

Adc::Adc ( ADC_HandleTypeDef & hadc , double conversionRatio) {
	this->active 			= false ;
	this->hadc			  	 = & hadc ;
	this->convertedValue.u32 = 0 ;
	this->adcRatio 			 = ( ADC_MAX_ANALOG_VALUE / ADC_MAX_DIGITAL_VALUE ) / float(conversionRatio) ;
}

void Adc::init ( void ) {
	if ( HAL_ADC_Start_DMA (  this->hadc , (uint32_t*) & convertedValue.u32 , 1 ) != HAL_OK ) {
		app_error ( ADC_ERROR ) ;
	}
	active = true ;
}

void Adc::stop ( void ) {
	active = false ;
	HAL_ADC_Stop ( hadc ) ;
	// HAL_ADC_Stop_DMA instead ?
}


// ARCHIVES : Read without DMA
#if 0
uint16_t Adc::readADC ( void ) {
	HAL_ADC_Start ( hadc ) ;

	if ( HAL_ADC_PollForConversion ( hadc, 1000 ) != HAL_OK ) {
		app_error ( ADC_ERROR ) ;
		}
	HAL_ADC_Stop ( hadc ) ;

	convertedValue.u32 = HAL_ADC_GetValue ( hadc ) ;
	return (uint16_t) convertedValue.u16 * adcRatio  ;
}
#endif



