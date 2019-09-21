/*
 * ewma_int.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */
#if 0
#include <ewma_int.h>

Ewma_int::Ewma_int ( uint8_t size ) {
	decay  = 2.0 / ( (size / 2) + 1 ) ;
	output = 0.0 ;
}

void Ewma_int::init ( int16_t & sample ) {
	output = sample ;
}

void Ewma_int::set ( int16_t & sample ) {
	output = (sample * decay) + (output * (1.0 - decay) ) ;
}

int16_t & Ewma_int::get ( void ) {
	return output ;
}
#endif

