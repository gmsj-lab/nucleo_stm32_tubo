/*
 * periodicity_check.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "periodicity_check.h"

Periodicity::Periodicity () {
	current  		= 0 ;
	last 	 		= 0 ;
	highestExpected = 0 ;
	lowestExpected 	= 0 ;
}

void Periodicity::init 	( uint32_t period_us, uint16_t accuracy_us ) {
	highestExpected = period_us + accuracy_us ;
	lowestExpected  = period_us - accuracy_us ;
}

uint32_t Periodicity::check ( void ) {
	current = micros () ;
	uint32_t elapsed = current - last ;

	if ( ( last != 0 ) ) {
		if ( ( elapsed > highestExpected ) || ( elapsed < lowestExpected ) ) {
			for ( int i = 0 ; i < 5 ; i ++ ) {
				BSP.ledErreur.on () ;
				BSP.ledWifi.on () ;
			}
		}
		else {
			BSP.ledErreur.off () ;
			BSP.ledWifi.off () ;
		}
	}
	last = current ;
	return elapsed ;
}




