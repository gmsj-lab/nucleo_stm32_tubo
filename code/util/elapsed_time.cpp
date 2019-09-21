/*
 * elapsed_time.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include <elapsed_time.h>

ElapsedTime::ElapsedTime () {
	startTime   = 0 ;
}

void ElapsedTime::start ( void ) {
	startTime = micros () ;
}

uint32_t ElapsedTime::stop ( void ) {
	return micros () - startTime ;
}


