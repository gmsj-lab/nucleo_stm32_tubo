/*
 * sliding_window_double.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include <sliding_window_double.h>
#include "utils.h"

SlidingWindowDouble::SlidingWindowDouble ( uint8_t size ) : window ( size ) {
	reset () ;
}

void SlidingWindowDouble::reset ( void ) {
	sum 	= 0.0 ;
	current = 0 ;
	length  = 0 ;
	for ( int i = 0 ; i < window.length () ; i ++ ) {
		window [ i ] = 0.0 ;
	}
}

// Add new sample in window
void SlidingWindowDouble::set ( double sample ) {
	current = ( current + 1 ) % window.length () ;
	sum -= window [ current ] ;
	sum	+= sample ;

	window [ current ] = sample ;
	if ( length < window.length () ) {
		length ++ ;
	}
}
// Get average value
double SlidingWindowDouble::get ( void ) {
	return ( sum / (double) length ) ;
}
// Get sum
double SlidingWindowDouble::getSum ( void ) {
	return ( sum ) ;
}


