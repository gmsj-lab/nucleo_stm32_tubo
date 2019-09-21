/*
 * sliding_window.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "sliding_window.h"
#include "utils.h"

SlidingWindow::SlidingWindow ( uint8_t size , int32_t averagingThreshold ) : window ( size ) {
	this->averagingThreshold = averagingThreshold ;
	reset () ;
}

void SlidingWindow::reset ( void ) {
	sum 	= 0 ;
	current = 0 ;
	length  = 0 ;
	for ( int i = 0 ; i < window.length () ; i ++ ) {
		window [ i ] = 0 ;
	}
}

// Add new sample in window
void SlidingWindow::set ( int32_t sample ) {
	current = ( current + 1 ) % window.length () ;
	sum -= window [ current ] ;
	sum	+= sample ;

	window [ current ] = sample ;
	if ( length < window.length () ) {
		length ++ ;
	}
}

// Get last sample if > threshold, windowed value otherwise
float SlidingWindow::get ( void ) {
	return ( ABS( window [ current ] ) > averagingThreshold ) ? (float) window [ current ] : ( sum / (float) length ) ;
}

float SlidingWindow::getSum ( void ) {
	return ( sum ) ;
}



