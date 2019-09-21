/*
 * sliding_window_double.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_SLIDING_WINDOW_DOUBLE_H_
#define CODE_UTIL_SLIDING_WINDOW_DOUBLE_H_


#include "dynamic_array.h"

typedef DynamicArray < double > ArrayDouble  ;


class SlidingWindowDouble
{
private:
	double  	sum ;
	uint8_t  	current ;
	uint8_t  	length ;
	ArrayDouble window ;

public:
	// size of the sliding window
	SlidingWindowDouble ( uint8_t size ) ;

	void 	reset   ( void ) ;
	void 	set    	( double sample ) ; // Add new sample in window
	double  get		( void ) ; 			// Get windowed value
	double  getSum 	( void ) ; 			// Get the sum
} ;
#endif /* CODE_UTIL_SLIDING_WINDOW_DOUBLE_H_ */
