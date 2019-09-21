/*
 * sliding_window.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_SLIDING_WINDOW_H_
#define CODE_UTIL_SLIDING_WINDOW_H_


#include "dynamic_array.h"

typedef DynamicArray < int32_t > Array ;


class SlidingWindow
{
private:
	int32_t  averagingThreshold ;
	int32_t  sum ;
	uint8_t  current ;
	uint8_t  length ;
	Array 	 window ;

public:
	// size of the sliding window and threshold to choose between last value if > threshold or windowed value. default = always windowed value
	SlidingWindow ( uint8_t size , int32_t averagingThreshold = INT32_MAX ) ;

	void 	reset 	( void ) ;
	void 	set   	( int32_t sample ) ; 	// Add new sample in window
	float  	get	  	( void ) ; 				// Get last sample if > threshold, windowed value otherwise
	float  	getSum 	( void ) ; 				// Get the sum
} ;

#endif /* CODE_UTIL_SLIDING_WINDOW_H_ */
