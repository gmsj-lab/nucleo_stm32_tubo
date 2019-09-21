/*
 * ewma_int.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_UTIL_EWMA_INT_H_
#define CODE_UTIL_EWMA_INT_H_

#if 0
#include <stdint.h>

class Ewma_int {

private:
	double 	decay  ;
	int16_t output ;

public:
	virtual ~Ewma_int () {} ;

	// size of the sliding window and threshold to choose between last value if > threshold or windowed value. default = always windowed value
	Ewma_int ( uint8_t size = 10 ) ;

	void 	  init ( int16_t & sample ) ; 	// Add first sample in window, if not called, first few result will be erroneous
	void 	  set  ( int16_t & sample ) ; 	// Add new sample in window
	int16_t & get  ( void ) ; 				// Get filtered value
};
#endif
#endif /* CODE_UTIL_EWMA_INT_H_ */
