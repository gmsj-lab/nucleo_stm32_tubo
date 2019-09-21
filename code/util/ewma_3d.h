/*
 * ewma_3d.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_EWMA_3D_H_
#define CODE_UTIL_EWMA_3D_H_

#if 0
#include "Sensor.h"

class Ewma3D {
private:
	double 		decay  ;
	double_3D_t output ;

public:
	virtual ~Ewma3D () {};

	// size of the filter
	Ewma3D ( uint8_t size = 20 ) ;

	void 		  init ( double_3D_t & sample ) ; 	// Add first sample in window, if not called, first few result will be erroneous
	void 		  set  ( double_3D_t & sample ) ; 	// Add new sample in window
	double_3D_t & get  ( void ) ; 				 	// Get filtered value
};
#endif
#endif /* CODE_UTIL_EWMA_3D_H_ */
