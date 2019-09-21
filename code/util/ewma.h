/*
 * ewma.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_EWMA_H_
#define CODE_UTIL_EWMA_H_

#include <stdint.h>

class Ewma {

private:
	double weight  ;
	double output ;

public:
	virtual ~Ewma () {} ;

	Ewma ( double weightLast = 0.2 )  { weight  = weightLast ; output = 0.0 ;	 				 	 					}
	// Add first sample in window, if not called, first few result will be erroneous
	inline void 	init ( double sample ) { output = sample ;									 	 					}
	inline double 	set  ( double sample ) { output = (sample * weight) + (output * (1.0 - weight) ) ; return output ;	}
	inline double 	get  ( void 		 ) { return output ;										 					}
};
#endif /* CODE_UTIL_EWMA_H_ */
