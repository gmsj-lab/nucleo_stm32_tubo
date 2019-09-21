/*
 * periodicity_check.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_PERIODICITY_CHECK_H_
#define CODE_UTIL_PERIODICITY_CHECK_H_

#include "utils.h"
#include "bsp.h"

class Periodicity {

private:
	uint32_t current  ;
	uint32_t last 	  ;
	uint32_t highestExpected ;
	uint32_t lowestExpected ;

public:
	Periodicity () ;
	virtual ~Periodicity () {};

	void 	 init 	( uint32_t period_us, uint16_t accuracy ) ; // define expected frequency and accuracy
	uint32_t check 	( void ) ; 									// Used to check timer callback frequencies
} ;

#endif /* CODE_UTIL_PERIODICITY_CHECK_H_ */
