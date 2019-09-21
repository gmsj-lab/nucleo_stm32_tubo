/*
 * elapsed_time.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_UTIL_ELAPSED_TIME_H_
#define CODE_UTIL_ELAPSED_TIME_H_

#include "utils.h"

class ElapsedTime {

private:
	uint32_t startTime   ;

public:
	ElapsedTime ( void ) ;
	virtual ~ElapsedTime () {}

	void	 start 		( void ) ;
	uint32_t stop 		( void ) ;
};
#endif /* CODE_UTIL_ELAPSED_TIME_H_ */
