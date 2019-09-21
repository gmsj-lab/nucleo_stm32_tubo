/*
 * odometer.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_APP_ODOMETER_H_
#define CODE_APP_ODOMETER_H_


/* Includes ------------------------------------------------------------------*/

#include "utils.h"
#include "stm32f7xx_hal.h"
#include "target_attribute.h"
#include "encoder.h"

class Position {
public:
	int32_t x ;
	int32_t y ;
	int32_t angle ;
} ;

class Distance {
public:
	int32_t leftWheel ;
	int32_t rightWheel ;
	int32_t mean ;
} ;
typedef Distance Velocity ;

class Odometer {

private:
	double	 		period ;
	Encoder *		leftEncoder ;
	Encoder *		rightEncoder ;

	Position		position ;
	Distance		distance ;
	Distance		totalDistance ;
	Velocity		speed ;

	inline void computePosition ( double meanDistance, double leftRightDelta ) ;

public:
					 Odometer () ;
	virtual 		~Odometer () {}

	void 			   init			  	( uint32_t timerPeriodInMs, Encoder & left , Encoder & right ) ;
	void 			   reset		   	( void ) ;
	void 			   update		  	( void ) ; 								// record the distance over the last period to compute velocity
	inline 	Position & getPosition 	  	( void ) { return position ; 	  } ; 	// In milimeters and angle in degree relative to beginning of time (i.e. last reset() )
	inline	Distance & getDistance 	  	( void ) { return distance ; 	  } ; 	// In milimeters forward - backwards since last update
	inline 	Velocity & getSpeed 	 	( void ) { return speed ;		  } ; 	// In milimeters/seconds since last update (or a little more if not enough samples )
	Distance & 		   getTotalDistance	( void ) ; 								// In milimeters forward - backwards since beginning of time
} ;


#endif /* CODE_APP_ODOMETER_H_ */
