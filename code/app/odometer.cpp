/*
 * odometer.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#include "odometer.h"

#define NUMBER_OF_PULSE_PER_REVOLUTION  1920.0
#define WHEEL_DIAMETER					144.0 // in milimeters
#define WHEEL_SPACING					300.0 // in milimeters
#define DISTANCE_PER_REVOLUTION			(PI * WHEEL_DIAMETER)
#define DISTANCE_PER_PULSE				(DISTANCE_PER_REVOLUTION / NUMBER_OF_PULSE_PER_REVOLUTION)

Odometer::Odometer () {
	period 			= 0.0 ;
	leftEncoder		= NULL ;
	rightEncoder	= NULL ;
}
void Odometer::init ( uint32_t timerFrequency, Encoder & left , Encoder & right ) {
	this->period 		= 1.0 / (double) timerFrequency ;
	this->leftEncoder	= & left ;
	this->rightEncoder	= & right ;

	this->leftEncoder ->setup () ;
	this->rightEncoder->setup () ;

	reset () ;
}
void Odometer::reset ( void ) {
	leftEncoder  ->reset () ;
	rightEncoder ->reset () ;

	position.x 			= 0 ;
	position.y 			= 0 ;
	position.angle 		= 0 ;

	distance.leftWheel 	= 0 ;
	distance.rightWheel = 0 ;
	distance.mean 		= 0 ;

	speed.leftWheel 	= 0 ;
	speed.rightWheel 	= 0 ;
	speed.mean 			= 0 ;
}
Distance & Odometer::getTotalDistance	( void ) {
	// TOTAL DISTANCE
	totalDistance.leftWheel  = DISTANCE_PER_PULSE * leftEncoder ->get () ;
	totalDistance.rightWheel = DISTANCE_PER_PULSE * rightEncoder->get () ;
	totalDistance.mean	     = ( totalDistance.leftWheel + totalDistance.rightWheel ) / 2.0 ;

	return totalDistance ;
}

void Odometer::update ( void ) {

	double delta_left   = DISTANCE_PER_PULSE * leftEncoder  ->getDelta () ;
	double delta_right  = DISTANCE_PER_PULSE * rightEncoder ->getDelta () ;

	// DISTANCE:
	distance.leftWheel  = delta_left ;
	distance.rightWheel = delta_right ;
	distance.mean	    = ( delta_left + delta_right ) / 2.0 ;

	// SPEED
	speed.leftWheel 	= delta_left  / period ;
	speed.rightWheel	= delta_right / period ;
	speed.mean	    	= ( speed.leftWheel + speed.rightWheel ) / 2.0 ;

	// POSITION: Compute position from origin
	computePosition ( distance.mean, delta_left - delta_right ) ;
}
inline void Odometer::computePosition ( double meanDistance, double leftRightDelta ) {
	  double 	curveRadius, curveAngle ;
	  Position 	curveCenter ;

	  if ( leftRightDelta == 0 ) {
	      // Approximate as a strait line if same number of pulse left and right
	      position.x 	+= ( meanDistance * cos ( position.angle ) ) ;
	      position.y 	+= ( meanDistance * sin ( position.angle ) ) ;
	  }
	  else {
		  // Approximate delta position from last update as a curve trajectory
		  curveAngle  	 = leftRightDelta / WHEEL_SPACING ;
	      curveRadius 	 = meanDistance / curveAngle ;

	      // Compute curve center
	      curveCenter.x = position.x - ( curveRadius * sin ( position.angle ) ) ;
	      curveCenter.y = position.y + ( curveRadius * cos ( position.angle ) ) ;

	      // TUBO position
	      position.angle += curveAngle ;
	      position.x = curveCenter.x + ( curveRadius * sin ( position.angle ) ) ;
	      position.y = curveCenter.y - ( curveRadius * cos ( position.angle ) ) ;
	 }
}


