/*
 * pid.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#include "pid.h"
#include "utils.h"

Pid::Pid ( void ) : integralError ( INTEGRAL_WINDOW_SIZE ) {
	 init  () ;
	 reset () ;
 }

void Pid::init ( void ) {
	set 				( PID_KP, PID_KI, PID_KD 	  ) ;
	setBounds 			( OUT_MAX, PROPORTIONAL_RANGE ) ;
}

void Pid::reset ( void ) {
	lastError 		= 0.0 ;
	integralError.reset () ;
}

void Pid::set   			( double kp, double ki, double kd ) 		{ this->kp = kp ; this->ki = ki ; this->kd = kd ; 	}
void Pid::setKp 			( double kp ) 								{ this->kp = kp ; 									}
void Pid::setKi 			( double ki ) 								{ this->ki = ki ; 									}
void Pid::setKd 			( double kd ) 								{ this->kd = kd ; 								  	}
void Pid::setBounds 		( double outMax, double proportionalRange ) {
	this->outMax 			= outMax ;
	this->proportionalRange = proportionalRange ;
}

double Pid::process ( double inError ) {

	// Compute derivative term
	double derivativeError = inError - lastError ;

	// Store to compute derivative term
    lastError = inError ;

	return ( process ( inError, derivativeError ) ) ;
}
double 	Pid::process ( double inError, double derivativeError ) {
	double correction ;

	// Compute integral term
	integralError.set ( inError * ki ) ;

	// Limit integral to reasonable value
	double integralTerm = CONSTRAIN ( integralError.getSum () , -INTEGRAL_LIMIT , INTEGRAL_LIMIT ) ;

	if ( ABS(inError) > proportionalRange ) {
		correction = SIGN(inError) * outMax ;
	}
	else {
		correction = inError * kp + integralTerm + derivativeError * kd ;
	}
	return ( CONSTRAIN ( correction , -outMax , outMax ) ) ;
}

#if 0
// With a better anti-windup for integral term
double Pid::process ( double newError ) {

	// Compute integral term
	double integralTerm = integralError.getSum () + ( newError * ki ) ;

	double correction = ( newError * kp + integralTerm + ( newError - lastError ) * kd ) ;

	if ( correction > outMax ) {
		// Limit integral not to exceed max
		integralError.set ( (int16_t) ( newError * ki - ( correction - outMax ) ) ) ;
		correction = outMax ;
	}
	else if ( correction < -outMax ) {
		// Limit integral not to exceed max
		integralError.set ( (int16_t) newError * ki  + ( - outMax - correction ) ) ;
		correction = - outMax ;
	}

    lastError = newError ;

	return ( CONSTRAIN ( correction , -outMax , outMax ) ) ;
}
#endif



