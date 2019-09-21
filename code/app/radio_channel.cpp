/*
 * radio_channel.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "radio_channel.h"

RadioChannel::RadioChannel () {
	pwm	 		  = NULL ;
	maxOutput 	  = 0 ;
	startCounter  = 0 ;
}

RadioChannel::~RadioChannel () {
}

void RadioChannel::init ( PwmIn & pwm, RadioChannelCalibration & calibration, uint32_t maxOutput ) {
	this->pwm  		  	= & pwm ;
	this->calibration 	= calibration ;
	this->maxOutput 	= maxOutput ;
	this->startCounter  = 0 ;
}

void RadioChannel::resetRange ( void ) {
	calibration.max  = 0 ;
	calibration.min  = UINT32_MAX ;
}

RadioChannelCalibration & RadioChannel::calibrateZero ( void ) {

	this->pwm->start() ;
	calibration.zero = pwm->getRaw () ;
	return calibration ;
}

RadioChannelCalibration & RadioChannel::calibrateRange ( void ) {
	uint32_t rawValue ;

	this->pwm->start() ;

	rawValue = pwm->getRaw () ;

	if ( rawValue >= calibration.max ) {
		calibration.max = rawValue ;
		BSP.ledM_A.on () ;
	}
	if ( rawValue <= calibration.min ) {
		calibration.min = rawValue ;
		BSP.ledErreur.on () ;
	}
	return calibration ;
}

int32_t RadioChannel::get ( void ) {
	int32_t value ;
	uint32_t rawValue ;

	this->pwm->start () ;

	rawValue = pwm->getRaw () ;

	if ( rawValue > ( calibration.zero + (NEUTRAL_HALF_RANGE * calibration.max) ) ) {
		// Forward : Positive values
		value = _MAP ( rawValue , calibration.zero , calibration.max , 0, maxOutput ) ;
	}
	else if ( rawValue < ( calibration.zero - (NEUTRAL_HALF_RANGE * calibration.max) ) ) {
		// Backward : Negative values
		value = _MAP ( rawValue , calibration.min , calibration.zero , 0, maxOutput ) - maxOutput ;
	}
	else {
		// Neutral
		value = 0 ;
	}
	// Make sure radio is on and working : wait to see 10 consecutive zero values before taking into account readings
	if ( startCounter < 10 ) {
		if ( value == 0 ) {
			startCounter ++ ;
		}
		else {
			startCounter = 0 ;
			value = 0 ;
		}
	}
	return ( value ) ;
}


