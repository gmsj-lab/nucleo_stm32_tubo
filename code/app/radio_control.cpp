/*
 * radioControl.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "radio_control.h"

RadioControl::RadioControl ( double radioEwmaWeight ) :
	steeringChannelFilter (radioEwmaWeight),
	speedChannelFilter (radioEwmaWeight) {
}

RadioControl::~RadioControl () {
}

void RadioControl::init	( PwmIn & steering , PwmIn &  speed, RadioCalibration & radioCalibration ) {

	steeringChannel.init ( steering , radioCalibration.direction, ROTATION_SPEED_MAX_OUTPUT ) ;
	speedChannel   .init ( speed	, radioCalibration.speed	, SPEED_MAX_OUTPUT ) ;
}

void RadioControl::read  ( void ) {
	speedChannelFilter   .set ( speedChannel   .get () ) ;
	steeringChannelFilter.set ( steeringChannel.get () ) ;
}

void RadioControl::resetRange ( void ) {
	steeringChannel.resetRange () ;
	speedChannel   .resetRange () ;
}

RadioCalibration RadioControl::calibrateZero ( void ) {
	RadioCalibration calibration ;
	calibration.direction 	= steeringChannel.calibrateZero () ;
	calibration.speed 		= speedChannel   .calibrateZero () ;

	return calibration ;
}

RadioCalibration RadioControl::calibrateRange ( void ) {
	RadioCalibration calibration ;

	calibration.direction 	= steeringChannel.calibrateRange () ;
	calibration.speed 		= speedChannel   .calibrateRange () ;

	return calibration ;
}


