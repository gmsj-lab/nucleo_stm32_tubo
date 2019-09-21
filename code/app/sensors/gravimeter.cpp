/*
 * gravimeter.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "gravimeter.h"

//----------------------------------------------------------------------------
// Gravimeter class
//----------------------------------------------------------------------------
Gravimeter::Gravimeter () : filter ( FILTER_SIZE ) , calibWindow ( CALIB_WINDOW_SIZE) {
	this->adc 			= NULL ;

	// default : perform automatic calibration
	doAutoCalibration () ;
}
void Gravimeter::init ( Adc & adc ) {
	this->adc = & adc ;
}
void Gravimeter::calibrateZero () {
	calibWindow.set ( readSensor () ) ;
	zeroRef = calibWindow.get () ;
}
void Gravimeter::doAutoCalibration () {
	low	  	= NEGATIVE_GRAVITY_VALUE ;
	high 	= GRAVITY_VALUE	;
	zeroRef = ZERO_GRAVITY_VALUE ;
}

void Gravimeter::startRangeCalibrate () {
	low	 = UINT10_MAX ;
	high = 0 ;
}

void Gravimeter::doRangeCalibrate () {
	uint32_t voltValue = readSensor () ;

	if ( voltValue < low  ) low  = voltValue ;
	if ( voltValue > high )	high = voltValue ;
	zeroRef =  ( high - low ) / 2.0 ;
}

double Gravimeter::readSensor () {
	// Read a sample from sensor
	filter.set ( adc->read () ) ;

	// provide the filtered adcValue
	return ( filter.get () ) ;
}

double Gravimeter::read () {
	return ( ToDeg ( asin ( ( readSensor () - zeroRef ) / SENSITIVITY ) ) ) ;
}
#if 0
uint32_t Gravimeter::readGforce () {
	double gValue ;
	uint32_t voltValue = readSensor () ;

	if ( voltValue < low  ) voltValue = low ;
	if ( voltValue > high )	voltValue = high ;

	gValue = ( voltValue > zeroRef ) ? _MAP ( voltValue , zeroRef , high    ,  0.0 , 1.0  )
									 : _MAP ( voltValue , low     , zeroRef , -1.0 , 0.0  ) ;
	return ( gValue ) ;
}

uint32_t Gravimeter::readDegrees () {
	return (uint32_t) ToDeg ( acos ( readGforce () ) ) ;
}
#endif


