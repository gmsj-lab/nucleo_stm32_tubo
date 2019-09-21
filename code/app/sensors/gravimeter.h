/*
 * gravimeter.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef Gravimeter_H_
#define Gravimeter_H_

#include "utils.h"
#include "adc_driver.h"
#include "sliding_window.h"

#define FILTER_SIZE				3											// Number of read for filtering
#define CALIB_WINDOW_SIZE	  100
#ifdef arduino
#define ZERO_GRAVITY_VALUE		((int) ( ( UINT10_MAX ) * ( 1.5 / 5.0 ) ))	// 1.5 Volts for 0 g, range of approx 5 v (vMax) for Arduino pin
#define GRAVITY_VALUE			((int) ( ( UINT10_MAX ) * ( 1.8 / 5.0 ) ))	// 1.8 Volts for + 1 g
#define NEGATIVE_GRAVITY_VALUE	((int) ( ( UINT10_MAX ) * ( 1.2 / 5.0 ) ))	// 1.2 Volts for - 1 g
#endif

#define NEGATIVE_GRAVITY_VALUE	 1800	// Should be 1023 if range is +/- 2 g (1/4 of range)
#define ZERO_GRAVITY_VALUE		 2300
#define GRAVITY_VALUE			 2800	// Should be 3076 if range is +/- 2 g (3/4 of range)
#define SENSITIVITY				(double)((GRAVITY_VALUE-NEGATIVE_GRAVITY_VALUE)/2.0)  // unit:  delta ADC value / delta g

class Gravimeter {

private:

	SlidingWindow 	filter ;
	SlidingWindow 	calibWindow ;
	Adc *			adc ;
	uint32_t 		zeroRef ;
	uint32_t 		low ;
	uint32_t 		high ;

	double readSensor 	 	 ( void ) ;

public:

	Gravimeter	 			 ( void ) ;
	void calibrateZero  	 ( void ) ;
	void startRangeCalibrate ( void ) ;
	void doAutoCalibration 	 ( void ) ;
	void doRangeCalibrate 	 ( void ) ;

	void calibrate 	 		 ( void ) ;

	void 	 init			 ( Adc & adc ) ;
	double	 read			 ( void ) ;
	#if 0
	uint32_t readGforce		 ( void ) ;
	uint32_t readDegrees	 ( void ) ;
	#endif
};
#endif /* Gravimeter_H_ */
