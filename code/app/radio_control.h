/*
 * radioControl.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_APP_RADIO_CONTROL_H_
#define CODE_APP_RADIO_CONTROL_H_


#include "utils.h"
#include "pwm_in.h"
#include "radio_channel.h"
#include "ewma.h"

#define ROTATION_SPEED_MAX_OUTPUT	 500	// degrees per seconds 		 Range : -ROTATION_SPEED_MAX_OUTPUT to +ROTATION_SPEED_MAX_OUTPUT
#define SPEED_MAX_OUTPUT			1000	// In millimeters / seconds. Range : -SPEED_MAX_OUTPUT 			to +SPEED_MAX_OUTPUT

class RadioCalibration {
public:
	RadioChannelCalibration speed 	  ;
	RadioChannelCalibration direction ;
} ;

class RadioControl {

private:

	RadioChannel steeringChannel ;
	RadioChannel speedChannel    ;
	Ewma		 steeringChannelFilter ;
	Ewma		 speedChannelFilter ;

public:
	RadioControl 						( double radioEwmaWeight ) ;
	virtual ~RadioControl () ;

	void 				init			( PwmIn & steering , PwmIn & speed, RadioCalibration & radioCalibration ) ;
	void 				read			( void ) ;  // Read raw values and feed ewma filters
	void 				resetRange		( void ) ; 	// Reset calibration parameters
	RadioCalibration	calibrateZero	( void ) ;
	RadioCalibration 	calibrateRange	( void ) ;
	inline int32_t 		getSpeed 		( void ) { return speedChannelFilter   .get () ; }
	inline int32_t 		getDirection 	( void ) { return steeringChannelFilter.get () ; }
} ;
#endif /* CODE_APP_RADIO_CONTROL_H_ */
