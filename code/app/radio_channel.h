/*
 * radio_channel.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_APP_RADIO_CHANNEL_H_
#define CODE_APP_RADIO_CHANNEL_H_

#include "utils.h"
#include "pwm_in.h"
#include "bsp.h"

#define NEUTRAL_HALF_RANGE	  0.02 // In percent of the max value

class RadioChannelCalibration {
public:
	uint32_t 	min ;
	uint32_t 	zero ;
	uint32_t 	max ;
} ;

class RadioChannel {

private:
	PwmIn * 				pwm ;
	uint32_t 				maxOutput ;
	RadioChannelCalibration calibration ;
	int 		 			startCounter ; 		// Used to detect radio : accept reading only after 10 consecutive readings at zero

public:
	RadioChannel () ;
	virtual ~RadioChannel () ;

	void 						init 			( PwmIn & pwm, RadioChannelCalibration & calibration, uint32_t maxOutput ) ;
	void 						resetRange		( void ) ; 	// Reset calibration parameters
	RadioChannelCalibration &	calibrateZero	( void ) ;
	RadioChannelCalibration &	calibrateRange	( void ) ;
	int32_t						get				( void ) ;

};

#endif /* CODE_APP_RADIO_CHANNEL_H_ */
