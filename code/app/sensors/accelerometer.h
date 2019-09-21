/*
 * accelrometer.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef APP_SENSORS_ACCELROMETER_H_
#define APP_SENSORS_ACCELROMETER_H_

#include <string.h>
#include "LSM303.h"
#include "sensor.h"

class Accelerometer : public Sensor
{
private:
	uint8_t			rate  ;

	void 			setScaleFactor ( void ) ;
	void			defaultCalibration ( void ) ;

public:
					Accelerometer 	() ;
	virtual 		~Accelerometer 	() {} ;

	void 		  	init			( I2cDriver & i2c ) ;
	void 			calibrateZero	( void ) ;
	double_3D_t & 	read 			( void ) ;
} ;

#endif /* APP_SENSORS_ACCELROMETER_H_ */
