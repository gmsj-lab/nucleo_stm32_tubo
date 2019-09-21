/*
 * gyrometer.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_SENSORS_GYROMETER_H_
#define CODE_APP_SENSORS_GYROMETER_H_

#include "L3GD20.h"
#include "sensor.h"
#include "utils.h"

class Gyrometer : public Sensor
{
private:
	gyroRange_t range ;
	uint8_t		rate  ;

	void 			setScaleFactor ( void ) ;
	void			defaultCalibration ( void ) ;
public:
					Gyrometer 		() ;
	virtual 		~Gyrometer 		() {} ;

	void 		 	init			( I2cDriver & i2c , gyroRange_t range = GYRO_RANGE_250DPS ) ;
	void 			calibrateZero	( void ) ;
	double_3D_t & 	read			( void ) ;
};

#endif /* CODE_APP_SENSORS_GYROMETER_H_ */
