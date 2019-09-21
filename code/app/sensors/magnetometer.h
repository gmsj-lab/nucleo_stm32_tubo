/*
 * magnetometer.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_APP_SENSORS_MAGNETOMETER_H_
#define CODE_APP_SENSORS_MAGNETOMETER_H_

#include <string.h>
#include "LSM303.h"
#include "sensor.h"
#include "utils.h"

class Magnetometer : public Sensor
{
private:
	lsm303MagRate 	rate ;
	lsm303MagGain 	gain ;
	uint8_t 		enableTemperatureSensor ;
public:
	Magnetometer () ;

	void 			init 				( I2cDriver & i2c , const int16_3D_t &calibrationOffset, const double_3D_t &calibrationScale, bool enableTemperatureSensor = false ) ;
	void 			enableTemperature	( bool enableTemperatureSensor = true ) ;
	void 			setMagGain			( lsm303MagGain gain ) ;
	void 			setMagRate			( lsm303MagRate rate ) ;
	double_3D_t & 	read 				( void ) ;
	void 			calibrateRange		( void ) ;
	void			defaultCalibration	( const int16_3D_t & calibrationOffset, const double_3D_t & calibrationScale ) ;
	int16_3D_t 		getCalibrationOffset( void ) { return offset ; 		} ;
	double_3D_t  	getCalibrationScale ( void ) { return scaleFactor ; } ;
};

#endif /* CODE_APP_SENSORS_MAGNETOMETER_H_ */
