/*
 * barometer.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_SENSORS_BAROMETER_H_
#define CODE_APP_SENSORS_BAROMETER_H_

#include <string.h>
#include "stdint.h"
#include "BMP180.h"
#include "sensor.h"
#include "utils.h"

class Barometer : public Sensor
{
private:
	bmp180_calib_data 		  calib_coeffs ;   // Last read accelerometer data will be available here
	uint8_t           		  mode ;

	void read16				  	( uint8_t reg, uint16_t *value ) ;
	void readS16 			  	( uint8_t reg, int16_t  *value ) ;
	void readCoefficients 	  	( void ) ;
	void readRawTemperature   	( int32_t *temperature ) ;
	void readRawPressure 	  	( int32_t *pressure ) ;

public:
	Barometer 				  	() ;

	void  init 				  	( I2cDriver & i2c , bmp180_mode_t mode = BMP180_MODE_ULTRAHIGHRES ) ;
	double_3D_t & read 		  	( void ) ;

	double getTemperature 	  	( void ) ;
	double getPressure		  	( void ) ;
	double pressureToAltitude  	( double seaLvel, double atmospheric ) ;
	double seaLevelForAltitude 	( double altitude, double atmospheric ) ;

private:
	int32_t computeB5		  ( int32_t ut ) ;
} ;

#endif /* CODE_APP_SENSORS_BAROMETER_H_ */
