/*
 * sensor.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#ifndef CODE_APP_SENSORS_SENSOR_H_
#define CODE_APP_SENSORS_SENSOR_H_

#include <string.h>
#include "utils.h"
#include "stdint.h"
#include "bsp.h"
#include "i2c_driver.h"
#include "sliding_window.h"


/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665)              /**< Earth's gravity in m/s^2 */
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                  /**< Gauss to micro-Tesla multiplier */
#define CALIBRATION_WINDOW_MAX_SIZE		  (100)

#define I2C_MULTIPLE_READ 				  0x80


typedef struct {
	union {
		int16_t vector [ 3 ] ;
		struct {
			int16_t x ;
			int16_t y ;
			int16_t z ;
		};
	};
} int16_3D_t ;

typedef struct {
	union {
		double vector [ 3 ] ;
		struct {
			double x ;
			double y ;
			double z ;
		};
		struct {
			double roll    ; // Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90<=roll<=90
			double pitch   ; // Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180<=pitch<=180)
			double heading ; // Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359
		};
		double pressure ;
	};
} double_3D_t ;

class Sensor {

protected:
	I2cDriver *		i2c ;
	bool 			autoRange ;
	int16_3D_t 		min ;
	int16_3D_t 		max ;
	int16_3D_t 		offset ;
	double_3D_t		scaledData ;
	double_3D_t		scaleFactor ;
	uint8_t   		i2cAdress ;
	SlidingWindow	xWindow ;
	SlidingWindow	yWindow ;
	SlidingWindow	zWindow ;

	void    init 		( I2cDriver & i2c ) ;
	uint8_t	readSensor  ( int reg ) ;
	void	readSensor  ( int reg , uint8_t * data , uint8_t length ) ;
	void	writeSensor ( uint8_t reg, uint8_t value ) ;

public:

	int16_3D_t 		rawData ;


	Sensor ( int32_t i2cAdress )
		: xWindow ( CALIBRATION_WINDOW_MAX_SIZE ),
		  yWindow ( CALIBRATION_WINDOW_MAX_SIZE ),
		  zWindow ( CALIBRATION_WINDOW_MAX_SIZE ) {

		this->i2cAdress = i2cAdress ;
		i2c				=  NULL ;
		autoRange 		=  false ;
		min.x 			=  INT16_MAX ;
		min.y 			=  INT16_MAX ;
		min.z 			=  INT16_MAX ;
		max.x 			=  INT16_MIN ;
		max.y 			=  INT16_MIN ;
		max.z 			=  INT16_MIN ;
		offset.x 		=  0 ;
		offset.y 		=  0 ;
		offset.z 		=  0 ;
	}
	virtual ~Sensor () {}

	virtual double_3D_t & read ( void ) = 0 ;

	void enableAutoRange ( bool enabled ) {
		autoRange = enabled ;
	} ;

	int16_3D_t & readRaw ( void ) {
		return rawData ;
	} ;

	double_3D_t & scale ( void ) {
		scaledData.x = (double)( rawData.x - offset.x ) * scaleFactor.x ;
		scaledData.y = (double)( rawData.y - offset.y ) * scaleFactor.y ;
		scaledData.z = (double)( rawData.z - offset.z ) * scaleFactor.z ;
		return scaledData ;
	} ;

	void sensorCalibrateZero ( int16_t x , int16_t y, int16_t z ) {

		xWindow.set( rawData.x ) ;
		yWindow.set( rawData.y ) ;
		zWindow.set( rawData.z ) ;

		offset.x = xWindow.get () - x ;
		offset.y = yWindow.get () - y ;
		offset.z = zWindow.get () - z ;
	} ;
} ;

#endif /* CODE_APP_SENSORS_SENSOR_H_ */

