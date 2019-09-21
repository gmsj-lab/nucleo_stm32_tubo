/*
 * ahrs.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 *
 *  Implementation of Madgwick's IMU and AHRS algorithms.
 *  See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 */

#ifndef CODE_APP_AHRS_H_
#define CODE_APP_AHRS_H_

#include "quaternion_euler_conversion.h"
#include "sensor.h"

class Ahrs {
private:
	double 				samplePeriod ;
	volatile double 	beta ;				// algorithm gain
	volatile double 	q0, q1, q2, q3 ;	// quaternion of sensor frame relative to auxiliary frame

	float 	invSqrt 		( float  x ) ;
	double 	invSqrt 		( double x ) ;
	void	getEulerAngles 	( EulerAngle & angle ) ;
	// Gyro in rad/s, Accelro and Magneto in any calibrated unit
	void	update    		( EulerAngle & angle, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz ) ;
	void	updateIMU    	( EulerAngle & angle, double gx, double gy, double gz, double ax, double ay, double az ) ;

public:
			Ahrs 			( void ) ;
	void 	init			( double sampleFrequency ) { setFrequency ( sampleFrequency ) ; }  	// sample frequence in Hz
	void 	setFrequency	( double sampleFrequency ) ;										// sample frequency in Hz
	void 	setBeta 		( double beta ) ;

	// Gyro in rad/s, Accelro and Magneto in any calibrated unit
	void	update    		( EulerAngle & angle, const double_3D_t & g, const double_3D_t & a, const double_3D_t & m ) ;
	void	updateIMU 		( EulerAngle & angle, const double_3D_t & g, const double_3D_t & a ) ;
} ;

#endif /* CODE_APP_AHRS_H_ */
