/*
 * quaternion_euler_conversion.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_QUATERNION_EULER_CONVERSION_H_
#define CODE_APP_QUATERNION_EULER_CONVERSION_H_


#include "utils.h"
#include "math.h"

struct Quaternion {
	volatile double q0 ;
	volatile double q1 ;
	volatile double q2 ;
	volatile double q3 ;
} ;

struct EulerAngle {
public:
	volatile double roll ;
	volatile double pitch ;
	volatile double yaw ;
} ;

void toRadians 	  ( const Quaternion & q, EulerAngle & angle ) ;
void toDegrees 	  ( const Quaternion & q, EulerAngle & angle ) ;
void toQuaternion ( const EulerAngle & angle, Quaternion & q ) ;

#endif /* CODE_APP_QUATERNION_EULER_CONVERSION_H_ */
