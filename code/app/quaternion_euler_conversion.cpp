/*
 * quaternion_euler_conversion.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */


#include <math.h>
#include "quaternion_euler_conversion.h"

void toRadians ( const Quaternion & q, EulerAngle & angle ) {

	double q2sqr  = q.q2 * q.q2 ;
	double t0 	 = -2.0 * ( q2sqr       + q.q3 * q.q3 ) + 1.0 ;
	double t1	 = +2.0 * ( q.q1 * q.q2 + q.q0 * q.q3 ) ;
	double t2	 = -2.0 * ( q.q1 * q.q3 - q.q0 * q.q2 ) ;
	double t3	 = +2.0 * ( q.q2 * q.q3 + q.q0 * q.q1 ) ;
	double t4	 = -2.0 * ( q.q1 * q.q1 + q2sqr       ) + 1.0 ;

	t2 = ( t2 >  1.0 ) ?  1.0 : t2 ;
	t2 = ( t2 < -1.0 ) ? -1.0 : t2 ;

	angle.roll  = atan2 ( t3, t4 ) ;
	angle.pitch = asin  ( t2 ) 	 ;
	angle.yaw   = atan2 ( t1, t0 ) ;

	if (isnan ( angle.roll  )) app_error ( NAN_ERROR ) ;
	if (isnan ( angle.pitch )) app_error ( NAN_ERROR ) ;
	if (isnan ( angle.yaw   )) app_error ( NAN_ERROR ) ;
}
void toDegrees ( const Quaternion & q, EulerAngle & angle ) {

	toRadians ( q, angle ) ;

	angle.roll  = ToDeg ( angle.roll  ) ;
	angle.pitch = ToDeg ( angle.pitch ) ;
	angle.yaw   = ToDeg ( angle.yaw   ) ;
}
void toQuaternion ( const EulerAngle & angle, Quaternion & q ) {

	double t0 = cos ( angle.yaw   * 0.5 ) ;
	double t1 = sin ( angle.yaw   * 0.5 ) ;
	double t2 = cos ( angle.roll  * 0.5 ) ;
	double t3 = sin ( angle.roll  * 0.5 ) ;
	double t4 = cos ( angle.pitch * 0.5 ) ;
	double t5 = sin ( angle.pitch * 0.5 ) ;

	q.q0 = t2 * t4 * t0 + t3 * t5 * t1 ;
	q.q1 = t3 * t4 * t0 - t2 * t5 * t1 ;
	q.q2 = t2 * t5 * t0 + t3 * t4 * t1 ;
	q.q3 = t2 * t4 * t1 - t3 * t5 * t0 ;
}

