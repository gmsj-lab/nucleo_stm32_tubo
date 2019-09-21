/*
 * pid.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_PID_H_
#define CODE_APP_PID_H_


#include "stm32f7xx_hal.h"
#include "sliding_window_double.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PID_KP			   	   		0.0
#define PID_KI			   	  		0.0
#define PID_KD			   	   		0.0
#define OUT_MAX			    	 1000.0
#define PROPORTIONAL_RANGE		1000000.0	// Large number, so all inputs are processed by PID
#define INTEGRAL_WINDOW_SIZE		100
#define INTEGRAL_LIMIT				100.0

class Pid {

	private:
		double 				kp ;
		double 				ki ;
		double 				kd ;
		double 				outMax ;
		double 				lastError ;
		double				proportionalRange ;
		SlidingWindowDouble integralError ;

	public:
		Pid () ;
		virtual ~Pid () {}

		void 	init 				( void ) ;
		void 	reset 	  			( void ) ;
		void 	set 				( double kp , double ki , double kd ) ;
		void 	setKp				( double kp ) ;
		void 	setKi				( double ki ) ;
		void 	setKd				( double kd ) ;
		void 	setBounds 			( double outMax,  double proportionalRange = PROPORTIONAL_RANGE ) ;
		double 	process 			( double error ) ;
		double 	process 			( double error, double derivativeError ) ;
} ;
#ifdef __cplusplus
}
#endif

#endif /* CODE_APP_PID_H_ */
