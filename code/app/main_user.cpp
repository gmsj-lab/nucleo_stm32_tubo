/*
 * main_user.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#include "main_user.h"
#include "main_support.h"

// -----------------------------------------------------------------------------------------------------
// 								MAIN SETUP
// -----------------------------------------------------------------------------------------------------
void mainSetup ()
{
	// mode = TRACKER_WIFI_AND_BLUETOOTH or TRACKER_WIFI_AND_IHM_BLUETOOTH
	initTracker (TRACKER_WIFI_AND_IHM_BLUETOOTH) ;

	BSP.init 	 		() ;
	setCallbacks 		() ;
	retrieveFromEeprom 	() ;
	initComponents		() ;
	pidCalibration		() ;
	BSP.timers.init 	() ;
	bip					() ;
}
// -----------------------------------------------------------------------------------------------------
// 								MAIN LOOP
// -----------------------------------------------------------------------------------------------------
void mainLoop ( void ) {
	PROFILING_BEGIN( PROFILING_MAIN_PROCESSING ) ;
	if ( sendTelemetry ) {
		ihm.process () ;
		sendTelemetry = false ;
	}
	PROFILING_END ;
}
// -----------------------------------------------------------------------------------------------------
// 								BSP TIMER CALLBACKS
// -----------------------------------------------------------------------------------------------------
void _2_5_ms_processing ( void ) {

}
// 200 Hz or 400 Hz Processing, depending on USE_HIGH_PROCESS_FREQUENCY --------------------------------
void _5_ms_processing ( void ) {
	CHECK_PERIODICITY ;
	CHRONO_START ;
	PROFILING_BEGIN( PROFILING_5_MS_PROCESSING ) ;

	// Update delta distance, speed and compute position
	odometer.update () ;

	readSensors () ;

	processMode () ;

	// Give order to motors
	leftMotor .setThrottle ( leftMotorCmd  ) ;
	rightMotor.setThrottle ( rightMotorCmd ) ;

	PROFILING_END ;
	CHRONO_STOP ;
}

// 50 Hz Processing -----------------------------------------------------------------------------------
void _20_ms_processing  ( void ) {
	PROFILING_BEGIN( PROFILING_20_MS_PROCESSING ) ;

	// Send over wifi
	sendTelemetry = true ;

#if USE_RADIO_CONTROL == true
	// Read Radio commands
	radioControl.read () ;
	velocityCommand = radioControl.getSpeed     () ; // Command given in millimeters / seconds ( +-1000 )
	steeringCommand = radioControl.getDirection () ; // The target orientation is given in degrees / seconds

#endif

	PROFILING_END ;
}

// 25 Hz Processing -----------------------------------------------------------------------------------
#if 0
void _40_ms_processing ( void ) {
	PROFILING_BEGIN( PROFILING_40_MS_PROCESSING ) ;

	PROFILING_END ;
}
#endif
// 10 Hz Processing -----------------------------------------------------------------------------------
void _100_ms_processing  ( void ) {
	PROFILING_BEGIN( PROFILING_100_MS_PROCESSING ) ;

	// Update delta distance, speed and compute position
	//odometer.update () ;

	BSP.ledM_A   .off () ;
	BSP.ledWifi  .off () ;
	BSP.ledErreur.off () ;
	BSP.buzzerA  .off () ;
	BSP.buzzerB  .off () ;
	PROFILING_END ;
}
// 1 Hz Processing -----------------------------------------------------------------------------------
void _1000_ms_processing ( void ) {
	BSP.ledM_A 	 .set ( *processMode == &defaultOperationMode 		) ;
	BSP.ledWifi	 .set ( *processMode == &positionControlMode		) ;
	BSP.ledErreur.set ( *processMode == &sensorZeroCalibrationMode 	) ;

	ihm.batteryLevel.set ( BSP.tensionBatterie.read () ) ;
	PROFILING_SHOW ;
}

// readSensors -----------------------------------------------------------------------------------------------------
inline void readSensorsBEURK ( void ) {
	gyro_event      = gyrometer    .read () ;
	accelero_event	= accelerometer.read () ;
	magneto_event 	= magnetometer .read () ;

#if USE_GRAVIMETER == true
	gravimeter_event 	 = gravimeter.read () ;
	accelero_event.pitch = ( accelero_event.pitch + gravimeter_event ) / 2 ;
#endif

	// Perform sensor fusion
	ahrs.update ( attitude, gyro_event, accelero_event, magneto_event ) ;

#if USE_COMPLEMENTARY_FILTER == true
	// Complementary filter method
	//attitude.pitch = G_FORCE_TO_DEGREES( complementaryFilter.update ( accelero_event.pitch , gyro_event.pitch ) ) ;
	attitude.pitch =  complementaryFilter.update ( accelero_event.pitch , gyro_event.pitch ) ;
#endif

	// Apply attitude correction
	calibrate () ;
}
inline void readSensors ( void ) {
	gyro_event      = gyrometer    .read () ;
	accelero_event	= accelerometer.read () ;
	magneto_event 	= magnetometer .read () ;

#if USE_GRAVIMETER == true
	gravimeter_event 	 = gravimeter.read () ;
	accelero_event.pitch = ( accelero_event.pitch + gravimeter_event ) / 2 ;
#endif

#if USE_COMPLEMENTARY_FILTER == true
	// Complementary filter method
	//attitude.pitch = G_FORCE_TO_DEGREES( complementaryFilter.update ( accelero_event.pitch , gyro_event.pitch ) ) ;
	attitude.pitch =  complementaryFilter.update ( accelero_event.pitch , gyro_event.pitch ) ;
#else
	// Perform sensor fusion
	ahrs.update ( attitude, gyro_event, accelero_event, magneto_event ) ;
#endif

	// Filter gyro angular speed
	angularSpeedFilter.set( gyro_event.pitch ) ;

	// Apply attitude correction
	calibrate () ;
}

void init_positionControlMode ( void ) {

	// Get current attitude
	readSensors () ;

	// Set default target direction to current orientation
	targetDirection = store.offset.heading ;

	// Set target position as current position
	targetPosition = odometer.getTotalDistance ().mean ;

    // Go to position control mode
	processMode = positionControlMode ;
}

void positionControlModeBEURK ( void ) {
	double headingCorrection ;

	if ( steeringCommand != 0 ) {
		targetDirection = attitude.yaw - ToRad ( steeringCommand / (double) _50_HZ_TIMER ) ;
	}
	if ( velocityCommand != 0 ) {
		targetPosition = odometer.getTotalDistance ().mean + velocityCommand ;
	}

	// PIDs
	requestedTilt     = positionPid   .process ( MM_TO_METER( odometer.getTotalDistance ().mean - targetPosition ) ) ;
	tiltCorrection    = tiltPid       .process ( attitude.pitch + requestedTilt ) ;
	headingCorrection = orientationPid.process ( attitude.yaw - targetDirection  ) ;

	// Filter (EWMA) tilt and orientation pid outputs to provide nice looking torque commands
	leftTorqueFilter .set ( tiltCorrection + headingCorrection ) ;
	rightTorqueFilter.set ( tiltCorrection - headingCorrection ) ;

	// Get the filtered command
	double leftTorque  = leftTorqueFilter .get () ;
	double rightTorque = rightTorqueFilter.get () ;

	leftMotorCmd  = ROUND_2_INT( leftTorque  ) ;
	rightMotorCmd = ROUND_2_INT( rightTorque ) ;
}

void positionControlMode ( void ) {

	velocityFilter.set( odometer.getSpeed().mean ) ;

	if ( steeringCommand != 0 ) {
		targetDirection = attitude.yaw - ToRad ( steeringCommand / (double) _50_HZ_TIMER ) ;
	}
	if ( velocityCommand != 0 ) {
		targetPosition = odometer.getTotalDistance ().mean + velocityCommand ;
	}

	double velocityError  = MM_TO_METER( velocityFilter.get() - velocityCommand	) ;
	double positionError  = MM_TO_METER( odometer.getTotalDistance ().mean - targetPosition ) ;
	double directionError = attitude.yaw - targetDirection ;

	lastRequestedTilt = requestedTilt ;
	// POSITION PID
	requestedTilt = positionPid.process ( positionError, velocityError ) ;
	double requestedTiltVelocity = requestedTilt - lastRequestedTilt ;

	double tiltError         = attitude.pitch + requestedTilt ;
	double tiltVelocityError = angularSpeedFilter.get () + requestedTiltVelocity ;

	// TILT PID
	tiltCorrection = tiltPid.process ( tiltError, tiltVelocityError ) ;

	// ORIENTATION PID
	double headingCorrection = orientationPid.process ( directionError ) ;

	// Filter (EWMA) tilt and orientation pid outputs to provide nice looking torque commands
	leftTorqueFilter .set ( tiltCorrection + headingCorrection ) ;
	rightTorqueFilter.set ( tiltCorrection - headingCorrection ) ;

	// Get the filtered command
	double leftTorque  = leftTorqueFilter .get () ;
	double rightTorque = rightTorqueFilter.get () ;

	leftMotorCmd  = ROUND_2_INT( leftTorque  ) ;
	rightMotorCmd = ROUND_2_INT( rightTorque ) ;
}

