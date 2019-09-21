/*
 * ihm.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "ihm.h"

#include <math.h>
#include <iostream>
#include <string>

#define BT_ATITUDE_START_OF_MSG			'@'
#define BT_ATITUDE_SEPARATOR			'#'

#define BT_CALIBRATION_START_OF_MSG		"Raw:"
#define BT_CALIBRATION_SEPARATOR		','

#define END_OF_MSG		'\n'
#define MAX_BT_ATITUDE_MSG_LENGTH		(( 4 * 2 ) + ( 3 * 20 ))
#define MAX_BT_RAW_SENSOR_MSG_LENGTH	(( 8 + 5 ) + ( 9 * 20 ))
#define DECIMAL							10
extern TargetTracker tracker ;

Ihm::Ihm ( void ) :
	blueLed				( "blue"			, "LEDS" 	, blueLedIhmCallback 				) ,
	greenLed			( "green"			, "LEDS" 	, greenLedIhmCallback 				) ,
	redLed				( "red"				, "LEDS" 	, redLedIhmCallback 				) ,
	m_aLed				( "m/a"				, "LEDS" 	, m_aLedIhmCallback 				) ,
	wifiLed				( "wifi"			, "LEDS" 	, wifiLedIhmCallback 				) ,
	erreurLed			( "erreur"			, "LEDS" 	, erreurLedIhmCallback 				) ,
	aBuzzer				( "A"				, "BUZZERS" , aBuzzerIhmCallback				) ,
	bBuzzer				( "B"				, "BUZZERS" , bBuzzerIhmCallback				) ,
	nucleoUserButton	( "Nucleo"			, "BUTTONS" , nucleoUserButtonIhmCallback		) ,
	userButton			( "User"			, "BUTTONS" , frontPanelUserButtonIhmCallback	) ,
	resetButton			( "Reset"			, "BUTTONS" , frontPanelResetButtonIhmCallback	) ,
	getTempAndAlt		( "getTempAndAlt"	, "SENSORS"  									) ,
	altitude    		( "altitude"		, "SENSORS" 									) ,
	temperature 		( "temperature"		, "SENSORS" 									) ,
	getSensors 			( "getSensors"		, "SENSORS" 									) ,
	calibration 		( "calibration"		, "SENSORS" , calibrationIhmCallback			) ,
#if USE_RAW_SENSOR_DISPLAY == true
	gyroX  				( "GyroX" 			, "SENSORS" 									) ,
	gyroY  				( "GyroY" 			, "SENSORS" 									) ,
	gyroZ  				( "GyroZ" 			, "SENSORS" 									) ,
	accelX  			( "AccelX" 			, "SENSORS" 									) ,
	accelY  			( "AccelY" 			, "SENSORS" 									) ,
	accelZ  			( "AccelZ" 			, "SENSORS" 									) ,
	magnetoX  			( "MagnetoX" 		, "SENSORS" 									) ,
	magnetoY  			( "MagnetoY" 		, "SENSORS" 									) ,
	magnetoZ  			( "MagnetoZ" 		, "SENSORS" 									) ,
#else
	gyroY  				( "GyroY" 			, "SENSORS" 									) ,
#endif
	pitch				( "Pitch" 			, "SENSORS" 									) ,
	roll				( "Roll" 			, "SENSORS" 									) ,
	yaw				  	( "Yaw" 			, "SENSORS" 									) ,
	gravity			 	( "Gravity" 		, "SENSORS" 									) ,
	getOdometry			( "getOdometry"		, "ODOMETRY" 									) ,
	resetOdometry		( "resetOdometry"	, "ODOMETRY" , resetOdometryCallback			) ,
	distanceLeft 		( "distanceLeft"	, "ODOMETRY" 									) ,
	distanceRight 		( "distanceRight"	, "ODOMETRY" 									) ,
	distanceTotal 		( "distanceTotal"	, "ODOMETRY" 									) ,
	speedLeft			( "speedLeft"		, "ODOMETRY" 									) ,
	speedRight			( "speedRight"		, "ODOMETRY" 									) ,
	targetPosition		( "targetPosition"	, "ODOMETRY" , targetPositionCallback			) ,
	target_Direction 	( "targetDirection"	, "ODOMETRY" , targetDirectionCallback			) ,
	leftMotor    		( "leftMotorCmd"	, "MOTORS" 	 , leftMotorCmdCallback	 			) ,
	rightMotor    		( "rightMotorCmd"	, "MOTORS" 	 , rightMotorCmdCallback	 		) ,
	tiltKp		    	( "tiltKp"			, "PID" 	 , tiltPidKpCallback			 	) ,
	tiltKi		    	( "tiltKi"			, "PID" 	 , tiltPidKiCallback			 	) ,
	tiltKd		   		( "tiltKd"			, "PID" 	 , tiltPidKdCallback				) ,
	orientKp	 		( "orientKp"		, "PID" 	 , orientPidKpCallback		 		) ,
	orientKi	 		( "orientKi"		, "PID" 	 , orientPidKiCallback		 		) ,
	orientKd	  		( "orientKd"		, "PID" 	 , orientPidKdCallback		 		) ,
	velocityKp	 		( "velocityKp"		, "PID" 	 , velocityKpCallback		 		) ,
	positionKp	 		( "posKp"			, "PID" 	 , positionPidKpCallback	 		) ,
	positionKi	 		( "posKi"			, "PID" 	 , positionPidKiCallback	 		) ,
	positionKd	  		( "posKd"			, "PID" 	 , positionPidKdCallback			) ,
	polebot_error 		( "Error"			, "STATUS" 										) ,
	operationMode		( "Mode"			, "CMD" 	 , operationModeCallback			) ,
	batteryLevel		( "Battery"			, "STATUS"										) ,
	timerPeriod			( "freq"			, "STATUS"										) ,
	processTime			( "processTime"		, "STATUS"										) ,
	positionTilt		( "posTilt"			, "PID"											) ,
	storeEeprom			( "storeEeprom"		, "CMD"		 , storeEepromCallback				) ,
	pitchOffset			( "pitchOffset"		, "PID"		 									) ,
	radio_zero			( "radio_zero"		, "RADIO"	 , radioCalibrateZeroCallback	 	) ,
	radio_range			( "radio_range"		, "RADIO"	 , radioCalibrateRangeCallback 		) ,
	radio_speed			( "radio_speed"		, "RADIO"		 								) ,
	radio_direction		( "radio_direction"	, "RADIO"		 								) ,
	beta 				( "beta"			, "AHRS"	 , betaCallback	 					)

{
	timerValue 			= 0 ;
	imuOnUsart			= false ;
}

Ihm::~Ihm () {
}

void Ihm::init ( int usartForIMU ) {

	polebot_error	.setReadOnly ( true ) ;
	batteryLevel 	.setReadOnly ( true ) ;
	radio_speed	 	.setReadOnly ( true ) ;
	radio_direction	.setReadOnly ( true ) ;

	getSensors.set  ( false ) ;
	getOdometry.set ( false ) ;
	calibration.set ( false ) ;

	if ( usartForIMU != -1 ) {
		imuOnUsart = true ;
		usartDriver.init ( usartForIMU ) ;
	}
}
extern Store	 		store ;
extern EulerAngle 		attitude ;
extern int16_t 			leftMotorCmd ;
extern int16_t			rightMotorCmd ;
extern uint32_t 		timerPeriodicity ;
extern uint32_t 		readSensorTime ;
#if USE_RAW_SENSOR_DISPLAY == true
extern double_3D_t		accelero_event ;
extern double_3D_t		magneto_event ;
#endif
extern float      		gravimeter_event ;
extern Accelerometer 	accelerometer ;
extern Gyrometer 		gyrometer ;
extern Magnetometer 	magnetometer ;
extern double		 	requestedTilt ;
extern Ewma 			angularSpeedFilter ;

extern int32_t 			velocityCommand ;
extern int32_t			steeringCommand ;

void Ihm::processOrientation ( void ) {

	if ( getSensors.get () ) {

#if USE_RAW_SENSOR_DISPLAY == true
		gyroX	  	.set ( gyro_event.x 			) ;
		gyroY	  	.set ( gyro_event.y     		) ;
		gyroZ	  	.set ( gyro_event.z 			) ;

		accelX	  	.set ( accelero_event.x 		) ;
		accelY	  	.set ( accelero_event.y 		) ;
		accelZ	  	.set ( accelero_event.z 		) ;

		magnetoX  	.set ( magneto_event.x  		) ;
		magnetoY  	.set ( magneto_event.y  		) ;
		magnetoZ  	.set ( magneto_event.z  		) ;
#else
		//gyroY	  	.set ( gyro_event.y     		) ;
		gyroY	  	.set ( angularSpeedFilter.get ()) ;

#endif
		pitch	  	.set ( ToDeg ( attitude.pitch - store.offset.pitch   ) ) ;
		roll	  	.set ( ToDeg ( attitude.roll  - store.offset.roll    ) ) ;
		yaw		  	.set ( ToDeg ( attitude.yaw   - store.offset.heading ) ) ;

		gravity	  	.set ( gravimeter_event ) ;

		positionTilt.set ( ToDeg (requestedTilt ) ) ;

		leftMotor 	.set ( leftMotorCmd  ) ;
		rightMotor	.set ( rightMotorCmd ) ;
	}
}
void Ihm::processTempAndAlt ( void ) {

	if ( getTempAndAlt.get ()  == true ) {
		temperature .set ( readTemperature () ) ;
		altitude	.set ( readAltitude    () ) ;
		// read just once upon request by user: reset
		getTempAndAlt.set ( false ) ;
	}
}
void Ihm::processOdometry ( void ) {

	if ( getOdometry.get () ) {

		Distance distance = readDistance () ;
		distanceLeft .set ( distance.leftWheel  ) ;
		distanceRight.set ( distance.rightWheel ) ;
		distanceTotal.set ( distance.mean ) ;

		Velocity velocity = readVelocity () ;
		speedLeft .set ( velocity.leftWheel  ) ;
		speedRight.set ( velocity.rightWheel ) ;

		timerPeriod.set ( timerPeriodicity ) ;
		processTime.set ( readSensorTime ) ;

		radio_speed.set      ( velocityCommand ) ;
		radio_direction.set  ( steeringCommand ) ;
	}
}
void Ihm::sendAtitudeOverBluetooth ( void ) {
	char * buf ;
	uint8_t msgMaxLen = MAX_BT_ATITUDE_MSG_LENGTH ;

	while ( usartDriver.getSendBuffer ( &buf , msgMaxLen ) < msgMaxLen ) {
		HAL_Delay ( 1 ) ;
	}
	int msglen = 0 ;
	buf [ msglen ++ ] = BT_ATITUDE_START_OF_MSG ;
	buf [ msglen ++ ] = BT_ATITUDE_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], ToDeg(attitude.pitch)  ) ;
	buf [ msglen ++ ] = BT_ATITUDE_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], ToDeg(attitude.roll) ) ;
	buf [ msglen ++ ] = BT_ATITUDE_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], ToDeg(attitude.yaw)   ) ;
	buf [ msglen ++ ] = END_OF_MSG ;

	usartDriver.send ( msglen ) ;
}
void Ihm::sendRawSensorOverBluetooth ( void ) {
	char * buf ;
	uint8_t msgMaxLen = MAX_BT_RAW_SENSOR_MSG_LENGTH ;

	while ( usartDriver.getSendBuffer ( &buf , msgMaxLen ) < msgMaxLen ) {
		HAL_Delay ( 1 ) ;
	}
	strcpy ( buf, BT_CALIBRATION_START_OF_MSG ) ;
	int msglen = strlen ( BT_CALIBRATION_START_OF_MSG ) ;

	msglen += ftoa ( &buf [ msglen ] , accelerometer.rawData.x ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ] , accelerometer.rawData.y ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], accelerometer	.rawData.z ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], gyrometer		.rawData.x ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], gyrometer		.rawData.y ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], gyrometer		.rawData.z ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], magnetometer	.rawData.x ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], magnetometer	.rawData.y ) ;
	buf [ msglen ++ ] = BT_CALIBRATION_SEPARATOR ;
	msglen += ftoa ( &buf [ msglen ], magnetometer	.rawData.z ) ;
	buf [ msglen ++ ] = END_OF_MSG ;

	usartDriver.send ( msglen ) ;
}

void Ihm::process ( void ) {

	processOrientation 	() ;
	if ( imuOnUsart == true ) {
#if IHM_BLUETOOTH_USAGE == IHM_ATTITUDE_ON_BLUETOOTH
	sendAtitudeOverBluetooth () ;
#elif IHM_BLUETOOTH_USAGE == IHM_CALIBRATION_ON_BLUETOOTH
	sendRawSensorOverBluetooth () ;
#endif
	}
	processOdometry   	() ;
	processTempAndAlt 	() ;

	tracker.process () ;
}


