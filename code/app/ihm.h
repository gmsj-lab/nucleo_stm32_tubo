/*
 * ihm.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_IHM_H_
#define CODE_APP_IHM_H_


#include "main_user.h"
#include "utils.h"
#include "usart_driver.h"

#include "target_attribute.h"
#include "quaternion_euler_conversion.h"
#include "sensor.h"

class Ihm {
private:

	uint32_t		timerValue ;
	bool 			imuOnUsart ;
	UsartDriver 	usartDriver ;

	void 			processOdometry				( void ) ;
	void 			processTempAndAlt			( void ) ;
	void			sendAtitudeOverBluetooth	( void ) ;
	void			sendRawSensorOverBluetooth	( void ) ;
public:

	// FRONT PANEL
	TT_bool 	blueLed	;
	TT_bool 	greenLed ;
	TT_bool 	redLed ;
	TT_bool 	m_aLed ;
	TT_bool 	wifiLed ;
	TT_bool 	erreurLed ;
	TT_bool 	aBuzzer	;
	TT_bool 	bBuzzer ;
	TT_bool 	nucleoUserButton ;
	TT_bool 	userButton ;
	TT_bool 	resetButton	;

	// TEMPERATURE & BAROMETRIC SENSORS
	TT_bool 	getTempAndAlt ;
	TT_float 	altitude , temperature ;

	// ATTITUDE SENSORS
	TT_bool 	getSensors, calibration ;
#if USE_RAW_SENSOR_DISPLAY == true
	TT_float 	gyroX  	  , gyroY  		, gyroZ ;
	TT_float 	accelX 	  , accelY 		, accelZ ;
//	TT_int16 	accelXraw , accelYraw	, accelZraw ;
	TT_float 	magnetoX  , magnetoY	, magnetoZ ;
#else
	TT_float 	gyroY ;
#endif
	TT_float 	pitch 	  , roll 		, yaw ;
	TT_float   	gravity;

	// ODOMETRY
	TT_bool		getOdometry ;
	TT_bool 	resetOdometry ;
	TT_int32	distanceLeft ;
	TT_int32	distanceRight ;
	TT_int32	distanceTotal ;
	TT_int32	speedLeft ;
	TT_int32	speedRight ;
	TT_int32	targetPosition ;
	TT_int32	target_Direction ;

	// MOTOR COMMANDS
	TT_int16	leftMotor ;
	TT_int16	rightMotor ;

	// PID PARAMETERS
	TT_int16	tiltKp ;
	TT_int16	tiltKi ;
	TT_int16	tiltKd ;

	TT_int16	orientKp ;
	TT_int16	orientKi ;
	TT_int16	orientKd ;

	TT_int16	velocityKp ;

	TT_int16	positionKp ;
	TT_int16	positionKi ;
	TT_int16	positionKd ;

	// ERROR MGT
	TT_uint32 	polebot_error ;

	// TUBO MODES
	TT_int8		operationMode ;

	// VOLTAGE CONTROL
	TT_int16	batteryLevel ;

	// CHECK TIMER FREQUECY
	TT_int32 	timerPeriod  ;
	TT_uint32 	processTime  ;

	// Position PID output
	TT_float 	positionTilt ;

	TT_bool 	storeEeprom ;

	TT_float	pitchOffset ;

	// Radio Control Commands
	TT_bool 	radio_zero ;
	TT_bool 	radio_range ;
	TT_float	radio_speed ;
	TT_float	radio_direction ;

	// Ahrs parameter
	TT_int16	beta ;

	Ihm ( void ) ;
	virtual ~Ihm () ;

	void init 			 	( int usartForIMU = - 1  ) ;
	void process		 	( void ) ;
	void processOrientation ( void ) ;
} ;

#endif /* CODE_APP_IHM_H_ */
