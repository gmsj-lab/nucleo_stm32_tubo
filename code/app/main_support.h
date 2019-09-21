/*
 * main_support.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_MAIN_SUPPORT_H_
#define CODE_APP_MAIN_SUPPORT_H_


// -----------------------------------------------------------------------------------------------------
// 								GLOBALS
// -----------------------------------------------------------------------------------------------------
Callback			processMode = defaultOperationMode ;

EulerAngle 			attitude 		;
double_3D_t 		accelero_event 	;
double_3D_t 		magneto_event 	;
double_3D_t 		gyro_event 		;
double      		gravimeter_event;

Motor 				leftMotor  		;
Motor 				rightMotor 		;

Ahrs				ahrs 			;
Accelerometer		accelerometer 	;
Gyrometer			gyrometer 		;
Magnetometer		magnetometer 	;

#if USE_BAROMETER == true
Barometer			barometer 		;
#endif
#if USE_GRAVIMETER == true
Gravimeter			gravimeter 		;
#endif
#if USE_RADIO_CONTROL == true
RadioControl 		radioControl ( RADIO_EWMA_WEIGHT ) ;
#endif
Odometer			odometer 	 	;
TargetTracker 		tracker 	( TARGET_NAME ) ;

Ihm					ihm 		 	;


int16_t 			motorCmd  		;
int16_t				leftMotorCmd 	;
int16_t				rightMotorCmd 	;

Ewma 				velocityFilter     (SPEED_EWMA_WEIGHT) ;
Ewma				angularSpeedFilter (ANGULAR_SPEED_EWMA_WEIGHT) ;
Ewma				leftTorqueFilter   (TORQUE_EWMA_WEIGHT) ;
Ewma				rightTorqueFilter  (TORQUE_EWMA_WEIGHT) ;

#if USE_COMPLEMENTARY_FILTER == true
ComplementaryFilter complementaryFilter ;
#endif
bool 				tiltMode 				= false ;
bool 				sendTelemetry 			= false ;

double 				tiltCorrection			= 0.0 ;
double 				requestedTilt 	 		= 0.0 ;
double 				lastRequestedTilt 	 	= 0.0 ;

int32_t 			velocityCommand 		= 0.0 ;
int32_t 			steeringCommand 		= 0.0 ;
double				targetPosition 			= 0.0 ;		// In millimeters
double 				targetVelocity		 	= 0.0 ;		// In millimeters/s
double 				targetDirection	 		= 0.0 ;		// in degrees/s. -180 to + 180
Pid 				tiltPid 	  	;
Pid 				positionPid 	;
Pid 				orientationPid 	;

SlidingWindowDouble	xWindow ( CALIBRATION_WINDOW_MAX_SIZE ) ;
SlidingWindowDouble	yWindow ( CALIBRATION_WINDOW_MAX_SIZE ) ;
SlidingWindowDouble	zWindow ( CALIBRATION_WINDOW_MAX_SIZE ) ;
// -----------------------------------------------------------------------------------------------------
// 								PROFILING
// -----------------------------------------------------------------------------------------------------
uint32_t 			timerPeriodicity = 0 ;
#if USE_PERIODICITY_CHECK == true
	Periodicity 	periodicity ;
#endif

volatile uint32_t 	readSensorTime = 0 ;
#if USE_TIMING == true
	ElapsedTime 	chrono ;
#endif

#if USE_PROFILING == true
	Profiling 		profiling  	( "PROFILING", "main", "p400Hz",  "p50Hz",  "p25Hz",  "p10Hz"  ) ;
#endif

// -----------------------------------------------------------------------------------------------------
void setCallbacks ( void )
{
// -----------------------------------------------------------------------------------------------------
// 								SET BSP CALLBACKS
// -----------------------------------------------------------------------------------------------------
	BSP.nucleoUserButton		.setCallback ( nucleoUserButtonBspCallback 	 			) ;
	BSP.frontPanelUserButton 	.setCallback ( frontPanelUserButtonBspCallback 			) ;
	BSP.frontPanelResetButton 	.setCallback ( frontPanelResetButtonBspCallback 		) ;
	BSP.itRaspberry				.setCallback ( itRaspberryBspCallback 		 			) ;
	BSP.imuGint					.setCallback ( imuGintBspCallback 			 			) ;
	BSP.imuGrdy					.setCallback ( imuGrdyBspCallback 						) ;
	BSP.capteurPing1			.setCallback ( capteurPing1BspCallback 					) ;
	BSP.capteurPing2			.setCallback ( capteurPing2BspCallback 					) ;
	BSP.capteurPing3			.setCallback ( capteurPing3BspCallback 					) ;
	BSP.capteurPing4			.setCallback ( capteurPing4BspCallback 					) ;
	BSP.gpsFix					.setCallback ( gpsFixBspCallback 						) ;
	BSP.gps1pps					.setCallback ( gps1ppsBspCallback 						) ;
//	BSP.timers  				.setCallback ( _400_HZ_TIMER,    _2_5_ms_processing 	) ;
	BSP.timers  				.setCallback ( _200_HZ_TIMER,    _5_ms_processing 	    ) ;
	BSP.timers  				.setCallback (  _50_HZ_TIMER,   _20_ms_processing 		) ;
//	BSP.timers  				.setCallback (  _25_HZ_TIMER,   _40_ms_processing 		) ;
	BSP.timers  				.setCallback (  _10_HZ_TIMER , _100_ms_processing  		) ;
	BSP.timers  				.setCallback (   _1_HZ_TIMER, _1000_ms_processing		) ;
}

// -----------------------------------------------------------------------------------------------------
// 								EEPROM EMULATION
// -----------------------------------------------------------------------------------------------------
Store store ;

void retrieveFromEeprom ( void ) {

	if ( ! BSP.flash.get( (uint32_t *) & store , sizeof ( store ) ) ) {
		app_error( FLASH_ERROR ) ;
	}
	if ( store.validity != EEPROM_VALIDITY_MARK_UP ) {
		store.tiltKp   		  	  				= 15990.0 ;
		store.tiltKi    		  				=   100.0 ;
		store.tiltKd    		  				=   658.0 ;

		store.orientKp  		  				=   200.0 ;
		store.orientKi  		  				=     0.0 ;
		store.orientKd  		  				=     0.0 ;

		store.velocityKp						=     0.0 ; // Not used anymore

		store.positionKp		  				=     5.0 ;
		store.positionKi		  				=     8.0 ;
		store.positionKd		  				=     0.0 ;

		store.offset.heading 					=     0.0 ;
		store.offset.pitch   					=     0.0 ;
		store.offset.roll 	  					=     0.0 ;

		store.magnetoOffset.x 	  				=     246 ;
		store.magnetoOffset.y 	  				=     239 ;
		store.magnetoOffset.z 	  				=    -703 ;

		store.magnetoScaleFactor.x				=     0.00093297746144721235 ;
		store.magnetoScaleFactor.y				=     0.00089937106918239001 ;
		store.magnetoScaleFactor.z				=     0.00122507788161993760 ;

		store.radioCalibration.speed.zero 		=  1494 ;
		store.radioCalibration.speed.max 		=  1999 ;
		store.radioCalibration.speed.min 		=   999 ;

		store.radioCalibration.direction.zero 	=  1222 ;
		store.radioCalibration.direction.max 	= 30922 ;
		store.radioCalibration.direction.min 	= 63322 ;
	}

	ihm.tiltKp		.set ( store.tiltKp ) ;
	ihm.tiltKi		.set ( store.tiltKi ) ;
	ihm.tiltKd		.set ( store.tiltKd ) ;

	ihm.orientKp	.set ( store.orientKp ) ;
	ihm.orientKi	.set ( store.orientKi ) ;
	ihm.orientKd	.set ( store.orientKd ) ;

	ihm.velocityKp	.set ( store.velocityKp * 100.0 ) ;

	ihm.positionKp	.set ( store.positionKp * 100.0 ) ;
	ihm.positionKi	.set ( store.positionKi * 1000.0 ) ;
	ihm.positionKd	.set ( store.positionKd * 100.0 ) ;

	ihm.pitchOffset .set ( store.offset.pitch ) ;

}
void storeToEeprom ( void ) {
	store.validity =  EEPROM_VALIDITY_MARK_UP ;

	if ( ! BSP.flash.set( (uint32_t *) & store , sizeof ( store ) ) ) {
		app_error( FLASH_ERROR ) ;
	}
}
// -----------------------------------------------------------------------------------------------------
// 								INIT APPLICATION COMPONENTS
// -----------------------------------------------------------------------------------------------------
void initComponents ( void )
	{
	BSP.tensionBatterie			.init () ;
#if USE_GRAVIMETER == true
	BSP.analogAccelerometer		.init () ;
#endif
	leftMotor 	 		.init ( BSP.pwmLeftMotor   	, BSP.leftInb  			, BSP.leftIna  				) ;
	rightMotor	 		.init ( BSP.pwmRightMotor 	, BSP.rightIna 			, BSP.rightInb 				) ;
	odometer	 		.init ( _200_HZ_TIMER  		, BSP.encoderLeftMotor 	, BSP.encoderRightMotor 	) ;
	ahrs		 		.init ( _200_HZ_TIMER 															) ;
	gyrometer	 		.init ( BSP.i2c 																) ;
	accelerometer		.init ( BSP.i2c 																) ;
	magnetometer 		.init ( BSP.i2c 			, store.magnetoOffset	, store.magnetoScaleFactor	) ;
#if USE_RADIO_CONTROL == true
	radioControl 		.init ( BSP.radioCmdSteering, BSP.radioCmdSpeed, store.radioCalibration			) ;
#endif

#if USE_COMPLEMENTARY_FILTER == true
	complementaryFilter	.init ( _200_HZ_TIMER  		, 0 											) ;
#endif

#if USE_BAROMETER == true
	barometer	 		.init ( BSP.i2c 															) ;
#endif

#if USE_GRAVIMETER == true
	gravimeter	 		.init ( BSP.analogAccelerometer 											) ;
#endif
#if USE_PERIODICITY_CHECK == true
	periodicity	 		.init ( 5000, 3  															) ;
#endif
}

void pidCalibration ( void ) {
// -----------------------------------------------------------------------------------------------------
// 								PID CALIBRATION
// -----------------------------------------------------------------------------------------------------
	tiltPid 	  .set 	( store.tiltKp     , store.tiltKi    , store.tiltKd 	) ;
	positionPid   .set 	( store.positionKp , store.positionKi, store.positionKd ) ;
	orientationPid.set  ( store.orientKp   , store.orientKi  , store.orientKd 	) ;

	tiltPid     	.setBounds ( MAX_THROTTLE_FOR_TILT_PID		  ) ; // Make the PID provide values in motor range
	positionPid		.setBounds ( MAX_ANGLE_FOR_POSITION_PID 	  ) ; // Output of position pid is normalize as an angle in radians
	orientationPid  .setBounds ( MAX_THROTTLE_FOR_ORIENTATION_PID ) ;
}

// -----------------------------------------------------------------------------------------------------
// 								BSP OTHER CALLBACKS
// -----------------------------------------------------------------------------------------------------
void nucleoUserButtonBspCallback 		( void ) { setOperationMode ( SENSOR_ZERO_CALIBRATION_MODE ) ; }
void frontPanelUserButtonBspCallback 	( void ) { setOperationMode ( POSITION_CONTROL_MODE 	   ) ; }
void frontPanelResetButtonBspCallback	( void ) { setOperationMode ( DEFAULT_OPERATION_MODE) 		 ; }
void imuGintBspCallback 				( void ) {} 											// TODO
void imuGrdyBspCallback 				( void ) {} 											// TODO
void itRaspberryBspCallback 			( void ) {} 											// TODO
void capteurPing1BspCallback 			( void ) {} 											// TODO
void capteurPing2BspCallback 			( void ) {} 											// TODO
void capteurPing3BspCallback 			( void ) {} 											// TODO
void capteurPing4BspCallback 			( void ) {} 											// TODO
void gpsFixBspCallback 					( void ) {} 											// TODO
void gps1ppsBspCallback 				( void ) {} 											// TODO

// ---------------------------------------------------------------------------------------------------------
// 								IHM CALLBACKS
// ---------------------------------------------------------------------------------------------------------
void   blueLedIhmCallback				( void ) { BSP.ledBlue	 .toggle () ;								}
void   greenLedIhmCallback				( void ) { BSP.ledGreen  .toggle () ;								}
void   redLedIhmCallback 				( void ) { BSP.ledRed	 .toggle () ;								}
void   m_aLedIhmCallback 				( void ) { BSP.ledM_A	 .toggle () ;								}
void   wifiLedIhmCallback				( void ) { BSP.ledWifi	 .toggle () ;								}
void   erreurLedIhmCallback 			( void ) { BSP.ledErreur .toggle () ; 								}
void   aBuzzerIhmCallback  				( void ) { BSP.buzzerA	 .toggle () ;								}
void   bBuzzerIhmCallback  				( void ) { BSP.buzzerB	 .toggle () ;								}
void   nucleoUserButtonIhmCallback 		( void ) { nucleoUserButtonBspCallback 		() ;					}
void   frontPanelUserButtonIhmCallback 	( void ) { frontPanelUserButtonBspCallback  () ;					}
void   frontPanelResetButtonIhmCallback ( void ) { frontPanelResetButtonBspCallback () ;					}
void   calibrationIhmCallback 			( void ) { setOperationMode ( SENSOR_RANGE_CALIBRATION_MODE ) ;		}
void   targetPositionCallback			( void ) { targetPosition  = ihm.targetPosition.get () ;			}
void   targetDirectionCallback			( void ) { targetDirection += ToRad( ihm.target_Direction.get () ) ;	}
void   resetOdometryCallback			( void ) { odometer.reset() ;										}
void   leftMotorCmdCallback 			( void ) { leftMotorCmd  = ihm.leftMotor .get () * PWM_RATIO ;	 	}
void   rightMotorCmdCallback 			( void ) { rightMotorCmd = ihm.rightMotor.get () * PWM_RATIO ;		}
double readTemperature 					( void ) { return getTemperature 			() ; 					}
double readAltitude 					( void ) { return getPressure 				() ;					}
Distance readDistance 					( void ) { return odometer.getTotalDistance () ;					}
Velocity readVelocity 					( void ) { return odometer.getSpeed 		() ;					}
void  operationModeCallback 			( void ) { setOperationMode ( ihm.operationMode.get () ) ;			}
void  storeEepromCallback 				( void ) { storeToEeprom () ;										}

void   tiltPidKpCallback		 		( void ) {
												   store.tiltKp   = ihm.tiltKp.get () ;
												   tiltPid.setKp  ( store.tiltKp ) ;
												   ihm.tiltKp.set ( store.tiltKp ) ;						}
void   tiltPidKiCallback		 		( void ) {
												   store.tiltKi   = ihm.tiltKi.get ();
												   tiltPid.setKi  ( store.tiltKi ) ;
												   ihm.tiltKi.set ( store.tiltKi ) ;						}
void   tiltPidKdCallback				( void ) {
												   store.tiltKd   = ihm.tiltKd.get ();
												   tiltPid.setKd  ( store.tiltKd ) ;
												   ihm.tiltKd.set ( store.tiltKd ) ;						}
void   orientPidKpCallback		 		( void ) {
												   store.orientKp = ihm.orientKp.get ();
												   orientationPid.setKp ( store.orientKp ) ;
												   ihm.orientKp.set     ( store.orientKp ) ;				}
void   orientPidKiCallback				( void ) {
												   store.orientKi = ihm.orientKi.get ();
												   orientationPid.setKi ( store.orientKi ) ;
												   ihm.orientKi.set     ( store.orientKi ) ;				}
void   orientPidKdCallback	 			( void ) {
												   store.orientKd = ihm.orientKd.get ();
												   orientationPid.setKd ( store.orientKd ) ;
												   ihm.orientKd.set     ( store.orientKd ) ;				}
void   velocityKpCallback			 	( void ) {
												   store.velocityKp = ihm.velocityKp.get () / 100.0 ;
												   ihm.velocityKp.set( store.velocityKp * 100.0 ) ;			}
void   positionPidKpCallback		 	( void ) {
												   store.positionKp = ihm.positionKp.get () / 100.0 ;
												   positionPid.setKp ( store.positionKp ) ;
												   ihm.positionKp.set( store.positionKp * 100.0 ) ;			}
void   positionPidKiCallback			( void ) {
												   store.positionKi = ihm.positionKi.get () / 1000.0 ;
												   positionPid.setKi ( store.positionKi ) ;
												   ihm.positionKi.set( store.positionKi * 1000.0 ) ;		}
void   positionPidKdCallback	 		( void ) {
												   store.positionKd = ihm.positionKd.get () / 100.0 ;
												   positionPid.setKd ( store.positionKd ) ;
												   ihm.positionKd.set( store.positionKd * 100.0 ) ;			}
void  radioCalibrateZeroCallback 		( void ) { setOperationMode ( RADIO_ZERO_CALIBRATION_MODE  ) ;		}
void  radioCalibrateRangeCallback 		( void ) { setOperationMode ( RADIO_RANGE_CALIBRATION_MODE ) ;		}
void betaCallback 						( void ) { ahrs.setBeta( ihm.beta.get() / 1000.0 );					}

// -----------------------------------------------------------------------------------------------------
// 								GLOBAL ROUTINES
// -----------------------------------------------------------------------------------------------------
void app_error ( app_error_t error ) {
	ihm.polebot_error.set ( error ) ;
#if USE_BLOCK_UPON_ERROR == true
	while ( 1 ) {
		bip	() ;
		HAL_Delay ( 1000 ) ;
	}
#else
	bip	() ;
	bip	() ;
	bip	() ;
#endif
}
// -----------------------------------------------------------------------------------------------------
// 								LOCAL ROUTINES
// -----------------------------------------------------------------------------------------------------
void initTracker ( bool mode ) {

	tracker.setTimeTagging ( TIME_TAGGING ) ;
	if ( mode == TRACKER_WIFI_AND_BLUETOOTH ) {
		ihm.init () ;
		tracker.begin ( UART_WIFI , UART_BLUETOOTH ) ;
	}
	else {
		ihm.init ( UART_BLUETOOTH ) ;
		tracker.begin ( UART_WIFI ) ;
	}

	tracker.print ( "======= TargetTracker " ) ;
	tracker.print ( TARGET_NAME ) ;
	tracker.println ( " =======" ) ;
}

void bip ( void ) {
	// Announce that we are ready
	BSP.buzzerA.on () ;
	HAL_Delay ( 20 ) ;
	BSP.buzzerA.off () ;
	HAL_Delay ( 20 ) ;
}
double getPressure 	( void ) {
#if USE_BAROMETER == true
	return ( barometer.pressureToAltitude ( SENSORS_PRESSURE_SEALEVELHPA , barometer.read ().pressure ) ) ;
#else
	return 0 ;
#endif
}

double getTemperature ( void ) {
#if USE_BAROMETER == true
	return barometer.getTemperature () ;
#else
	return 0 ;
#endif
}

// -----------------------------------------------------------------------------------------------------
// 								LOCAL ROUTINES : MODES OF OPERATION
// -----------------------------------------------------------------------------------------------------
void setOperationMode ( uint8_t mode ) {
	switch ( mode ) {
		case DEFAULT_OPERATION_MODE 		: 	init_defaultOperationMode () 			 ; break ;
		case SENSOR_ZERO_CALIBRATION_MODE 	: 	processMode = sensorZeroCalibrationMode  ; break ;
		case SENSOR_RANGE_CALIBRATION_MODE 	: 	processMode = sensorRangeCalibrationMode ; break ;
		case RADIO_ZERO_CALIBRATION_MODE 	:   radioZeroCalibrationMode ()				 ; break ;
		case RADIO_RANGE_CALIBRATION_MODE 	: 	init_radioRangeCalibrationMode ()		 ; break ;
		case POSITION_CONTROL_MODE 			:	init_positionControlMode () 		 	 ; break ;
		default: break ;
	}
}

// -----------------------------------defaultOperationMode-------------------------------------------------
void init_defaultOperationMode 	( void ) {

    // Give order to motors TO STOP
    leftMotorCmd  = 0.0 ;
    rightMotorCmd = 0.0 ;

    // Reset odometer
    odometer.reset () ;

    // Go to default mode
	processMode = defaultOperationMode ;
}

void defaultOperationMode 	( void ) {
	// Do nothing
}
// -----------------------------------sensorZeroCalibrationMode-------------------------------------------------
void sensorZeroCalibrationMode ( void ) {

	calibrateZero () ;
	//gyrometer.calibrateZero () ;

#if USE_GRAVIMETER == true
	gravimeter.calibrateZero () ;
#endif
}
void calibrateZero ( void ) {

	xWindow.set ( attitude.roll  ) ;
	yWindow.set ( attitude.pitch ) ;
	zWindow.set ( attitude.yaw   ) ;

	store.offset.roll 		= xWindow.get () ;
	store.offset.pitch 		= yWindow.get () ;
	store.offset.heading 	= zWindow.get () ;

	ihm.pitchOffset .set ( (float) store.offset.pitch ) ;
}
void calibrate 	( void ) {
	attitude.roll  -= store.offset.roll ;
	attitude.pitch -= store.offset.pitch ;
	attitude.yaw   -= store.offset.heading ;
}

// -----------------------------------sensor Calibration Modes-------------------------------------------------
void sensorRangeCalibrationMode ( void ) {
	magnetometer.calibrateRange () ;
	store.magnetoOffset 	 = magnetometer.getCalibrationOffset () ;
	store.magnetoScaleFactor = magnetometer.getCalibrationScale  () ;
#if USE_GRAVIMETER == true
	gravimeter.calibrateRange () ;
#endif
}

// -----------------------------------radio Calibration Modes -------------------------------------------------

#if USE_RADIO_CONTROL == true
void radioZeroCalibrationMode ( void ) {

	RadioCalibration calibration = radioControl.calibrateZero  () ;

	store.radioCalibration.direction.zero = calibration.direction.zero ;
	store.radioCalibration.speed.zero 	  = calibration.speed.zero ;

	processMode = init_defaultOperationMode ;
}

void init_radioRangeCalibrationMode ( void ) {
	radioControl.resetRange () ;
	processMode = radioRangeCalibrationMode ;
}
void radioRangeCalibrationMode ( void ) {
	RadioCalibration calibration = radioControl.calibrateRange () ;

	store.radioCalibration.direction.max = calibration.direction.max ;
	store.radioCalibration.direction.min = calibration.direction.min ;
	store.radioCalibration.speed.max 	 = calibration.speed.max ;
	store.radioCalibration.speed.min 	 = calibration.speed.min ;
}
#else
void radioZeroCalibrationMode 		( void ) {}
void init_radioRangeCalibrationMode ( void ) {}
#endif

#endif /* CODE_APP_MAIN_SUPPORT_H_ */
