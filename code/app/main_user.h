/*
 * main_user.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_APP_MAIN_USER_H_
#define CODE_APP_MAIN_USER_H_


#define TARGET_NAME	"TUBO"

// CONDITIONAL COMPILATION
#define	USE_COMPLEMENTARY_FILTER			false
#define	USE_GRAVIMETER						false
#define	USE_BAROMETER						false
#define	USE_RADIO_CONTROL					true
#define	USE_PROFILING						false
#define USE_PERIODICITY_CHECK				true
#define USE_TIMING							true
#define USE_BLOCK_UPON_ERROR				false
#define USE_RAW_SENSOR_DISPLAY				false

// WIFI AND BLEUTOOTH USAGE
#define TRACKER_WIFI_AND_BLUETOOTH			true	// WIFI AND BLUETOOTH USED BY TRACKER
#define TRACKER_WIFI_AND_IHM_BLUETOOTH  	false	// WIFI USED BY TRACKER, BLUETOOTH BY IHM FOR IMU PROCESSING CODE

#define IHM_ATTITUDE_ON_BLUETOOTH  			false	// WIFI USED BY IHM FOR IMU PROCESSING CODE (BUNNY)
#define IHM_CALIBRATION_ON_BLUETOOTH  		true	// WIFI USED BY IHM FOR SENSOR CALIBRATION (MOTION CAL)

#define IHM_BLUETOOTH_USAGE		  			IHM_ATTITUDE_ON_BLUETOOTH	// IHM_ATTITUDE_ON_BLUETOOTH or IHM_CALIBRATION_ON_BLUETOOTH

// OPERATION MODES
#define	DEFAULT_OPERATION_MODE				0
#define	SENSOR_ZERO_CALIBRATION_MODE		1
#define RADIO_ZERO_CALIBRATION_MODE			2
#define	SENSOR_RANGE_CALIBRATION_MODE		3
#define RADIO_RANGE_CALIBRATION_MODE		4
#define	POSITION_CONTROL_MODE				5

#define MAX_ANGLE_FOR_POSITION_PID			( ToRad( 90.0 )    ) // Output of position pid is normalize as an angle in radians
#define MAX_THROTTLE_FOR_TILT_PID	 		( MAX_THROTTLE/1.0 )
#define MAX_THROTTLE_FOR_ORIENTATION_PID 	( MAX_THROTTLE/3.0 )

#define RADIO_EWMA_WEIGHT			 		0.1
#define SPEED_EWMA_WEIGHT 					0.3
#define ANGULAR_SPEED_EWMA_WEIGHT			0.2
#define TORQUE_EWMA_WEIGHT					0.5
#define PWM_RATIO				 			 10  	// 100% -> 1000
#define ROTATION_SPEED_LIMIT				 45 	// degrees

// PHYSICAL DATA ON TUBO
#define COG_HEIGHT_MM						(500.0 + 7.22)				// Center Of Gravity 500 mm above wheel axis

#define CALIBRATION_WINDOW_MAX_SIZE		  (100)

// -----------------------------------------------------------------------------------------------------
// 								PROFILING
// -----------------------------------------------------------------------------------------------------

// PROFILING THREADS
#define	PROFILING_OTHER_PROCESSING			O
#define	PROFILING_MAIN_PROCESSING			1
#define	PROFILING_2_5_MS_PROCESSING			2	// CHOOSE BETWEEN 2,5 OR 5 MS
#define	PROFILING_5_MS_PROCESSING			2	// CHOOSE BETWEEN 2,5 OR 5 MS
#define PROFILING_20_MS_PROCESSING			3
#define	PROFILING_40_MS_PROCESSING			4
#define PROFILING_100_MS_PROCESSING			5

#if USE_PROFILING == true
	#define PROFILING_BEGIN(x) 	profiling.begin ( x ) ;
	#define PROFILING_END   	profiling.end 	() ;
	#define PROFILING_SHOW		profiling.show 	() ;
#else
	#define PROFILING_BEGIN(x)
	#define PROFILING_END
	#define PROFILING_SHOW
#endif

#if USE_PERIODICITY_CHECK == true
	#define	CHECK_PERIODICITY timerPeriodicity = periodicity.check () ;
#else
	#define	CHECK_PERIODICITY
#endif

#if USE_TIMING == true
	#define CHRONO_START chrono.start() ;
	#define CHRONO_STOP	 readSensorTime = chrono.stop() ;
#else
	#define CHRONO_STOP
	#define CHRONO_START
#endif

// -----------------------------------------------------------------------------------------------------
// INCLUDES
#include <io_redirection.h>
#include "utils.h"
#include "stm32f7xx_nucleo_144.h"
#include "target_tracker_stm32.h"
#include "usart_driver.h"
#include "bsp.h"
#include "eeprom_store.h"
#include "ihm.h"
#include "motor.h"
#include "pid.h"
#include "odometer.h"
#include "accelerometer.h"
#include "gyrometer.h"
#include "magnetometer.h"
#include "ahrs.h"
#include "ewma.h"
#include "quaternion_euler_conversion.h"

#if USE_PROFILING == true
	#include "profiling.h"
#endif
#if USE_PERIODICITY_CHECK == true
	#include "periodicity_check.h"
#endif
#if USE_TIMING == true
	#include "elapsed_time.h"
#endif
#if USE_BAROMETER == true
	#include "barometer.h"
#endif
#if USE_GRAVIMETER == true
	#include "gravimeter.h"
#endif
#if USE_COMPLEMENTARY_FILTER == true
	#include "complementary_filter.h"
#endif
#if USE_RADIO_CONTROL == true
	#include "radio_control.h"
#endif

// Function called from C code :
#ifdef __cplusplus
  extern "C" {
#endif

   void mainSetup 									( void ) ;
   void mainLoop									( void ) ;

#ifdef __cplusplus
  }
#endif
   // BSP TIMER CALLBACKS
   void 	   _1_ms_processing 					( void ) ;
   void 	   _2_ms_processing 					( void ) ;
   void 	 _2_5_ms_processing 					( void ) ;
   void 	   _5_ms_processing 					( void ) ;
   void 	  _20_ms_processing 					( void ) ;
   void 	  _40_ms_processing 					( void ) ;
   void 	 _100_ms_processing 					( void ) ;
   void 	_1000_ms_processing 					( void ) ;

   // BSP CALLBACKS
   void 	nucleoUserButtonBspCallback 			( void ) ;
   void 	frontPanelUserButtonBspCallback 		( void ) ;
   void 	frontPanelResetButtonBspCallback		( void ) ;
   void 	imuGintBspCallback						( void ) ;
   void 	imuGrdyBspCallback 						( void ) ;
   void 	itRaspberryBspCallback 					( void ) ;
   void 	capteurPing1BspCallback 				( void ) ;
   void 	capteurPing2BspCallback 				( void ) ;
   void 	capteurPing3BspCallback 				( void ) ;
   void 	capteurPing4BspCallback 				( void ) ;
   void 	gpsFixBspCallback						( void ) ;
   void 	gps1ppsBspCallback 						( void ) ;

   // IHM CALLBACKS
   void 	blueLedIhmCallback						( void ) ;
   void 	greenLedIhmCallback						( void ) ;
   void 	redLedIhmCallback 						( void ) ;
   void 	m_aLedIhmCallback 						( void ) ;
   void 	wifiLedIhmCallback						( void ) ;
   void 	erreurLedIhmCallback 					( void ) ;
   void 	aBuzzerIhmCallback  					( void ) ;
   void 	bBuzzerIhmCallback  					( void ) ;
   void 	nucleoUserButtonIhmCallback 			( void ) ;
   void 	frontPanelUserButtonIhmCallback 		( void ) ;
   void 	frontPanelResetButtonIhmCallback		( void ) ;
   void     calibrationIhmCallback					( void ) ;
   void 	leftMotorCmdCallback					( void ) ;
   void 	rightMotorCmdCallback					( void ) ;
   void		resetOdometryCallback					( void ) ;
   void		targetPositionCallback					( void ) ;
   void		targetDirectionCallback					( void ) ;
   void 	tiltPidKpCallback						( void ) ;
   void 	tiltPidKiCallback						( void ) ;
   void 	tiltPidKdCallback						( void ) ;
   void 	orientPidKpCallback						( void ) ;
   void 	orientPidKiCallback						( void ) ;
   void 	orientPidKdCallback						( void ) ;
   void 	positionKCallback						( void ) ;
   void 	velocityKpCallback						( void ) ;
   void 	positionPidKpCallback					( void ) ;
   void 	positionPidKiCallback					( void ) ;
   void 	positionPidKdCallback					( void ) ;
   double 	readTemperature 						( void ) ;
   double 	readAltitude	 						( void ) ;
   Distance readDistance	 						( void ) ;
   Velocity readVelocity	 						( void ) ;
   void 	operationModeCallback					( void ) ;
   void 	storeEepromCallback						( void ) ;
   void 	radioCalibrateZeroCallback				( void ) ;
   void 	radioCalibrateRangeCallback				( void ) ;
   void 	betaCallback 							( void ) ;
   // GLOBAL ROUTINES
   void 	app_error 								( app_error_t error ) ;

   // LOCAL ROUTINES
   void 	initTracker 							( bool mode ) ;
   void 	bip										( void ) ;
   double 	getPressure 							( void ) ;
   double    getTemperature	 						( void ) ;
   inline void 	readSensors 						( void ) ;
   void 	calibrateZero 							( void ) ;

   // LOCAL ROUTINES : MODES OF OPERATION
   void 	setOperationMode 						( uint8_t mode ) ;
   void 	init_defaultOperationMode 				( void ) ;
   void 	defaultOperationMode 					( void ) ;
   void 	sensorZeroCalibrationMode				( void ) ;
   void		sensorRangeCalibrationMode				( void ) ;
   void		init_radioRangeCalibrationMode			( void ) ;
   void		radioZeroCalibrationMode 				( void ) ;
   void 	radioRangeCalibrationMode 				( void ) ;
   void 	init_positionControlMode 				( void ) ;
   inline void 	positionControlMode 				( void ) ;

#endif /* CODE_APP_MAIN_USER_H_ */
