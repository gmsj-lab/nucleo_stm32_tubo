/*
 * bsp.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "bsp.h"
#include "boot_link.h"

_BSP _BSP::bsp =_BSP();


_BSP::_BSP ()  :
	nucleoUserButton 	  	( USER_BUTTON_PIN 			 				, USER_BUTTON_GPIO_PORT 			 ) ,
	frontPanelUserButton  	( FRONT_PANEL_USER_BUTTON_Pin   			, FRONT_PANEL_USER_BUTTON_GPIO_Port  ) ,
	frontPanelResetButton 	( FRONT_PANEL_RESET_BUTTON_Pin 				, FRONT_PANEL_RESET_BUTTON_GPIO_Port ) ,
	gpsFix 					( GPS_FIX_Pin								, GPS_FIX_GPIO_Port					 ) ,
	gps1pps 				( GPS_1PPS_Pin								, GPS_1PPS_GPIO_Port 				 ) ,
	imuGrdy					( IMU_GRDY_Pin								, IMU_GRDY_GPIO_Port				 ) ,
	imuGint					( IMU_GINT_Pin								, IMU_GINT_GPIO_Port				 ) ,
	itRaspberry 			( IT_RASPBERRY_Pin							, IT_RASPBERRY_GPIO_Port			 ) ,
	capteurPing1			( CAPTEUR_PING_1_Pin						, CAPTEUR_PING_1_GPIO_Port			 ) ,
	capteurPing2			( CAPTEUR_PING_2_Pin						, CAPTEUR_PING_2_GPIO_Port			 ) ,
	capteurPing3			( CAPTEUR_PING_3_Pin						, CAPTEUR_PING_3_GPIO_Port			 ) ,
	capteurPing4			( CAPTEUR_PING_4_Pin						, CAPTEUR_PING_4_GPIO_Port			 ) ,
	ledWifi					( LED_WIFI_Pin								, LED_WIFI_GPIO_Port	, true		 ) ,
	ledM_A					( LED_M_A_Pin								, LED_M_A_GPIO_Port		, true		 ) ,
	ledErreur				( LED_ERREUR_Pin							, LED_ERREUR_GPIO_Port	, true		 ) ,
	ledGreen				( LED_GREEN_Pin								, LED_GREEN_GPIO_Port				 ) ,
	ledBlue					( LED_BLUE_Pin								, LED_BLUE_GPIO_Port				 ) ,
	ledRed					( LED_RED_Pin								, LED_RED_GPIO_Port					 ) ,
	buzzerA					( BUZZER_A_Pin								, BUZZER_A_GPIO_Port				 ) ,
	buzzerB					( BUZZER_B_Pin								, BUZZER_B_GPIO_Port				 ) ,
	leftIna					( LEFT_INA_Pin								, LEFT_INA_GPIO_Port				 ) ,
	leftInb					( LEFT_INB_Pin								, LEFT_INB_GPIO_Port				 ) ,
	rightIna				( RIGHT_INA_Pin								, RIGHT_INA_GPIO_Port				 ) ,
	rightInb				( RIGHT_INB_Pin								, RIGHT_INB_GPIO_Port				 ) ,
	gpsEn					( GPS_EN_Pin								, GPS_EN_GPIO_Port					 ) ,
	spiSs1					( SPI_SS_1_Pin								, SPI_SS_1_GPIO_Port				 ) ,
	spiSs2					( SPI_SS_2_Pin								, SPI_SS_2_GPIO_Port				 ) ,
	spiSs3					( SPI_SS_3_Pin								, SPI_SS_3_GPIO_Port				 ) ,
	timers					( TIMER_HTIM_2_5_MS_PERIODIC_PROCESSING 										 ) ,
//	timer_1_ms				( TIMER_HTIM_1_MS_PERIODIC_PROCESSING 											 ) ,
//	timer_5_ms				( TIMER_HTIM_5_MS_PERIODIC_PROCESSING 											 ) ,
//	timer_20_ms				( TIMER_HTIM_20_MS_PERIODIC_PROCESSING 											 ) ,
//	timer_1000_ms			( TIMER_HTIM_1000_MS_PERIODIC_PROCESSING 										 ) ,
	pwmLeftMotor			( TIMER_HTIM_PWM_MOTORS 					, TIMER_HTIM_PWM_LEFT_MOTOR_CHANNEL	 ) ,
	pwmRightMotor 			( TIMER_HTIM_PWM_MOTORS 					, TIMER_HTIM_PWM_RIGHT_MOTOR_CHANNEL ) ,
	pwmServoA	 			( TIMER_HTIM_PWM_SERVOS 					, TIMER_HTIM_PWM_SERVO_A_CHANNEL	 ) ,
	pwmServoB	 			( TIMER_HTIM_PWM_SERVOS 					, TIMER_HTIM_PWM_SERVO_B_CHANNEL	 ) ,
	radioCmdSpeed			( TIMER_HTIM_PWM_INPUT_RADIO_CMD_SPEED		, TIMER_HTIM_PWM_IN_RC_SPEED_CHANNEL ) ,
	radioCmdSteering		( TIMER_HTIM_PWM_INPUT_RADIO_CMD_STEERING	, TIMER_HTIM_PWM_IN_RC_STEER_CHANNEL ) ,
	encoderLeftMotor		( TIMER_HTIM_PWM_LEFT_ENCODER 													 ) ,
	encoderRightMotor		( TIMER_HTIM_PWM_RIGHT_ENCODER													 ) ,
	tensionBatterie			( ADC_HADC_BATTERY							, BATTERY_CONVERSION_RATIO		 	 ) ,
	analogAccelerometer		( ADC_HADC_GRAVIMETER						, ACCELEROMETER_CONVERSION_RATIO	 ) ,
	i2c						( & I2C_HANDLE																	 ) ,
	spi						( 																				 )
{
}

void _BSP::init ( void ) {
	ledBlue  .off () ;
	ledRed   .off () ;
	ledGreen .off () ;
	ledM_A	 .off () ;
	ledWifi	 .off () ;
	ledErreur.off () ;
}
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim ) {
	if ( htim->Instance == TIMER_2_5_MS ) {
			BSP.timers.timersCallback () ;
		}

//	if ( htim->Instance == TIMER_1_MS ) {
//		BSP.timer_1_ms.timerCallback () ;
//	}
//	else if ( htim->Instance == TIMER_5_MS ) {
//		BSP.timer_5_ms.timerCallback () ;
//	}
//	else if ( htim->Instance == TIMER_20_MS ) {
//		BSP.timer_20_ms.timerCallback () ;
//	}
//	else if ( htim->Instance == TIMER_1000_MS ) {
//		BSP.timer_1000_ms.timerCallback () ;
//	}
}

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin ) {
	switch ( GPIO_Pin ) {
		case FRONT_PANEL_USER_BUTTON_Pin  :	BSP.frontPanelUserButton	.gpioInterrupt () ;	break ;
		case FRONT_PANEL_RESET_BUTTON_Pin :	BSP.frontPanelResetButton	.gpioInterrupt () ; break ;
		case USER_BUTTON_PIN 			  :	BSP.nucleoUserButton		.gpioInterrupt () ; break ;
		case GPS_FIX_Pin 				  :	BSP.gpsFix					.gpioInterrupt () ; break ;
		case IMU_GINT_Pin 				  : BSP.imuGint					.gpioInterrupt () ; break ;
		case IMU_GRDY_Pin 				  :	BSP.imuGrdy					.gpioInterrupt () ; break ;
		case GPS_1PPS_Pin 				  :	BSP.gps1pps					.gpioInterrupt () ; break ;
		case IT_RASPBERRY_Pin 			  :	BSP.itRaspberry				.gpioInterrupt () ; break ;
		case CAPTEUR_PING_1_Pin 		  :	BSP.capteurPing1			.gpioInterrupt () ; break ;
		case CAPTEUR_PING_2_Pin 		  :	BSP.capteurPing2			.gpioInterrupt () ; break ;
		case CAPTEUR_PING_3_Pin 		  :	BSP.capteurPing3			.gpioInterrupt () ; break ;
		case CAPTEUR_PING_4_Pin 		  :	BSP.capteurPing4			.gpioInterrupt () ; break ;
		default :
			break ;
	}
}
void stop ( void ) {
	BSP.timers				.stop () ;
//	BSP.timer_1_ms			.stop () ;
//	BSP.timer_5_ms			.stop () ;
//	BSP.timer_20_ms			.stop () ;
//	BSP.timer_1000_ms		.stop () ;
	BSP.tensionBatterie		.stop () ;
	BSP.analogAccelerometer	.stop () ;
	BSP.encoderLeftMotor	.stop () ;
	BSP.encoderRightMotor	.stop () ;
}

void softwareRebootReq ( void ) {
	stop () ;
	performSoftwareReboot () ;
}
void softwareReprogrammingReq ( void ) {
	stop () ;
	setupSoftwareReprogramming () ;
}


