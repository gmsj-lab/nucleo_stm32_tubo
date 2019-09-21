/*
 * bsp.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_BSP_BSP_H_
#define CODE_BSP_BSP_H_


#include "utils.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "main.h"
#include "gpio_input.h"
#include "gpio_output.h"
#include "i2c_driver.h"
#include "spi_driver.h"
#include "usart_driver.h"
#include "adc_driver.h"
#include "encoder.h"
#include "pwm_in.h"
#include "pwm_out.h"
#include "timer.h"
#include "timers.h"
#include "flash.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi2;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;

/* ------------------------------------ */
/* Hardware ressources usage definition */
/* ------------------------------------ */

/* Timer usage definition */

#define TIMER_PWM_MOTORS 							TIM1
#define TIMER_PWM_LEFT_ENCODER						TIM2
#define TIMER_PWM_INPUT_RADIO_CMD_SPEED 			TIM3
#define TIMER_PWM_INPUT_RADIO_CMD_STEERING 			TIM4
#define TIMER_PWM_RIGHT_ENCODER 					TIM5
#define TIMER_1_MS			 						TIM6
#define TIMER_1000_MS			 					TIM7
#define TIMER_PWM_SERVOS		 					TIM9
#define TIMER_2_5_MS		 						TIM13
#define TIMER_5_MS		 							TIM14

#define TIMER_HTIM_PWM_MOTORS 						htim1
#define TIMER_HTIM_PWM_RIGHT_ENCODER 				htim2
#define TIMER_HTIM_PWM_LEFT_ENCODER					htim5
#define TIMER_HTIM_PWM_INPUT_RADIO_CMD_SPEED	 	htim3
#define TIMER_HTIM_PWM_INPUT_RADIO_CMD_STEERING 	htim4
#define TIMER_HTIM_PWM_SERVOS	 					htim9
#define TIMER_HTIM_1_MS_PERIODIC_PROCESSING			htim6
#define TIMER_HTIM_1000_MS_PERIODIC_PROCESSING		htim7
#define TIMER_HTIM_2_5_MS_PERIODIC_PROCESSING		htim13
#define TIMER_HTIM_5_MS_PERIODIC_PROCESSING			htim14

#define TIMER_HTIM_PWM_RIGHT_MOTOR_CHANNEL 			TIM_CHANNEL_1
#define TIMER_HTIM_PWM_LEFT_MOTOR_CHANNEL 			TIM_CHANNEL_2
#define TIMER_HTIM_PWM_SERVO_A_CHANNEL	 			TIM_CHANNEL_1
#define TIMER_HTIM_PWM_SERVO_B_CHANNEL	 			TIM_CHANNEL_2
#define TIMER_HTIM_PWM_IN_RC_SPEED_CHANNEL	 		TIM_CHANNEL_1
#define TIMER_HTIM_PWM_IN_RC_STEER_CHANNEL		 	TIM_CHANNEL_1


/* Adc usage definition */

#define ADC_BATTERY	 								ADC1
#define ADC_GRAVIMETER 								ADC2

#define ADC_HADC_BATTERY	 						hadc1
#define ADC_HADC_GRAVIMETER 						hadc2


/* I2C usage definition */
#define I2C_HANDLE									hi2c1

/* Uart usage definition */
#define UART_WIFI		 							2
#define UART_RASPBERRY		 						3
#define UART_BLUETOOTH		 						6
#define UART_GPS			 						7


#define BATTERY_CONVERSION_RATIO					( 10.0 / ( 100.0 + 10.0 ) )	// Resistor bridge : 10 KOhms, 100 KOhms
#define ACCELEROMETER_CONVERSION_RATIO				( 1.0  )					// TBD

#define BSP		_BSP::bsp

// Function called from C code :
#ifdef __cplusplus
  extern "C" {
#endif
void softwareRebootReq 		  ( void ) ;
void softwareReprogrammingReq ( void ) ;
#ifdef __cplusplus
  }
#endif
// Singleton class, accessed by the BSP define above
class _BSP {

private:
    _BSP();
    ~_BSP(){};
    _BSP(const _BSP&) ;

public:
    static _BSP 	bsp ;

	GpioInput		nucleoUserButton ;
	GpioInput		frontPanelUserButton ;
	GpioInput		frontPanelResetButton ;
	GpioInput		gpsFix ;
	GpioInput		gps1pps ;
	GpioInput		imuGrdy ;
	GpioInput		imuGint ;
	GpioInput		itRaspberry ;
	GpioInput		capteurPing1 ;
	GpioInput		capteurPing2 ;
	GpioInput		capteurPing3 ;
	GpioInput		capteurPing4 ;

	GpioOutput		ledWifi ;
	GpioOutput		ledM_A ;
	GpioOutput		ledErreur ;
	GpioOutput		ledGreen ;
	GpioOutput		ledBlue ;
	GpioOutput		ledRed ;
	GpioOutput		buzzerA ;
	GpioOutput		buzzerB ;
	GpioOutput		leftIna ;
	GpioOutput		leftInb ;
	GpioOutput		rightIna ;
	GpioOutput		rightInb ;
	GpioOutput		gpsEn ;
	GpioOutput		spiSs1 ;
	GpioOutput		spiSs2 ;
	GpioOutput		spiSs3 ;

	Timers			timers ;
//	Timer			timer_1_ms ;
//	Timer			timer_5_ms ;
//	Timer			timer_20_ms ;
//	Timer			timer_1000_ms ;

	PwmOut			pwmLeftMotor ;
	PwmOut			pwmRightMotor ;
	PwmOut			pwmServoA ;
	PwmOut			pwmServoB ;

	PwmIn			radioCmdSpeed ;
	PwmIn			radioCmdSteering ;

	Encoder			encoderLeftMotor ;
	Encoder			encoderRightMotor ;

	Adc				tensionBatterie ;
	Adc				analogAccelerometer ;

	I2cDriver 		i2c ;
	SpiDriver		spi ;
	UsartDriver		gps ;

	Flash			flash ;

	void 			init ( void ) ;
};

#endif /* CODE_BSP_BSP_H_ */
