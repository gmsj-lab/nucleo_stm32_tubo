/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_SERVO_A_TIM9_Pin GPIO_PIN_5
#define PWM_SERVO_A_TIM9_GPIO_Port GPIOE
#define PWM_SERVO_B_TIM9_Pin GPIO_PIN_6
#define PWM_SERVO_B_TIM9_GPIO_Port GPIOE
#define USER_BLUE_BUTTON_Pin GPIO_PIN_13
#define USER_BLUE_BUTTON_GPIO_Port GPIOC
#define USER_BLUE_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_A_Pin GPIO_PIN_1
#define BUZZER_A_GPIO_Port GPIOF
#define SPI_SS_1_Pin GPIO_PIN_2
#define SPI_SS_1_GPIO_Port GPIOF
#define SPI_SS_2_Pin GPIO_PIN_8
#define SPI_SS_2_GPIO_Port GPIOF
#define SPI_SS_3_Pin GPIO_PIN_9
#define SPI_SS_3_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define TENSION_BATTERIE_Pin GPIO_PIN_0
#define TENSION_BATTERIE_GPIO_Port GPIOC
#define ENCODER_LEFT_CH1_Pin GPIO_PIN_0
#define ENCODER_LEFT_CH1_GPIO_Port GPIOA
#define ENCODER_LEFT_CH2_Pin GPIO_PIN_1
#define ENCODER_LEFT_CH2_GPIO_Port GPIOA
#define GPS_FIX_Pin GPIO_PIN_3
#define GPS_FIX_GPIO_Port GPIOA
#define ENCODER_RIGHT_CH1_Pin GPIO_PIN_5
#define ENCODER_RIGHT_CH1_GPIO_Port GPIOA
#define IMU_GINT_Pin GPIO_PIN_6
#define IMU_GINT_GPIO_Port GPIOA
#define IMU_GINT_EXTI_IRQn EXTI9_5_IRQn
#define IMU_GRDY_Pin GPIO_PIN_7
#define IMU_GRDY_GPIO_Port GPIOA
#define IMU_GRDY_EXTI_IRQn EXTI9_5_IRQn
#define ACCELERO_X_ANALOGIQUE_Pin GPIO_PIN_5
#define ACCELERO_X_ANALOGIQUE_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define FRONT_PANEL_USER_BUTTON_Pin GPIO_PIN_0
#define FRONT_PANEL_USER_BUTTON_GPIO_Port GPIOG
#define FRONT_PANEL_USER_BUTTON_EXTI_IRQn EXTI0_IRQn
#define GPS_RX_Pin GPIO_PIN_7
#define GPS_RX_GPIO_Port GPIOE
#define GPS_TX_Pin GPIO_PIN_8
#define GPS_TX_GPIO_Port GPIOE
#define CAPTEUR_PING_1_Pin GPIO_PIN_9
#define CAPTEUR_PING_1_GPIO_Port GPIOE
#define CAPTEUR_PING_1_EXTI_IRQn EXTI9_5_IRQn
#define CAPTEUR_PING_2_Pin GPIO_PIN_10
#define CAPTEUR_PING_2_GPIO_Port GPIOE
#define CAPTEUR_PING_2_EXTI_IRQn EXTI15_10_IRQn
#define CAPTEUR_PING_3_Pin GPIO_PIN_14
#define CAPTEUR_PING_3_GPIO_Port GPIOE
#define CAPTEUR_PING_3_EXTI_IRQn EXTI15_10_IRQn
#define CAPTEUR_PING_4_Pin GPIO_PIN_15
#define CAPTEUR_PING_4_GPIO_Port GPIOE
#define CAPTEUR_PING_4_EXTI_IRQn EXTI15_10_IRQn
#define LEFT_INB_Pin GPIO_PIN_12
#define LEFT_INB_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define GPS_1PPS_Pin GPIO_PIN_11
#define GPS_1PPS_GPIO_Port GPIOD
#define GPS_1PPS_EXTI_IRQn EXTI15_10_IRQn
#define GPS_EN_Pin GPIO_PIN_12
#define GPS_EN_GPIO_Port GPIOD
#define RIGHT_INA_Pin GPIO_PIN_6
#define RIGHT_INA_GPIO_Port GPIOC
#define RIGHT_INB_Pin GPIO_PIN_8
#define RIGHT_INB_GPIO_Port GPIOC
#define LEFT_INA_Pin GPIO_PIN_9
#define LEFT_INA_GPIO_Port GPIOC
#define PWM_RIGHT_MOTOR_Pin GPIO_PIN_8
#define PWM_RIGHT_MOTOR_GPIO_Port GPIOA
#define PWM_LEFT_MOTOR_Pin GPIO_PIN_9
#define PWM_LEFT_MOTOR_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define USART3_TX_RASPBERRY_Pin GPIO_PIN_10
#define USART3_TX_RASPBERRY_GPIO_Port GPIOC
#define USART3_RX_RASPBERRY_Pin GPIO_PIN_11
#define USART3_RX_RASPBERRY_GPIO_Port GPIOC
#define IT_RASPBERRY_Pin GPIO_PIN_12
#define IT_RASPBERRY_GPIO_Port GPIOC
#define IT_RASPBERRY_EXTI_IRQn EXTI15_10_IRQn
#define BUZZER_B_Pin GPIO_PIN_0
#define BUZZER_B_GPIO_Port GPIOD
#define USART2_TX_WIFI_Pin GPIO_PIN_5
#define USART2_TX_WIFI_GPIO_Port GPIOD
#define USART2_RX_WIFI_Pin GPIO_PIN_6
#define USART2_RX_WIFI_GPIO_Port GPIOD
#define USART6_RX_BLUETOOTH_Pin GPIO_PIN_9
#define USART6_RX_BLUETOOTH_GPIO_Port GPIOG
#define LED_WIFI_Pin GPIO_PIN_10
#define LED_WIFI_GPIO_Port GPIOG
#define LED_ERREUR_Pin GPIO_PIN_12
#define LED_ERREUR_GPIO_Port GPIOG
#define USART6_TX_BLUETOOTH_Pin GPIO_PIN_14
#define USART6_TX_BLUETOOTH_GPIO_Port GPIOG
#define LED_M_A_Pin GPIO_PIN_15
#define LED_M_A_GPIO_Port GPIOG
#define ENCODER_RIGHT_CH2_Pin GPIO_PIN_3
#define ENCODER_RIGHT_CH2_GPIO_Port GPIOB
#define PWM_INPUT_RADIO_CMD_SPEED_Pin GPIO_PIN_4
#define PWM_INPUT_RADIO_CMD_SPEED_GPIO_Port GPIOB
#define PWM_INPUT_RADIO_CMD_STEERING_Pin GPIO_PIN_6
#define PWM_INPUT_RADIO_CMD_STEERING_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
#define FRONT_PANEL_RESET_BUTTON_Pin GPIO_PIN_1
#define FRONT_PANEL_RESET_BUTTON_GPIO_Port GPIOE
#define FRONT_PANEL_RESET_BUTTON_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

#define LED_GREEN_Pin			LED1_Pin
#define LED_GREEN_GPIO_Port		GPIOB
#define LED_BLUE_Pin			LED2_Pin
#define LED_BLUE_GPIO_Port		GPIOB
#define LED_RED_Pin				LED3_Pin
#define LED_RED_GPIO_Port		GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
