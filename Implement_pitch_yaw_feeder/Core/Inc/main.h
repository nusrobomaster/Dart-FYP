/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* ---------------------------------------------------------------------------*
 * LCD (SPI)
 * PE6=MOSI, PE5=MISO, PE12=SCK, PC1=LED, PC5=RS, PA5=RST, PA4=CS
 * --------------------------------------------------------------------------- */
#define LCD_LED_GPIO_Port    GPIOC
#define LCD_LED_Pin          GPIO_PIN_1

#define LCD_RS_GPIO_Port     GPIOC
#define LCD_RS_Pin           GPIO_PIN_5

#define LCD_RST_GPIO_Port    GPIOA
#define LCD_RST_Pin          GPIO_PIN_5

#define LCD_CS_GPIO_Port     GPIOA
#define LCD_CS_Pin           GPIO_PIN_4

#define LCD_MOSI_GPIO_Port   GPIOE
#define LCD_MOSI_Pin         GPIO_PIN_6

#define LCD_MISO_GPIO_Port   GPIOE
#define LCD_MISO_Pin         GPIO_PIN_5

#define LCD_SCK_GPIO_Port    GPIOE
#define LCD_SCK_Pin          GPIO_PIN_12

/* ---------------------------------------------------------------------------*
 * Touch (I2C + GPIO)
 * PF10=INT, PF0=SDA, PF1=SCL, PI9=RST
 * --------------------------------------------------------------------------- */
#define CTP_INT_GPIO_Port    GPIOF
#define CTP_INT_Pin          GPIO_PIN_10

#define CTP_SDA_GPIO_Port    GPIOF
#define CTP_SDA_Pin          GPIO_PIN_0

#define CTP_SCL_GPIO_Port    GPIOF
#define CTP_SCL_Pin          GPIO_PIN_1

#define CTP_RST_GPIO_Port    GPIOI
#define CTP_RST_Pin          GPIO_PIN_9

/* ---------------------------------------------------------------------------*
 * Electronic lock
 * PI0 = lock status (e.g. HAL_GPIO_ReadPin(LOCK_GPIO_Port, LOCK_Pin))
 * --------------------------------------------------------------------------- */
#define LOCK_GPIO_Port       GPIOI
#define LOCK_Pin             GPIO_PIN_0

/* ---------------------------------------------------------------------------*
 * Pitch motor (H-bridge + PWM)
 * PB1=FWD, PC3=BWD, PB0=PWM TIM3_CH3
 * --------------------------------------------------------------------------- */
#define FWD_GPIO_Port        GPIOB
#define FWD_Pin              GPIO_PIN_1

#define BWD_GPIO_Port        GPIOC
#define BWD_Pin              GPIO_PIN_3

#define PITCH_PWM_GPIO_Port  GPIOB
#define PITCH_PWM_Pin        GPIO_PIN_0

extern TIM_HandleTypeDef htim3;
#define PITCH_PWM_TIM        htim3
#define PITCH_PWM_CHANNEL    TIM_CHANNEL_3
/* Legacy names */
#define PWM_TIM              htim3
#define PWM_CHANNEL          TIM_CHANNEL_3

/* ---------------------------------------------------------------------------*
 * Pitch optical endstops
 * PA2=upper (45°), PA3=lower (35°)
 * --------------------------------------------------------------------------- */
#define PITCH_ENDSTOP_UPPER_GPIO_Port  GPIOA
#define PITCH_ENDSTOP_UPPER_Pin        GPIO_PIN_2

#define PITCH_ENDSTOP_LOWER_GPIO_Port  GPIOA
#define PITCH_ENDSTOP_LOWER_Pin        GPIO_PIN_3

/* ---------------------------------------------------------------------------*
 * Pitch encoder (TIM2, incremental)
 * PA0=ChA, PA1=ChB
 * e.g. HAL_TIM_Encoder_Start(&PITCH_ENC_TIM, TIM_CHANNEL_ALL);
 * --------------------------------------------------------------------------- */
#define PITCH_ENC_CHA_GPIO_Port  GPIOA
#define PITCH_ENC_CHA_Pin        GPIO_PIN_0

#define PITCH_ENC_CHB_GPIO_Port  GPIOA
#define PITCH_ENC_CHB_Pin        GPIO_PIN_1

extern TIM_HandleTypeDef htim2;
#define PITCH_ENC_TIM        htim2

/* ---------------------------------------------------------------------------*
 * Launcher
 * PH12=optical endstop, PD15=servo release (TIM4_CH4)
 * --------------------------------------------------------------------------- */
#define LAUNCHER_ENDSTOP_GPIO_Port  GPIOH
#define LAUNCHER_ENDSTOP_Pin        GPIO_PIN_12

#define LAUNCHER_SERVO_GPIO_Port   GPIOD
#define LAUNCHER_SERVO_Pin         GPIO_PIN_15

extern TIM_HandleTypeDef htim4;
#define LAUNCHER_SERVO_TIM         htim4
#define LAUNCHER_SERVO_CHANNEL     TIM_CHANNEL_4

/* ---------------------------------------------------------------------------*
 * Feeder (servos TIM4_CH1..CH3 + endstop)
 * PD12=servo1, PD13=servo2, PD14=servo3, PI5=optical endstop
 * --------------------------------------------------------------------------- */
#define FEEDER_SERVO1_GPIO_Port    GPIOD
#define FEEDER_SERVO1_Pin         GPIO_PIN_12

#define FEEDER_SERVO2_GPIO_Port   GPIOD
#define FEEDER_SERVO2_Pin         GPIO_PIN_13

#define FEEDER_SERVO3_GPIO_Port   GPIOD
#define FEEDER_SERVO3_Pin         GPIO_PIN_14

#define FEEDER_ENDSTOP_GPIO_Port  GPIOI
#define FEEDER_ENDSTOP_Pin        GPIO_PIN_5

#define FEEDER_SERVO_TIM          htim4
#define FEEDER_SERVO1_CHANNEL      TIM_CHANNEL_1
#define FEEDER_SERVO2_CHANNEL      TIM_CHANNEL_2
#define FEEDER_SERVO3_CHANNEL      TIM_CHANNEL_3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
