/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define DIO_SCK_Pin GPIO_PIN_2
#define DIO_SCK_GPIO_Port GPIOE
#define DIO_NSS_Pin GPIO_PIN_4
#define DIO_NSS_GPIO_Port GPIOE
#define DIO_MOSO_Pin GPIO_PIN_5
#define DIO_MOSO_GPIO_Port GPIOE
#define DIO_MOSI_Pin GPIO_PIN_6
#define DIO_MOSI_GPIO_Port GPIOE
#define USER_BTN_Pin GPIO_PIN_13
#define USER_BTN_GPIO_Port GPIOC
#define MISC_SDA_Pin GPIO_PIN_0
#define MISC_SDA_GPIO_Port GPIOF
#define MISC_SCL_Pin GPIO_PIN_1
#define MISC_SCL_GPIO_Port GPIOF
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define A4_SDA_Pin GPIO_PIN_5
#define A4_SDA_GPIO_Port GPIOF
#define U7_TX_Pin GPIO_PIN_7
#define U7_TX_GPIO_Port GPIOF
#define SONAR_ECHO7_Pin GPIO_PIN_9
#define SONAR_ECHO7_GPIO_Port GPIOF
#define A5_SCL_Pin GPIO_PIN_10
#define A5_SCL_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_3
#define A2_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_0
#define SERVO1_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define A0_Pin GPIO_PIN_3
#define A0_GPIO_Port GPIOA
#define LENCODER_B_Pin GPIO_PIN_4
#define LENCODER_B_GPIO_Port GPIOA
#define D13_SCK_Pin GPIO_PIN_5
#define D13_SCK_GPIO_Port GPIOA
#define D12_MISO_Pin GPIO_PIN_6
#define D12_MISO_GPIO_Port GPIOA
#define D11_PWM_MOSI_Pin GPIO_PIN_7
#define D11_PWM_MOSI_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define GRN_LED_Pin GPIO_PIN_0
#define GRN_LED_GPIO_Port GPIOB
#define RMOTOR__Pin GPIO_PIN_1
#define RMOTOR__GPIO_Port GPIOB
#define LEDS_MOSI_Pin GPIO_PIN_2
#define LEDS_MOSI_GPIO_Port GPIOB
#define U7_TXE7_Pin GPIO_PIN_7
#define U7_TXE7_GPIO_Port GPIOE
#define SONAR2_ECHO_Pin GPIO_PIN_8
#define SONAR2_ECHO_GPIO_Port GPIOE
#define D6_PWM_Pin GPIO_PIN_9
#define D6_PWM_GPIO_Port GPIOE
#define SONAR3_ECHO_Pin GPIO_PIN_10
#define SONAR3_ECHO_GPIO_Port GPIOE
#define D5_PWM_Pin GPIO_PIN_11
#define D5_PWM_GPIO_Port GPIOE
#define SONAR4_ECHO_Pin GPIO_PIN_12
#define SONAR4_ECHO_GPIO_Port GPIOE
#define D3_PWM_Pin GPIO_PIN_13
#define D3_PWM_GPIO_Port GPIOE
#define SONAR5_ECHO_Pin GPIO_PIN_14
#define SONAR5_ECHO_GPIO_Port GPIOE
#define SONAR6_ECHO_Pin GPIO_PIN_15
#define SONAR6_ECHO_GPIO_Port GPIOE
#define SERVO3_Pin GPIO_PIN_10
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_11
#define SERVO4_GPIO_Port GPIOB
#define U5_TX_Pin GPIO_PIN_12
#define U5_TX_GPIO_Port GPIOB
#define U5_RX_Pin GPIO_PIN_13
#define U5_RX_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define U1_TX_Pin GPIO_PIN_15
#define U1_TX_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define LENCODER_A_Pin GPIO_PIN_12
#define LENCODER_A_GPIO_Port GPIOD
#define LIDAR_PWM_Pin GPIO_PIN_13
#define LIDAR_PWM_GPIO_Port GPIOD
#define D10_PWM_NSS_Pin GPIO_PIN_14
#define D10_PWM_NSS_GPIO_Port GPIOD
#define D9_PWM_Pin GPIO_PIN_15
#define D9_PWM_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define RENCODER_A_Pin GPIO_PIN_6
#define RENCODER_A_GPIO_Port GPIOC
#define RENCODER_B_Pin GPIO_PIN_7
#define RENCODER_B_GPIO_Port GPIOC
#define RMOTOR_C8_Pin GPIO_PIN_8
#define RMOTOR_C8_GPIO_Port GPIOC
#define A4_SDAC9_Pin GPIO_PIN_9
#define A4_SDAC9_GPIO_Port GPIOC
#define USB_VBUS__Pin GPIO_PIN_9
#define USB_VBUS__GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LEDS_NSS_Pin GPIO_PIN_15
#define LEDS_NSS_GPIO_Port GPIOA
#define LEDS_SCK_Pin GPIO_PIN_10
#define LEDS_SCK_GPIO_Port GPIOC
#define LEDS_MISO_Pin GPIO_PIN_11
#define LEDS_MISO_GPIO_Port GPIOC
#define U2_TX_Pin GPIO_PIN_5
#define U2_TX_GPIO_Port GPIOD
#define U2_RX_Pin GPIO_PIN_6
#define U2_RX_GPIO_Port GPIOD
#define D0_RX_Pin GPIO_PIN_9
#define D0_RX_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define D1_TX_Pin GPIO_PIN_14
#define D1_TX_GPIO_Port GPIOG
#define SERVO2_Pin GPIO_PIN_3
#define SERVO2_GPIO_Port GPIOB
#define LMOTOR__Pin GPIO_PIN_4
#define LMOTOR__GPIO_Port GPIOB
#define LMOTOR_B5_Pin GPIO_PIN_5
#define LMOTOR_B5_GPIO_Port GPIOB
#define U1_RX_Pin GPIO_PIN_6
#define U1_RX_GPIO_Port GPIOB
#define BLUE_LED_Pin GPIO_PIN_7
#define BLUE_LED_GPIO_Port GPIOB
#define D15_Pin GPIO_PIN_8
#define D15_GPIO_Port GPIOB
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOB
#define SONAR1_ECHO_Pin GPIO_PIN_0
#define SONAR1_ECHO_GPIO_Port GPIOE
#define LENCODER_BE1_Pin GPIO_PIN_1
#define LENCODER_BE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
