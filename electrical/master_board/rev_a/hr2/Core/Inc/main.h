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
#define FPGA_RX_Pin GPIO_PIN_7
#define FPGA_RX_GPIO_Port GPIOF
#define SERVO_POS_Pin GPIO_PIN_8
#define SERVO_POS_GPIO_Port GPIOF
#define ECHO2_Pin GPIO_PIN_9
#define ECHO2_GPIO_Port GPIOF
#define A5_SCL_Pin GPIO_PIN_10
#define A5_SCL_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define SERVO_CUR_Pin GPIO_PIN_2
#define SERVO_CUR_GPIO_Port GPIOC
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
#define LQUAD_B_Pin GPIO_PIN_4
#define LQUAD_B_GPIO_Port GPIOA
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
#define RMOTOR_CTL2_Pin GPIO_PIN_1
#define RMOTOR_CTL2_GPIO_Port GPIOB
#define LEDS_MOSI_Pin GPIO_PIN_2
#define LEDS_MOSI_GPIO_Port GPIOB
#define FPGA_TX_Pin GPIO_PIN_7
#define FPGA_TX_GPIO_Port GPIOE
#define ECHO7_Pin GPIO_PIN_8
#define ECHO7_GPIO_Port GPIOE
#define D6_PWM_Pin GPIO_PIN_9
#define D6_PWM_GPIO_Port GPIOE
#define ECHO3_Pin GPIO_PIN_10
#define ECHO3_GPIO_Port GPIOE
#define D5_PWM_Pin GPIO_PIN_11
#define D5_PWM_GPIO_Port GPIOE
#define ECHO4_Pin GPIO_PIN_12
#define ECHO4_GPIO_Port GPIOE
#define D3_PWM_Pin GPIO_PIN_13
#define D3_PWM_GPIO_Port GPIOE
#define ECHO5_Pin GPIO_PIN_14
#define ECHO5_GPIO_Port GPIOE
#define ECHO6_Pin GPIO_PIN_15
#define ECHO6_GPIO_Port GPIOE
#define SERVO3_Pin GPIO_PIN_10
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_11
#define SERVO4_GPIO_Port GPIOB
#define LIDAR_TX_Pin GPIO_PIN_12
#define LIDAR_TX_GPIO_Port GPIOB
#define LIDAR_RX_Pin GPIO_PIN_13
#define LIDAR_RX_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define SBC_TX_Pin GPIO_PIN_15
#define SBC_TX_GPIO_Port GPIOB
#define STL_RX_Pin GPIO_PIN_8
#define STL_RX_GPIO_Port GPIOD
#define STL_TX_Pin GPIO_PIN_9
#define STL_TX_GPIO_Port GPIOD
#define LQUAD_A_Pin GPIO_PIN_12
#define LQUAD_A_GPIO_Port GPIOD
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
#define RQUAD_A_Pin GPIO_PIN_6
#define RQUAD_A_GPIO_Port GPIOC
#define RQUAD_B_Pin GPIO_PIN_7
#define RQUAD_B_GPIO_Port GPIOC
#define RMOTOR_CTL1_Pin GPIO_PIN_8
#define RMOTOR_CTL1_GPIO_Port GPIOC
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
#define WOW_EN_Pin GPIO_PIN_12
#define WOW_EN_GPIO_Port GPIOC
#define SBC_ALIVE_Pin GPIO_PIN_0
#define SBC_ALIVE_GPIO_Port GPIOD
#define INT1_Pin GPIO_PIN_1
#define INT1_GPIO_Port GPIOD
#define ECHO1_Pin GPIO_PIN_2
#define ECHO1_GPIO_Port GPIOD
#define ESTOP_CLR_Pin GPIO_PIN_3
#define ESTOP_CLR_GPIO_Port GPIOD
#define WOW_RX_Pin GPIO_PIN_5
#define WOW_RX_GPIO_Port GPIOD
#define WOW_TX_Pin GPIO_PIN_6
#define WOW_TX_GPIO_Port GPIOD
#define ESTOP_Pin GPIO_PIN_7
#define ESTOP_GPIO_Port GPIOD
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
#define LMOTOR_CTL1_Pin GPIO_PIN_4
#define LMOTOR_CTL1_GPIO_Port GPIOB
#define LMOTOR_CTL2_Pin GPIO_PIN_5
#define LMOTOR_CTL2_GPIO_Port GPIOB
#define SBC_RX_Pin GPIO_PIN_6
#define SBC_RX_GPIO_Port GPIOB
#define BLUE_LED_Pin GPIO_PIN_7
#define BLUE_LED_GPIO_Port GPIOB
#define D15_Pin GPIO_PIN_8
#define D15_GPIO_Port GPIOB
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOB
#define INT0_Pin GPIO_PIN_0
#define INT0_GPIO_Port GPIOE
#define LQUAD_BE1_Pin GPIO_PIN_1
#define LQUAD_BE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
