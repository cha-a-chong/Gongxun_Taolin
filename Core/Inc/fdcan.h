/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.h
 * @brief   This file contains all the function prototypes for
 *          the fdcan.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdbool.h>
extern uint8_t Emm_mode;
extern uint8_t RxFlag;
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void FDCAN_Send_Msg(uint8_t *msg, uint32_t len);
//u8 FDCAN1_Receive_Msg(u8 *buf);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

