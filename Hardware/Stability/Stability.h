/*
 * Stability.h
 *
 *  Created on: Dec 9, 2024
 *      Author: Taolin
 */

#ifndef STABILITY_STABILITY_H_
#define STABILITY_STABILITY_H_

#include "stm32h7xx_hal.h"
#include "Arm/Arm.h"
#include "Run/Run.h"
#include "SCSLib/SCServo.h"

extern uint8_t Zero_data;
extern uint8_t RxState;
// 使用方差判断稳定性
uint8_t Check_Stability(float check_x,float check_y,uint8_t Check_flag);
// 飞特舵机回零
uint8_t FT_Servo_Zero(uint8_t Number);
// 飞特舵机调整为向外
uint8_t FT_Servo_Orth(void);
// 发车前检查状态
uint8_t Check_Status(void);
// 飞特舵机等待回零到位
uint8_t FT_Servo_Wait_Zero(uint8_t ID);

// 菲特舵机转动，从抓取物料到车上
uint8_t FT_Servo_Put(uint8_t Servo_ID, uint8_t Pub_Number);

#endif /* STABILITY_STABILITY_H_ */
