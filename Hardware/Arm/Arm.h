/*
 * Arm.h
 *
 *  Created on: Nov 1, 2024
 *      Author: Taolin
 */

#ifndef ARM_ARM_H_
#define ARM_ARM_H_

#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03

#include "main.h"
#include "usart.h"
#include "SCSLib/SCServo.h"
#include "SCSLib/SCS_Uart.h"
#include "Emm_V5/Emm_V5.h"
#include "Run/Run.h"
#define jixiebi_usart huart5





void Move_Arm(uint8_t servoID, int16_t Position, uint16_t Time);
void catch_Frist(uint16_t Pos1, uint16_t Pos2);
void catch_Second(uint16_t Pos1, uint16_t Pos2);
void Start(void);
void put(uint16_t Pos1, uint16_t Pos2);


/*************************第一次抓取***********************************************/
void Frist_Grab_Wuliao(void);
void put_wuliao_to_circular_frist(void);
void put_wuliao_to_Car_frist(void);
void put_wuliao_to_circular_second(void);
/*************************第二次抓取***********************************************/
void Second_Run_Frist_Grab_Wuliao(void);
void Second_Run_put_wuliao_to_Car_frist(void);
void Second_Run_put_wuliao_to_circular_frist(void);
void Second_Run_put_wuliao_to_circular_second(void);

#endif /* ARM_ARM_H_ */
