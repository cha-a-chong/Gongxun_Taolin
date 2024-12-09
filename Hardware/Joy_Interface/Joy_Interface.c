/*
 * Joy_Interface.c
 *
 *  Created on: Nov 1, 2024
 *      Author: Taolin
 */

#include "Joy_Interface.h"
#include "Arm/Arm.h"
#include "Emm_V5/Emm_V5.h"
#include "SCSLib/SCServo.h"

float Left_axis_X = 0;
float Left_axis_Y = 0;
float Right_axis_X = 0;
float Right_axis_Y = 0;
float LT = 0;
float RT = 0;
float Hat_X = 0;
float Hat_Y = 0;

void Check_Joydata(void)
{
	static uint16_t Angle  = 2048;
	uint16_t speed = 100;
	uint16_t acc = 0;
	//如果左侧扳机按下则爪子张开，右侧扳机按下则爪子闭合

	if(Left_axis_Y < -0.4)
	{
		Emm_V5_Vel_Control(1,0,speed*(-Left_axis_Y),acc,true);
		Emm_V5_Vel_Control(2,0,speed*(-Left_axis_Y),acc,true);
		Emm_V5_Vel_Control(3,0,speed*(-Left_axis_Y),acc,true);
		Emm_V5_Vel_Control(4,0,speed*(-Left_axis_Y),acc,true);
		Emm_V5_Synchronous_motion();  // 触发多机同步开始运动
	}
	else if (Left_axis_Y > 0.4)
	{
		Emm_V5_Vel_Control(1,1,speed*Left_axis_Y,acc,true);
		Emm_V5_Vel_Control(2,1,speed*Left_axis_Y,acc,true);
		Emm_V5_Vel_Control(3,1,speed*Left_axis_Y,acc,true);
		Emm_V5_Vel_Control(4,1,speed*Left_axis_Y,acc,true);
		Emm_V5_Synchronous_motion();  // 触发多机同步开始运动
		HAL_Delay(10);
	}
	if (Left_axis_X < -0.4)
	{
		Emm_V5_Vel_Control(1,1,speed*(-Left_axis_X),acc,true);
		Emm_V5_Vel_Control(2,0,speed*(-Left_axis_X),acc,true);
		Emm_V5_Vel_Control(3,1,speed*(-Left_axis_X),acc,true);
		Emm_V5_Vel_Control(4,0,speed*(-Left_axis_X),acc,true);
		Emm_V5_Synchronous_motion();  // 触发多机同步开始运动
		HAL_Delay(10);
	}
	else if (Left_axis_X > 0.4)
	{
		Emm_V5_Vel_Control(1,0,speed*Left_axis_X,acc,true);
		Emm_V5_Vel_Control(2,1,speed*Left_axis_X,acc,true);
		Emm_V5_Vel_Control(3,0,speed*Left_axis_X,acc,true);
		Emm_V5_Vel_Control(4,1,speed*Left_axis_X,acc,true);
		Emm_V5_Synchronous_motion();  // 触发多机同步开始运动
		HAL_Delay(10);
	}
	if ((Left_axis_X == 0) && (Left_axis_Y == 0) )
	{
		Emm_V5_Stop_Now(1,true);
		Emm_V5_Stop_Now(2,true);
		Emm_V5_Stop_Now(3,true);
		Emm_V5_Stop_Now(4,true);
		Emm_V5_Synchronous_motion();
	}
	if(LT > 0.4)
	{
		Move_Arm(1,500,500);
	}
	else if(RT > 0.4)
	{
		Move_Arm(1,800,500);
	}
	if(Hat_Y > 0.4)
	{
		Emm_V5_Pos_Control(5, 0, 128, 32, 1024, 0, false);
		HAL_Delay(50);
	}
	else if (Hat_Y < -0.4)
	{
		Emm_V5_Pos_Control(5, 1, 128, 32, 1024, 0, false);
		HAL_Delay(50);
	}
	if(Hat_X > 0.4)
	{
		Angle += 100;
	}
	else if (Hat_X < -0.4)
	{
		Angle -= 100;
	}
	WritePosEx(1, Angle, 200, 50);
}

// 在串口中断函数中调用这个函数, 处理接收到的手柄数据
void Recive_Joydata(uint8_t rec)
{
//	静态变量存储数据
	static uint8_t ch;
//	共用体提供数据
	static union
	{
	//		+0.0 是4个字符，一共有八个这样的，就是32个字节，然后每个都需要插入'\0'，所以数组大小是32+8 = 40
		uint8_t date[40];
	//		存放八个数据的字符串，每个长度为5字节
		char ActVal[8][5];
	} Recive;
	static uint8_t count = 0;
	static uint8_t i = 0;
	static char *endpointer;
	ch = rec;
//	状态机
	switch (count)
	{
//	接收帧头, 准备开始接收数据
	case 0:
		if (ch == 0x0d)
			count++;
		else
			count = 0;
		break;
	case 1:
		if (ch == 0x0a)
		{
			i = 0;
			count++;
		}
		else if (ch == 0x0d)
			;
		else
			count = 0;
		break;
	case 2:
		if((i == 4) || (i == 9) || (i == 14) || (i == 19) || (i == 24) || (i == 29) || (i == 34))
		{
			Recive.date[i] = '\0';
			Recive.date[i+1] = ch;
			i = i+2;
		}
		else if(i == 38)
		{
			Recive.date[i] = ch;
			Recive.date[39] = '\0';
			count++;
			i = 0;
			break;
		}
		else
		{
			Recive.date[i] = ch;
			i++;
		}
		break;
	case 3:
		if (ch == 0x0a)
		{
			Left_axis_X = strtof(Recive.ActVal[0], &endpointer);
			Left_axis_Y = strtof(Recive.ActVal[1], &endpointer);
			Right_axis_X = strtof(Recive.ActVal[2], &endpointer);
			Right_axis_Y = strtof(Recive.ActVal[3], &endpointer);
			count++;
		}
		else
			count = 0;
		break;
	case 4:
		if (ch == 0x0d)
		{
			LT = strtof(Recive.ActVal[4], &endpointer);
			RT = strtof(Recive.ActVal[5], &endpointer);
			Hat_X = strtof(Recive.ActVal[6], &endpointer);
			Hat_Y = strtof(Recive.ActVal[7], &endpointer);
			count = 0;
		}
		break;
	default:
		count = 0;
		break;
	}
}
