/*
 * Tx.c
 *
 *  Created on: Nov 5, 2024
 *      Author: HU230
 */

#include "Tx.h"
#include "stm32h7xx_hal.h"
//float X_out=0;
//float Y_out=0;
//float A_out=0;
//int location_x_pid(float goal)
//{
//    static float Bias;
//	static float Integral_bias;
//	static float Last_Bias;
//	float Kp=1;
//    float Ki=0.0;
//    float Kd=0.0;


//	Bias=goal- X_NOW; //计算偏差
//	Integral_bias += Bias; //求出偏差的积分
//	X_out = Kp*Bias + Ki*Integral_bias + Kd*(Bias-Last_Bias);
//	Last_Bias = Bias; //保存上一次偏差
//
//	return X_out; //输出
//}

//int location_y_pid(float goal)
//{
//    static float Bias;
//	static float Integral_bias;
//	static float Last_Bias;
//	float Kp=1;
//    float Ki=0.0;
//    float Kd=0.0;


//	Bias=goal- Y_NOW; //计算偏差
//	Integral_bias += Bias; //求出偏差的积分
//	Y_out = Kp*Bias + Ki*Integral_bias + Kd*(Bias-Last_Bias);
//	Last_Bias = Bias; //保存上一次偏差
//	return Y_out; //输出
//}

//int location_A_pid(float goal)
//{
//    static float Bias;
//	static float Integral_bias;
//	static float Last_Bias;
//	float Kp=1;
//    float Ki=0.0;
//    float Kd=0.0;


//	Bias=goal- A_NOW; //计算偏差
//	Integral_bias += Bias; //求出偏差的积分
//	A_out = Kp*Bias + Ki*Integral_bias + Kd*(Bias-Last_Bias);
//	Last_Bias = Bias; //保存上一次偏差
//	return A_out; //输出
//}

//float x_target=0;
//float y_target=0;
//float a_target=0;

//void Move_Ctrl(float x_goal, float y_goal, float a_goal)
//{
////	x_target=x_goal;
////    y_target=y_goal;
////	a_target=a_goal;
//
//	if(location_x_pid(x_goal)>=0)
//	{
//	   Move_Left(64,64,location_x_pid(x_goal)*12.9);
//	}
//	else
//	{
//		Move_Right(64,64,-location_x_pid(x_goal)*12.9);
//	}
//
//	if(location_y_pid(y_goal)>=0)
//	{
//		Move_Line(64,64,location_y_pid(y_goal)*12.9);
//	}
//	else
//	{
//		Move_Back(64,64,-location_y_pid(y_goal)*12.9);
//	}
//	if(location_A_pid(90)>=0)
//	{
//	   Move_zhengxuzhuan(64,64,location_A_pid(a_goal)*12.9);
//	}
//	else
//	{
//		Move_fanxuzhuan(64,64,-location_A_pid(a_goal)*12.9);
//	}
//
//}

