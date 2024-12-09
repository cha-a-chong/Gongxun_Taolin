/*
 * Run.c
 *
 *  Created on: Nov 1, 2024
 *      Author: Taolin
 */

#include "Run.h"

#define RunSpeed 100
#define RunAcc 120

int time5_jiancha = 0;

uint8_t GetRxFlag(void) {
	if (RxFlag == 1) {
		RxFlag = 0;
		return 1;
	}
	return 0;
}

/**
 * @brief 实现四个电机同步向前运动 Emm_V5_Pos_Control(1, 1, speed, acc, length, 0, true)
 * @param  第一列 电机地址
 * @param  第二列 方向 ，0为CW，其余值为CCW
 * @param  第三列 速度 ，范围0 - 5000RPM
 * @param  第四列 加速度 ，范围0 - 255，注意：0是直接启动
 * @param  第五列 脉冲数 ，范围0- (2^32 - 1)个
 * @param  第六列 相位/绝对标志，false为相对运动，true为绝对值运动
 * @param  第七列 snF ：多机同步标志 ，false为不启用，true为启用
 * @retval
 */
////0正传，1反转
////1.8度电机,16细分，3200个脉冲，电机转动一圈
////曲线加减速时间计算公式：t2 - t1 = (256 - acc) * 50(us)，Vt2 = Vt1 + 1(RPM)；
////（注：acc 为加速度档位，Vt1为 t1 时刻的转速，，Vt2为 t2 时刻的转速）
/////////////////////////////////////////////////////////////////////////////////////////////
//后续根据实物再进行调整
void Move_Line(u16 speed, u8 acc, u32 length)  //初始设定130.5个脉冲是一厘米
{
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(1, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(2, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(3, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(4, 0, speed, acc, length, 0, true);

	Emm_V5_Synchronous_motion();  // 触发多机同步开始运动
	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0)&& (time5_jiancha != 0) )
		;
//	RxFlag = 0;·
	HAL_Delay(30);
}

void Move_Left(u16 speed, u8 acc, u32 length) {
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(1, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(2, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(3, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(4, 0, speed, acc, length, 0, true);

	Emm_V5_Synchronous_motion(); // 触发多机同步开始运动

	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
//	RxFlag = 0;
	HAL_Delay(30);
}

void Move_Back(u16 speed, u8 acc, u32 length) {
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(1, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(2, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(3, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(4, 1, speed, acc, length, 0, true);

	Emm_V5_Synchronous_motion(); // 触发多机同步开始运动


	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
//	RxFlag = 0;
	HAL_Delay(30);
}

void Move_Right(u16 speed, u8 acc, u32 length) {
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(1, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(2, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(3, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(4, 1, speed, acc, length, 0, true);

	Emm_V5_Synchronous_motion();


	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
//	RxFlag = 0;
	HAL_Delay(30);
}
void Move_zhengxuzhuan(u16 speed, u8 acc, u32 length) {
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(1, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(2, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(3, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(4, 0, speed, acc, length, 0, true);

	Emm_V5_Synchronous_motion(); // 触发多机同步开始运动

	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
//	RxFlag = 0;
	HAL_Delay(30);
}

void Move_fanxuzhuan(u16 speed, u8 acc, u32 length) //4340是90度
{
	int time;
	time = length / speed * 20;  //毫秒

	Emm_V5_Pos_Control(1, 1, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(2, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(3, 0, speed, acc, length, 0, true);
	Emm_V5_Pos_Control(4, 1, speed, acc, length, 0, true);

	Emm_V5_Synchronous_motion(); // 触发多机同步开始运动


	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
//	RxFlag = 0;
	HAL_Delay(30);
}
uint16_t High_Length=0;
//void Move_Xiajiang(u16 speed, u8 acc, u32 length) {
//	int time;
//	time = length / speed * 20;  //毫秒
//	High_Length+=length;
//	Emm_V5_Pos_Control(5, 1, speed, acc, length, 1, 0);
//
//
//	Emm_mode = 1;
//	time5_jiancha = (time + 800) / 10;
//	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
//		;
////	RxFlag = 0;
//	if(High_Length>=14000)
//		Emm_V5_Stop_Now(5,0);
//	HAL_Delay(30);
//}
void Drop_Location_jiang(u16 speed, u8 acc, u32 length) {
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(5, 1, speed, acc, length, 1, 0);
//	检查到位回读
	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
	HAL_Delay(30);
}
void Drop_Location_Sheng(u16 speed, u8 acc, u32 length) {
	int time;
	time = length / speed * 20;  //毫秒
	Emm_V5_Pos_Control(5, 1, speed, acc, length, 1, 0);
	Emm_mode = 1;
	time5_jiancha = (time + 800) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;

	HAL_Delay(30);
}
/////////////////////////////////////////////////////////////////////////////////////

/***********将当前位置清零**********************/
void motor_clear_all(void) {
	Emm_V5_Reset_CurPos_To_Zero(1);
	Emm_V5_Reset_CurPos_To_Zero(2);
	Emm_V5_Reset_CurPos_To_Zero(3);
	Emm_V5_Reset_CurPos_To_Zero(4);
}
/*********************************************/

/******************______速度模式控制______***************************/
void Move_Val_Line(u16 speed, u8 acc)
{
	Emm_V5_Vel_Control(1, 0, speed,acc,true);
	Emm_V5_Vel_Control(2, 0, speed,acc,true);
	Emm_V5_Vel_Control(3, 0, speed,acc,true);
	Emm_V5_Vel_Control(4, 0, speed,acc,true);
	Emm_V5_Synchronous_motion();  // 触发多机同步开始运动

}
void Move_Val_Back(u16 speed, u8 acc)
{
	Emm_V5_Vel_Control(1, 1, speed,acc,true);
	Emm_V5_Vel_Control(2, 1, speed,acc,true);
	Emm_V5_Vel_Control(3, 1, speed,acc,true);
	Emm_V5_Vel_Control(4, 1, speed,acc,true);
	Emm_V5_Synchronous_motion();  // 触发多机同步开始运动

}
void Move_Val_Left(u16 speed, u8 acc)
{
	Emm_V5_Vel_Control(1, 1, speed,acc,true);
	Emm_V5_Vel_Control(2, 0, speed,acc,true);
	Emm_V5_Vel_Control(3, 1, speed,acc,true);
	Emm_V5_Vel_Control(4, 0, speed,acc,true);
	Emm_V5_Synchronous_motion();  // 触发多机同步开始运动

}
void Move_Val_Right(u16 speed, u8 acc)
{
	Emm_V5_Vel_Control(1, 0, speed,acc,true);
	Emm_V5_Vel_Control(2, 1, speed,acc,true);
	Emm_V5_Vel_Control(3, 0, speed,acc,true);
	Emm_V5_Vel_Control(4, 1, speed,acc,true);
	Emm_V5_Synchronous_motion();  // 触发多机同步开始运动

}

void Move_Stop(void)
{
	Emm_V5_Stop_Now(1,true);
	Emm_V5_Stop_Now(2,true);
	Emm_V5_Stop_Now(3,true);
	Emm_V5_Stop_Now(4,true);
	Emm_V5_Synchronous_motion();  // 触发多机同步开始运动
}
/*********************_______END_________************************/
void Move_GO(void) {
	int time;
	time = 5000 / RunSpeed * 20;  //毫秒
	Emm_V5_Pos_Control(1, 1, 0, 0, 0, 0, true);
	Emm_V5_Pos_Control(2, 0, RunSpeed, RunAcc, 5000, 0, true);
	Emm_V5_Pos_Control(3, 1, 0, 0, 0, 0, true);
	Emm_V5_Pos_Control(4, 0, RunSpeed, RunAcc, 5000, 0, true);

	Emm_V5_Synchronous_motion();

	RxFlag = 0;
	Emm_mode = 1;
	time5_jiancha = (time - 500) / 10;
	while ((GetRxFlag() == 0) && (time5_jiancha != 0))
		;
	HAL_Delay(30);

}
extern void Start(void);
void Move_TO_Saomaqu(float Leftclk, float Lineclk) {
	Move_Left(RunSpeed, RunAcc, Leftclk);                                                                                         //爪子目前在最低处
	HAL_Delay(20);
	HAL_Delay(yanshi);

	Move_Line(RunSpeed, RunAcc, Lineclk); ////130.5
	HAL_Delay(yanshi);
}
void Move_TO_yuanliaoqu(float Lineclk) //物料区
{
	Move_Line(RunSpeed, RunAcc, Lineclk);
	HAL_Delay(yanshi);
}

void Move_TO_jianzhi1(float Backclk, float fanzhuanclk) //粗加工区
{
	Move_Back(RunSpeed, RunAcc, Backclk);
	HAL_Delay(yanshi);
	Move_fanxuzhuan(RunSpeed, RunAcc, fanzhuanclk);
	HAL_Delay(yanshi);
}

void Move_TO_zancunqu(float Lineclk, float fanzhuanclk) //暂存区
{
	Move_Line(RunSpeed, RunAcc, Lineclk);
	HAL_Delay(yanshi);
	Move_fanxuzhuan(RunSpeed, RunAcc, fanzhuanclk);
	HAL_Delay(yanshi);
}
void Move_TO_jianzhi2(float Backclk, float Zhengzhuanclk) //从暂存区回到原料区
{
	Move_Back(RunSpeed, RunAcc, Backclk);
	HAL_Delay(yanshi);
	Move_zhengxuzhuan(RunSpeed, RunAcc, Zhengzhuanclk);
	HAL_Delay(yanshi);
}
void Move_TO_cujiagongqu(float Backclk) //粗加工区
{
	Move_Back(RunSpeed, RunAcc, Backclk);
	HAL_Delay(yanshi);
}
void Move_TO_jianzhi3(float Backclk, float Zhengzhuanclk) //从暂存区回到原料区
{
	Move_Back(RunSpeed, RunAcc, Backclk);
	HAL_Delay(yanshi);
	Move_zhengxuzhuan(RunSpeed, RunAcc, Zhengzhuanclk);
	HAL_Delay(yanshi);
}
void Move_TO_fanyuanliaoqu(float Lineclk) //物料区
{
	Move_Back(RunSpeed, RunAcc, Lineclk);
	HAL_Delay(20000);
}

void Move_Action_Left_Ctrl(void) {

	if (X_out > 0) {
		Move_Left(100, 100, X_out);
		HAL_Delay(yanshi);
	} else {
		Move_Right(100, 100, -X_out);
		HAL_Delay(yanshi);
	}
	if (Y_out > 0) {
		Move_Line(100, 100, Y_out);
		HAL_Delay(yanshi);
	} else {
		Move_Back(100, 100, -Y_out);
		HAL_Delay(yanshi);
	}
}

void Move_Action_Forward_Ctrl(void) {

	if (X_out > 0) {
		Move_Line(100, 100, X_out);
		HAL_Delay(yanshi);
	} else {
		Move_Back(100, 100, -X_out);
		HAL_Delay(yanshi);
	}
	if (Y_out > 0) {
		Move_Right(100, 100, Y_out);
		HAL_Delay(yanshi);
	} else {
		Move_Left(100, 100, -Y_out);
		HAL_Delay(yanshi);
	}

}
void Move_Action_Right_Ctrl(void) {
	if (X_out > 0) {
		Move_Right(100, 100, X_out);
		HAL_Delay(yanshi);
	} else {
		Move_Left(100, 100, -X_out);
		HAL_Delay(yanshi);
	}
	if (Y_out > 0) {
		Move_Back(100, 100, Y_out);
		HAL_Delay(yanshi);
	} else {
		Move_Line(100, 100, -Y_out);
		HAL_Delay(yanshi);
	}
}
#define		ABS(x)		((x) > 0 ? (x) : -(x))
void put_location(float x_goal, float y_goal) {
	if ((X_NOW - x_goal) < 0) {

		Move_Right(100, 100, ABS(X_NOW - x_goal) * 13.8);
		HAL_Delay(yanshi);
	} else {
		Move_Left(100, 100, ABS(X_NOW - x_goal) * 13.8);
		HAL_Delay(yanshi);
	}
	if ((Y_NOW - y_goal) < 0) {
		Move_Back(100, 100, ABS(Y_NOW - y_goal) * 13.8);
		HAL_Delay(yanshi);
	} else {
		Move_Line(100, 100, ABS(Y_NOW - y_goal) * 13.8);
		HAL_Delay(yanshi);
	}
}
void Move_Action_Nopid_Left_Ctrl(float x_goal, float y_goal) {
	while (1) {
		if ((X_NOW - x_goal) < 0) {
			Move_Left(100, 100, ABS(X_NOW - x_goal) * 13.8);
			HAL_Delay(yanshi);
		} else {
			Move_Right(100, 100, ABS(X_NOW - x_goal) * 13.8);
			HAL_Delay(yanshi);
		}
		if (Y_NOW - y_goal < 0) {
			Move_Line(100, 100, ABS(Y_NOW - y_goal) * 13.8);
			HAL_Delay(yanshi);
		} else {
			Move_Back(100, 100, ABS(Y_NOW - y_goal) * 13.8);
			HAL_Delay(yanshi);
		}
		if ( ABS(X_NOW - x_goal) < 2 && ABS(Y_NOW - y_goal) < 2) {
			break; /* 到达目标 */
		}
	}
}
void Move_Action_Nopid_Forward_Ctrl(float x_goal, float y_goal) {
	//ACTION调整
	while (1) {

		if ((X_NOW - x_goal) < 0) {
			Move_Line(100, 100, ABS(X_NOW - x_goal) * 13.8);
			HAL_Delay(yanshi);
		} else {
			Move_Back(100, 100, ABS(X_NOW - x_goal) * 13.8);
			HAL_Delay(yanshi);
		}
		if ((Y_NOW - y_goal) < 0) {
			Move_Right(100, 100, ABS(Y_NOW - y_goal) * 13.8);
			HAL_Delay(yanshi);
		} else {
			Move_Left(100, 100, ABS(Y_NOW - y_goal) * 13.8);
			HAL_Delay(yanshi);
		}
		if ( ABS(X_NOW - x_goal) < 5 && ABS(Y_NOW - y_goal) < 5) {
			break; /* 到达目标 */
		}
	}
}

void Move_Action_Nopid_Right_Ctrl(float x_goal, float y_goal) {
	while (1) {
		if ((X_NOW - x_goal) < 0) {

			Move_Right(100, 100, ABS(X_NOW - x_goal) * 13.8);
			HAL_Delay(yanshi);
		} else {
			Move_Left(100, 100, ABS(X_NOW - x_goal) * 13.8);
			HAL_Delay(yanshi);
		}
		if ((Y_NOW - y_goal) < 0) {
			Move_Back(100, 100, ABS(Y_NOW - y_goal) * 13.8);
			HAL_Delay(yanshi);
		} else {
			Move_Line(100, 100, ABS(Y_NOW - y_goal) * 13.8);
			HAL_Delay(yanshi);
		}
		if ( ABS(X_NOW - x_goal) < 5 && ABS(Y_NOW - y_goal) < 5) {
			break; /* 到达目标 */
		}
	}
}

/* Tx姿态调整 */
uint16_t time_tx=0;
void Move_Tx_Pid_Ctrl(float TX_X_Goal,float TX_Y_Goal) {

	while (1) {
	if (TX_Y_out >= 0) {
			Move_Right(80, 80, TX_Y_out * 5);
			HAL_Delay(5);
		} else {
			Move_Left(80, 80, -TX_Y_out * 5);
			HAL_Delay(5);
		}

		if (TX_X_out >= 0) {
			Move_Back(80, 80, TX_X_out * 5);
			HAL_Delay(5);
		} else {
			Move_Line(80, 80, -TX_X_out * 5);
			HAL_Delay(5);
		}

		if (ABS(Tx_NOW - 370.5) < 5 && ABS(Ty_NOW -230.5) < 5) {
			break;
		}
		if(time_tx>=250)
		{time_tx=0;
			break;
		}
	}

}

void Move_Tx_NO_Pid_Ctrl(float x2 ,float y2) {
	while (1) {
	if ((Tx_NOW - x2)< 0) {
			Move_Right(80, 80, ABS(Tx_NOW - x2) * 10);
			HAL_Delay(yanshi);
		} else {
			Move_Left(80, 80, ABS(Tx_NOW - x2) * 10);
			HAL_Delay(yanshi);
		}

		if ((Ty_NOW - y2) < 0) {
			Move_Back(80, 80, ABS(Ty_NOW - y2) * 10);
			HAL_Delay(yanshi);
		} else {
			Move_Line(80, 80, ABS(Ty_NOW - y2) * 10 );
			HAL_Delay(yanshi);
		}

		if (ABS(Tx_NOW - x2) < 2 && ABS(Ty_NOW - y2) < 2) {
			break;
		}
	}
}


void Move_Tx_Val_Ctrl(float* tx_target,float* ty_target,float x1,float y1) {
	 if (tx_target != NULL && ty_target != NULL)
	 {
	*tx_target = x1;
	*ty_target = y1;
	 }
	while (1) {
	if (TX_Y_out >= 0) {
			Move_Val_Right(50, 50);
			HAL_Delay(yanshi);
		} else {
			Move_Val_Left(50, 50);
			HAL_Delay(yanshi);
		}

		if (TX_X_out >= 0) {
			Move_Val_Back(50, 50);
			HAL_Delay(yanshi);
		} else {
			Move_Val_Line(50, 50);
			HAL_Delay(yanshi);
		}
		if (ABS(Tx_NOW - x1) < 5 && ABS(Ty_NOW - y1) < 5) {
			Move_Stop();
			break;
		}
	}
}

void Move_Tx_Ctrl(void){
	if (TX_Y_out >= 0) {
				Move_Right(80, 80, TX_Y_out * 2);
				HAL_Delay(yanshi);
			} else {
				Move_Left(80, 80, -TX_Y_out * 2);
				HAL_Delay(yanshi);
			}

			if (TX_X_out >= 0) {
				Move_Back(80, 80, TX_X_out * 2);
				HAL_Delay(yanshi);
			} else {
				Move_Line(80, 80, -TX_X_out * 2);
				HAL_Delay(yanshi);
			}


}

