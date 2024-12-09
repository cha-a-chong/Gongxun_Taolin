#include "arm.h"
#include "Stability/Stability.h"
extern int frist_wuliao, second_wuliao, three_wuliao, wuliao_falg;
extern int colour;
int Grab_count;
#define time 1000

//爪子落地 步进最低，爪子 780
#define Arm_close 20
#define Arm_low 600
#define Arm_high 1000
#define Emm_Speed  50     //步进电机
#define Emm_Acc  50
#define Max_High 12000
//控制单个舵机运动
//参数：sevoID:舵机ID，Position:转动角度（这里以270°为例）,Time:转动时间
void Move_Arm(uint8_t servoID, int16_t Position, uint16_t Time) {
	uint8_t TxBuf[11];
	if (servoID > 31 || !(Time > 0) || Position > 32767) {
		return;
	}
	TxBuf[0] = FRAME_HEADER;
	TxBuf[1] = FRAME_HEADER;
	TxBuf[2] = 8;
	TxBuf[3] = CMD_SERVO_MOVE;
	TxBuf[4] = 1;
	TxBuf[5] = (uint8_t) Time;
	TxBuf[6] = (uint8_t) (Time >> 8);
	TxBuf[7] = servoID;
	TxBuf[8] = (uint8_t) Position;
	TxBuf[9] = (uint8_t) (Position >> 8);

	HAL_UART_Transmit(&jixiebi_usart, TxBuf, 10, 0xffff);
	HAL_Delay(5);

}

void QR(void) {

}
//步进上升1800  下降1800
extern float x, y;
void catch_Frist(uint16_t Pos1, uint16_t Pos2) //5400
{
	WritePosEx(0, Pos1, 500, 50); //物料底盘移动

	Move_Arm(1, Arm_close, time); //爪子闭合
	HAL_Delay(time);
	Drop_Location_Sheng(Emm_Speed, Emm_Acc, 500);
//	HAL_Delay(3000);
	WritePosEx(1, Pos2, 500, 50); //爪子底盘向左
	HAL_Delay(3000);
//	if(x-317.5>=50&&y-230.5>=50)
//	{
//		WritePosEx(1, 4045, 500, 50); //爪子底盘向左
//		HAL_Delay(1500);
//		Grab_count--;
//	}
	Move_Arm(6, 550, time); //爪子下降
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 4500);
	Move_Arm(1, 190, time); //爪子张开
	HAL_Delay(time);
	Move_Arm(6, 1000, time); //爪子上升
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 500);

	WritePosEx(1, 4050, 500, 50); //爪子底盘向左
	HAL_Delay(1500);
	Move_Arm(1, 500, time); //爪子张开
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 8600);
}
//上升1800，下降1800
void catch_Second(uint16_t Pos1, uint16_t Pos2) //从地上抓取到车上
{

	WritePosEx(0, Pos1, 500, 50); //物料底盘移动
	Move_Arm(1, 500, time); //爪子张开
	HAL_Delay(time);
	Move_Arm(6, 360, time); //爪子下降
	HAL_Delay(time);

	Move_Arm(1, Arm_close, time); //爪子闭合
	HAL_Delay(time);
	Move_Arm(6, 1000, time); //爪子上升
	HAL_Delay(time);
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 1000);
	WritePosEx(1, Pos2, 500, 50); //爪子底盘向左
	HAL_Delay(5000);
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 5000);
	Move_Arm(6, 550, time); //爪子下降
	HAL_Delay(time);
	Move_Arm(1, 190, time); //爪子张开
	HAL_Delay(time);
	Move_Arm(6, 1000, time); //爪子上升
	HAL_Delay(time);
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 5000);
	WritePosEx(1, 4045, 500, 50); //爪子底盘向左
	HAL_Delay(1500);
	Move_Arm(1, 500, time); //爪子张开
	HAL_Delay(time);
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 11000);

}

extern int QR_Flag;
void Start(void) //下降14000
{
//	先将爪子升起来，避免打到TX2或者物料盘
	Drop_Location_jiang(200, 120, 4000);
//	将舵机向外转动
	FT_Servo_Orth();
//	HAL_Delay(2000);
	Drop_Location_jiang(200, 120, 12000);
	Move_Arm(1, 500, 300); //爪子张开
	while (!QR_Flag)
		;  //等待扫码完成
//	扫码完成后将爪子提起俩
	Drop_Location_jiang(200, 120, 8600);
}
void put(uint16_t Pos1, uint16_t Pos2) {
	WritePosEx(0, Pos1, 1000, 100); //物料底盘移动
	Move_Arm(6, 1000, time); //爪子上升
	Move_Arm(1, 150, time); //爪子张开
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 3500);
	HAL_Delay(1000);
	WritePosEx(1, Pos2, 800, 50); //爪子底盘向左
	HAL_Delay(2000);                                       //延时是为了抓取底盘降下来，防止变转变降
	Move_Arm(6, 410, time); //爪子下降
	HAL_Delay(time);
	Move_Arm(1, Arm_close, time); //爪子闭合
	HAL_Delay(time);
	Move_Arm(6, 1000, time); //爪子上升
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 1000);
	WritePosEx(1, 4100, 800, 50); //抓取底盘向摆放位置
	HAL_Delay(2000);                                        //延时是为了让抓取底盘转过来
	Move_Arm(6, 360, time); //爪子下降
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 11000);
	Move_Arm(1, 500, time); //爪子张开

}

void put_second(uint16_t Pos1, uint16_t Pos2) {
	WritePosEx(0, Pos1, 1000, 100); //物料底盘移动
	Move_Arm(6, 1000, time); //爪子上升
	Move_Arm(1, 150, time); //爪子张开
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 3500);
	HAL_Delay(1000);
	WritePosEx(1, Pos2, 800, 50); //爪子底盘向左
	HAL_Delay(2000);                                       //延时是为了抓取底盘降下来，防止变转变降
	Move_Arm(6, 410, time); //爪子下降
	HAL_Delay(time);
	Move_Arm(1, Arm_close, time); //爪子闭合
	HAL_Delay(time);
	Move_Arm(6, 1000, time); //爪子上升
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 1000);
	WritePosEx(1, 4100, 800, 50); //抓取底盘向摆放位置
	HAL_Delay(2000);                                        //延时是为了让抓取底盘转过来
	Move_Arm(6, 1000, time); //爪子下降
	Drop_Location_jiang(Emm_Speed, Emm_Acc, 11000);
	Move_Arm(1, 500, time); //爪子张开
}
//爪子在高5000处，必须搭配一个下降

extern int QR_data[6];
void Realize_Stop() {
	float err_x, err_y;

	while (1) {
		err_x = x;
		err_y = y;
		if (x - err_x <= 5 && y - err_y <= 5) {
			break;
		}
	}
}

void Frist_Grab_Wuliao(void) {
	static uint8_t Grab_count = 0;
	while (Grab_count < 3) {
		if (QR_data[0] == colour && wuliao_falg == 0) {
			Realize_Stop();
			catch_Frist(860, 2375);
			wuliao_falg = 1;
			Grab_count += 1;
		}
		if (QR_data[1] == colour && wuliao_falg == 1) {
			Realize_Stop();
			catch_Frist(1910, 2370);
			wuliao_falg = 2;
			Grab_count += 1;
		}
		if (QR_data[2] == colour && wuliao_falg == 2) {
			Realize_Stop();


//(2960, 2370);
			wuliao_falg = 0;
			Grab_count += 1;
		}

	}
}

extern int tx_target, ty_target;
void put_wuliao_to_circular_frist(void) {

	for (int i = 0; i < 3; i++) {
		if (QR_data[i] + 3 == 4) {
			Move_Action_Nopid_Right_Ctrl(1850, 1200);

		}
		if (QR_data[i] + 3 == 5) {
			Move_Action_Nopid_Right_Ctrl(1850, 1050);
		}
		if (QR_data[i] + 3 == 6) {
			Move_Action_Nopid_Right_Ctrl(1850, 885);
		}
		if (i == 0) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			put(2150, 2680);
		} else if (i == 1) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			put(3205, 2680);
		} else if (i == 2) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			put(2960, 2370);
		}

	}
}

void put_wuliao_to_Car_frist(void) {
	for (int i = 0; i < 3; i++) {
		if (QR_data[i] == 1) {
			Move_Action_Nopid_Right_Ctrl(1850, 1200);
		}
		if (QR_data[i] == 2) {
			Move_Action_Nopid_Right_Ctrl(1850, 1050);
		}
		if (QR_data[i] == 3) {
			Move_Action_Nopid_Right_Ctrl(1850, 885);
		}
		if (i == 0) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			catch_Second(860, 2375);

		} else if (i == 1) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			catch_Second(1910, 2370);
		} else if (i == 2) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			catch_Second(2960, 2370);
		}

	}
}

void put_wuliao_to_circular_second(void) {
	for (int i = 0; i < 3; i++) {
		if (QR_data[i] + 3 == 4) {
			Move_Action_Nopid_Forward_Ctrl(954, 1850);
		}
		if (QR_data[i] + 3 == 5) {
			Move_Action_Nopid_Forward_Ctrl(1050, 1850);
		}
		if (QR_data[i] + 3 == 6) {
			Move_Action_Nopid_Forward_Ctrl(1208, 1850);
		}
		if (i == 0) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			put(2150, 2680);

		} else if (i == 1) {

			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			put(3205, 2680);
		} else if (i == 2) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			put(2960, 2370);
		}

	}
	Move_Action_Nopid_Forward_Ctrl(1000, 1850);
}

/*************************第二次抓取***********************************************/
void Second_Run_Frist_Grab_Wuliao(void) {
	static uint8_t Grab_count = 0;
	wuliao_falg = 0;
	while (Grab_count < 3) {
		if (QR_data[3] == colour && wuliao_falg == 0) {
			catch_Frist(860, 2375);
			wuliao_falg = 1;
			Grab_count += 1;
		}
		if (QR_data[4] == colour && wuliao_falg == 1) {
			catch_Frist(1910, 2370);
			wuliao_falg = 2;
			Grab_count += 1;
		}
		if (QR_data[5] == colour && wuliao_falg == 2) {
			catch_Frist(2960, 2370);
			wuliao_falg = 0;
			Grab_count += 1;
		}

	}
}
void Second_Run_put_wuliao_to_Car_frist(void) {
	for (int i = 3; i < 6; i++) {
		if (QR_data[i] == 1) {
			Move_Action_Nopid_Right_Ctrl(1850, 1200);
		}
		if (QR_data[i] == 2) {
			Move_Action_Nopid_Right_Ctrl(1850, 1050);
		}
		if (QR_data[i] == 3) {
			Move_Action_Nopid_Right_Ctrl(1850, 885);
		}
		if (i == 3) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			catch_Second(860, 2375);

		} else if (i == 4) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			catch_Second(1910, 2370);
		} else if (i == 5) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			catch_Second(2960, 2370);
		}

	}
}

void Second_Run_put_wuliao_to_circular_frist(void) {

	for (int i = 3; i < 6; i++) {
		if (QR_data[i] == 1) {
			Move_Action_Nopid_Right_Ctrl(910, 1910);

		}
		if (QR_data[i] == 2) {
			Move_Action_Nopid_Right_Ctrl(1060, 1910);
		}
		if (QR_data[i] == 3) {
			Move_Action_Nopid_Right_Ctrl(1185, 1910);
		}
		if (i == 3) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			put(2150, 2680);
		} else if (i == 4) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			put(3205, 2680);
		} else if (i == 5) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX调整
			put(2960, 2370);
		}

	}
}

void Second_Run_put_wuliao_to_circular_second(void) {
	for (int i = 3; i < 6; i++) {
		if (QR_data[i] + 3 == 4) {
			Move_Action_Nopid_Forward_Ctrl(954, 1870);
		}
		if (QR_data[i] + 3 == 5) {
			Move_Action_Nopid_Forward_Ctrl(1050, 1870);
		}
		if (QR_data[i] + 3 == 6) {
			Move_Action_Nopid_Forward_Ctrl(1208, 1870);
		}
		if (i == 3) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			put_second(2150, 2680);

		} else if (i == 4) {

			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX

			put_second(3205, 2680);
		} else if (i == 5) {
			Move_Tx_Pid_Ctrl(tx_target, ty_target); //TX
			put_second(2960, 2370);
		}

	}
	Move_Action_Nopid_Forward_Ctrl(1000, 1820);
}

