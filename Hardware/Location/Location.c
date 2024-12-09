/*
 * Location.c
 *
 *  Created on: Nov 4, 2024
 *      Author: Taolin
 */

#include "Location.h"

/* 定义PID结构体 */
_PID XPID;
_PID YPID;
_PID APID;
_PID TXPID;
_PID TYPID;
//	定义三个轴的输出
float X_out;
float Y_out;
float A_out;
float TX_X_out;
float TX_Y_out;

// 输出限幅
float limit(float x, float min, float max) {
	x = (x > max) ? max : ((x < min) ? min : x);
	return x;
}
//	取绝对值函数
float my_abs(float in) {
	if (in < 0) {
		return -in;
	} else {
		return in;
	}
}

//	初始化PID结构体
void PID_Init(void) {
	//======================X=====================//   20   60
	XPID.P = 20;    //0.57    0  0.1
	XPID.I = 0;
	XPID.D = 60;

	XPID.I_LIMIT = 100.0;
	XPID.OUT_LIMIT = 100.0;

	XPID.P_OUT = 0.0;
	XPID.I_OUT = 0.0;
	XPID.D_OUT = 0.0;

	XPID.OUT = 0.0;

	XPID.err = 0.0;
	XPID.err_err = 0.0;
	XPID.err_old = 0.0;

	//========= =============Y=====================//
	YPID.P = 20;    //0.89                     //0.8 0 0.01
	YPID.I = 0;
	YPID.D = 60;    //60
	YPID.I_LIMIT = 100.0;
	YPID.OUT_LIMIT = 100.0;

	YPID.P_OUT = 0.0;
	YPID.I_OUT = 0.0;
	YPID.D_OUT = 0.0;

	YPID.OUT = 0.0;

	YPID.err = 0.0;
	YPID.err_err = 0.0;
	YPID.err_old = 0.0;

	//======================A=====================//
	APID.P = 0.9;
	APID.I = 0.0;
	APID.D = 0.0;

	APID.I_LIMIT = 1000.0;
	APID.OUT_LIMIT = 2350.0;

	APID.P_OUT = 0.0;
	APID.I_OUT = 0.0;
	APID.D_OUT = 0.0;

	APID.OUT = 0.0;

	APID.err = 0.0;
	APID.err_err = 0.0;
	APID.err_old = 0.0;
	//======================TX=====================//
	TXPID.P = 1;
	TXPID.I = 0;
	TXPID.D = 0.3;

	TXPID.I_LIMIT = 100.0;
	TXPID.OUT_LIMIT = 300.0;

	TXPID.P_OUT = 0.0;
	TXPID.I_OUT = 0.0;
	TXPID.D_OUT = 0.0;

	TXPID.OUT = 0.0;

	TXPID.err = 0.0;
	TXPID.err_err = 0.0;
	TXPID.err_old = 0.0;

	//======================TY=====================//
	TYPID.P = -1;
	TYPID.I = -0;      //0.9  0.01   0.3
	//0.9 0 0.2
	TYPID.D = -0.3;

	TYPID.I_LIMIT = 100.0;
	TYPID.OUT_LIMIT = 300.0;

	TYPID.P_OUT = 0.0;
	TYPID.I_OUT = 0.0;
	TYPID.D_OUT = 0.0;

	TYPID.OUT = 0.0;

	TYPID.err = 0.0;
	TYPID.err_err = 0.0;
	TYPID.err_old = 0.0;
}

/* X轴调整 */
short X_Ctrl(_PID *PID, float goal) {
	PID->err = goal - X_NOW;

	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT = PID->D * PID->err_err;

	PID->I_OUT = limit(PID->I_OUT, -PID->I_LIMIT, PID->I_LIMIT);

	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

	PID->OUT = limit(PID->OUT, -PID->OUT_LIMIT, PID->OUT_LIMIT);

	PID->err_old = PID->err;

	return PID->OUT;
}

/* Y轴调整 */
short Y_Ctrl(_PID *PID, float goal) {
	PID->err = goal - Y_NOW;

	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT = PID->D * PID->err_err;
//
	PID->I_OUT = limit(PID->I_OUT, -PID->I_LIMIT, PID->I_LIMIT);

	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

	PID->OUT = limit(PID->OUT, -PID->OUT_LIMIT, PID->OUT_LIMIT);

	PID->err_old = PID->err;

	return PID->OUT;
}

/* 角度保持 */
short A_Ctrl(_PID *PID, float goal) {
	PID->err = goal - A_NOW;

	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT = PID->D * PID->err_err;

	PID->I_OUT = limit(PID->I_OUT, -PID->I_LIMIT, PID->I_LIMIT);

	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

	PID->OUT = limit(PID->OUT, -PID->OUT_LIMIT, PID->OUT_LIMIT);

	PID->err_old = PID->err;

	return PID->OUT;
}
/* TX的X轴调整 */
short Tx_X_Ctrl(_PID *PID, float goal) {
	PID->err = Tx_NOW - goal;

	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT = PID->D * PID->err_err;

	PID->I_OUT = limit(PID->I_OUT, -PID->I_LIMIT, PID->I_LIMIT);

	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

	PID->OUT = limit(PID->OUT, -PID->OUT_LIMIT, PID->OUT_LIMIT);

	PID->err_old = PID->err;

	return PID->OUT;
}

/* TX的Y轴调整 */
short Tx_Y_Ctrl(_PID *PID, float goal) {
	PID->err = Ty_NOW - goal;

	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT = PID->D * PID->err_err;

	PID->I_OUT = limit(PID->I_OUT, -PID->I_LIMIT, PID->I_LIMIT);

	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;

	PID->OUT = limit(PID->OUT, -PID->OUT_LIMIT, PID->OUT_LIMIT);

	PID->err_old = PID->err;

	return PID->OUT;
}

