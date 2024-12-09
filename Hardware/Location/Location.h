/*
 * Location.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Taolin
 */

#ifndef LOCATION_LOCATION_H_
#define LOCATION_LOCATION_H_

#include "main.h"
#include "Action/Action.h"
#include "Run/Run.h"
#include "tim.h"

// 将从Action得到的数据修改为从TX2回传的数据,但Z轴保持不变
extern float x;
extern float y;

// 输出的数据
extern float X_out;
extern float Y_out;
extern float A_out;
extern float TX_X_out;
extern float TX_Y_out;

#define Tx_NOW x   //x,y为TX得到的数据
#define Ty_NOW y

// 从Action得到的数据，浮点格式,此处进行修改
#define X_NOW pos_x    //pos_x pos_y  zangle为Action的到的数据
#define Y_NOW -pos_y
#define A_NOW zangle

typedef struct {
	float P;
	float P_OUT;

	float I;
	float I_OUT;
	float I_LIMIT;

	float D;
	float D_OUT;

	float OUT;
	float OUT_LIMIT;

	short err;
	short err_err;
	short err_old;

} _PID;

float my_abs(float in);
void PID_Init(void);
short X_Ctrl(_PID *PID, float goal);
short Y_Ctrl(_PID *PID, float goal);
short A_Ctrl(_PID *PID, float goal);
short Tx_X_Ctrl(_PID *PID, float goal);
short Tx_Y_Ctrl(_PID *PID, float goal);

extern _PID XPID;
extern _PID YPID;
extern _PID APID;
extern _PID TXPID;
extern _PID TYPID;

#endif /* LOCATION_LOCATION_H_ */
