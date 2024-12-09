/*
 * Joy_Interface.h
 *
 *  Created on: Nov 1, 2024
 *      Author: Taolin
 */

#ifndef JOY_INTERFACE_JOY_INTERFACE_H_
#define JOY_INTERFACE_JOY_INTERFACE_H_

#include "main.h"
#include "stm32h7xx.h"
#include <stdio.h>
#include <stdlib.h>

extern float Left_axis_X;
extern float Left_axis_Y;
extern float Right_axis_X;
extern float Right_axis_Y;
extern float LT;
extern float RT;
extern float Hat_X;
extern float Hat_Y;

void Recive_Joydata(uint8_t rec);
void Check_Joydata(void);

#endif /* JOY_INTERFACE_JOY_INTERFACE_H_ */
