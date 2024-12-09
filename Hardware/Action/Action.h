/*
 * Action.h
 *
 *  Created on: Oct 26, 2024
 *      Author: Taolin
 */

#ifndef ACTION_ACTION_H_
#define ACTION_ACTION_H_

#include "usart.h"

// extern扩展,向外提供变量
extern float pos_x;
extern float pos_y;
extern float zangle;


void Action_It_Init(void);
void Data_Analyse(uint8_t rec);

#endif /* ACTION_ACTION_H_ */
