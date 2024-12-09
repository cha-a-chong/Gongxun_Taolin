/*
 * Screen.h
 *
 *  Created on: Oct 26, 2024
 *      Author: Taolin
 */

#ifndef SCREEM_SCREEN_H_
#define SCREEM_SCREEN_H_

#include "stm32h7xx_hal.h"

void Action_Show();
void QR_Show(void);
void Point_Show(void);

void Check_Flag(uint8_t rec);

#endif /* SCREEM_SCREEN_H_ */
