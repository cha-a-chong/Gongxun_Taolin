/*
 * HWT101CT.h
 *
 *  Created on: Nov 27, 2024
 *      Author: yinnanzhao
 */

#ifndef HWT101CT_HWT101CT_H_
#define HWT101CT_HWT101CT_H_
#include "main.h"

void ParseAndPrintData(uint8_t *data, uint16_t length);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type);

#endif /* HWT101CT_HWT101CT_H_ */
