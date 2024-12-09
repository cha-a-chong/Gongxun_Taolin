/*
 * SCS_Uart.c
 *
 *  Created on: Nov 1, 2024
 *      Author: Taolin
 */

#include "stm32h7xx_hal.h"
#include "SCS_Uart.h"
#include "usart.h"

void Uart_Send(uint8_t *buf, uint16_t len)
{
	HAL_UART_Transmit(&huart6, buf, len, HAL_MAX_DELAY);
}

int16_t Uart_Read(uint8_t *buf, uint16_t len, uint32_t timeout)
{
	if (HAL_UART_Receive(&huart6, buf, len, timeout) == HAL_OK)
	{
		return len;
	}
	else
	{
		return 0;
	}
}

