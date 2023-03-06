#ifndef CONTROLBOARD_HARDWARE_USART_H_
#define CONTROLBOARD_HARDWARE_USART_H_

#include "main.h"

uint8_t SerialGetCrc8Value(uint8_t *data, uint8_t len);
void SerialWrite(UART_HandleTypeDef *huart,
								 uint8_t botton_LU, uint8_t botton_LD, uint8_t botton_LL, uint8_t botton_LR,
								 uint8_t botton_RU, uint8_t botton_RD, uint8_t botton_RL, uint8_t botton_RR,
								 uint8_t botton_top_LT, uint8_t botton_top_LB, uint8_t botton_top_RT, uint8_t botton_top_RB,
								 uint8_t botton_A, uint8_t botton_B, uint8_t botton_center,
								 uint16_t *rocker);
#endif
