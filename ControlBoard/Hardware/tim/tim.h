#ifndef CONTROLBOARD_HARDWARE_TIM_H_
#define CONTROLBOARD_HARDWARE_TIM_H_

#include "main.h"

void servoControl(TIM_HandleTypeDef *htim, int angle, uint8_t num);
void motorControl(TIM_HandleTypeDef *htim, int speed, uint8_t num);

#endif
