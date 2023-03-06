#include "tim.h"


/**
	* @brief Control Servo
  * @param timer 4, angle(0~180), num(0 or 1)
  * @retval None 
  */
void servoControl(TIM_HandleTypeDef *htim, uint8_t angle, uint8_t num) {
	if (angle < 0 || angle > 180 || num > 1 || num < 0) {
		return;
	}
	
	float cmp_val;
	cmp_val = 10.0 * angle / 9.0 + 50.0; // 50 ~ 2500
	
	if (num == 0) { // No.0 -> CH1
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t)cmp_val);
	} 
	else if (num == 1) { // No.1 -> CH2
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint16_t)cmp_val);
	}
}


/**
	* @brief Control Motor
  * @param timer 5, speed(-100~100), num(0 or 1)
  * @retval None 
  */
void motorControl(TIM_HandleTypeDef *htim, int speed, uint8_t num) {
	if (speed < -100 || speed > 100 || num > 1 || num < 0) {
		return;
	}
	
	int cmp_val;
	cmp_val = speed + 150;
	
	if (num == 0) { // No.0 -> CH1
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t)cmp_val);
	} 
	else if (num == 1) { // No.1 -> CH2
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint16_t)cmp_val);
	}
}

