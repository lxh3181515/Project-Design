#include "tim.h"


/**
	* @brief Control Servo
  * @param timer 4, angle(-90~90), num(0 or 1)
  * @retval None 
  */
void servoControl(TIM_HandleTypeDef *htim, int angle, uint8_t num) {
	if (angle < -90 || angle > 90) {
		return;
	}
	if (num !=1 && num != 0){
		return;
	}
	
	angle = angle + 90; //ÁãÎ»ÊÇ90
	if(num == 1) angle = 180 - angle;
		
	float cmp_val;
	cmp_val = 10.0 * angle / 9.0 + 50.0; // 50 ~ 250
	
	
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
	if (speed < -100 || speed > 100) {
		return;
	}
	if (num !=1 && num != 0){
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

