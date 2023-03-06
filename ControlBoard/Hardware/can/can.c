#include "can.h"
#include "usart.h"


// button
extern uint8_t botton_LU, botton_LD, botton_LL, botton_LR;
extern uint8_t botton_RU, botton_RD, botton_RL, botton_RR;
extern uint8_t botton_top_LT, botton_top_LB, botton_top_RT, botton_top_RB;
extern uint8_t botton_A, botton_B, botton_center;
// rocker
extern uint16_t rocker[4];


/**
  * @brief HAL_CAN_RxFifo0MsgPendingCallback
  * @param None
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t aRxData[8];

	CAN_RxHeaderTypeDef hCAN1_RxHeader;
	
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN1_RxHeader, aRxData) == HAL_OK)
	{
		if (hCAN1_RxHeader.StdId == 0x201)
		{
			botton_LU = (aRxData[0] >> 0) & 0x01;
			botton_LD = (aRxData[0] >> 1) & 0x01;
			botton_LL = (aRxData[0] >> 2) & 0x01;
			botton_LR = (aRxData[0] >> 3) & 0x01;
			botton_RU = (aRxData[0] >> 4) & 0x01;
			botton_RD = (aRxData[0] >> 5) & 0x01;
			botton_RL = (aRxData[0] >> 6) & 0x01;
			botton_RR = (aRxData[0] >> 7) & 0x01;
			botton_top_LT = (aRxData[1] >> 0) & 0x01;
			botton_top_LB = (aRxData[1] >> 1) & 0x01;
			botton_top_RT = (aRxData[1] >> 2) & 0x01;
			botton_top_RB = (aRxData[1] >> 3) & 0x01;
			botton_A = (aRxData[1] >> 4) & 0x01;
			botton_B = (aRxData[1] >> 5) & 0x01;
			botton_center = (aRxData[1] >> 6) & 0x01;
		} 
		else if (hCAN1_RxHeader.StdId == 0x202) 
		{
			rocker[0] = (aRxData[0] << 8) | aRxData[1];
			rocker[1] = (aRxData[2] << 8) | aRxData[3];
			rocker[2] = (aRxData[4] << 8) | aRxData[5];
			rocker[3] = (aRxData[6] << 8) | aRxData[7];
		}
	}
}




