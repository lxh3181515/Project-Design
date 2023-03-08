#ifndef CONTROLBOARD_HARDWARE_USART_H_
#define CONTROLBOARD_HARDWARE_USART_H_

#include "main.h"
#include "string.h"

uint8_t SerialGetCrc8Value(uint8_t *data, uint8_t len);
void SerialWrite(UART_HandleTypeDef *huart,
								 uint8_t botton_LU, uint8_t botton_LD, uint8_t botton_LL, uint8_t botton_LR,
								 uint8_t botton_RU, uint8_t botton_RD, uint8_t botton_RL, uint8_t botton_RR,
								 uint8_t botton_top_LT, uint8_t botton_top_LB, uint8_t botton_top_RT, uint8_t botton_top_RB,
								 uint8_t botton_A, uint8_t botton_B, uint8_t botton_center,
								 uint16_t *rocker);

#define g 9.8
typedef struct IMU
{
	//������ٶ�
	float ax;
	float ay;
	float az;
	
	//������ٶ�
	float wx;
	float wy;
	float wz;
	
	//����Ƕ�
	float roll; //x
	float pitch;//y
	float yaw;  //z
	
}IMU;

enum getImuDataFLAG
{
	GetAcceleration,//��ȡ���ٶ�flag
	GetAngularVelocity,//��ȡ���ٶ�flag
	GetAngle,//��ȡ�Ƕ�flag
	WaitingGetImu//�ȴ�
};
void Get_ImuData(void);
unsigned char ImuCountSum(unsigned char * startIndex,short len);


#endif
