#include "usart.h"

const uint8_t header[2] = {0x55,0xaa};
const uint8_t ender[2] = {0x0d,0x0a};

uint8_t SerialGetCrc8Value(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	uint8_t i;
	while(len--)
	{
		crc ^= *data++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
				crc=(crc>>1)^0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}
uint8_t high;
uint8_t low;
void SerialWrite(UART_HandleTypeDef *huart,
								 uint8_t botton_LU, uint8_t botton_LD, uint8_t botton_LL, uint8_t botton_LR,
								 uint8_t botton_RU, uint8_t botton_RD, uint8_t botton_RL, uint8_t botton_RR,
								 uint8_t botton_top_LT, uint8_t botton_top_LB, uint8_t botton_top_RT, uint8_t botton_top_RB,
								 uint8_t botton_A, uint8_t botton_B, uint8_t botton_center,
								 uint16_t *rocker)
{

	uint8_t sendData[28];
	sendData[0] = header[0];
	sendData[1] = header[1];
	sendData[2] = botton_LU;
	sendData[3] = botton_LD;
	sendData[4] = botton_LL;
	sendData[5] = botton_LR;
	sendData[6] = botton_RU;
	sendData[7] = botton_RD;
	sendData[8] = botton_RL;
	sendData[9] = botton_RR;
	sendData[10] = botton_top_LT;
	sendData[11] = botton_top_LB;
	sendData[12] = botton_top_RT;
	sendData[13] = botton_top_RB;
	sendData[14] = botton_A;
	sendData[15] = botton_B;
	sendData[16] = botton_center;
	for(int i=0; i<4; i++)
	{
		sendData[16 + 1 + 2*i] = (uint8_t)(rocker[i] & 0x00ff);
		sendData[16 + 2 + 2*i] = (uint8_t)((rocker[i] >> 8) & 0x00ff);
	}
	sendData[25] = SerialGetCrc8Value(sendData, 25);
	sendData[26] = ender[0];
	sendData[27] = ender[1];

	HAL_UART_Transmit(huart, sendData, 8*28, 0xffff);
}
