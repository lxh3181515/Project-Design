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

	HAL_UART_Transmit(huart, sendData, 28, 0xffff);
}


IMU imu; // imu结构体
unsigned char  ImuReceiveBuff[10] = {0}; // 包括数据头不包括校验和
const short ImudataLength   = 8; // 不包括数据头和校验和的数据个数
unsigned char ImuReceiver = 0; // 用于接收数据
unsigned char aRxBuffer; // 串口接收变量
void Get_ImuData(void)
{
	static short ImuReceiveDataNum = 0; //用于记录目前接受了的数据个数
	static unsigned char ImuReceiverFront   = 0; // 接受的数据的上一个 用于判断数据头
	static unsigned char Flag = WaitingGetImu; // 标志位
	
	ImuReceiver = aRxBuffer;   

	//根据数据头 确定获取的数据
	if(Flag == WaitingGetImu)
	{
		if(ImuReceiver == 0x51)                   
		{  
			if(ImuReceiverFront == 0x55)        
			{			
				Flag = GetAcceleration;	 //0x55 0x51 接收加速度	
				ImuReceiveBuff[0]=ImuReceiverFront;     //buf[0]
				ImuReceiveBuff[1]=ImuReceiver;         //buf[1]	
			}
		}
		else if(ImuReceiver == 0x52)            
		{  
			if(ImuReceiverFront == 0x55)        
			{
				Flag = GetAngularVelocity; //0x55 0x52 接收角速度
				ImuReceiveBuff[0]=ImuReceiverFront;     //buf[0]
				ImuReceiveBuff[1]=ImuReceiver;         //buf[1]	
			}
		}
		else if(ImuReceiver == 0x53)              
		{  
			if(ImuReceiverFront == 0x55)       
			{		
				Flag = GetAngle;          //0x55 0x53 接收角度
				ImuReceiveBuff[0]=ImuReceiverFront;     //buf[0]
				ImuReceiveBuff[1]=ImuReceiver;         //buf[1]	
			}
		}
		else 
		{
			ImuReceiverFront = ImuReceiver;  
		}
	}
	
	//接受数据
	else
  {
		if(ImuReceiveDataNum < ImudataLength)
		{
			ImuReceiveBuff[ImuReceiveDataNum + 2] = ImuReceiver;               
			ImuReceiveDataNum++;
		}
		else
		{
			//检验校验和
			if(ImuReceiver != ImuCountSum(&ImuReceiveBuff[0],ImudataLength+2))
			{
				return;
			}
			else
			{
				//赋值
				switch(Flag)
				{
					case GetAcceleration:
					{
						// (m/s2)
						imu.ax = (short)(((short)ImuReceiveBuff[3]<<8)|ImuReceiveBuff[2])/32768.0*16.0*g;
						imu.ay = (short)(((short)ImuReceiveBuff[5]<<8)|ImuReceiveBuff[4])/32768.0*16.0*g;
						imu.az = (short)(((short)ImuReceiveBuff[7]<<8)|ImuReceiveBuff[6])/32768.0*16.0*g;
						break;
					}
					case GetAngularVelocity:
					{
						// (°/s)
						imu.wx = (short)(((short)ImuReceiveBuff[3]<<8)|ImuReceiveBuff[2])/32768.0*2000.0; 
						imu.wy = (short)(((short)ImuReceiveBuff[5]<<8)|ImuReceiveBuff[4])/32768.0*2000.0;
						imu.wz = (short)(((short)ImuReceiveBuff[7]<<8)|ImuReceiveBuff[6])/32768.0*2000.0;
						break;
					}
					case GetAngle:
					{
						// (°)
						imu.roll  = (short)(((short)ImuReceiveBuff[3]<<8)|ImuReceiveBuff[2])/32768.0*180.0; // x
						imu.pitch = (short)(((short)ImuReceiveBuff[5]<<8)|ImuReceiveBuff[4])/32768.0*180.0; // y
						imu.yaw   = (short)(((short)ImuReceiveBuff[7]<<8)|ImuReceiveBuff[6])/32768.0*180.0; // z
						break;
					}
					default:
					{
						break;
					}
				}	
			}
			
			//不论校验有没有通过都 清零 
			ImuReceiverFront = 0;
			Flag = WaitingGetImu;
			ImuReceiveDataNum = 0;
		}		
	}
}
unsigned char ImuCountSum(unsigned char * startIndex,short len)
{
	unsigned char CheckSum = 0x00;
	while(len>0)
	{
		CheckSum += (*startIndex); //(uint8_t)
		startIndex ++;
		len--;
	}
	
	return CheckSum;
}



/**
  * @brief HAL_UART_RxCpltCallback - USART2
  * @param None
  * @retval None
  */

extern UART_HandleTypeDef huart2;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	Get_ImuData();

	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);
}
