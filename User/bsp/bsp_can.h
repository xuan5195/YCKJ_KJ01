#ifndef __BSP_CAN_H
#define __BSP_CAN_H	 

#include "stm32f10x.h"

//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 Can_Send_Msg(u8* msg,u8 len);						//��������
u8 Can_Receive_Msg(u8 *buf);							//��������
void Package_Send(u8 _mode,u8 *Package_Dat);
void Package0_Send(void);

#endif

















