#ifndef __BSP_TM1639_H
#define __BSP_TM1639_H

#include "stm32f10x.h"

#define TM1639_DIO_High()   	GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define TM1639_DIO_Low()  		GPIO_ResetBits(GPIOB, GPIO_Pin_14) 	   		//
#define TM1639_CLK_High()   	GPIO_SetBits(GPIOB, GPIO_Pin_13)  	   		//
#define TM1639_CLK_Low()  		GPIO_ResetBits(GPIOB, GPIO_Pin_13) 	   		//
#define TM1639_STB_High()   	GPIO_SetBits(GPIOB, GPIO_Pin_12)  	   		//
#define TM1639_STB_Low()  		GPIO_ResetBits(GPIOB, GPIO_Pin_12) 	   		//

#define DIG0	0xC0
#define DIG1	0xC2
#define DIG2	0xC4
#define DIG3	0xC6
#define DIG4	0xC8
#define DIG5	0xCA
#define DIG6	0xCC
#define DIG7	0xCE

#define LEVEL_OFF	0x80
#define LEVEL_1		0x88
#define LEVEL_2		0x89
#define LEVEL_4		0x8A
#define LEVEL_10	0x8B
#define LEVEL_11	0x8C
#define LEVEL_12	0x8D
#define LEVEL_13	0x8E
#define LEVEL_14	0x8F

#define LEVEL_USE	LEVEL_4	//Ŀǰʹ������ 

#define MD_WRITE	0x40
#define MD_AUTO		MD_WRITE
#define MD_NORMAL	MD_WRITE


void BspTm1639_Config(void); 
void BspTm1639_Writebyte(uint8_t datx);
void BspTm1639_Test(void);
void BspTM1639_ClearALL(void);	//����ʾ���� 
void BspTm1639_Show(uint8_t ShowMode,uint32_t ShowDate);
void BspTm1639_ShowSNDat(uint8_t ShowMode,uint8_t *ShowDate);

#endif
