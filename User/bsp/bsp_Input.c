#include "stm32f10x.h"		/* ���Ҫ��ST�Ĺ̼��⣬�����������ļ� */
#include <stdio.h>			/* ��Ϊ�õ���printf���������Ա����������ļ� */
#include "bsp_timer.h"		/* systick��ʱ��ģ�� */
#include "bsp_FireData.h"	/**/
#include "bsp_Input.h"		/**/

#ifdef PressureOpen
  uint8_t PressureDat	= PressureOpen; 
#elif defined PressureClose
  uint8_t PressureDat	= PressureClose; 
#else
  uint8_t PressureDat	= 0; 
#endif

void Input_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//���� ������������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

//��ȡ���ݱ���
u8 Read_PressureData(void)
{
	u8 temp=0;
	if(Pressure1 == PressureDat)	{temp = 0xDD;}
	else	{temp = 0xAA;}//�쳣
	return temp;
}

