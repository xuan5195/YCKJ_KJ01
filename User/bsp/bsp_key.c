#include "stm32f10x.h"		/* ���Ҫ��ST�Ĺ̼��⣬�����������ļ� */
#include <stdio.h>			/* ��Ϊ�õ���printf���������Ա����������ļ� */
#include "bsp_key.h"		/**/
#include "bsp_timer.h"

KEY_ENUM KEY_ID;
/*
*********************************************************************************************************
* �������ƣ�BspKey_Delay
* ����˵��: ������ʱ����
* ��  	�Σ���
* �� �� ֵ����
*********************************************************************************************************
*/
void BspKey_Delay(u8 z) 
{ 
	bsp_DelayMS(z);
}

/*
*********************************************************************************************************
* �������ƣ�BspKey_Config
* ����˵��: ����GPIO�ڳ�ʼ�� ��������
* ��  	�Σ���
* �� �� ֵ����
*********************************************************************************************************
*/
void BspKey_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_4|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//���� ������������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
* �������ƣ�BspKey_Read
* ����˵��: ɨ�������ֵ
* ��  	�Σ���
* �� �� ֵ������ֵ
*********************************************************************************************************
*/
u8 BspKey_Read(void)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(1)key_up=1;  //֧������		  
	if(key_up&&(Key1_Dat()==0||Key2_Dat()==0||Key3_Dat()==0||Key4_Dat()==0))
	{
		BspKey_Delay(10);//ȥ���� 
		key_up=0;
		if(Key1_Dat()==0)		return KEY_1;
		else if(Key2_Dat()==0)	return KEY_2;
		else if(Key3_Dat()==0)	return KEY_3;
		else if(Key4_Dat()==0)	return KEY_4;
	}
	else if((Key1_Dat()==1)&&(Key2_Dat()==1)&&(Key3_Dat()==1)&&(Key4_Dat()==1))	key_up=1; 	    
 	return KEY_NONE;// �ް�������
}



