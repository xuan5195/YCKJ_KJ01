#include "stm32f10x.h"		/* ���Ҫ��ST�Ĺ̼��⣬�����������ļ� */
#include <stdio.h>			/* ��Ϊ�õ���printf���������Ա����������ļ� */
#include "bsp_timer.h"		/* systick��ʱ��ģ�� */
#include "bsp_FireData.h"	/**/
#include "bsp_Input.h"		/**/
#include "bsp_ADC.h"		/**/

u8 gADCTemp[2] = {0};

void Fire_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void OUT_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	Fire_OUT1_OFF();//�������1
	Fire_OUT2_OFF();//�������2

}

u8 Test_ADC_Data(u8 TestADCCount)
{
	u8 Test_ADC_temp = 0x00;
	Fire_Test1_OFF();Fire_Test2_OFF();Fire_Test3_OFF();Fire_Test4_OFF();
	Fire_Test5_OFF();Fire_Test6_OFF();
	switch (TestADCCount)
	{
		case 0:
			//////ͨ��1����----------------------------
			Fire_Test1_ON();	bsp_DelayMS(400);
			if(Get_FireData()==0xAA)	{Test_ADC_temp += 0x01;} //�쳣
			Fire_Test1_OFF();	bsp_DelayMS(1);
			
			//////ͨ��2����----------------------------
			Fire_Test2_ON();	bsp_DelayMS(400);
			if(Get_FireData()==0xAA)	{Test_ADC_temp += 0x02;} //�쳣
			Fire_Test2_OFF();	bsp_DelayMS(1);
				
			//////ͨ��3����----------------------------
			Fire_Test3_ON();	bsp_DelayMS(400);
			if(Get_FireData()==0xAA)	{Test_ADC_temp += 0x04;} //�쳣
			Fire_Test3_OFF();	bsp_DelayMS(1);
			gADCTemp[0] = Test_ADC_temp;
			break;
		case 1:	
			//////ͨ��4����----------------------------
			Fire_Test4_ON();	bsp_DelayMS(400);
			if(Get_FireData()==0xAA)	{Test_ADC_temp += 0x08;} //�쳣
			Fire_Test4_OFF();	bsp_DelayMS(1);
				
			//////ͨ��5����----------------------------
			Fire_Test5_ON();	bsp_DelayMS(400);
			if(Get_FireData()==0xAA)	{Test_ADC_temp += 0x10;} //�쳣
			Fire_Test5_OFF();	bsp_DelayMS(1);
			
			//////ͨ��6����----------------------------
			Fire_Test6_ON();	bsp_DelayMS(400);
			if(Get_FireData()==0xAA)	{Test_ADC_temp += 0x20;} //�쳣
			Fire_Test6_OFF();	bsp_DelayMS(1);
			gADCTemp[1] = Test_ADC_temp;
			break;
		default:
			break;
	}
	Test_ADC_temp = (gADCTemp[0] | gADCTemp[1]);	
	return Test_ADC_temp;	
}
void FireStart(u8 Fire_OUT_No)
{
	switch(Fire_OUT_No)
	{
		case 0x01:
			Fire_Start1_ON();
			break;
		case 0x02:
			Fire_Start2_ON();
			break;
		case 0x04:
			Fire_Start3_ON();
			break;
		case 0x08:
			Fire_Start4_ON();
			break;
		case 0x10:
			Fire_Start5_ON();
			break;
	}
}

