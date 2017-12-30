#ifndef __BSP_KEY_H
#define __BSP_KEY_H	


#define Key1_Dat()  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)		//����
#define Key2_Dat()  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)	//��һ����
#define Key3_Dat()  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)	//�ֶ�
#define Key4_Dat()  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)	//������һ

#define EmerKey_Dat()  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)	//��������

typedef enum
{
	KEY_NONE = 0,			/* 0 ��ʾ�����¼� */
	KEY_1,					/* 1�� */
	KEY_2,					/* 2�� */
	KEY_3,					/* 3�� */
	KEY_4,					/* 4�� */
}KEY_ENUM;

void BspKey_Config(void);
u8 BspKey_Read(void);
		 				    
#endif
