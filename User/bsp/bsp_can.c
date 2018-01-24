#include "stm32f10x.h"
#include <stdio.h>

#include "bsp_can.h"
#include "bsp_crc8.h"
#include "bsp_tm1639.h"	
#include "bsp.h"

extern uint8_t Physical_ADD[4];//�����ַ
extern uint8_t Logic_ADD;
extern uint8_t g_RxMessage[8];	//CAN��������
extern uint8_t g_RxMessFlag;	//CAN�������� ��־
extern unsigned char FM1702_Key[7];
extern uint8_t WaterCost,CostNum;	//WaterCost=ˮ�� ��С�ۿ���  //������
extern uint8_t g_MemoryBuf[5][10];	//���ݻ��棬[0]=0xAA��ʾ�в忨���ݣ�[0]=0xBB��ʾ�аο����ݣ�[1-4]���ţ�[5-7]��[8]�������룻[9]ͨ����
extern void PutOutMemoryBuf(void);	//���һ������
extern uint8_t re_RxMessage[16];
extern unsigned char UID[5];
extern uint8_t gErrorDat[6];		//�쳣����洢
extern uint8_t Flash_UpdateFlag;	//Flash�����ݸ��±�־��0xAA��ʾ������Ҫ����
extern uint8_t g_IAP_Flag;			//����������־
extern uint8_t g_LoseContact;		//ʧ������������200ʱ����ʾʧ�����Զ���λ

extern void Delay (uint16_t nCount);


//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.��Χ:1~8;
//tbs1:ʱ���1��ʱ�䵥Ԫ.��Χ:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//ע�����ϲ����κ�һ����������Ϊ0,�������.
//������=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,��ͨģʽ;1,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(1,8,7,5,1);
//������Ϊ:36M/((1+8+7)*5)=450Kbps
//������Ϊ:36M/((1+13+2)*18)=125Kbps CAN_Normal_Init(1,13,2,18,1);
//����ֵ:0,��ʼ��OK;	����,��ʼ��ʧ��;


u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	u16 std_id = 0x200,std_id1 = 0x200|Logic_ADD;
	u32 ext_id = 0x1800F001;
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��IO
	  
 	//CAN��Ԫ����
 	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ֹʱ�䴥��ͨ��ģʽ
 	CAN_InitStructure.CAN_ABOM=DISABLE;	//�����CAN_MCR�Ĵ�����INRQλ������1�����0��һ��Ӳ����⵽128��11λ����������λ�����˳�����״̬��
  	CAN_InitStructure.CAN_AWUM=DISABLE;	//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)//
  	CAN_InitStructure.CAN_NART=ENABLE;	//ENABLE:CAN����ֻ������1�Σ����ܷ��͵Ľ����Σ��ɹ���������ٲö�ʧ��
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//�ڽ������ʱFIFOδ��������������FIFO�ı���δ����������һ���յ��ı��ĻḲ��ԭ�еı���
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//����FIFO���ȼ��ɱ��ĵı�ʶ��������
  	CAN_InitStructure.CAN_Mode= mode;	//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; //
  	//���ò�����
  	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;        //��Ƶϵ��(Fdiv)Ϊbrp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);         // ��ʼ��CAN1 

 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //���ù�������0����ΧΪ0~13  
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;  //CAN_FilterMode_IdMask:����ģʽ  CAN_FilterMode_IdList:�б�ģʽ
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //���ù�����λ��Ϊ32λ  
	if(std_id1!=std_id)
	{
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  		//���ñ�ʶ���Ĵ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;		//���ñ�ʶ���Ĵ������ֽ�	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=std_id1<<5; 	//�������μĴ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0|CAN_ID_STD;	//�������μĴ������ֽ�  
	}
	else
	{
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  	//���ñ�ʶ���Ĵ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;	//���ñ�ʶ���Ĵ������ֽ�	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((ext_id<<3)>>16) & 0xffff; 		//�������μĴ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=((ext_id<<3)& 0xffff) | CAN_ID_EXT;	//�������μĴ������ֽ�  
	}
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
 	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0	
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

	std_id = 0x100; std_id1 = 0x100|Physical_ADD[3];
 	CAN_FilterInitStructure.CAN_FilterNumber=9;	  //���ù�������1����ΧΪ0~13  
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;  //CAN_FilterMode_IdMask:����ģʽ  CAN_FilterMode_IdList:�б�ģʽ
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //���ù�����λ��Ϊ32λ  
	{
		CAN_FilterInitStructure.CAN_FilterIdHigh=std_id<<5;  		//���ñ�ʶ���Ĵ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterIdLow=0|CAN_ID_STD;		//���ñ�ʶ���Ĵ������ֽ�	
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=std_id1<<5; 	//�������μĴ������ֽ�  
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0|CAN_ID_STD;	//�������μĴ������ֽ�  
	}
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������1������FIFO0
 	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������1	
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��	
	
#if CAN_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	uint8_t SendCAN_Buf[8]={0};
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	if(g_RxMessFlag != 0xAA)
	{
		for(i=0;i<8;i++)	
		{
			//printf("%02X",RxMessage.Data[i]);
			g_RxMessage[i] = RxMessage.Data[i];
		}
		g_RxMessFlag = 0xAA;
		//if( g_RxMessFlag == 0xAA )//���յ������ݣ����豸���жϷ�ʽ����
		if((Logic_ADD==0)&&( g_RxMessFlag == 0xAA ))//���յ�������
		{
			if((g_RxMessage[0]==0xC1)\
				&&(g_RxMessage[1]==Physical_ADD[0])&&(g_RxMessage[2]==Physical_ADD[1])\
				&&(g_RxMessage[3]==Physical_ADD[2])&&(g_RxMessage[4]==Physical_ADD[3]))
			{
				Logic_ADD = g_RxMessage[5];	//ȡ��������߼���ַ
				Package_Send(0xC2,(u8 *)Physical_ADD);
			}
		}
		else if((Logic_ADD!=0)&&( g_RxMessFlag == 0xAA ))//���յ������ݣ����豸���жϷ�ʽ����
		{
			if(g_RxMessage[0] == 0xA4)	//������������
			{
				g_IAP_Flag = 0xAA;	//����±�־
				Flash_UpdateFlag = 0xAA;
			}
			else if(g_RxMessage[0] == 0xA2)	//�㲥 RFID���洢��Ϣ ���뼰��
			{
				FM1702_Key[0] = g_RxMessage[1];	FM1702_Key[1] = g_RxMessage[2];		FM1702_Key[2] = g_RxMessage[3];
				FM1702_Key[3] = g_RxMessage[4];	FM1702_Key[4] = g_RxMessage[5];		FM1702_Key[5] = g_RxMessage[6];
				FM1702_Key[6] = g_RxMessage[7]; Flash_UpdateFlag = 0xAA;
			}
			else if(g_RxMessage[0] == 0xA3)	//ˮ�ѹ㲥
			{
				WaterCost = g_RxMessage[1];  	CostNum = g_RxMessage[2];   //ˮ�ѡ�����
				Flash_UpdateFlag = 0xAA;
			}
			else if(g_RxMessage[0] == 0xC1)	//д���߼���ַ
			{
				if((g_RxMessage[1]==Physical_ADD[0])&&(g_RxMessage[2]==Physical_ADD[1])&&\
				   (g_RxMessage[3]==Physical_ADD[2])&&(g_RxMessage[4]==Physical_ADD[3]))
				{
					Logic_ADD = g_RxMessage[5];	//ȡ��������߼���ַ
					Package_Send(0xC2,(u8 *)SendCAN_Buf);
				}
			}
			else if((g_RxMessage[1] == Logic_ADD)&&(g_RxMessage[0] == 0x01))	//��ѭ��ʽ������
			{
				gErrorDat[0] = 0x00;	gErrorDat[1] = 0x00;	gErrorDat[2] = 0x00;
				g_LoseContact = 0;	//��.ʧ������ֵ��
				if(g_MemoryBuf[0][0]==0x00)			//�����ݰ�
				{
					Package_Send(0x02,(u8 *)SendCAN_Buf);
				}
				else if(g_MemoryBuf[0][0]==0xAA)	//�忨���ݰ�
				{
					SendCAN_Buf[0] = g_MemoryBuf[0][1];	//������1 ����0
					SendCAN_Buf[1] = g_MemoryBuf[0][2];	//������2 ����1
					SendCAN_Buf[2] = g_MemoryBuf[0][3];	//������3 ����2
					SendCAN_Buf[3] = g_MemoryBuf[0][4];	//������4 ����3
					Package_Send(0x03,(u8 *)SendCAN_Buf);Delay(0xFF);
					SendCAN_Buf[0] = g_MemoryBuf[0][5];	//������5 ���1
					SendCAN_Buf[1] = g_MemoryBuf[0][6];	//������6 ���2
					SendCAN_Buf[2] = g_MemoryBuf[0][7];	//������6 ���3
					SendCAN_Buf[3] = g_MemoryBuf[0][8];	//������7 У��
					SendCAN_Buf[4] = g_MemoryBuf[0][9];	//ͨ����
					Package_Send(0x04,(u8 *)SendCAN_Buf);//Delay(0xFF);
					PutOutMemoryBuf();
				}
				else if(g_MemoryBuf[0][0]==0x55)	//�ο����ݰ�
				{
					SendCAN_Buf[0] = g_MemoryBuf[0][1];	//������1 ����0
					SendCAN_Buf[1] = g_MemoryBuf[0][2];	//������2 ����1
					SendCAN_Buf[2] = g_MemoryBuf[0][3];	//������3 ����2
					SendCAN_Buf[3] = g_MemoryBuf[0][4];	//������4 ����3
					Package_Send(0x05,(u8 *)SendCAN_Buf);Delay(0xFF);
					SendCAN_Buf[0] = g_MemoryBuf[0][5];	//������5 ���1
					SendCAN_Buf[1] = g_MemoryBuf[0][6];	//������6 ���2
					SendCAN_Buf[2] = g_MemoryBuf[0][7];	//������6 ���3
					SendCAN_Buf[3] = g_MemoryBuf[0][8];	//������7 У��
					SendCAN_Buf[4] = g_MemoryBuf[0][9];	//ͨ����
					Package_Send(0x06,(u8 *)SendCAN_Buf);//Delay(0xFF);
					PutOutMemoryBuf();
				}
			}
			else if((g_RxMessage[1] == Logic_ADD)&&(g_RxMessage[0] == 0x07))	//����ȡ�߳ɹ�
			{
//				if((g_RxMessage[2] == g_MemoryBuf[0][1])&&(g_RxMessage[3] == g_MemoryBuf[0][2])&&\
//				   (g_RxMessage[4] == g_MemoryBuf[0][3])&&(g_RxMessage[5] == g_MemoryBuf[0][4])&&\
//				   (g_RxMessage[6] == g_MemoryBuf[0][0]))	//�忨/�ο���־
//				{
//					PutOutMemoryBuf();
//				}
			}
			else if((g_RxMessage[1] == Logic_ADD)&&(g_RxMessage[0] == 0x08))	//�������·�������ݰ� ����
			{
				re_RxMessage[0] = g_RxMessage[2];	//����0
				re_RxMessage[1] = g_RxMessage[3];	//����1
				re_RxMessage[2] = g_RxMessage[4];	//����2
				re_RxMessage[3] = g_RxMessage[5];	//����3
				re_RxMessage[4] = g_RxMessage[6];	//ͨ����
			}
			else if((g_RxMessage[1] == Logic_ADD)&&(g_RxMessage[0] == 0x09))	//�������·�������ݰ� ���
			{
				re_RxMessage[5] = g_RxMessage[2];	//���1
				re_RxMessage[6] = g_RxMessage[3];	//���2
				re_RxMessage[7] = g_RxMessage[4];	//���3
				re_RxMessage[8] = g_RxMessage[5];	//��У��
				re_RxMessage[9] = g_RxMessage[6];	//ͨ����
				re_RxMessage[10]= g_RxMessage[7];	//CRC8 �������ݰ���CRC
				if(re_RxMessage[4]==re_RxMessage[9])
				{
					if(re_RxMessage[10]==CRC8_Table(re_RxMessage,10))	//CRCУ��
							SendCAN_Buf[0] = 0x01;	//���ݣ�0x01��ʾ�ɹ���0x02��ʾʧ��
					else	SendCAN_Buf[0] = 0x02;	//���ݣ�0x01��ʾ�ɹ���0x02��ʾʧ��
				}
				else	SendCAN_Buf[0] = 0x02;	//���ݣ�0x01��ʾ�ɹ���0x02��ʾʧ��
				Package_Send(0x0A,(u8 *)SendCAN_Buf);
			}
			else if((g_RxMessage[1] == Logic_ADD)&&(g_RxMessage[0] == 0x11))	//�������·��쳣���ݰ�
			{
				gErrorDat[0] = g_RxMessage[3];		gErrorDat[1] = g_RxMessage[4];		gErrorDat[2] = g_RxMessage[5];
				SendCAN_Buf[0] = g_RxMessage[3];	SendCAN_Buf[1] = g_RxMessage[4];	SendCAN_Buf[2] = g_RxMessage[5];
				Package_Send(0x12,(u8 *)SendCAN_Buf);
			}
			else if((g_RxMessage[1] == Logic_ADD)&&(g_RxMessage[0] == 0x13))	//�������·��쳣���ݰ� �뿨���û����
			{
				gErrorDat[0] = g_RxMessage[2];	SendCAN_Buf[0] = g_RxMessage[2];//����1		
				gErrorDat[1] = g_RxMessage[3];	SendCAN_Buf[1] = g_RxMessage[3];//����2				
				gErrorDat[2] = g_RxMessage[4];	SendCAN_Buf[2] = g_RxMessage[4];//����3		
				gErrorDat[3] = g_RxMessage[5];	SendCAN_Buf[3] = g_RxMessage[5];//����4		
				gErrorDat[4] = g_RxMessage[6];	SendCAN_Buf[4] = g_RxMessage[6];//�������		
				gErrorDat[5] = g_RxMessage[7];	SendCAN_Buf[5] = g_RxMessage[7];//ͨ����		
				Package_Send(0x14,(u8 *)SendCAN_Buf);
			}
			for(i=0;i<8;i++)	SendCAN_Buf[i] = 0x00;
			g_RxMessFlag = 0x00;
		}	
	}
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ16)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;	����,ʧ��;
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	if(msg[0]==0xB3)	
		TxMessage.StdId=0x100|Physical_ADD[3];			// ��׼��ʶ��Ϊ0
	else if(msg[0]==0xC2)	
		TxMessage.StdId=0x100|Physical_ADD[3];			// ��׼��ʶ��Ϊ0
	else				
		TxMessage.StdId=0x200|msg[1];	// ��׼��ʶ��Ϊ0
	TxMessage.ExtId=0x1800F001;			// ������չ��ʾ����29λ��
	TxMessage.IDE=0;			// ��ʹ����չ��ʶ��
	TxMessage.RTR=0;		// ��Ϣ���ͣ�CAN_RTR_DataΪ����֡;CAN_RTR_RemoteΪԶ��֡
	TxMessage.DLC=len;		// ������֡��Ϣ
	for(i=0;i<8;i++)
	TxMessage.Data[i]=msg[i];	// ֡��Ϣ          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;	 ����,���յ����ݳ���;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);				//��ȡ����	
    for(i=0;i<8;i++)    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

void Package_Send(u8 _mode,u8 *Package_Dat)
{
	uint8_t res;
	uint8_t Package_SendBuf[8]={0x00};//���ͻ�����
	switch (_mode)
    {
    	case 0xB3:	//δע�����ݰ�
			Package_SendBuf[0] = 0xB3;
			Package_SendBuf[1] = Physical_ADD[0];	//�����ַ
 			Package_SendBuf[2] = Physical_ADD[1];
			Package_SendBuf[3] = Physical_ADD[2];
			Package_SendBuf[4] = Physical_ADD[3];
			Package_SendBuf[5] = CRC8_Table(Physical_ADD,4);
			break;
    	case 0xC2:	//ע��ظ� (д���߼���ַ�ظ�)
			Package_SendBuf[0] = 0xC2;
			Package_SendBuf[1] = Physical_ADD[0];	//�����ַ
 			Package_SendBuf[2] = Physical_ADD[1];
			Package_SendBuf[3] = Physical_ADD[2];
			Package_SendBuf[4] = Physical_ADD[3];
			Package_SendBuf[5] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[6] = VERSION;	//�汾��
    		break;
    	case 0x02:	//��ѭ�ظ� ������
 			Package_SendBuf[0] = 0x02;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = VERSION;	//�汾��
			break;
    	case 0x03:	//��ѭ�ظ� �忨����1
 			Package_SendBuf[0] = 0x03;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = Package_Dat[0];	//����1
 			Package_SendBuf[3] = Package_Dat[1];	//����2
			Package_SendBuf[4] = Package_Dat[2];	//����3
			Package_SendBuf[5] = Package_Dat[3];	//����4
			break;
    	case 0x04:	//��ѭ�ظ� �忨����2
 			Package_SendBuf[0] = 0x04;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = Package_Dat[0];	//���1
 			Package_SendBuf[3] = Package_Dat[1];	//���2
			Package_SendBuf[4] = Package_Dat[2];	//���3
			Package_SendBuf[5] = Package_Dat[3];	//��У��
			Package_SendBuf[6] = Package_Dat[4];	//ͨ����
			break;
    	case 0x05:	//��ѭ�ظ� �ο�����1
 			Package_SendBuf[0] = 0x05;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = Package_Dat[0];	//����1
 			Package_SendBuf[3] = Package_Dat[1];	//����2
			Package_SendBuf[4] = Package_Dat[2];	//����3
			Package_SendBuf[5] = Package_Dat[3];	//����4
    		break;
    	case 0x06:	//��ѭ�ظ� �ο�����2
 			Package_SendBuf[0] = 0x06;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = Package_Dat[0];	//���1
 			Package_SendBuf[3] = Package_Dat[1];	//���2
			Package_SendBuf[4] = Package_Dat[2];	//���3
			Package_SendBuf[5] = Package_Dat[3];	//��У��
			Package_SendBuf[6] = Package_Dat[4];	//ͨ����
    		break;
    	case 0x0A:	//д��ظ�
 			Package_SendBuf[0] = 0x0A;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = Package_Dat[0];	//���ݣ�0x01��ʾ�ɹ���0x02��ʾʧ��
    		break;
    	case 0x12:	//�쳣�ظ�
 			Package_SendBuf[0] = 0x12;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = 0x45;				//����1 "E"
 			Package_SendBuf[3] = Package_Dat[0];	//����2
			Package_SendBuf[4] = Package_Dat[1];	//����3
			Package_SendBuf[5] = Package_Dat[2];	//����4
    		break;
    	case 0x14:	//�쳣�ظ�
 			Package_SendBuf[0] = 0x14;
			Package_SendBuf[1] = Logic_ADD;	//�߼���ַ
			Package_SendBuf[2] = Package_Dat[0];	//����1
 			Package_SendBuf[3] = Package_Dat[1];	//����2
			Package_SendBuf[4] = Package_Dat[2];	//����3
			Package_SendBuf[5] = Package_Dat[3];	//����4
			Package_SendBuf[6] = Package_Dat[4];	//�������
    		break;
    	default:
    		break;
    }
	res = Can_Send_Msg(Package_SendBuf,8);//����8���ֽ�
	if(res)	{printf(" F, ");}	
	else 	{printf(" S, ");}	
}



