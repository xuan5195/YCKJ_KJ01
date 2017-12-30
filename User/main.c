#include "bsp.h"

static void InitBoard(void);
static void Delay (uint16_t nCount);

uint8_t Time_250ms=0;
uint8_t ShowFlag=0x00;
uint16_t ShowCount=0; 	//������������ʱ������ʾ
uint8_t gErrorShow=0;	//�쳣��ʾ���� �ڷ�����δ����ǰ��ʾʹ�ã������������E000
uint8_t gErrorDat[4]={0};	//�쳣����洢
uint8_t Logic_ADD=0;	//�߼���ַ
uint8_t Physical_ADD[4]={0x20,0x17,0x12,0x01};//�����ַ
uint8_t WaterCost=50,CostNum=29;	//WaterCost=ˮ�� ��С�ۿ���  //������
uint8_t g_RxMessage[8]={0};	//CAN��������
uint8_t g_RxMessFlag=0;		//CAN�������� ��־
//uint8_t OldOutFlag = 0;	//���ڱ�ǰο�����
uint8_t OutFlag=0,PowerUpFlag=0;	//��ˮ��־,�ϵ��־
uint8_t g_MemoryBuf[5][10]={0};	//���ݻ��棬[0]=0xAA��ʾ�в忨���ݣ�[0]=0xBB��ʾ�аο����ݣ�[1-4]���ţ�[5-7]��[8]�������룻[9]ͨ����
uint8_t FlagBit = 0x00;		//ͨ�ű�־λ��ÿ���뿨һ�����ݼ�1����ֵ�ﵽ199ʱ����0
uint16_t Beforeupdate = 0;	//Զ�̸���ǰ�Ŀ۷ѽ��;
uint16_t OvertimeCount=0;	//��ʱ���� 15000Ϊ1����
uint8_t OvertimeMinutes=0,OvertimeFlag=0;	//��ʱ���ӣ���ʱ��־
uint8_t BeforeFlag = 0xAA;	//���±�־ 0xAA��ʾδ����; 
u8 InPutCount=0;	//�����������
uint8_t re_RxMessage[16]={0};
uint32_t RFID_Money=0,OldRFID_Money = 0,u32TempDat=0,RFID_MoneyTemp=0;	//���ڽ��
extern unsigned char FM1702_Key[7];
extern unsigned char UID[5];
extern unsigned char FM1702_Buf[16];

void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
}

void RCC_Configuration(void)
{
//    RCC_DeInit();
//    RCC_HSEConfig(RCC_HSE_OFF);
//    RCC_HSICmd(ENABLE);
    RCC_HSEConfig(RCC_HSE_ON);
}
void PutOutMemoryBuf(void)	//���һ������
{
	uint8_t i,j;
	for(i=0;i<4;i++)
	{
		for(j=0;j<10;j++)
		{
			g_MemoryBuf[i][j] = g_MemoryBuf[i+1][j];
		}
	}
	for(j=0;j<10;j++)
	{
		g_MemoryBuf[4][j] = 0x00;
	}	
}
u8 SeekMemoryDat(u8 *Buf)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
		if((g_MemoryBuf[i][0]==Buf[0])&&(g_MemoryBuf[i][1]==Buf[1])\
          &&(g_MemoryBuf[i][2]==Buf[2])&&(g_MemoryBuf[i][3]==Buf[3])\
            &&(g_MemoryBuf[i][4]==Buf[4])&&(g_MemoryBuf[i][5]==Buf[5])\
              &&(g_MemoryBuf[i][6]==Buf[6])&&(g_MemoryBuf[i][7]==Buf[7])\
               &&(g_MemoryBuf[i][8]==Buf[8]) )	return i;
	} 
    return 0xF1;    //Error_Flag
}
void PutInMemoryBuf(u8 *Buf)	//ĩβ�����һ������
{
	uint8_t i,j;
	if(SeekMemoryDat((u8 *)Buf)==0xF1)  //��ѯ�Ƿ��ڻ����б���
    {
        if(g_MemoryBuf[4][0]!=0)	//Ŀǰ�Ѿ�����������һ����
        {
            for(i=0;i<4;i++)
            {
                for(j=0;j<10;j++)
                {
                    g_MemoryBuf[i][j] = g_MemoryBuf[i+1][j];
                }
            }
			if(FlagBit==200)FlagBit=0;else ++FlagBit;	//ͨ����
            for(j=0;j<9;j++)	g_MemoryBuf[4][j] = Buf[j];//����ĩβ
			g_MemoryBuf[4][9] = FlagBit;
        }
        else
        {
            for(i=0;i<5;i++)
            {
                if(g_MemoryBuf[i][0]!=0)	continue;
                else
                {
					if(FlagBit==200)FlagBit=0;else ++FlagBit;	//ͨ����
                    for(j=0;j<9;j++)	g_MemoryBuf[i][j] = Buf[j];//����ĩβ
					g_MemoryBuf[i][9] = FlagBit;
                    break;
                }
            }
        }    
    }
	
}

void Init_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{
	uint8_t DelayCount=0;
	uint8_t PUSendCount = 0x00;	//�ϵ緢�������������
	uint8_t CardUnReadFlag=0xAA;	//������־��������ʱִ��ȡ������
	uint8_t OldeUID[5];			//���ţ�OldeUID[4]ΪУ������
	uint8_t MemoryBuffer[10]={0};	//���ݻ��棬[0]=0xAA��ʾ�в忨���ݣ�[0]=0xBB��ʾ�аο����ݣ�[1-4]���ţ�[5-7]��[8]�������룻[9]Ԥ��
	uint8_t RFID_Count=0;	//������ˮ����ʱʱ��;3��
	uint8_t ErrorTemp[5]={0};
	uint8_t UseErrFlag = 0x00;	//�û����쳣��־,���յ��������������ݺ����ʾ�쳣����ֵ ������ʾ��E000��
	uint8_t i=0;
	uint32_t RFID_Money_Dat=0;	//���ڽ��,���ڴ���ʱʹ��
	uint8_t CardInFlag = 0,OldCardInFlag = 0;	//���ڱ�ǲ忨���ο�����
	
	InitBoard();			//Ӳ����ʼ��
	
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������450Kbps    
	printf("Starting Up...\r\n");

	BspTm1639_Show(0x01,0x00);
	ShowFlag = 0xAA;
	//Logic_ADD = 1;	//����ʹ��
	//PowerUpFlag=0xAA;	//����ʹ��
	while(Logic_ADD==0)	//�߼���ַΪ0ʱ����ʾ���豸δע�ᣬ����ȴ�ע�����
	{
		if(PowerUpFlag==0xAA)
		{
			DelayCount = Read_Rand();  //����0-50�������
			while(DelayCount--)		Delay(0x8F);	//����ʱ
			if(PUSendCount<30)	{	Package_Send(0xB3,(u8 *)Physical_ADD);	PUSendCount++;	Delay(0xFF);}	
			else
			{
				PUSendCount = 200;
			}				
		}

		if( g_RxMessFlag == 0xAA )//���յ�������
		{
			if((g_RxMessage[0]==0xA1)&&(g_RxMessage[1]==0xA1))	//δע��㲥
			{
				PowerUpFlag=0xAA;//���������ʱδע��ظ�
			}
			else if((g_RxMessage[0]==0xC1)\
				&&(g_RxMessage[1]==Physical_ADD[0])&&(g_RxMessage[2]==Physical_ADD[1])\
				&&(g_RxMessage[3]==Physical_ADD[2])&&(g_RxMessage[4]==Physical_ADD[3]))
			{
				Logic_ADD = g_RxMessage[5];	//ȡ��������߼���ַ
				Package_Send(0xC2,(u8 *)Physical_ADD);
			}
			g_RxMessFlag = 0x00;
		}
		
		if( ReadRFIDCard() == FM1702_OK )						//ֻ���ڼ���Ƿ��п�����У������
		{	BspTm1639_Show(0x06,0x01);	Delay(1500);	}		//��ʾ�����ʾ���� ---XX--- //����汾��
		else
		{
			if(ShowCount<1980)			BspTm1639_ShowSNDat(0x11,(u8 *)Physical_ADD);
			else if(ShowCount<2000)		BspTm1639_Show(0x00,0x00);		//����ʾ
			else if(ShowCount<3980)		BspTm1639_Show(0x01,0x00);		//��ʾ����
			else						BspTm1639_Show(0x00,0x00);		//����ʾ			
		}
	}
	CAN_DeInit(CAN1);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������450Kbps    
	BspTm1639_Show(0x03,WaterCost);	//��ʾÿ��ˮ���ֵ
	bsp_StartTimer(1, 200);		//��ʱ��1���� 200����
	while (1)
	{
		CPU_IDLE();
		if ( bsp_CheckTimer(1) )	//��ʱ��
		{
			bsp_StartTimer(1, 200);	//�����¸���ʱ����/
			if((gErrorDat[0]==0x30)||(gErrorDat[0]==0x31)||(gErrorDat[0]==0x32)||(gErrorDat[0]==0x33))	//�쳣���
			{//01.��ʾ�쳣����
				ShowFlag = 0x00;
				ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
				ErrorTemp[1] = gErrorDat[0]-0x30;
				BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);
				InPutCount = 0;
				OutPut_OFF();	//�ر�ˮ������ˮ
			}
			else
			{
				if( ReadRFID() == FM1702_OK )	CardUnReadFlag = 0;	//���� δ����������
				else if(CardUnReadFlag<100)		CardUnReadFlag ++;	//�ۼ� δ����������
				if( (CardUnReadFlag<3) )	//������
				{
					if((OldeUID[0]!=UID[0])||(OldeUID[1]!=UID[1])||(OldeUID[2]!=UID[2])||(OldeUID[3]!=UID[3]))
					{	//���Ų�һ��
						if(OldCardInFlag != CardInFlag)	//�հο�
						{
							MemoryBuffer[0] = 0x55;		//���հγ�
							MemoryBuffer[5] = RFID_Money>>16;	//������5 ���1��λ
							MemoryBuffer[6] = RFID_Money>>8;	//������6 ���2
							MemoryBuffer[7] = RFID_Money;		//������7 ���3
							MemoryBuffer[8] = RFID_Money>>24;	//������8 У��
							if((RFID_Money&0x00FFFFFF) > 0x00EA0000);	//�����쳣ʱ�����ύ�ο�����
							else PutInMemoryBuf((u8 *)MemoryBuffer);
							MemoryBuffer[0] = 0x00;		
							OldCardInFlag = CardInFlag;
						}
						OldeUID[0]=UID[0];OldeUID[1]=UID[1];OldeUID[2]=UID[2];OldeUID[3]=UID[3];
						CardUnReadFlag=0xAA;	//��־��Ϊδ������
						OutFlag = 0x00;			//��ˮ��־ 0x00->ֹͣ��ˮ
						//CardFlag = 0x00;
						RFID_Count=0; 
						continue;
					}
					if(RFID_Count<1)//1*200=200ms
					{
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//��ȡ���ڽ�� ʹ�õ�ַ��FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//������ȷ
						else				{ RFID_Money = OldRFID_Money; RFID_Count=0; continue; }		//�������� �˳����¶���
						OldRFID_Money = RFID_Money;				
						CardInFlag = 0xAA;	//�����ɹ�					
						BspTm1639_Show(0x00,0x00);		//����ʾ
					}
					else if(RFID_Count<6)//6*200=1200ms
					{	//��ʾԭ���ڽ��, ������0.4��ʱ���ύ�忨����
						if( (RFID_Money&0x00FFFFFF) == (ErrorMoney+7) )	//�������ݴ��ڽ��ֵΪ�û��쳣���� E700
						{	//E700���ύ�忨����
							InPutCount = 0;	OutPut_OFF();		//�ر�ˮ����ֹͣ��ˮ
							RFID_Count=200;		ErrorTemp[1] = 0x07;	ErrorTemp[0] = 0x00;
							BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//��ʾ�쳣����						
						}
						else if( (RFID_Money&0x00FFFFFF) >= ErrorMoney )	//�û��쳣���� E000
						{
							InPutCount = 0;	OutPut_OFF();		//�ر�ˮ����ֹͣ��ˮ
							if(UseErrFlag==0x00)
							{
								ErrorTemp[0] = gErrorShow%10;	ErrorTemp[1] = gErrorShow/10;	//��λ����λ
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//��ʾ�쳣����
							}
							else
							{
								ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);	//ʮλ����λ
								ErrorTemp[1] = gErrorDat[0]-0x30;	//��λ
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//.��ʾ�쳣����
							}
							CardInFlag = 0xAA;	//�����ɹ�	
							if(OldCardInFlag != CardInFlag)	//�ղ���
							{
								RFID_Money_Dat = ReadRFIDMoney(FM1702_Key[6]);	//�������ڽ�� �����쳣ʱ���ؿ����
								if(ErrorMoney==RFID_Money_Dat)	continue;	//�������󣬷����˳�
								MemoryBuffer[0] = 0xAA;		//���ղ���
								MemoryBuffer[1] = UID[0];	//����0��
								MemoryBuffer[2] = UID[1];	//����1��
								MemoryBuffer[3] = UID[2];	//����2��
								MemoryBuffer[4] = UID[3];	//����3��
								MemoryBuffer[5] = RFID_Money_Dat>>16;	//������5 ���1��λ
								MemoryBuffer[6] = RFID_Money_Dat>>8;	//������6 ���2
								MemoryBuffer[7] = RFID_Money_Dat;		//������7 ���3
								MemoryBuffer[8] = RFID_Money_Dat>>24;	//������8 У��
								PutInMemoryBuf((u8 *)MemoryBuffer);
								MemoryBuffer[0] = 0x00;	
								RFID_Money_Dat = 0;
								OldCardInFlag = CardInFlag;
							}							
						}
						else	//��������Ϊ�������,�ύ�忨����
						{
							CardInFlag = 0xAA;	//�����ɹ�								
							BspTm1639_Show(0x04,RFID_Money);	//�Ʒѿۿ�ģʽ
							if(OldCardInFlag != CardInFlag)	//�ղ���
							{
								MemoryBuffer[0] = 0xAA;		//���ղ���
								MemoryBuffer[1] = UID[0];	//����0��
								MemoryBuffer[2] = UID[1];	//����1��
								MemoryBuffer[3] = UID[2];	//����2��
								MemoryBuffer[4] = UID[3];	//����3��
								MemoryBuffer[5] = RFID_Money>>16;	//������5 ���1��λ
								MemoryBuffer[6] = RFID_Money>>8;	//������6 ���2
								MemoryBuffer[7] = RFID_Money;		//������7 ���3
								MemoryBuffer[8] = RFID_Money>>24;	//������8 У��
								PutInMemoryBuf((u8 *)MemoryBuffer);
								MemoryBuffer[0] = 0x00;	
								OldCardInFlag = CardInFlag;
							}							
						}						
						
					}
					else if(RFID_Count<15)//15*200=3000ms
					{	//��ʱ��ˮ
						if( (RFID_Money&0x00FFFFFF) > 0x00EA0000 )	//�������ݴ��ڽ��ֵΪ�û��쳣����
						{
							InPutCount = 0;	OutPut_OFF();		//�ر�ˮ������ˮ
							if(UseErrFlag==0x00)
							{
								ErrorTemp[0] = gErrorShow%10;	ErrorTemp[1] = gErrorShow/10;	//��λ//��λ
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.��ʾ�쳣����
							}							
							else
							{
								ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
								ErrorTemp[1] = gErrorDat[0]-0x30;
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.��ʾ�쳣����
							}
						}
						else	BspTm1639_Show(0x04,RFID_Money);	//�Ʒѿۿ�ģʽ				
					}
					

					if((RFID_Money&0x00FFFFFF) > 0x00EA0000)//�������ݴ��ڽ��ֵΪ�û��쳣����
					{
						if(gErrorDat[0]!=0x00)
						{
							InPutCount = 0;	OutPut_OFF();		//�ر�ˮ������ˮ
							ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
							ErrorTemp[1] = gErrorDat[0]-0x30;
							BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.��ʾ�쳣����
						}
						if(OldCardInFlag != CardInFlag)	
						{
							RFID_Money_Dat = ReadRFIDMoney(FM1702_Key[6]);	//�������ڽ�� �����쳣ʱ���ؿ����
							MemoryBuffer[0] = 0xAA;		//���ղ���
							MemoryBuffer[1] = UID[0];	//����0��
							MemoryBuffer[2] = UID[1];	//����1��
							MemoryBuffer[3] = UID[2];	//����2��
							MemoryBuffer[4] = UID[3];	//����3��
							MemoryBuffer[5] = RFID_Money_Dat>>16;	//������5 ���1��λ 
							MemoryBuffer[6] = RFID_Money_Dat>>8;	//������6 ���2	
							MemoryBuffer[7] = RFID_Money_Dat;		//������7 ���3	
							MemoryBuffer[8] = RFID_Money_Dat>>24;	//������8 У��
							PutInMemoryBuf((u8 *)MemoryBuffer);
							MemoryBuffer[0] = 0x00;
							RFID_Money_Dat = 0;							
							OutFlag = 0xAA;	//��ˮ��־
							OldCardInFlag = CardInFlag;
						}
					}
					else if((RFID_Count>15)&&((RFID_Money&0x00FFFFFF)>=WaterCost))	//���ڽ�������С�ۿ����
					{
						if(OutFlag == 0x00)	
						{
							OutFlag = 0xAA;	//��ˮ��־
							RFID_Money = DecMoney(FM1702_Key[6],WaterCost);
							if(ErrorMoney==RFID_Money)	{	RFID_Count=0;	continue;	}
							if((OldRFID_Money-RFID_Money)>(WaterCost*2))	RFID_Money = OldRFID_Money;
							else 											OldRFID_Money = RFID_Money; 
							Beforeupdate = WaterCost;	//����ǰ�۷� ���ο۷�
							//OldOutFlag = OutFlag;
						}
						if(InPutCount>=CostNum)
						{
							InPutCount = InPutCount-CostNum;
							OvertimeCount=0;OvertimeMinutes=0;//�п۷ѣ��峬ʱ��־
							RFID_Money = DecMoney(FM1702_Key[6],WaterCost);
							if(ErrorMoney==RFID_Money)	{	RFID_Count=0; continue;	}
							if(RFID_Money != 0x00)	//��ֹ���ʱ��ΰγ������������Ϊ0
							{
								OldRFID_Money = RFID_Money;
								if(BeforeFlag == 0xAA)	Beforeupdate = Beforeupdate + WaterCost;	//����ǰ�۷�
								if(Beforeupdate>(10*WaterCost))	Beforeupdate = 10*WaterCost;	
							}
							else
							{
								RFID_Money = OldRFID_Money;
							}						
						}
						BspTm1639_Show(0x04,RFID_Money);			//�Ʒѿۿ�ģʽ
						if(OvertimeFlag == 0x00)	OutPut_ON();	//��ˮ������ˮ
						else						OutPut_OFF();	//�ر�ˮ����ֹͣ��ˮ
					}
					if(RFID_Count<100)	RFID_Count++;	//100*200=10,000ms
					
					if(((re_RxMessage[4]==re_RxMessage[9]))&&(re_RxMessage[10]==CRC8_Table(re_RxMessage,10)))	//CRCУ��
					{
						if((re_RxMessage[0] == UID[0])&&(re_RxMessage[1] == UID[1])&&
						   (re_RxMessage[2] == UID[2])&&(re_RxMessage[3] == UID[3])&&\
						   (re_RxMessage[4] == FlagBit))
						{
							u32TempDat = (((u32)re_RxMessage[5]<<16)|((u32)re_RxMessage[6]<<8)|re_RxMessage[7]);	//��ȡ�ƶ˷�������
							if((u32TempDat<=WaterCost)||(Beforeupdate>=u32TempDat))	u32TempDat = 0x00;	//���Ϊ0��ֱ��д��0��
							else													u32TempDat = u32TempDat-Beforeupdate;
							u32TempDat = u32TempDat | ((u32)re_RxMessage[8]<<24);
							u32TempDat = u32TempDat;
							if( WriteMoney(FM1702_Key[6],u32TempDat) == FM1702_OK )	//д���� �ɹ�
							{
								Beforeupdate = 0x00;	//��ո�������ǰ�Ŀ۷ѽ��	
								BeforeFlag = 0x00;		//���ݸ������
								Delay(0xFF);			//��ʱ
								RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//��ȡ���ڽ�� ʹ�õ�ַ��FM1702_Key[6]
								if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//������ȷ
								else	{ RFID_Money = OldRFID_Money;OldRFID_Money = RFID_Money;}	//�ݴ濨�ڽ��
								BspTm1639_Show(0x04,RFID_Money);	//�Ʒѿۿ�ģʽ	
							}
						}
						for(i=0;i<16;i++)	re_RxMessage[i] = 0x00;
					}
					if((gErrorDat[0]==0x34)||(gErrorDat[0]==0x35)||(gErrorDat[0]==0x36)||(gErrorDat[0]==0x37))	//�쳣���
					{
						WriteMoney( FM1702_Key[6] , ErrorMoney+( (gErrorDat[0]-0x30)*10 + (gErrorDat[2]-0x30)) );
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//��ȡ���ڽ�� ʹ�õ�ַ��FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)		RFID_Money = RFID_MoneyTemp;	//������ȷ
						else	{ RFID_Money = OldRFID_Money;	OldRFID_Money = RFID_Money;	}	//�ݴ濨�ڽ��
					}
					
				}
				else	//û�ж�����
				{
					CardUnReadFlag=0xAA;RFID_Count=0;
					if((gErrorDat[0]==0x34)||(gErrorDat[0]==0x35))	//���ڿ���ػ��û���أ��ڿ�ȡ�ߺ����쳣�����ֹ�쳣������˸
					{gErrorDat[0] = 0x00;	gErrorDat[1] = 0x00;	gErrorDat[2] = 0x00;}	
					//CardFlag = 0x00;ReceiveFlag = 0x00;
					ShowFlag = 0xAA;
					UseErrFlag = 0x00;	//���û����쳣��־
					CardInFlag = 0x00;
					if(OldCardInFlag != CardInFlag)	//�հο�
					{
						MemoryBuffer[0] = 0x55;		//���հγ�
						MemoryBuffer[5] = RFID_Money>>16;	//������5 ���1��λ
						MemoryBuffer[6] = RFID_Money>>8;	//������6 ���2
						MemoryBuffer[7] = RFID_Money;		//������7 ���3
						MemoryBuffer[8] = RFID_Money>>24;	//������8 У��
						if((RFID_Money&0x00FFFFFF) > 0x00EA0000);	//�����쳣ʱ�����ύ�ο�����
						else PutInMemoryBuf((u8 *)MemoryBuffer);
						MemoryBuffer[0] = 0x00;		
						OldCardInFlag = CardInFlag;
					}
					OutFlag = 0x00;
					Beforeupdate = 0x00;	BeforeFlag = 0xAA;
					InPutCount = 0;
					OvertimeFlag = 0x00;
					OvertimeCount=0;OvertimeMinutes=0;//�п۷ѣ��峬ʱ��־
					OutPut_OFF();	//�ر�ˮ������ˮ
					if(ShowCount<1980)			BspTm1639_ShowSNDat(0x11,(u8 *)Physical_ADD);
					else if(ShowCount<2000)		BspTm1639_Show(0x00,0x00);		//����ʾ
					else if(ShowCount<3980)		BspTm1639_Show(0x03,WaterCost);	//��ʾÿ��ˮ���ֵ
					else						BspTm1639_Show(0x00,0x00);		//����ʾ	
					if(ShowCount==2000)	{	SoftReset1702();	}//���RFIDģ��
				}
			
			}
		}

	}
}

/*
*********************************************************************************************************
*	�� �� ��: InitBoard
*	����˵��: ��ʼ��Ӳ���豸
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitBoard(void)
{	
	RCC_Configuration();
	/* ��ʼ��systick��ʱ������������ʱ�ж� */
	bsp_InitTimer(); 
	NVIC_Configuration();
	
	bsp_InitUart(); 	/* ��ʼ������ */
	BspTm1639_Config();	//TM1639��ʼ��
	BspFM1701_Config();	//FM1701 GPIO��ʼ��
	TIM3_Int_Init(99,1599);//10Khz�ļ���Ƶ�ʣ�������100  

 	Adc_Init();		  		//ADC��ʼ��
	
}


//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{
		if( ShowFlag == 0xAA )
		{
			if( ShowCount < 4000 )  ShowCount++;
			else                    ShowCount = 0;
		}
		else    ShowCount = 0;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 					
	}
}

void Delay(__IO uint16_t nCount)
{
	while (nCount != 0)
	{
		nCount--;
	}
}



