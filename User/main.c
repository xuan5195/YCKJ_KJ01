//更新说明：
//2018.01.24  V5.1 增加在未注册时也能够远程复位
//2018.01.26  V5.1 增加在未注册时3min自动软复位，提高带电接入卡机时上线成功率
//2018.01.27  V5.1 修复上卡金额为0、错误代码E300时，刷卡造成放水不停止、不计费及显示金额错误等问题。
//2018.02.03  V5.1 增加在刷卡过程与服务器通信上金额界面闪烁一下。
//2018.02.23  V5.2 修改CAN波特率为50kbps;失联软复位时间由原来3min改为10Sec。

#include "bsp.h"
#include <stdlib.h>  
#define IAP_Flag	1		//IAP标志 0表示没使用在线升级功能


static void InitBoard(void);
void Delay (uint16_t nCount);

uint8_t Time_250ms=0;
uint8_t ShowFlag=0x00;
uint16_t ShowCount=0; 	//用于正常待机时交替显示
uint8_t gErrorShow=0;	//异常显示代码 在服务器未更新前显示使用，这样不会出现E000
uint8_t gErrorDat[6]={0};	//异常代码存储
uint8_t Logic_ADD=0;		//逻辑地址
uint8_t g_RxMessage[8]={0};	//CAN接收数据
uint8_t g_RxMessFlag=0;		//CAN接收数据 标志
uint8_t OutFlag=0,PowerUpFlag=0;	//放水标志,上电标志
uint8_t g_MemoryBuf[5][10]={0};	//数据缓存，[0]=0xAA表示有插卡数据，[0]=0xBB表示有拔卡数据，[1-4]卡号；[5-7]金额；[8]卡核验码；[9]通信码
uint8_t FlagBit = 0x00;		//通信标志位，每插入卡一次数据加1，数值达到199时，清0
uint16_t Beforeupdate = 0;	//远程更新前的扣费金额;
uint16_t OvertimeCount=0;	//超时计数
uint8_t Time20msCount=0;	//
uint8_t OvertimeMinutes=0;	//超时分钟，超时标志 0xAA表示超时
uint8_t BeforeFlag = 0xAA;	//更新标志 0xAA表示未更新; 
uint8_t InPutCount=0;		//输入脉冲计数
uint8_t re_RxMessage[16]={0};
uint32_t RFID_Money=0,OldRFID_Money = 0,u32TempDat=0,RFID_MoneyTemp=0;	//卡内金额
uint8_t CardInFlag;
uint8_t Flash_UpdateFlag=0x00;	//Flash有数据更新标志，0xAA表示有数据要更新
uint8_t DelayCount=0;			//上电随机延时
uint8_t g_LoseContact=0;		//失联计数，大于200时，表示失联，自动复位
extern uint8_t UID[5];
extern uint8_t FM1702_Buf[16];
extern uint8_t Physical_ADD[4];//物理地址
extern uint8_t FM1702_Key[7];
extern uint8_t WaterCost,CostNum;	//WaterCost=水费 最小扣款金额  //脉冲数
extern uint8_t g_IAP_Flag;	//在线升级标志

void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
}

void RCC_Configuration(void)
{
//    RCC_DeInit();
//    RCC_HSEConfig(RCC_HSE_OFF);
//    RCC_HSICmd(ENABLE);
    RCC_HSEConfig(RCC_HSE_ON);
}
void PutOutMemoryBuf(void)	//清第一个缓存
{
	uint8_t i,j;
	for(i=0;i<4;i++)
		for(j=0;j<10;j++)
			g_MemoryBuf[i][j] = g_MemoryBuf[i+1][j];
	for(j=0;j<10;j++)
		g_MemoryBuf[4][j] = 0x00;
}
u8 SeekMemoryDat(u8 *Buf)
{
	uint8_t i;
	for(i=0;i<5;i++)
		if((g_MemoryBuf[i][0]==Buf[0])&&(g_MemoryBuf[i][1]==Buf[1])\
          &&(g_MemoryBuf[i][2]==Buf[2])&&(g_MemoryBuf[i][3]==Buf[3])\
            &&(g_MemoryBuf[i][4]==Buf[4])&&(g_MemoryBuf[i][5]==Buf[5])\
              &&(g_MemoryBuf[i][6]==Buf[6])&&(g_MemoryBuf[i][7]==Buf[7])\
               &&(g_MemoryBuf[i][8]==Buf[8]) )	return i;
    return 0xF1;    //Error_Flag
}
void PutInMemoryBuf(u8 *Buf)	//末尾加入第一个缓存
{
	uint8_t i,j;
	if(SeekMemoryDat((u8 *)Buf)==0xF1)  //查询是否在缓存列表中
    {
        if(g_MemoryBuf[4][0]!=0)	//目前已经存满，丢第一个；
        {
            for(i=0;i<4;i++)
                for(j=0;j<10;j++)
                    g_MemoryBuf[i][j] = g_MemoryBuf[i+1][j];
			if(FlagBit==200)FlagBit=0;else ++FlagBit;	//通信码
            for(j=0;j<9;j++)	g_MemoryBuf[4][j] = Buf[j];//存入末尾
			g_MemoryBuf[4][9] = FlagBit;
        }
        else
        {
            for(i=0;i<5;i++)
            {
                if(g_MemoryBuf[i][0]!=0)	continue;
                else
                {
					if(FlagBit==200)FlagBit=0;else ++FlagBit;	//通信码
                    for(j=0;j<9;j++)	g_MemoryBuf[i][j] = Buf[j];//存入末尾
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

void SoftReset(void)
{
	__set_FAULTMASK(1);		// 关闭所有中端
	NVIC_SystemReset();		// 复位
}

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
	uint8_t PUSendCount = 0x00;	//上电发送随机数次数；
	uint8_t CardUnReadFlag=0xAA;	//读卡标志，用于延时执行取卡动作
	uint8_t OldeUID[5];			//卡号，OldeUID[4]为校验数据
	uint8_t MemoryBuffer[10]={0};	//数据缓存，[0]=0xAA表示有插卡数据，[0]=0xBB表示有拔卡数据，[1-4]卡号；[5-7]金额；[8]卡核验码；[9]预留
	uint8_t RFID_Count=0;	//卡开启水阀延时时间;3秒
	uint8_t ErrorTemp[5]={0};
	uint8_t UseErrFlag = 0x00;	//用户或卡异常标志,接收到服务器返回数据后才显示异常代码值 否则显示‘E000’
	uint8_t i=0;
	uint32_t RFID_Money_Dat=0;	//卡内金额,用于错误时使用
	uint8_t OldCardInFlag = 0;	//用于标记插卡、拔卡动作
	
	SystemInit();
	
	#if IAP_Flag
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x5000); //设置中断向量表的位置在 0x5000，并且将Target的IROM1起始改为0x08005000.
	#endif
	
	InitBoard();			//硬件初始化
	Delay(0xFFFF); 	//上电简单延时一下  
	
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_1tq,90,CAN_Mode_Normal);	//CAN初始化正常模式,波特率40Kbps  //则波特率为:36M/((1+8+1)*90)= 40Kbps CAN_Normal_Init(1,13,1,60,1);   
//	CAN_Mode_Init(CAN_SJW_2tq,CAN_BS1_16tq,CAN_BS2_2tq,90,CAN_Mode_Normal);	//CAN初始化正常模式,波特率20Kbps  //则波特率为:36M/((2+16+2)*90)= 20Kbps CAN_Normal_Init(2,16,2,90,1);   
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_4tq,36,CAN_Mode_Normal);	//CAN初始化正常模式,波特率50Kbps  //则波特率为:36M/((1+15+4)*36)= 50Kbps CAN_Normal_Init(1,15,4,36,1);   
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_4tq,CAN_BS2_3tq,75,CAN_Mode_Normal);	//CAN初始化正常模式,波特率60Kbps  //则波特率为:36M/((1+2+1)*150)= 60Kbps CAN_Normal_Init(1,2,1,150,1);   
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,25,CAN_Mode_Normal);	//CAN初始化正常模式,波特率90Kbps    
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,18,CAN_Mode_Normal);	//CAN初始化正常模式,波特率125Kbps    
	printf("\r\nStarting Up...\r\nYCKJ-KJ01 V3.0...\r\n");
	printf("VersionNo: %02X...\r\n",VERSION);
	Read_Flash_Dat();	//读取Flash数据
	printf("Physical_ADD:%02X%02X%02X%02X;\r\n",Physical_ADD[0],Physical_ADD[1],Physical_ADD[2],Physical_ADD[3]);
	printf("FM1702_Key:%02X%02X%02X%02X%02X%02X; %02d;\r\n",FM1702_Key[0],FM1702_Key[1],FM1702_Key[2],FM1702_Key[3],FM1702_Key[4],FM1702_Key[5],FM1702_Key[6]);
	printf("WaterCost:0.%03d; CostNum:%02d; g_IAP_Flag:0x%02X;\r\n",WaterCost,CostNum,g_IAP_Flag);
	if(g_IAP_Flag == 0xAA)	//更新标志
	{
		g_IAP_Flag = 0x00;	//清更新标志
		Write_Flash_Dat();
		printf("g_IAP_Flag:0x%02X;\r\n",g_IAP_Flag);
	}
	BspTm1639_Show(0x01,0x00);
	ShowFlag = 0xAA;	//交替显示标志,0xAA为交替显示
	//Logic_ADD = 1;	//测试使用
	//PowerUpFlag=0xAA;	//测试使用
	srand((Physical_ADD[2]<<8)|(Physical_ADD[3]));	//使用物理地址后二位作为种子
	while(Logic_ADD==0)	//逻辑地址为0时，表示该设备未注册，进入等待注册过程
	{
		if(PowerUpFlag==0xAA)
		{
			if(DelayCount==0)
			{				
				DelayCount = rand()% 100;  //产生0-99的随机数
				if(Logic_ADD==0)
 				{
					if(PUSendCount<30)	{	Package_Send(0xB3,(u8 *)Physical_ADD);	PUSendCount++;	Delay(0xFF);}	
					else				{	PUSendCount = 200;	}
				}
			}				
		}

		if( g_RxMessFlag == 0xAA )//接收到有数据
		{
			if((g_RxMessage[0]==0xA1)&&(g_RxMessage[1]==0xA1))	{	PowerUpFlag=0xAA;	}//进入随机延时未注册回复
//			else if((g_RxMessage[0]==0xC1)\
//				&&(g_RxMessage[1]==Physical_ADD[0])&&(g_RxMessage[2]==Physical_ADD[1])\
//				&&(g_RxMessage[3]==Physical_ADD[2])&&(g_RxMessage[4]==Physical_ADD[3]))
//			{
//				Logic_ADD = g_RxMessage[5];	//取出分配的逻辑地址
//				Package_Send(0xC2,(u8 *)Physical_ADD);
//			}
			g_RxMessFlag = 0x00;
		}
		
		if( ReadRFIDCard() == FM1702_OK )						//只用于检测是否有卡，不校验密码
		{	BspTm1639_Show(0x06,0x01);	Delay(1500);	}		//显示代码表示到卡 ---XX--- //软件版本号
		else
		{
			if(ShowCount<148)			BspTm1639_ShowSNDat(0x11,(u8 *)Physical_ADD);
			else if(ShowCount<150)		BspTm1639_Show(0x00,0x00);		//关显示
			else if(ShowCount<298)		BspTm1639_Show(0x01,0x00);		//显示方框
			else						BspTm1639_Show(0x00,0x00);		//关显示			
		}
		if(Flash_UpdateFlag == 0xAA)
		{
			Write_Flash_Dat();
			Flash_UpdateFlag = 0;
			if(g_IAP_Flag == 0xAA)		SoftReset();//更新标志 软件复位
		}
		if(g_LoseContact>200)		SoftReset();//失联 软件复位 10秒钟累加一；在定时器累加
	}
	CAN_DeInit(CAN1);
	
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_1tq,90,CAN_Mode_Normal);	//CAN初始化正常模式,波特率40Kbps  //则波特率为:36M/((1+8+1)*90)= 40Kbps CAN_Normal_Init(1,13,1,60,1);   
//	CAN_Mode_Init(CAN_SJW_2tq,CAN_BS1_16tq,CAN_BS2_2tq,90,CAN_Mode_Normal);	//CAN初始化正常模式,波特率20Kbps  //则波特率为:36M/((2+16+2)*90)= 20Kbps CAN_Normal_Init(2,16,2,90,1);   
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_4tq,36,CAN_Mode_Normal);	//CAN初始化正常模式,波特率50Kbps  //则波特率为:36M/((1+15+4)*36)= 50Kbps CAN_Normal_Init(1,15,4,36,1);   
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_4tq,CAN_BS2_3tq,75,CAN_Mode_Normal);	//CAN初始化正常模式,波特率60Kbps  //则波特率为:36M/((1+2+1)*150)= 60Kbps CAN_Normal_Init(1,2,1,150,1);   
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,25,CAN_Mode_Normal);	//CAN初始化正常模式,波特率90Kbps    
//	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,18,CAN_Mode_Normal);//CAN初始化正常模式,波特率125Kbps    
	ShowCount = 0;g_RxMessFlag = 0x00;	
	BspTm1639_Show(0x03,WaterCost);	//显示每升水金额值
	bsp_StartTimer(1, 200);		//定时器1周期 200毫秒
	while (1)
	{
		CPU_IDLE();
		if ( bsp_CheckTimer(1) )	//软定时器
		{
			bsp_StartTimer(1, 200);	//启动下个定时周期/
			if((gErrorDat[0]==0x30)||(gErrorDat[0]==0x31)||(gErrorDat[0]==0x32)||(gErrorDat[0]==0x33))	//异常情况
			{//01.显示异常代码
				ShowFlag = 0x00;	//关交替显示标志,只显示异常代码
				ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
				ErrorTemp[1] = gErrorDat[0]-0x30;
				BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);
				InPutCount = 0;
				OutFlag = 0x00;	OutPut_OFF();	//关闭水阀，放水
			}
			else
			{
				if( ReadRFID() == FM1702_OK )	CardUnReadFlag = 0;	//清零 未读到卡计数
				else if(CardUnReadFlag<100)		CardUnReadFlag ++;	//累计 未读到卡计数
				if( (CardUnReadFlag<3) )	//读到卡
				{
					if((OldeUID[0]!=UID[0])||(OldeUID[1]!=UID[1])||(OldeUID[2]!=UID[2])||(OldeUID[3]!=UID[3]))
					{	//卡号不一致
						if(OldCardInFlag != CardInFlag)	//刚拔卡
						{
							MemoryBuffer[0] = 0x55;		//卡刚拔出
							MemoryBuffer[5] = RFID_Money>>16;	//数据域5 金额1高位
							MemoryBuffer[6] = RFID_Money>>8;	//数据域6 金额2
							MemoryBuffer[7] = RFID_Money;		//数据域7 金额3
							MemoryBuffer[8] = RFID_Money>>24;	//数据域8 校验
							if((RFID_Money&0x00FFFFFF) > 0x00EA0000);	//错误异常时，不提交拔卡动作
							else PutInMemoryBuf((u8 *)MemoryBuffer);
							MemoryBuffer[0] = 0x00;
							gErrorDat[0]=0;	gErrorDat[1]=0;	gErrorDat[2]=0;	gErrorDat[3]=0;	gErrorDat[4]=0;	gErrorDat[5]=0;
							OldCardInFlag = CardInFlag;
						}
						OldeUID[0]=UID[0];OldeUID[1]=UID[1];OldeUID[2]=UID[2];OldeUID[3]=UID[3];
						CardUnReadFlag=0xAA;	//标志改为未读到卡
						OutFlag = 0x00;			//放水标志 0x00->停止放水
						RFID_Count=0; 
						continue;
					}
					if(RFID_Count<1)//1*200=200ms
					{
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//读卡正确
						else				{ RFID_Money = OldRFID_Money; RFID_Count=0; continue; }		//读卡错误 退出重新读卡
						OldRFID_Money = RFID_Money;				
						CardInFlag = 0xAA;	//读卡成功
						ShowFlag = 0x00;	//关交替显示标志,只显示异常代码						
						BspTm1639_Show(0x00,0x00);		//关显示
					}
					else if(RFID_Count<6)//6*200=1200ms
					{	//显示原卡内金额, 卡插入0.4秒时，提交插卡动作
						if( (RFID_Money&0x00FFFFFF) == (ErrorMoney+7) )	//读出数据大于金额值为用户异常代码 E700
						{	//E700不提交插卡动作
							InPutCount = 0;	OutPut_OFF();		//关闭水阀，停止放水
							RFID_Count=200;		ErrorTemp[1] = 0x07;	ErrorTemp[0] = 0x00;
							BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//显示异常代码						
						}
						else if( (RFID_Money&0x00FFFFFF) >= ErrorMoney )	//用户异常代码 E000
						{
							InPutCount = 0;	OutPut_OFF();		//关闭水阀，停止放水
							if(UseErrFlag==0x00)
							{
								ErrorTemp[0] = gErrorShow%10;	ErrorTemp[1] = gErrorShow/10;	//百位，个位
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//显示异常代码
							}
							else
							{
								ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);	//十位，个位
								ErrorTemp[1] = gErrorDat[0]-0x30;	//百位
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//.显示异常代码
							}
							CardInFlag = 0xAA;	//读卡成功	
							if(OldCardInFlag != CardInFlag)	//刚插入
							{
								RFID_Money_Dat = ReadRFIDMoney(FM1702_Key[6]);	//纯读卡内金额 用于异常时返回卡金额
								if(ErrorMoney==RFID_Money_Dat)	continue;	//读卡错误，返回退出
								MemoryBuffer[0] = 0xAA;		//卡刚插入
								MemoryBuffer[1] = UID[0];	//卡号0；
								MemoryBuffer[2] = UID[1];	//卡号1；
								MemoryBuffer[3] = UID[2];	//卡号2；
								MemoryBuffer[4] = UID[3];	//卡号3；
								MemoryBuffer[5] = RFID_Money_Dat>>16;	//数据域5 金额1高位
								MemoryBuffer[6] = RFID_Money_Dat>>8;	//数据域6 金额2
								MemoryBuffer[7] = RFID_Money_Dat;		//数据域7 金额3
								MemoryBuffer[8] = RFID_Money_Dat>>24;	//数据域8 校验
								PutInMemoryBuf((u8 *)MemoryBuffer);
								MemoryBuffer[0] = 0x00;	
								RFID_Money_Dat = 0;
								OldCardInFlag = CardInFlag;
							}							
						}
						else	//读出数据为正常金额,提交插卡动作
						{
							CardInFlag = 0xAA;	//读卡成功								
							BspTm1639_Show(0x04,RFID_Money);	//计费扣款模式
							if(OldCardInFlag != CardInFlag)	//刚插入
							{
								MemoryBuffer[0] = 0xAA;		//卡刚插入
								MemoryBuffer[1] = UID[0];	//卡号0；
								MemoryBuffer[2] = UID[1];	//卡号1；
								MemoryBuffer[3] = UID[2];	//卡号2；
								MemoryBuffer[4] = UID[3];	//卡号3；
								MemoryBuffer[5] = RFID_Money>>16;	//数据域5 金额1高位
								MemoryBuffer[6] = RFID_Money>>8;	//数据域6 金额2
								MemoryBuffer[7] = RFID_Money;		//数据域7 金额3
								MemoryBuffer[8] = RFID_Money>>24;	//数据域8 校验
								PutInMemoryBuf((u8 *)MemoryBuffer);
								MemoryBuffer[0] = 0x00;	
								OldCardInFlag = CardInFlag;
							}							
						}						
						
					}
					else if(RFID_Count<12)//12*200=2400ms
					{	//延时放水
						if( (RFID_Money&0x00FFFFFF) > 0x00EA0000 )	//读出数据大于金额值为用户异常代码
						{
							InPutCount = 0;	OutPut_OFF();		//关闭水阀，放水
							if(UseErrFlag==0x00)
							{
								ErrorTemp[0] = gErrorShow%10;	ErrorTemp[1] = gErrorShow/10;	//个位//百位
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码
							}							
							else
							{
								ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
								ErrorTemp[1] = gErrorDat[0]-0x30;
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码
							}
						}
						else	BspTm1639_Show(0x04,RFID_Money);	//计费扣款模式				
					}
					

					if((RFID_Money&0x00FFFFFF) > 0x00EA0000)//读出数据大于金额值为用户异常代码
					{
						if(gErrorDat[0]!=0x00)
						{
							InPutCount = 0;	OutPut_OFF();		//关闭水阀，放水
							ErrorTemp[0] = gErrorDat[4]%10;
							ErrorTemp[1] = gErrorDat[4]/10;
							BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码
						}
						if(OldCardInFlag != CardInFlag)	
						{
							RFID_Money_Dat = ReadRFIDMoney(FM1702_Key[6]);	//纯读卡内金额 用于异常时返回卡金额
							MemoryBuffer[0] = 0xAA;		//卡刚插入
							MemoryBuffer[1] = UID[0];	//卡号0；
							MemoryBuffer[2] = UID[1];	//卡号1；
							MemoryBuffer[3] = UID[2];	//卡号2；
							MemoryBuffer[4] = UID[3];	//卡号3；
							MemoryBuffer[5] = RFID_Money_Dat>>16;	//数据域5 金额1高位 
							MemoryBuffer[6] = RFID_Money_Dat>>8;	//数据域6 金额2	
							MemoryBuffer[7] = RFID_Money_Dat;		//数据域7 金额3	
							MemoryBuffer[8] = RFID_Money_Dat>>24;	//数据域8 校验
							PutInMemoryBuf((u8 *)MemoryBuffer);
							MemoryBuffer[0] = 0x00;
							RFID_Money_Dat = 0;							
							OldCardInFlag = CardInFlag;
						}
					}
					else if((RFID_Count>12)&&((RFID_Money&0x00FFFFFF)>=WaterCost))	//卡内金额大于最小扣款基数
					{
						if(OutFlag == 0x00)	
						{
							OutFlag = 0xAA;	//放水标志
							RFID_Money = DecMoney(FM1702_Key[6],WaterCost);
							if(ErrorMoney==RFID_Money)	{	RFID_Count=0;	continue;	}
							if((OldRFID_Money-RFID_Money)>(WaterCost*2))	RFID_Money = OldRFID_Money;
							else 											OldRFID_Money = RFID_Money; 
							Beforeupdate = WaterCost;	//更新前扣费 初次扣费
							OvertimeCount=0;OvertimeMinutes=0;//有扣费，清超时标志
						}
						if(InPutCount>=CostNum)
						{
							InPutCount = InPutCount-CostNum;
							OvertimeCount=0;OvertimeMinutes=0;//有扣费，清超时标志
							RFID_Money = DecMoney(FM1702_Key[6],WaterCost);
							if(ErrorMoney==RFID_Money)	{	RFID_Count=0; continue;	}
							if(RFID_Money != 0x00)	//防止这个时间段拔出卡，造成数据为0
							{
								OldRFID_Money = RFID_Money;
								if(BeforeFlag == 0xAA)	Beforeupdate = Beforeupdate + WaterCost;	//更新前扣费
								if(Beforeupdate>(10*WaterCost))	Beforeupdate = 10*WaterCost;	
							}
							else	{	RFID_Money = OldRFID_Money;	}						
						}
						BspTm1639_Show(0x04,RFID_Money);			//计费扣款模式
						if(OvertimeMinutes == 0xAA)	OutPut_OFF();	//关闭水阀，停止放水
						else						OutPut_ON();	//打开水阀，放水
					}
					else if((RFID_Money&0x00FFFFFF)<WaterCost)
					{
						OutPut_OFF();	//关闭水阀，停止放水
					}
					
					if(RFID_Count<100)	RFID_Count++;	//100*200=10,000ms
					
					if((re_RxMessage[0]!=0)&&(re_RxMessage[4]==re_RxMessage[9])&&(re_RxMessage[10]==CRC8_Table(re_RxMessage,10)))	//CRC校验
					{
						if((re_RxMessage[0] == UID[0])&&(re_RxMessage[1] == UID[1])&&
						   (re_RxMessage[2] == UID[2])&&(re_RxMessage[3] == UID[3])&&\
						   (re_RxMessage[4] == FlagBit))
						{
							u32TempDat = (((u32)re_RxMessage[5]<<16)|((u32)re_RxMessage[6]<<8)|re_RxMessage[7]);	//读取云端发回数据
							if((u32TempDat<=WaterCost)||(Beforeupdate>=u32TempDat))	{u32TempDat = 0x00;	RFID_Count=0;}//金额为0，直接写入0；
							else													u32TempDat = u32TempDat-Beforeupdate;
							u32TempDat = u32TempDat | ((u32)re_RxMessage[8]<<24);
							u32TempDat = u32TempDat;
							if( WriteMoney(FM1702_Key[6],u32TempDat) == FM1702_OK )	//写入金额 成功
							{
								Beforeupdate = 0x00;	//清空更新数据前的扣费金额	
								BeforeFlag = 0x00;		//数据更新完成
								Delay(0xFF);			//延时
								RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
								if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//读卡正确
								else	{ RFID_Money = OldRFID_Money;OldRFID_Money = RFID_Money;}	//暂存卡内金额
								if(RFID_Money<WaterCost)	RFID_Count=0;	//返回金额小于最小扣款金额
								BspTm1639_Show(0x00,0x00);		Delay(0x2FFF); 	//关显示，延时闪烁一下
								BspTm1639_Show(0x04,RFID_Money);		//计费扣款模式	
								OldRFID_Money = RFID_Money;
							}
						}
						for(i=0;i<16;i++)	re_RxMessage[i] = 0x00;
					}
					if((gErrorDat[0]==UID[0])&&(gErrorDat[1]==UID[1])&&(gErrorDat[2]==UID[2])&&(gErrorDat[3]==UID[3])\
						&&(gErrorDat[5] == FlagBit))	//异常情况
					{
						WriteMoney( FM1702_Key[6] , ErrorMoney+gErrorDat[4] );
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)		RFID_Money = RFID_MoneyTemp;	//读卡正确
						else	{ RFID_Money = OldRFID_Money;	OldRFID_Money = RFID_Money;	}	//暂存卡内金额
						gErrorDat[5] = 0xAA;//防止再次进入，重新写卡数据
					}
					
				}
				else	//没有读到卡
				{
					CardUnReadFlag=0xAA;RFID_Count=0;
					if((gErrorDat[4]>=40))	//用于卡相关或用户相关，在卡取走后，清异常代码防止异常代码闪烁
					{gErrorDat[0]=0;	gErrorDat[1]=0;	gErrorDat[2]=0;	gErrorDat[3]=0;	gErrorDat[4]=0;	gErrorDat[5]=0;}	
					ShowFlag = 0xAA;	//交替显示标志,0xAA为交替显示
					UseErrFlag = 0x00;	//清用户或卡异常标志
					CardInFlag = 0x00;
					if(OldCardInFlag != CardInFlag)	//刚拔卡
					{
						MemoryBuffer[0] = 0x55;		//卡刚拔出
						MemoryBuffer[5] = RFID_Money>>16;	//数据域5 金额1高位
						MemoryBuffer[6] = RFID_Money>>8;	//数据域6 金额2
						MemoryBuffer[7] = RFID_Money;		//数据域7 金额3
						MemoryBuffer[8] = RFID_Money>>24;	//数据域8 校验
						if((RFID_Money&0x00FFFFFF) > 0x00EA0000);	//错误异常时，不提交拔卡动作
						else PutInMemoryBuf((u8 *)MemoryBuffer);
						MemoryBuffer[0] = 0x00;		
						OldCardInFlag = CardInFlag;
					}
					Beforeupdate = 0x00;	BeforeFlag = 0xAA;
					InPutCount = 0;
					OvertimeCount=0;OvertimeMinutes=0;//有扣费，清超时标志
					OutFlag = 0x00;		OutPut_OFF();	//关闭水阀，放水
					if(ShowCount<148)			BspTm1639_Show(0x03,WaterCost);	//显示每升水金额值
					else if(ShowCount<150)		BspTm1639_Show(0x00,0x00);		//关显示
					else if(ShowCount<298)		BspTm1639_ShowSNDat(0x11,(u8 *)Physical_ADD);
					else						BspTm1639_Show(0x00,0x00);		//关显示	
					if(ShowCount==200)			{	SoftReset1702();	}		//检测RFID模块
				}
			
			}

			if(Flash_UpdateFlag == 0xAA)
			{
				Write_Flash_Dat();
				Flash_UpdateFlag = 0;
				if(g_IAP_Flag == 0xAA)		SoftReset();//更新标志 软件复位
			}
			if(g_LoseContact>200)		SoftReset();//失联 软件复位 10秒钟累加一；在定时器累加
		}

	}
}

/*
*********************************************************************************************************
*	函 数 名: InitBoard
*	功能说明: 初始化硬件设备
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void InitBoard(void)
{	
	RCC_Configuration();
	/* 初始化systick定时器，并启动定时中断 */
	bsp_InitTimer(); 
	//NVIC_Configuration();
	
	Init_GPIO();		//输出初始化
	#if IAP_Flag
	#else
		bsp_InitUart(); 	//初始化串口,因为IAP中初始过
	#endif
	BspTm1639_Config();	//TM1639初始化
	BspFM1701_Config();	//FM1701 GPIO初始化
	TIM3_Int_Init(1999,720-1);//以100khz的频率计数，0.01ms中断，计数到2000 为20ms 
	TIM2_Cap_Init(0xFFFF,72-1);	//以1Mhz的频率计数 
 	Adc_Init();		  		//ADC初始化
	
}


//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //TIM3更新中断 20ms中断
	{
		if( ShowFlag == 0xAA )	//更新显示
		{
			if( ShowCount < 300 )  	ShowCount++;	//300*20=6000，6秒一个显示周期
			else					ShowCount = 0;
			
			if( DelayCount > 0 )	{	if( (ShowCount%2)==0 )	DelayCount--;	}	//20ms中断	20*2=40ms
		}
		else    ShowCount = 0;
		
		//定时时间 10min内无流量计费关阀门
		if(OvertimeCount<3000)	OvertimeCount++;	//超时计数，60s=60,000ms=3000*20ms
		else
		{
			OvertimeCount=0;
			if(OvertimeMinutes<10)	OvertimeMinutes++;	//超时10分钟
			else				  	OvertimeMinutes = 0xAA;//超时标志
		}
		if(Time20msCount<10)	Time20msCount++;	//0.2秒 10 *20ms = 200ms
		else	
		{	
			Time20msCount=0;
			if(g_LoseContact<150)	g_LoseContact++;	//失联计数，0.2秒*150 = 30Sec；
			else					g_LoseContact=255;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 					
	}
}

void Delay(__IO uint16_t nCount)
{
	while (nCount != 0)
	{
		nCount--;
	}
}



