#include "bsp.h"

static void InitBoard(void);
static void Delay (uint16_t nCount);

uint8_t Time_250ms=0;
uint8_t ShowFlag=0x00;
uint16_t ShowCount=0; 	//用于正常待机时交替显示
uint8_t gErrorShow=0;	//异常显示代码 在服务器未更新前显示使用，这样不会出现E000
uint8_t gErrorDat[4]={0};	//异常代码存储
uint8_t Logic_ADD=0;	//逻辑地址
uint8_t Physical_ADD[4]={0x20,0x17,0x12,0x01};//物理地址
uint8_t WaterCost=50,CostNum=29;	//WaterCost=水费 最小扣款金额  //脉冲数
uint8_t g_count=0;
uint8_t g_RxMessage[8]={0};	//CAN接收数据
uint8_t g_RxMessFlag=0;		//CAN接收数据 标志
//uint8_t OldOutFlag = 0;	//用于标记拔卡动作
uint8_t CardInFlag = 0,OldCardInFlag = 0;	//用于标记插卡、拔卡动作
uint8_t OutFlag=0,PowerUpFlag=0;	//放水标志,上电标志
uint8_t g_MemoryBuf[5][10]={0};	//数据缓存，[0]=0xAA表示有插卡数据，[0]=0xBB表示有拔卡数据，[1-4]卡号；[5-7]金额；[8]卡核验码；[9]通信码
uint8_t FlagBit = 0x00;		//通信标志位，每插入卡一次数据加1，数值达到199时，清0
uint16_t Beforeupdate = 0;	//远程更新前的扣费金额;
uint16_t OvertimeCount=0;	//超时计数 15000为1分钟
uint8_t OvertimeMinutes=0,OvertimeFlag=0;	//超时分钟，超时标志
uint8_t BeforeFlag = 0xAA;	//更新标志 0xAA表示未更新; 
u8 InPutCount=0;	//输入脉冲计数
uint8_t re_RxMessage[16]={0};
uint32_t RFID_Money=0,OldRFID_Money = 0,u32TempDat=0,RFID_MoneyTemp=0;	//卡内金额
uint32_t RFID_Money_Dat=0;	//卡内金额
extern unsigned char FM1702_Key[7];
extern unsigned char UID[5];
extern unsigned char FM1702_Buf[16];

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
void PutInMemoryBuf(u8 *Buf)	//末尾加入第一个缓存
{
	uint8_t i,j;
	if(SeekMemoryDat((u8 *)Buf)==0xF1)  //查询是否在缓存列表中
    {
        if(g_MemoryBuf[4][0]!=0)	//目前已经存满，丢第一个；
        {
            for(i=0;i<4;i++)
            {
                for(j=0;j<10;j++)
                {
                    g_MemoryBuf[i][j] = g_MemoryBuf[i+1][j];
                }
            }
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
	uint8_t DelayCount=0;
	uint8_t PUSendCount = 0x00;	//上电发送随机数次数；
//	uint8_t SendCAN_Buf[8]={0};
//	uint8_t g_ShowBuf[8]={0};
	uint8_t CardUnReadFlag=0;	//读卡标志，用于延时执行取卡动作
	uint8_t OldeUID[5];			//卡号，OldeUID[4]为校验数据
	uint8_t MemoryBuffer[10]={0};	//数据缓存，[0]=0xAA表示有插卡数据，[0]=0xBB表示有拔卡数据，[1-4]卡号；[5-7]金额；[8]卡核验码；[9]预留
	uint8_t RFID_Count=0;	//卡开启水阀延时时间;3秒
	uint8_t ErrorTemp[5]={0};
	uint8_t UseErrFlag = 0x00;	//用户或卡异常标志,接收到服务器返回数据后才显示异常代码值 否则显示‘E000’
	uint8_t i=0;
	
	InitBoard();			//硬件初始化
	
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN初始化正常模式,波特率450Kbps    
	printf("Starting Up...\r\n");

	BspTm1639_Show(0x01,0x00);
	ShowFlag = 0xAA;
	//Logic_ADD = 1;	//测试使用
	//PowerUpFlag=0xAA;	//测试使用
	while(Logic_ADD==0)	//逻辑地址为0时，表示该设备未注册，进入等待注册过程
	{
		if(PowerUpFlag==0xAA)
		{
			DelayCount = Read_Rand();  //产生0-50的随机数
			while(DelayCount--)		Delay(0x8F);	//简单延时
			if(PUSendCount<30)	{	Package_Send(0xB3,(u8 *)Physical_ADD);	PUSendCount++;	Delay(0xFF);}	
			else
			{
				PUSendCount = 200;
			}				
		}

		if( g_RxMessFlag == 0xAA )//接收到有数据
		{
			if((g_RxMessage[0]==0xA1)&&(g_RxMessage[1]==0xA1))	//未注册广播
			{
				PowerUpFlag=0xAA;//进入随机延时未注册回复
			}
			else if((g_RxMessage[0]==0xC1)\
				&&(g_RxMessage[1]==Physical_ADD[0])&&(g_RxMessage[2]==Physical_ADD[1])\
				&&(g_RxMessage[3]==Physical_ADD[2])&&(g_RxMessage[4]==Physical_ADD[3]))
			{
				Logic_ADD = g_RxMessage[5];	//取出分配的逻辑地址
				Package_Send(0xC2,(u8 *)Physical_ADD);
			}
			g_RxMessFlag = 0x00;
		}
		
		if( ReadRFIDCard() == FM1702_OK )						//只用于检测是否有卡，不校验密码
		{	BspTm1639_Show(0x06,0x01);	Delay(1500);	}		//显示代码表示到卡 ---XX--- //软件版本号
		else
		{
			if(ShowCount<1980)			BspTm1639_ShowSNDat(0x11,(u8 *)Physical_ADD);
			else if(ShowCount<2000)		BspTm1639_Show(0x00,0x00);		//关显示
			else if(ShowCount<3980)		BspTm1639_Show(0x01,0x00);		//显示方框
			else						BspTm1639_Show(0x00,0x00);		//关显示			
		}
	}
	CAN_DeInit(CAN1);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN初始化正常模式,波特率450Kbps    
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
				ShowFlag = 0x00;
				ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
				ErrorTemp[1] = gErrorDat[0]-0x30;
				BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);
				InPutCount = 0;
				OutPut_OFF();	//关闭水阀，放水
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
							OldCardInFlag = CardInFlag;
							MemoryBuffer[0] = 0x55;		//卡刚拔出
							MemoryBuffer[5] = RFID_Money>>16;	//数据域5 金额1高位
							MemoryBuffer[6] = RFID_Money>>8;	//数据域6 金额2
							MemoryBuffer[7] = RFID_Money;		//数据域7 金额3
							MemoryBuffer[8] = RFID_Money>>24;	//数据域8 校验
							PutInMemoryBuf((u8 *)MemoryBuffer);
							MemoryBuffer[0] = 0x00;		
						}
						OldeUID[0]=UID[0];OldeUID[1]=UID[1];OldeUID[2]=UID[2];OldeUID[3]=UID[3];
						CardUnReadFlag=0xAA;
						OutFlag = 0x00;	//放水标志 0x00->停止放水
						//CardFlag = 0x00;
						RFID_Count=0; 
						continue;
					}
					if(RFID_Count<1)//1*200=200ms
					{
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//读卡正确
						else				{ RFID_Money = OldRFID_Money; continue; }		//读卡错误 退出重新读卡
						BspTm1639_Show(0x00,0x00);		//关显示
					}
					else if(RFID_Count<6)//6*200=1200ms
					{	//显示原卡内金额,提交插卡动作
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//读卡正确
						else				{ RFID_Money = OldRFID_Money; continue; }		//读卡错误 退出重新读卡
						OldRFID_Money = RFID_Money;				
						if( (RFID_Money&0x00FFFFFF) == (ErrorMoney+7) )	//读出数据大于金额值为用户异常代码 E700
						{	//E700不提交插卡动作
							InPutCount = 0;	OutPut_OFF();		//关闭水阀，放水
							RFID_Count=200;		ErrorTemp[1] = 0x07;	ErrorTemp[0] = 0x00;
							BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码						
						}
						else if( (RFID_Money&0x00FFFFFF) >= ErrorMoney )	//读出数据大于金额值为用户异常代码 E000
						{
							InPutCount = 0;	OutPut_OFF();		//关闭水阀，放水
							if(UseErrFlag==0x00)
							{
								ErrorTemp[0] = gErrorShow%10;	//个位
								ErrorTemp[1] = gErrorShow/10;	//百位
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码
							}
							else
							{
								ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);	//十位，个位
								ErrorTemp[1] = gErrorDat[0]-0x30;	//百位
								BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码
							}
							CardInFlag = 0xAA;						
							if(OldCardInFlag != CardInFlag)	//刚插入
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
								OldCardInFlag = CardInFlag;
							}							
						}
						else
						{
							CardInFlag = 0xAA;						
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
					else if(RFID_Count<15)//15*200=3000ms
					{	//延时放水，
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)	RFID_Money = RFID_MoneyTemp;	//读卡正确
						else				{ RFID_Money = OldRFID_Money; continue; }		//读卡错误 退出重新读卡
						OldRFID_Money = RFID_Money;				
						if( (RFID_Money&0x00FFFFFF) > 0x00EA0000 )	//读出数据大于金额值为用户异常代码
						{
							InPutCount = 0;	//OutPut_OFF();		//关闭水阀，放水
							if(UseErrFlag==0x00)
							{
								ErrorTemp[0] = gErrorShow%10;	//个位
								ErrorTemp[1] = gErrorShow/10;	//百位
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
							ErrorTemp[0] = ((gErrorDat[1]-0x30)<<4)|(gErrorDat[2]-0x30);
							ErrorTemp[1] = gErrorDat[0]-0x30;
							BspTm1639_ShowSNDat(0x12,(u8 *)ErrorTemp);//01.显示异常代码
						}
						if(OldCardInFlag != CardInFlag)	
						{
							OldCardInFlag = CardInFlag;
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
							OutFlag = 0xAA;	//放水标志
						}
					}
					else if((RFID_Count>15)&&((RFID_Money&0x00FFFFFF)>=WaterCost))	//卡内金额大于最小扣款基数
					{
						if(OutFlag == 0x00)	
						{
							OutFlag = 0xAA;	//放水标志
							RFID_Money = DecMoney(FM1702_Key[6],WaterCost);
							if(ErrorMoney==RFID_Money)	continue;
							if((OldRFID_Money-RFID_Money)>(WaterCost*2))	RFID_Money = OldRFID_Money;
							else 											OldRFID_Money = RFID_Money; 
							Beforeupdate = WaterCost;	//更新前扣费 初次扣费
							//OldOutFlag = OutFlag;
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
							else
							{
								RFID_Money = OldRFID_Money;
							}						
						}
						BspTm1639_Show(0x04,RFID_Money);			//计费扣款模式
						if(OvertimeFlag == 0x00)	OutPut_ON();	//打开水阀，放水
						else						OutPut_OFF();	//关闭水阀，停止放水
					}
					if(RFID_Count<100)	RFID_Count++;	//100*200=10,000ms
					
					if(((re_RxMessage[4]==re_RxMessage[9]))&&(re_RxMessage[10]==CRC8_Table(re_RxMessage,10)))	//CRC校验
					{
						if((re_RxMessage[0] == UID[0])&&(re_RxMessage[1] == UID[1])&&
						   (re_RxMessage[2] == UID[2])&&(re_RxMessage[3] == UID[3])&&\
						   (re_RxMessage[4] == FlagBit))
						{
							u32TempDat = (((u32)re_RxMessage[5]<<16)|((u32)re_RxMessage[6]<<8)|re_RxMessage[7]);	//读取云端发回数据
							if((u32TempDat<=WaterCost)||(Beforeupdate>=u32TempDat))	u32TempDat = 0x00;	//金额为0，直接写入0；
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
								BspTm1639_Show(0x04,RFID_Money);	//计费扣款模式	
							}
						}
						for(i=0;i<16;i++)	re_RxMessage[i] = 0x00;
					}
					if((gErrorDat[0]==0x34)||(gErrorDat[0]==0x35)||(gErrorDat[0]==0x36)||(gErrorDat[0]==0x37))	//异常情况
					{
						WriteMoney( FM1702_Key[6] , ErrorMoney+( (gErrorDat[0]-0x30)*10 + (gErrorDat[2]-0x30)) );
						RFID_MoneyTemp = ReadMoney(FM1702_Key[6]);	//读取卡内金额 使用地址块FM1702_Key[6]
						if(RFID_MoneyTemp != ErrorMoney)		RFID_Money = RFID_MoneyTemp;	//读卡正确
						else	{ RFID_Money = OldRFID_Money;	OldRFID_Money = RFID_Money;	}	//暂存卡内金额
					}
					
				}
				else	//没有读到卡
				{
					CardUnReadFlag=0xAA;RFID_Count=0;
					if((gErrorDat[0]==0x34)||(gErrorDat[0]==0x35))	//用于卡相关或用户相关，在卡取走后，清异常代码防止异常代码闪烁
					{gErrorDat[0] = 0x00;	gErrorDat[1] = 0x00;	gErrorDat[2] = 0x00;}	
					//CardFlag = 0x00;ReceiveFlag = 0x00;
					ShowFlag = 0xAA;
					UseErrFlag = 0x00;	//清用户或卡异常标志
					CardInFlag = 0x00;
					if(OldCardInFlag != CardInFlag)	//刚拔卡
					{
						OldCardInFlag = CardInFlag;
						MemoryBuffer[0] = 0x55;		//卡刚拔出
						MemoryBuffer[5] = RFID_Money>>16;	//数据域5 金额1高位
						MemoryBuffer[6] = RFID_Money>>8;	//数据域6 金额2
						MemoryBuffer[7] = RFID_Money;		//数据域7 金额3
						MemoryBuffer[8] = RFID_Money>>24;	//数据域8 校验
						PutInMemoryBuf((u8 *)MemoryBuffer);
						MemoryBuffer[0] = 0x00;		
					}
					OutFlag = 0x00;
					Beforeupdate = 0x00;	BeforeFlag = 0xAA;
					InPutCount = 0;
					OvertimeFlag = 0x00;
					OvertimeCount=0;OvertimeMinutes=0;//有扣费，清超时标志
					OutPut_OFF();	//关闭水阀，放水
					if(ShowCount<1980)			BspTm1639_ShowSNDat(0x11,(u8 *)Physical_ADD);
					else if(ShowCount<2000)		BspTm1639_Show(0x00,0x00);		//关显示
					else if(ShowCount<3980)		BspTm1639_Show(0x03,WaterCost);	//显示每升水金额值
					else						BspTm1639_Show(0x00,0x00);		//关显示	
					if(ShowCount==2000)	{	SoftReset1702();	}//检测RFID模块
				}
			
			}
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
	NVIC_Configuration();
	
	bsp_InitUart(); 	/* 初始化串口 */
	BspTm1639_Config();	//TM1639初始化
	BspFM1701_Config();	//FM1701 GPIO初始化
	TIM3_Int_Init(99,1599);//10Khz的计数频率，计数到100  

 	Adc_Init();		  		//ADC初始化
	
}


//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		if( ShowFlag == 0xAA )
		{
			if( ShowCount < 4000 )  ShowCount++;
			else                    ShowCount = 0;
		}
		else    ShowCount = 0;
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



