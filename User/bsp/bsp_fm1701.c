#include "stm32f10x.h"
#include "bsp_fm1701.h"
#include "bsp_crc8.h"
#include "bsp.h"


unsigned char FM1702_Buf[16];
unsigned char FM1702_Key[7]={0xFF,0xFF,0xFF,0xFF,0xFF,0xF1,41};
unsigned char UID[5];			//���ţ����һ�ֽ�ΪУ������
extern uint8_t gErrorShow;	//�쳣��ʾ���� �ڷ�����δ����ǰ��ʾʹ�ã������������E000


u8 GetFanma(u8 dat1)
{
	u8 i,dat2=0;
	for(i=0;i<8;i++)
	{
	  	dat2<<=1;
		if((dat1&0x80)==0x00)	dat2 = dat2+1;
		dat1<<=1;
	}
	return dat2;
}

void BspFM1701_Delay(unsigned int dlength)
{ 
	unsigned int  i;
	unsigned char j;
	for (i=0;i<dlength;i++)
	{
		for (j=0;j<100;j++);
	}
}

void BspFM1701_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 

	Init_FM1702();			//FM1701��ʼ��
}

void SPI_Send(unsigned char Data)          //����SPI�ķ���
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		BspFM1701_Delay(1);
		if((Data&0x80)==0x80)	MOSI_SET;
		else					MOSI_CLR;
		BspFM1701_Delay(1);
		SCK_SET;
		Data<<=1;
		BspFM1701_Delay(1);
		SCK_CLR;
	}
}

unsigned char  SPI_Receve(void)         //ģ��SPI�н���
{
	unsigned char i,Data=0;
	for(i=0;i<8;i++)   
	{
		BspFM1701_Delay(1);
		Data<<=1;
		if(MISO)	Data+=1;
		SCK_SET;
		BspFM1701_Delay(1);
		SCK_CLR;
		BspFM1701_Delay(1);
	}
	return Data;
}

unsigned char Read_REG(unsigned char SpiAddr)           //���Ĵ���
{
	unsigned char Data;
	SpiAddr<<=1;
	SpiAddr|=0x80;
	NSS_CLR;
	BspFM1701_Delay(1);
	SPI_Send(SpiAddr);
	Data=SPI_Receve();
	NSS_SET;
	BspFM1701_Delay(1);
	return Data;
}

void Write_REG(unsigned char SpiAddr,unsigned char Data)  //д�Ĵ���
{
	SpiAddr<<=1;
	SpiAddr&=0x7f;
	NSS_CLR;
	BspFM1701_Delay(1);
	SPI_Send(SpiAddr);
	SPI_Send(Data);
	BspFM1701_Delay(1);
	NSS_SET;
	BspFM1701_Delay(1);
}
/***************************************************************
����: Clear_FIFO                                             
����: �ú���ʵ����FFIFO������                                	       			                          
����:   N/A                                                                                                                  
���:   TRUE, FIFO�����                                       
	FALSE, FIFOδ�����  	                                  
****************************************************************/
unsigned char Clear_FIFO(void)
{
	unsigned char i,temp;
	temp = Read_REG(Control);
	temp|=0x01;
	Write_REG(Control,temp);
	for(i=0;i<0xFF;i++)
	{
		if(Read_REG(FIFO_Length)==0)
		{
			return TRUE;
		}   
	}
	return FALSE;
}

/*********************************************************************
����: Write_FIFO 
����: �ú���ʵ����RC531��FIFO��д��x bytes����
����: count, ��д���ֽڵĳ���
      buff, ָ���д�����ݵ�ָ��
�������
**********************************************************************/
void Write_FIFO(unsigned char count,unsigned char *Buff)
{
	unsigned char i;
	for (i=0;i<count;i++)
	{
		Write_REG(FIFO,*(Buff +i));
	}
}

/********************************************************************
����: Read_FIFO   
����: �ú���ʵ�ִ�RC531��FIFO�ж���x bytes����
����: buff, ָ��������ݵ�ָ��
���: ���յ����ֽ���
*******************************************************************/
unsigned char Read_FIFO(unsigned char *Buff)
{
	unsigned char i,temp;
	temp=Read_REG(FIFO_Length);
	if(temp==0)
	{
		return 0;
	}
	if(temp>=24)
	{
		temp=24;
	}
	for(i=0;i<temp;i++)
	{
		*(Buff +i) = Read_REG(FIFO);
	}
	return temp; 
}

/********************************************************************
����: Judge_Req 
����: �ú���ʵ�ֶԿ�Ƭ��λӦ���źŵ��ж� 
����: *buff, ָ��Ӧ�����ݵ�ָ��  
���: TRUE, ��ƬӦ���ź���ȷ                                 
      FALSE, ��ƬӦ���źŴ���
********************************************************************/
unsigned char Judge_Req(uchar *Buff)
{
	uchar temp1,temp2;
	temp1 = *Buff;
	temp2 = *(Buff+1);
	if((temp1 !=0x00)&&(temp2 ==0x00))
	{
		return TRUE;
	}
	return FALSE;
}

/*******************************************************************
����: Command_Send 
����: �ú���ʵ����FM1702��������Ĺ���
����: count, ����������ĳ���                              
      buff, ָ����������ݵ�ָ��                             
      Comm_Set, ������  
���: TRUE, �����ȷִ��
      FALSE, ����ִ�д���
******************************************************************/
uchar Command_Send(uchar count,uchar *Buff,uchar Comm_Set)
{
    uchar i;
    Write_REG(Command,Idle);   //��յ�ǰָ�
    Clear_FIFO();
    if(count)
    {
      Write_FIFO(count,Buff);
    }
    Read_REG(FIFO_Length);
    Write_REG(Command,Comm_Set);
    for(i=0;i<0x7F;i++)	//20171020 ��i<250��Ϊi<0x7F
    {
      if(Read_REG(Command)==0)
      {
        return TRUE;
      }
    }
    return FALSE;
}

unsigned char Request(unsigned char mode)
{
    uchar a;
    Write_REG(TxControl,0x58);
    for(a=0;a<100;a++);
    Write_REG(TxControl,0x5b);
    Write_REG(CRCPresetLSB,0x63);
    Write_REG(CWConductance,0x3f);
    Write_REG(0x19,0x72);
    FM1702_Buf[0]=mode;
    Write_REG(Bit_Frame,0x07);
    Write_REG(ChannelRedundancy,3);//x03);
    Write_REG(TxControl,0x5b);
    Write_REG(Control,0x01);          //�ط�����
    
    if(Command_Send(1,FM1702_Buf,Transceive)==FALSE)
    {
      return FM1702_NOTAGERR;
    }
    Read_FIFO(FM1702_Buf);
    if(Judge_Req(FM1702_Buf)== TRUE)
    {
      return FM1702_OK;
    }
    return FM1702_REQERR;
}

/*******************************************************************
���ƣ�KeyConvert  ��Կת����ʽ
���ã���6λ��Կת����12λ��Կ��ʽ
���룺* Uncodeָ����Ҫת������Կ��ַ
    ��*��coded ת�ܻ��õ���Կ��ŵ�ַ��
�������
******************************************************************/
/*
void KeyConvert(uchar *Coding,uchar *Coded)
{
  uchar i,temp;
  for(i=0;i<6;i++)
  {
    temp=(*(Coding+i)&0x0f);
    *(Coded+i*2)=(~temp<<4)|temp;
    temp=(*(Coding+i)>>4);
    *(Coded+i*2+1)=(~temp<<4)|temp;
  }
}*/
void KeyConvert(uchar *Coding,uchar *Coded)
{
  uchar i,temp;
  for(i=0;i<6;i++)
  {
    temp=(*(Coding+i)&0x0f);
    *(Coded+i*2+1)=(~temp<<4)|temp;
    temp=(*(Coding+i)>>4);
    *(Coded+i*2)=(~temp<<4)|temp;
  }
}


/*********************************************************************
����: Load_keyE2 
����: �ú���ʵ�ֽ��������FM1702��keybuf��
���룺KeyCode :ָ���ֽڵ�ָ��
�����True: ��Կװ�سɹ�                                      
      False: ��Կװ��ʧ�� 
*********************************************************************/
uchar Load_Key(uchar *Keybuf)
{
  KeyConvert(Keybuf,FM1702_Buf);
  Command_Send(12,FM1702_Buf,LoadKey);  //��������
  if(Read_REG(ErrorFlag)&0x40)
  {
    return FALSE;
  }
  return TRUE;
}

uchar Check_UID(void)
{
  uchar	temp;
  uchar	i;

  temp = 0x00;
  for(i = 0; i < 5; i++)
  {
    temp = temp ^ UID[i];
  }

  if(temp == 0)
  {
    return TRUE;
  }

  return FALSE;
}

void Save_UID(uchar row, uchar col, uchar length)
{
  uchar	i;
  uchar	temp;
  uchar	temp1;

  if((row == 0x00) && (col == 0x00))
  {
    for(i = 0; i < length; i++)
    {
       UID[i] = FM1702_Buf[i];
    }
  }
  else
  {
    temp = FM1702_Buf[0];
    temp1 = UID[row - 1];
    switch(col)
    {
      case 0:		temp1 = 0x00; row = row + 1; break;
      case 1:		temp = temp & 0xFE; temp1 = temp1 & 0x01; break;
      case 2:		temp = temp & 0xFC; temp1 = temp1 & 0x03; break;
      case 3:		temp = temp & 0xF8; temp1 = temp1 & 0x07; break;
      case 4:		temp = temp & 0xF0; temp1 = temp1 & 0x0F; break;
      case 5:		temp = temp & 0xE0; temp1 = temp1 & 0x1F; break;
      case 6:		temp = temp & 0xC0; temp1 = temp1 & 0x3F; break;
      case 7:		temp = temp & 0x80; temp1 = temp1 & 0x7F; break;
      default:	        break;
    }

    FM1702_Buf[0] = temp;
    UID[row - 1] = temp1 | temp;
    for(i = 1; i < length; i++)
    {
      UID[row - 1 + i] = FM1702_Buf[i];
    }
  }
}

void Set_BitFraming(uchar row, uchar col)
{
  switch(row)
  {
    case 0:		FM1702_Buf[1] = 0x20; break;
    case 1:		FM1702_Buf[1] = 0x30; break;
    case 2:		FM1702_Buf[1] = 0x40; break;
    case 3:		FM1702_Buf[1] = 0x50; break;
    case 4:		FM1702_Buf[1] = 0x60; break;
    default:	        break;
  }

  switch(col)
  {
    case 0:		Write_REG(0x0F,0x00);  break;
    case 1:		Write_REG(0x0F,0x11); FM1702_Buf[1] = (FM1702_Buf[1] | 0x01); break;
    case 2:		Write_REG(0x0F,0x22); FM1702_Buf[1] = (FM1702_Buf[1] | 0x02); break;
    case 3:		Write_REG(0x0F,0x33); FM1702_Buf[1] = (FM1702_Buf[1] | 0x03); break;
    case 4:		Write_REG(0x0F,0x44); FM1702_Buf[1] = (FM1702_Buf[1] | 0x04); break;
    case 5:		Write_REG(0x0F,0x55); FM1702_Buf[1] = (FM1702_Buf[1] | 0x05); break;
    case 6:		Write_REG(0x0F,0x66); FM1702_Buf[1] = (FM1702_Buf[1] | 0x06); break;
    case 7:		Write_REG(0x0F,0x77); FM1702_Buf[1] = (FM1702_Buf[1] | 0x07); break;
    default:	        break;
  }
}

uchar AntiColl(void)
{
  uchar	temp;
  uchar	i;
  uchar	row, col;
  uchar	pre_row;

  row = 0;
  col = 0;
  pre_row = 0;
  Write_REG(0x23,0x63);
  Write_REG(0x12,0x3f);
  Write_REG(0x13,0x3f);
  FM1702_Buf[0] = RF_CMD_ANTICOL;
  FM1702_Buf[1] = 0x20;
  Write_REG(0x22,0x03);	
  temp = Command_Send(2, FM1702_Buf, Transceive);
  while(1)
  {
    if(temp == FALSE)
    {
      return(FM1702_NOTAGERR);
    }

    temp = Read_REG(0x04);
    if(temp == 0)
    {
      return FM1702_BYTECOUNTERR;
    }

    Read_FIFO(FM1702_Buf);
    Save_UID(row, col, temp);	

    temp = Read_REG(0x0A);		
    temp = temp & 0x01;
    if(temp == 0x00)
    {
      temp = Check_UID();			
      if(temp == FALSE)
      {
        return(FM1702_SERNRERR);
      }

      return(FM1702_OK);
    }
    else
    {
      temp = Read_REG(0x0B);           
      row = temp / 8;
      col = temp % 8;
      FM1702_Buf[0] = RF_CMD_ANTICOL;
      Set_BitFraming(row + pre_row, col);	
      pre_row = pre_row + row;
      for(i = 0; i < pre_row + 1; i++)
      {
        FM1702_Buf[i + 2] = UID[i];
      }

      if(col != 0x00)
      {
        row = pre_row + 1;
      }
      else
      {
        row = pre_row;
      }
      temp = Command_Send(row + 2, FM1702_Buf, Transceive);
    }
  }
}

uchar Select_Card(void)
{
  uchar	temp, i;

  Write_REG(0x23,0x63);
  Write_REG(0x12,0x3f);
  FM1702_Buf[0] = RF_CMD_SELECT;
  FM1702_Buf[1] = 0x70;
  for(i = 0; i < 5; i++)
  {
    FM1702_Buf[i + 2] = UID[i];
  }

  Write_REG(0x22,0x0f);	
  temp = Command_Send(7, FM1702_Buf, Transceive);
  if(temp == FALSE)
  {
    return(FM1702_NOTAGERR);
  }
  else
  {
    temp = Read_REG(0x0A);
    if((temp & 0x02) == 0x02) return(FM1702_PARITYERR);
    if((temp & 0x04) == 0x04) return(FM1702_FRAMINGERR);
    if((temp & 0x08) == 0x08) return(FM1702_CRCERR);
    temp = Read_REG(0x04);
    if(temp != 1) return(FM1702_BYTECOUNTERR);
    Read_FIFO(FM1702_Buf);
    temp = *FM1702_Buf;

    if((temp == 0x08) || (temp == 0x88) || (temp == 0x53) ||(temp == 0x18)) 
            return(FM1702_OK);
    else
            return(FM1702_SELERR);
  }
}

/****************************************************************/
/*����: Authentication                                          */
/*����: �ú���ʵ��������֤�Ĺ���                                */
/*����: UID: ��Ƭ���кŵ�ַ                                     */
/* SecNR: ������                                                */
/* mode: ģʽ                                                   */
/*���: FM1702_NOTAGERR: �޿�                                   */
/* FM1702_PARITYERR: ��żУ���                                 */
/* FM1702_CRCERR: CRCУ���                                     */
/* FM1702_OK: Ӧ����ȷ                                          */
/* FM1702_AUTHERR: Ȩ����֤�д�                                 */
/****************************************************************/
uchar Authentication(uchar *UID, uchar SecNR, uchar mode)
{
	uchar i;
	uchar temp, temp1;

	Write_REG(0x23,0x63);
	Write_REG(0x12,0x3f);
	Write_REG(0x13,0x3f);
	temp1 = Read_REG(0x09);
	temp1 = temp1 & 0xf7;
	Write_REG(0x09,temp1);
	if(mode == RF_CMD_AUTH_LB)	FM1702_Buf[0] = RF_CMD_AUTH_LB;
	else						FM1702_Buf[0] = RF_CMD_AUTH_LA;
	FM1702_Buf[1] = SecNR * 4 + 3;
	for(i = 0; i < 4; i++)
	{
		FM1702_Buf[2 + i] = UID[i];
	}

	Write_REG(0x22,0x0f);	
	temp = Command_Send(6, FM1702_Buf, Authent1);
	if(temp == FALSE)			return FM1702_NOTAGERR;/* �޿� */

	temp = Read_REG(0x0A);
	if((temp & 0x02) == 0x02) 	return FM1702_PARITYERR;/* ��Ƭ��żУ����� */
	if((temp & 0x04) == 0x04) 	return FM1702_FRAMINGERR;/* FM1702֡���� */
	if((temp & 0x08) == 0x08) 	return FM1702_CRCERR;/* ��ƬCRCУ����� */
	temp = Command_Send(0, FM1702_Buf, Authent2);
	if(temp == FALSE)	return FM1702_NOTAGERR;/* �޿� */

	temp = Read_REG(0x0A);
	if((temp & 0x02) == 0x02) return FM1702_PARITYERR;
	if((temp & 0x04) == 0x04) return FM1702_FRAMINGERR;
	if((temp & 0x08) == 0x08) return FM1702_CRCERR;
	temp1 = Read_REG(0x09);
	temp1 = temp1 & 0x08;	
	if(temp1 == 0x08)	return FM1702_OK;

	return FM1702_AUTHERR;
}

uchar MIF_READ(uchar *buff, uchar Block_Adr)
{
  uchar temp;

  Write_REG(0x23,0x63);
  Write_REG(0x12,0x3f);
  Write_REG(0x13,0x3f);
  Write_REG(0x22,0x0f);

  buff[0] = RF_CMD_READ;
  buff[1] = Block_Adr;
  temp = Command_Send(2, buff, Transceive);
  if(temp == 0)
  {
    return FM1702_NOTAGERR;
  }

  temp = Read_REG(0x0A);
  if((temp & 0x02) == 0x02) return FM1702_PARITYERR;
  if((temp & 0x04) == 0x04) return FM1702_FRAMINGERR;
  if((temp & 0x08) == 0x08) return FM1702_CRCERR;
  temp = Read_REG(0x04);
  if(temp == 0x10)
  {
    Read_FIFO(buff);
    return FM1702_OK;
  }
  else if(temp == 0x04)
  {
    Read_FIFO(buff);
    return FM1702_OK;
  }
  else
  {
    return FM1702_BYTECOUNTERR;
  }
}

uchar MIF_Write(uchar *buff, uchar Block_Adr)
{
  uchar temp;
  uchar *F_buff;

  Write_REG(0x23,0x63);
  Write_REG(0x12,0x3f);
  F_buff = buff + 0x10;
  Write_REG(0x22,0x07);
  *F_buff = RF_CMD_WRITE;
  *(F_buff + 1) = Block_Adr;
  temp = Command_Send(2, F_buff, Transceive);
  if(temp == FALSE)
  {
    return(FM1702_NOTAGERR);
  }

  temp = Read_REG(0x04);
  if(temp == 0)
  {
    return(FM1702_BYTECOUNTERR);
  }

  Read_FIFO(F_buff);
  temp = *F_buff;
  switch(temp)
  {
  case 0x00:	return(FM1702_NOTAUTHERR);
  case 0x04:	return(FM1702_EMPTY);
  case 0x0a:	break;
  case 0x01:	return(FM1702_CRCERR);
  case 0x05:	return(FM1702_PARITYERR);
  default:	return(FM1702_WRITEERR);
  }

  temp = Command_Send(16, buff, Transceive);
  if(temp == TRUE)
  {
    return(FM1702_OK);
  }
  else
  {
    temp = Read_REG(0x0A);
    if((temp & 0x02) == 0x02)
      return(FM1702_PARITYERR);
    else if((temp & 0x04) == 0x04)
      return(FM1702_FRAMINGERR);
    else if((temp & 0x08) == 0x08)
      return(FM1702_CRCERR);
    else
      return(FM1702_WRITEERR);
  }
}

/*********************************************************************
���ƣ���ʼ��FM1702SL
���ܣ�ʵ�ֳ�ʼ������

*********************************************************************/
void Init_FM1702(void)
{
    uchar i;
    Reset_On;
    for(i=0;i<0xff;i++)
    {
      ;
    }
    Reset_Off; 
    for(i=0;i<0xff;i++)
    {
      ;
    }
    
    SCK_CLR;
    NSS_SET;
    {   
		while(Read_REG(Command));
		Write_REG(Page_Sel,0x80);
		while(Read_REG(Command));
		Write_REG(Page_Sel,0);
    }
        
    for(i=0;i<6;i++)
    {
      FM1702_Buf[i]=0xff;
    }
  
}

void delay_10ms(unsigned int _10ms)
{
  unsigned int i;
  
  while (_10ms--)
  {
     for(i=0;i<1000;i++);
  }
}
/*********************************************************************
����: ReadMoney 
����: ������ȡ���ڽ��
���룺Block_Adr:���ַ������ 
��������ڽ����ΪFFFFFF=16777215��ʵ��Ϊ16777.215                                      
*********************************************************************/
u32 ReadMoney(uchar Block_Adr)
{
    u32 u32Dat=0x0;
	u8 ReadMoney_Buf[16];
	u8 status;
	status = MIF_READ(ReadMoney_Buf,Block_Adr);	//��ȡ���
	if(status== FM1702_OK)
    {
		if((CRC8_Table(ReadMoney_Buf,6)==ReadMoney_Buf[6])&&(ReadMoney_Buf[4]==RFID_OPENFLAG))	//CRC  �� ������־
		{
			if(ReadMoney_Buf[5]==0x00)	//������
			{
				if(ReadMoney_Buf[7]==0x01)	//�����־
				{
					if((ReadMoney_Buf[0]==GetFanma(ReadMoney_Buf[8]))\
					 &&(ReadMoney_Buf[1]==GetFanma(ReadMoney_Buf[9]))\
					 &&(ReadMoney_Buf[2]==GetFanma(ReadMoney_Buf[10]))\
					 &&(ReadMoney_Buf[3]==GetFanma(ReadMoney_Buf[11])) )
					{
						u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
						u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
						u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
						u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
					}
					else	u32Dat = ErrorMoney;	//����
				}
				else
				{
					u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
					u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
					u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
					u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
				}
			}
			else if(ReadMoney_Buf[5]==0x02)		{u32Dat = ErrorMoney+7;gErrorShow = ReadMoney_Buf[12];}	//�����쳣 E700//���ƿ�
			else 								{u32Dat = ErrorMoney+5;gErrorShow = ReadMoney_Buf[12];}	//�û��쳣 E000
		}
		else 	u32Dat = ErrorMoney;
    }
	else    u32Dat = ErrorMoney;
	return 	u32Dat;

}

/*********************************************************************
����: ReadRFIDMoney  
����: ������ȡ���ڽ�� �������
���룺Block_Adr:���ַ������ 
��������ڽ�� ���=ErrorMoney����������                                    
*********************************************************************/
u32 ReadRFIDMoney(uchar Block_Adr)
{
    u32 u32Dat=0x0;
	u8 ReadMoney_Buf[16];
	u8 status;
	status = MIF_READ(ReadMoney_Buf,Block_Adr);	//��ȡ���
	if(status== FM1702_OK)
    {
		if((CRC8_Table(ReadMoney_Buf,6)==ReadMoney_Buf[6])&&(ReadMoney_Buf[4]==RFID_OPENFLAG))	//CRC  �� ������־
		{
			if(ReadMoney_Buf[7]==0x01)	//�����־
			{
				if((ReadMoney_Buf[0]==GetFanma(ReadMoney_Buf[8]))\
				 &&(ReadMoney_Buf[1]==GetFanma(ReadMoney_Buf[9]))\
				 &&(ReadMoney_Buf[2]==GetFanma(ReadMoney_Buf[10]))\
				 &&(ReadMoney_Buf[3]==GetFanma(ReadMoney_Buf[11])) )
				{
					u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
					u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
					u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
					u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
				}
				else	u32Dat = ErrorMoney;	//����
			}
			else
			{
				u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
				u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
				u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
				u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
			}
		}
		else 	u32Dat = ErrorMoney;
    }
	else    u32Dat = ErrorMoney;
	return 	u32Dat;

}


/*********************************************************************
����: DecMoney 
����: �ݼ����ڽ�� 
���룺Block_Adr:���ַ������ 
	  Dec_dat:�ݼ����
����������������Ϊ50000��ʵ��Ϊ500.00                                      
*********************************************************************/
u32 DecMoney(uchar Block_Adr,uchar Dec_dat)
{
	u8 ReadMoney_Buf[16]={0};
	u8 status;
//	u32 uTemp=0;
	u32 u32Dat=0x0;
	status = MIF_READ(ReadMoney_Buf,Block_Adr);	//��ȡ���
	if(status== FM1702_OK)
	{
		if((CRC8_Table(ReadMoney_Buf,6)==ReadMoney_Buf[6])&&(ReadMoney_Buf[4]==RFID_OPENFLAG))	//CRC  �� ������־
		{
			if(ReadMoney_Buf[7]==0x01)	//�����־
			{
				if((ReadMoney_Buf[0]==GetFanma(ReadMoney_Buf[8]))\
				 &&(ReadMoney_Buf[1]==GetFanma(ReadMoney_Buf[9]))\
				 &&(ReadMoney_Buf[2]==GetFanma(ReadMoney_Buf[10]))\
				 &&(ReadMoney_Buf[3]==GetFanma(ReadMoney_Buf[11])) )
				{
					//u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
					u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
					u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
					u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
				}
				else	return ErrorMoney;	//����
			}
			else
			{
				//u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
				u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
				u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
				u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
			}
		}
		else 	return ErrorMoney;	//����

	}
	
	u32Dat = u32Dat - Dec_dat;	//��ֵ;
	
	ReadMoney_Buf[1] = u32Dat>>16;	//���1 ��λ
	ReadMoney_Buf[2] = u32Dat>>8;	//���2 
	ReadMoney_Buf[3] = u32Dat;		//���3	
	
	ReadMoney_Buf[6] = CRC8_Table(ReadMoney_Buf,6);
	ReadMoney_Buf[7] = 0x01;	//�����־  20171019 ���ӷ���
	ReadMoney_Buf[8] = GetFanma(ReadMoney_Buf[0]);	//���� ͨ����
	ReadMoney_Buf[9] = GetFanma(ReadMoney_Buf[1]);	//���� ���1 ��λ
	ReadMoney_Buf[10] = GetFanma(ReadMoney_Buf[2]);	//���� ���2
	ReadMoney_Buf[11] = GetFanma(ReadMoney_Buf[3]);	//���� ���3
	status = MIF_Write(ReadMoney_Buf,Block_Adr);
	status = MIF_READ(ReadMoney_Buf,Block_Adr);	//20171020 ���Ӷ�ȡ���
	if((status== FM1702_OK)&&\
	(u32Dat == (((u32)ReadMoney_Buf[1]<<16)|((u32)ReadMoney_Buf[2]<<8)|(u32)ReadMoney_Buf[3])))
    {	//20171020 ���Ӽ������ �ٶ������Ա�
		if((CRC8_Table(ReadMoney_Buf,6)==ReadMoney_Buf[6])&&(ReadMoney_Buf[4]==RFID_OPENFLAG))
		{
			if(ReadMoney_Buf[5]==0x00)	//������
			{
				u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
				u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
				u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
				u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
			}
			else 	u32Dat = ErrorMoney;
		}
		else 	u32Dat = ErrorMoney;
    }
	else    u32Dat = ErrorMoney;
	return 	u32Dat;
}

/*********************************************************************
����: WriteMoney 
����: д�뿨�ڽ�� 
���룺Block_Adr:���ַ������ 
	  Write_dat:д����
�����                                     
*********************************************************************/
u8 WriteMoney(uchar Block_Adr,u32 Write_dat)
{
	u8 ReadMoney_Buf[16]={0};
	u8 status;
	u32 u32Dat=0x0; 
	status = MIF_READ(ReadMoney_Buf,Block_Adr);	//��ȡ���
	if(status== FM1702_OK)
	{
		if((CRC8_Table(ReadMoney_Buf,6)==ReadMoney_Buf[6])&&(ReadMoney_Buf[4]==RFID_OPENFLAG))	//CRC  �� ������־
		{
			if(ReadMoney_Buf[7]==0x01)	//�����־
			{
				if((ReadMoney_Buf[0]==GetFanma(ReadMoney_Buf[8]))\
				 &&(ReadMoney_Buf[1]==GetFanma(ReadMoney_Buf[9]))\
				 &&(ReadMoney_Buf[2]==GetFanma(ReadMoney_Buf[10]))\
				 &&(ReadMoney_Buf[3]==GetFanma(ReadMoney_Buf[11])) )
				{
					u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
					u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
					u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
					u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
				}
			}
			else
			{
				u32Dat = u32Dat|((u32)ReadMoney_Buf[0]<<24);	//������ͨ��ʹ��
				u32Dat = u32Dat|((u32)ReadMoney_Buf[1]<<16);	//���1 ��λ
				u32Dat = u32Dat|((u32)ReadMoney_Buf[2]<<8);		//���2 
				u32Dat = u32Dat|((u32)ReadMoney_Buf[3]);		//���3							
			}
		}
	}
	if((Write_dat&0x00FFFFFF) >= ErrorMoney)
	{
		ReadMoney_Buf[0] = (u8)(u32Dat>>24); 	//У��
		ReadMoney_Buf[1] = (u8)(u32Dat>>16); 	//���ݸ�λ
		ReadMoney_Buf[2] = (u8)(u32Dat>>8);  	//����
		ReadMoney_Buf[3] = (u8)(u32Dat);   	//���ݵ�λ
		ReadMoney_Buf[4] = 0x01;    //�������
		ReadMoney_Buf[5] = 0x01;	//�û��쳣
    	ReadMoney_Buf[12] = Write_dat - ErrorMoney;  //�쳣��ʾ����
	}
	else
	{
		ReadMoney_Buf[0] = (u8)(Write_dat>>24); //У��
		ReadMoney_Buf[1] = (u8)(Write_dat>>16); //���ݸ�λ
		ReadMoney_Buf[2] = (u8)(Write_dat>>8);  //����
		ReadMoney_Buf[3] = (u8)(Write_dat);   	//���ݵ�λ
		ReadMoney_Buf[4] = 0x01;    //�������
		ReadMoney_Buf[5] = 0x00;
    	ReadMoney_Buf[12] = 0x00;  //�쳣��ʾ����
	}
	ReadMoney_Buf[6] = CRC8_Table(ReadMoney_Buf,6);
	ReadMoney_Buf[7] = 0x01;	//�����־  20171019 ���ӷ���
	ReadMoney_Buf[8] = GetFanma(ReadMoney_Buf[0]);	//���� ͨ����
	ReadMoney_Buf[9] = GetFanma(ReadMoney_Buf[1]);	//���� ���1 ��λ
	ReadMoney_Buf[10] = GetFanma(ReadMoney_Buf[2]);	//���� ���2
	ReadMoney_Buf[11] = GetFanma(ReadMoney_Buf[3]);	//���� ���3
	status = MIF_Write(ReadMoney_Buf,Block_Adr);
	return status;
}

u8 ReadRFIDCard(void)	//��ȡCARD״̬ ==FM1702_OK��ʾֻ���ڼ���Ƿ��п�����У������
{
    u8 status;
	status = Request(RF_CMD_REQUEST_ALL);		// Ѱ��
	if(status != FM1702_OK)	
	{	/*delay_10ms(2);*/ 		return status;	}

	status = AntiColl();						//��ͻ���
	if(status != FM1702_OK)    	return status;	
	else 						return status;

}

u8 ReadRFID(void)	//��ȡRFID״̬ ==FM1702_OK��ʾ��ȡ���ݳɹ�
{
	u8 status,password[6] = {0x0};
	u8 SectionNum = 0;
	password[0] = FM1702_Key[0];	password[1] = FM1702_Key[1];	password[2] = FM1702_Key[2];
	password[3] = FM1702_Key[3];	password[4] = FM1702_Key[4];	password[5] = FM1702_Key[5];
	SectionNum = FM1702_Key[6]/4;	//ȡ��������
	status = Request(RF_CMD_REQUEST_ALL);		// Ѱ��
	if(status != FM1702_OK) 
	{		/*delay_10ms(2);*/		return status;	}

	status = AntiColl();						//��ͻ���
	if(status != FM1702_OK)    	return status;	
	
	status=Select_Card();                       //ѡ��
	if(status != FM1702_OK)    	return status;

	status = Load_Key(password);                //��������
	if(status != TRUE)         	return status;

	status = Authentication(UID, SectionNum, RF_CMD_AUTH_LA);//У������
	if(status != FM1702_OK)    	return status;
	else 						return status;
}


u8 WriteRFID_Key_test(void)	//
{
	u8 status,password[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xF1};
	u8 SectionNum = 8;
	u8 RFID_Dat[16] = {0};
	status = Request(RF_CMD_REQUEST_ALL);		// Ѱ��
	if(status != FM1702_OK) 
	{		/*delay_10ms(2);*/		return status;	}

	status = AntiColl();						//��ͻ���
	if(status != FM1702_OK)    	return status;	

	status=Select_Card();                       //ѡ��
	if(status != FM1702_OK)    	return status;

	status = Load_Key(password);                //��������
	if(status != TRUE)         	return status;

	status = Authentication(UID, 2, RF_CMD_AUTH_LA);//У������
	if(status != FM1702_OK)    	return status;
	
    status=MIF_READ(RFID_Dat,SectionNum);	             //����
	if(status != FM1702_OK)    	return status;
	else 						return status;
	
}

void SoftReset1702(void)
{
    if(Read_REG(CRCPresetLSB)!=0x63)
    {
        Init_FM1702();  //FM1701��ʼ��
    }
}


