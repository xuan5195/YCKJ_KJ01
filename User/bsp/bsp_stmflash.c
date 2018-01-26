#include "bsp_stmflash.h"
#include "bsp_crc8.h"

#define FLASH_SAVE_ADD		0x0800FC00		//Flash�洢��ʼ��ַ ΪFlash���һҳ����СΪ1K

uint8_t Physical_ADD[4]={0x20,0x17,0x12,0x02};//�����ַ 0x5E
uint8_t FM1702_Key[7]={0xFF,0xFF,0xFF,0xFF,0xFF,0xF1,0x29};
uint8_t WaterCost=50,CostNum=29;	//WaterCost=ˮ�� ��С�ۿ���  //������
uint8_t g_IAP_Flag=0x00;	//����������־
 
//��ȡָ����ַ�İ���(16λ����)
//faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}


void STMFLASH_Write(u32 WriteAddr,u8 *pBuffer,u16 NumToWrite)	
{
 	u16 i,DateTemp=0;    
	FLASH_Unlock();		//����
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(FLASH_SAVE_ADD);	
	for(i=0;i<NumToWrite;(i=i+2))
	{
		DateTemp = ((u16)pBuffer[i+1])<<8 ;
		DateTemp = DateTemp|pBuffer[i];
		FLASH_ProgramHalfWord(WriteAddr,DateTemp);
	    WriteAddr+=2;//��ַ����2.
	}  
	FLASH_Lock();//����
}

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Read(u32 ReadAddr,u8 *pBuffer,u16 NumToRead)   	
{
	u16 i,DateTemp=0;
	for(i=0;i<NumToRead;(i=i+2))
	{
		DateTemp = STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		pBuffer[i]	=(u8)(DateTemp);
		pBuffer[i+1]=(u8)(DateTemp>>8);
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}



void Read_Flash_Dat(void)
{
	u8 datatemp[16]={0};
	STMFLASH_Read(FLASH_SAVE_ADD,datatemp,16);
	if((datatemp[0]!=0x00)&&(datatemp[0]!=0xFF)&&(datatemp[1]!=0x00)&&(datatemp[1]!=0xFF))
	{
		Physical_ADD[0] = datatemp[0];	//�����ַ0
		Physical_ADD[1] = datatemp[1];	//�����ַ1
		Physical_ADD[2] = datatemp[2];	//�����ַ2
		Physical_ADD[3] = datatemp[3];	//�����ַ3
		if(( CRC8_Table(datatemp+4,11) == datatemp[15] )&&( datatemp[4] == 0xAA ))
		{
			FM1702_Key[0] 	= datatemp[5];	//RFID_key0
			FM1702_Key[1] 	= datatemp[6];	//RFID_key1
			FM1702_Key[2] 	= datatemp[7];	//RFID_key2
			FM1702_Key[3] 	= datatemp[8];	//RFID_key3
			FM1702_Key[4] 	= datatemp[9];	//RFID_key4
			FM1702_Key[5] 	= datatemp[10];	//RFID_key5
			FM1702_Key[6] 	= datatemp[11];	//���ַ
			WaterCost 		= datatemp[12];	//ˮ�� ��С�ۿ���
			CostNum 		= datatemp[13];	//������
			g_IAP_Flag 		= datatemp[14];	//����������־
		}
		else
		{
			datatemp[ 5] = FM1702_Key[0];	//RFID_key0
			datatemp[ 6] = FM1702_Key[1];	//RFID_key1
			datatemp[ 7] = FM1702_Key[2];	//RFID_key2
			datatemp[ 8] = FM1702_Key[3];	//RFID_key3
			datatemp[ 9] = FM1702_Key[4];	//RFID_key4
			datatemp[10] = FM1702_Key[5];	//RFID_key5
			datatemp[11] = FM1702_Key[6];	//���ַ
			datatemp[12] = WaterCost;		//ˮ�� ��С�ۿ���
			datatemp[13] = CostNum;			//������
			datatemp[14] = g_IAP_Flag;		//����������־
			datatemp[15] = CRC8_Table(datatemp+4,11);
			STMFLASH_Write(FLASH_SAVE_ADD,datatemp,16);			
		}			
	}
	else
	{
		datatemp[ 0] = Physical_ADD[0];	//�����ַ0
		datatemp[ 1] = Physical_ADD[1];	//�����ַ1
		datatemp[ 2] = Physical_ADD[2];	//�����ַ2
		datatemp[ 3] = Physical_ADD[3];	//�����ַ3
		datatemp[ 4] = 0xAA;			//��־λ
		datatemp[ 5] = FM1702_Key[0];	//RFID_key0
		datatemp[ 6] = FM1702_Key[1];	//RFID_key1
		datatemp[ 7] = FM1702_Key[2];	//RFID_key2
		datatemp[ 8] = FM1702_Key[3];	//RFID_key3
		datatemp[ 9] = FM1702_Key[4];	//RFID_key4
		datatemp[10] = FM1702_Key[5];	//RFID_key5
		datatemp[11] = FM1702_Key[6];	//���ַ
		datatemp[12] = WaterCost;		//ˮ�� ��С�ۿ���
		datatemp[13] = CostNum;			//������
		datatemp[14] = g_IAP_Flag;		//����������־
		datatemp[15] = CRC8_Table(datatemp+4,11);
		STMFLASH_Write(FLASH_SAVE_ADD,datatemp,16);
	}
}

void Write_Flash_Dat(void)
{
	u8 datatemp1[16]={0},datatemp2[16]={0};
	STMFLASH_Read(FLASH_SAVE_ADD,datatemp1,16);	//������ֵ �Ա� ����Ҫд���ֵ
	datatemp2[ 0] = Physical_ADD[0];//�����ַ0
	datatemp2[ 1] = Physical_ADD[1];//�����ַ1
	datatemp2[ 2] = Physical_ADD[2];//�����ַ2
	datatemp2[ 3] = Physical_ADD[3];//�����ַ3
	datatemp2[ 4] = 0xAA;			//��־λ
	datatemp2[ 5] = FM1702_Key[0];	//RFID_key0
	datatemp2[ 6] = FM1702_Key[1];	//RFID_key1
	datatemp2[ 7] = FM1702_Key[2];	//RFID_key2
	datatemp2[ 8] = FM1702_Key[3];	//RFID_key3
	datatemp2[ 9] = FM1702_Key[4];	//RFID_key4
	datatemp2[10] = FM1702_Key[5];	//RFID_key5
	datatemp2[11] = FM1702_Key[6];	//���ַ
	datatemp2[12] = WaterCost;		//ˮ�� ��С�ۿ���
	datatemp2[13] = CostNum;		//������
	datatemp2[14] = g_IAP_Flag;		//�̶�������־
	datatemp2[15] = CRC8_Table(datatemp2+4,11);
	if((datatemp2[15]!=datatemp1[15])||(datatemp2[13]!=datatemp1[13])||(datatemp2[12]!=datatemp1[12])||\
	   (datatemp2[11]!=datatemp1[11])||(datatemp2[10]!=datatemp1[10])||(datatemp2[ 9]!=datatemp1[ 9])||\
	   (datatemp2[ 8]!=datatemp1[ 8])||(datatemp2[ 7]!=datatemp1[ 7])||(datatemp2[ 6]!=datatemp1[ 6])||\
	   (datatemp2[ 5]!=datatemp1[ 5]))	//���ݲ�һ����дFlash
		{STMFLASH_Write(FLASH_SAVE_ADD,datatemp2,16);}
}


