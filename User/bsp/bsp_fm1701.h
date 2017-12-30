#ifndef __BSP_FM1701_H
#define __BSP_FM1701_H

#include "stm32f10x.h"

#define ErrorMoney		0x00EF0000         /*��������*/
#define RFID_OPENFLAG	0x01               /*������־*/

#define mifare1			1
#define mifarepro		2
#define mifarelight		3
#define unkowncard		4
#define unknowncard     4

/* FM1702������ */
#define Idle       	0x00          	/* ��ָ��*/
#define Transceive	0x1E			/* ���ͽ������� */
#define Transmit	0x1a			/* �������� */
#define ReadE2		0x03			/* ��FM1702 EEPROM���� */
#define WriteE2		0x01			/* дFM1702 EEPROM���� */
#define Authent1	0x0c			/* ��֤������֤���̵�1�� */
#define Authent2	0x14			/* ��֤������֤���̵�2�� */
#define LoadKeyE2	0x0b			/* ����Կ��EEPROM���Ƶ�KEY���� */
#define LoadKey		0x19			/* ����Կ��FIFO���渴�Ƶ�KEY���� */
//#define RF_TimeOut	0xfff			/* ����������ʱʱ�� */
#define RF_TimeOut	0x7f
#define Req		    0x01
#define Sel		    0x02

/* �������Ͷ��� */
#define uchar	unsigned char
#define uint	unsigned int

/* ��Ƭ���Ͷ��嶨�� */
#define TYPEA_MODE	    0			/* TypeAģʽ */
#define TYPEB_MODE	    1			/* TypeBģʽ */
#define SHANGHAI_MODE	2			/* �Ϻ�ģʽ */
#define TM0_HIGH	    0xf0		/* ��ʱ��0��λ,4MS��ʱ */
#define TM0_LOW		    0x60		/* ��ʱ��0��λ */
#define TIMEOUT		    100			/* ��ʱ������4MS��100=0.4�� */

/* ��Ƶ��ͨ�������붨�� */
#define RF_CMD_REQUEST_STD	0x26
#define RF_CMD_REQUEST_ALL	0x52
#define RF_CMD_ANTICOL		0x93
#define RF_CMD_SELECT		0x93
#define RF_CMD_AUTH_LA		0x60
#define RF_CMD_AUTH_LB		0x61
#define RF_CMD_READ		    0x30
#define RF_CMD_WRITE		0xa0
#define RF_CMD_INC		    0xc1
#define RF_CMD_DEC		    0xc0
#define RF_CMD_RESTORE		0xc2
#define RF_CMD_TRANSFER		0xb0
#define RF_CMD_HALT		    0x50

/* Status Values */
#define ALL	    0x01
#define KEYB	0x04
#define KEYA	0x00
#define _AB	    0x40
#define CRC_A	1
#define CRC_B	2
#define CRC_OK	0
#define CRC_ERR 1
#define BCC_OK	0
#define BCC_ERR 1

/* �����Ͷ��� */
#define MIFARE_8K	    0			/* MIFAREϵ��8KB��Ƭ */
#define MIFARE_TOKEN	1			/* MIFAREϵ��1KB TOKEN��Ƭ */
#define SHANGHAI_8K	    2			/* �Ϻ���׼ϵ��8KB��Ƭ */
#define SHANGHAI_TOKEN	3			/* �Ϻ���׼ϵ��1KB TOKEN��Ƭ */

/* ����������붨�� */
#define FM1702_OK		    0		/* ��ȷ */
#define FM1702_NOTAGERR		1		/* �޿� */
#define FM1702_CRCERR		2		/* ��ƬCRCУ����� */
#define FM1702_EMPTY		3		/* ��ֵ������� */
#define FM1702_AUTHERR		4		/* ��֤���ɹ� */
#define FM1702_PARITYERR	5		/* ��Ƭ��żУ����� */
#define FM1702_CODEERR		6		/* ͨѶ����(BCCУ���) */
#define FM1702_SERNRERR		8		/* ��Ƭ���кŴ���(anti-collision ����) */
#define FM1702_SELECTERR	9		/* ��Ƭ���ݳ����ֽڴ���(SELECT����) */
#define FM1702_NOTAUTHERR	10		/* ��Ƭû��ͨ����֤ */
#define FM1702_BITCOUNTERR	11		/* �ӿ�Ƭ���յ���λ������ */
#define FM1702_BYTECOUNTERR	12		/* �ӿ�Ƭ���յ����ֽ����������������Ч */
#define FM1702_RESTERR		13		/* ����restore�������� */
#define FM1702_TRANSERR		14		/* ����transfer�������� */
#define FM1702_WRITEERR		15		/* ����write�������� */
#define FM1702_INCRERR		16		/* ����increment�������� */
#define FM1702_DECRERR		17		/* ����decrement�������� */
#define FM1702_READERR		18		/* ����read�������� */
#define FM1702_LOADKEYERR	19		/* ����LOADKEY�������� */
#define FM1702_FRAMINGERR	20		/* FM1702֡���� */
#define FM1702_REQERR		21		/* ����req�������� */
#define FM1702_SELERR		22		/* ����sel�������� */
#define FM1702_ANTICOLLERR	23		/* ����anticoll�������� */
#define FM1702_INTIVALERR	24		/* ���ó�ʼ���������� */
#define FM1702_READVALERR	25		/* ���ø߼�����ֵ�������� */
#define FM1702_DESELECTERR	26
#define FM1702_CMD_ERR		42		/* ������� */

#define Page_Sel		0x00	/* ҳд�Ĵ��� */
#define Command			0x01	/* ����Ĵ��� */
#define FIFO			0x02	/* 64�ֽ�FIFO�������������Ĵ��� */
#define PrimaryStatus	0x03	/* ��������������FIFO��״̬�Ĵ���1 */
#define FIFO_Length		0x04	/* ��ǰFIFO���ֽ����Ĵ��� */
#define SecondaryStatus	0x05	/* ����״̬�Ĵ���2 */
#define InterruptEn		0x06	/* �ж�ʹ��/��ֹ�Ĵ��� */
#define Int_Req			0x07	/* �ж������ʶ�Ĵ��� */
#define Control			0x09	/* ���ƼĴ��� */
#define ErrorFlag		0x0A	/* ����״̬�Ĵ��� */
#define CollPos			0x0B	/* ��ͻ���Ĵ��� */
#define TimerValue		0x0c	/* ��ʱ����ǰֵ */
#define Bit_Frame		0x0F	/* λ֡�����Ĵ��� */
#define TxControl		0x11	/* ���Ϳ��ƼĴ��� */
#define CWConductance	0x12	/* ѡ�����TX1��TX2�������ߵ��迹 */
#define ModConductance	0x13	/* ������������迹 */
#define CoderControl	0x14	/* �������ģʽ��ʱ��Ƶ�� */
#define TypeBFraming	0x17	/* ����ISO14443B֡��ʽ */
#define DecoderControl	0x1a	/* ������ƼĴ��� */
#define RxThreshold     0x1C    
#define Rxcontrol2		0x1e	/* ������Ƽ�ѡ�����Դ */
#define RxWait			0x21	/* ѡ����ͽ���֮���ʱ���� */
#define ChannelRedundancy	0x22	/* RFͨ������ģʽ���üĴ��� */
#define CRCPresetLSB	0x23
#define CRCPresetMSB	0x24
#define MFOUTSelect		0x26	/* mf OUT ѡ�����üĴ��� */
#define TimerClock		0x2a	/* ��ʱ���������üĴ��� */
#define TimerControl	0x2b	/* ��ʱ�����ƼĴ��� */
#define TimerReload		0x2c	/* ��ʱ����ֵ�Ĵ��� */
#define TypeSH			0x31	/* �Ϻ���׼ѡ��Ĵ��� */
#define TestDigiSelect	0x3d	/* ���Թܽ����üĴ��� */


//�˿ڶ���
#define MISO            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define Reset_On        GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define Reset_Off       GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define SCK_SET         GPIO_SetBits(GPIOA, GPIO_Pin_7)
#define SCK_CLR         GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define MOSI_SET        GPIO_SetBits(GPIOA, GPIO_Pin_6)
#define MOSI_CLR        GPIO_ResetBits(GPIOA, GPIO_Pin_6)
#define NSS_SET         GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define NSS_CLR         GPIO_ResetBits(GPIOA, GPIO_Pin_5)


//======================================================================

void BspFM1701_Config(void);

void SPI_Send(unsigned char );
unsigned char SPI_Receve(void);
void Write_REG(unsigned char,unsigned char );
unsigned char Read_REG(unsigned char );

unsigned char Clear_FIFO(void);
void Write_FIFO(unsigned char count,unsigned char *Buff);
unsigned char Read_FIFO(unsigned char *Buff);
uchar Command_Send(uchar count,uchar *Buff,uchar Comm_Set);
unsigned char Request(unsigned char mode);
uchar Load_Key(uchar *Keybuf);
void KeyConvert(uchar *Coding,uchar *Coded);
unsigned char Load_Key(unsigned char *Keybuf);
unsigned char AntiColl(void);
unsigned char Select_Card(void);
unsigned char Authentication(unsigned char *UID, unsigned char SecNR, unsigned char mode);
unsigned char MIF_READ(unsigned char *buff, unsigned char Block_Adr);
unsigned char MIF_Write(unsigned char *buff, unsigned char Block_Adr);
void delay_10ms(unsigned int _10ms);
void Init_FM1702(void);
u32 ReadMoney(uchar Block_Adr);
u32 DecMoney(uchar Block_Adr,uchar Dec_dat);
u8 WriteMoney(uchar Block_Adr,u32 Write_dat);
u8 ReadRFID(void);	//��ȡRFID״̬ ==FM1702_OK��ʾ��ȡ���ݳɹ�
u8 ReadRFIDCard(void);
u32 ReadRFIDMoney(uchar Block_Adr);
u8 WriteRFID_Key_test(void);	//
void SoftReset1702(void);

#endif
