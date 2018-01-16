/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��
*	�ļ����� : bsp.h
*	˵    �� : ���ǵײ�����ģ�����е�h�ļ��Ļ����ļ��� Ӧ�ó���ֻ�� #include bsp.h ���ɣ�
*			  ����Ҫ#include ÿ��ģ��� h �ļ�
*
*	Copyright (C), 2017-2018, �����״ϿƼ����޹�˾
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

/* ���� BSP �汾�� */
#define __STM32F1_BSP_VERSION		"1.1"

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

#define VERSION		    0x12		/* �̼��汾�� */


#include "bsp_timer.h"		/* systick��ʱ��ģ�� */
#include "bsp_TimerTim3.h"
#include "bsp_Tim2.h"
#include "bsp_tm1639.h"		
#include "bsp_fm1701.h"		
#include "bsp_can.h"
#include "bsp_uart_fifo.h"
#include "bsp_crc8.h"
#include "bsp_adc.h"
#include "bsp_stmflash.h"

#define OutPut_ON()  GPIO_SetBits(GPIOB, GPIO_Pin_5)  	       	//
#define OutPut_OFF()  GPIO_ResetBits(GPIOB, GPIO_Pin_5) 	   	//

#endif

