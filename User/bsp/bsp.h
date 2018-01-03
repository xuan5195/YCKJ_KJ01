/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
*	说    明 : 这是底层驱动模块所有的h文件的汇总文件。 应用程序只需 #include bsp.h 即可，
*			  不需要#include 每个模块的 h 文件
*
*	Copyright (C), 2017-2018, 厦门易聪科技有限公司
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

/* 定义 BSP 版本号 */
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

#define VERSION		    0xA1		/* 固件版本号 */


#include "bsp_timer.h"		/* systick定时器模块 */
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

