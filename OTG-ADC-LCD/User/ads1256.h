/*
*********************************************************************************************************
*
*	模块名称 : ADS1256 驱动模块(8通道带PGA的24位ADC)
*	文件名称 : bsp_ads1256.h
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _ADS1256_H
#define _ADS1256_H

#include "defines.h"
#include "stm32f4xx.h"
#include "misc.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_delay.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "tm_stm32f4_spi.h"
#include "tm_stm32f4_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

/* 增益选项 */
typedef enum
{
	ADS1256_GAIN_1 = (0),	/* 增益1（缺省） */
	ADS1256_GAIN_2 = (1),	/* 增益2 */
	ADS1256_GAIN_4 = (2),	/* 增益4 */
	ADS1256_GAIN_8 = (3),	/* 增益8 */
	ADS1256_GAIN_16 = (4),	/* 增益16 */
	ADS1256_GAIN_32 = (5),	/* 增益32 */
	ADS1256_GAIN_64 = (6),	/* 增益64 */
}ADS1256_GAIN_E;

/* 采样速率选项 */
/* 数据转换率选择
11110000 = 30,000SPS (default)
11100000 = 15,000SPS
11010000 = 7,500SPS
11000000 = 3,750SPS
10110000 = 2,000SPS
10100001 = 1,000SPS
10010010 = 500SPS
10000010 = 100SPS
01110010 = 60SPS
01100011 = 50SPS
01010011 = 30SPS
01000011 = 25SPS
00110011 = 15SPS
00100011 = 10SPS
00010011 = 5SPS
00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int32_t AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;			/* 当前通道 */
}ADS1256_VAR_T;

void InitADS1256(void);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

uint8_t ADS1256_ReadChipID(void);
int32_t ADS1256_ReadAdc(void);
void ADS1256_StartScan(void);
void ADS1256_StopScan(void);
int32_t ADS1256_GetAdc(uint8_t _ch);
uint8_t SPI_SendByte(uint8_t byte);
extern ADS1256_VAR_T g_tADS1256;

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
