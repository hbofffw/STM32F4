#ifndef __ADS1256_H_
#define	__ADS1256_H_

//***************************
//		Pin assign	   	
//		GPIOC10	---	RESET
//		GPIOC11	--- DRDY
//		GPIOB12	--- CS
//		GPIOB13	--- SCK
//		GPIOB14	--- DOUT
//		GPIOB15	--- DIN
//***************************					

/*//FOR F401
+5V <------  5.0V      5V供电
GND	------ - GND       地
DRDY------>  PC9       准备就绪
CS    <------PB12       SPI_CS
DIN   <------PC2       SPI_MOSI
DOUT------>  PC3       SPI_MISO
SCLK  <------PB10      SPI时钟
GND  ------- GND       地
//-PDWN  <------  PB0       掉电控制
RST   <------PC4       复位信号
*/
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

typedef struct
{
	ADS1256_GAIN_E Gain;		/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int32_t AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;			/* 当前通道 */	
}ADS1256_VAR_T;

void SPI_Init2(void);

extern u8  results1;
extern u8  results2;
extern u8  results3;
void SPI_ADS1256_Init(void);
void ADS1256_GPIO_init(void);
void ADS1256_Init(void);
void ADS_sum(void);
u8 SPI_SendByte(u8 byte);
int32_t ADS1256_ReadData(void);
void ADS1256WREG(unsigned char regaddr, unsigned char databyte);
void ADS1256_ResetHard(void);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
void ADS1256_SetChannal(uint8_t _ch);
void ADS1256_SetDiffChannal(uint8_t _ch);

void ADS1256_SetDiffChannal2(uint8_t _ch);

extern ADS1256_VAR_T g_tADS1256;

#endif



