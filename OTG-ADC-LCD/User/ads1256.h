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
+5V <------  5.0V      5V����
GND	------ - GND       ��
DRDY------>  PC9       ׼������
CS    <------PB12       SPI_CS
DIN   <------PC2       SPI_MOSI
DOUT------>  PC3       SPI_MISO
SCLK  <------PB10      SPIʱ��
GND  ------- GND       ��
//-PDWN  <------  PB0       �������
RST   <------PC4       ��λ�ź�
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
	ADS1256_GAIN_1 = (0),	/* ����1��ȱʡ�� */
	ADS1256_GAIN_2 = (1),	/* ����2 */
	ADS1256_GAIN_4 = (2),	/* ����4 */
	ADS1256_GAIN_8 = (3),	/* ����8 */
	ADS1256_GAIN_16 = (4),	/* ����16 */
	ADS1256_GAIN_32 = (5),	/* ����32 */
	ADS1256_GAIN_64 = (6),	/* ����64 */
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
	ADS1256_GAIN_E Gain;		/* ���� */
	ADS1256_DRATE_E DataRate;	/* ����������� */
	int32_t AdcNow[8];			/* 8·ADC�ɼ������ʵʱ���з����� */
	uint8_t Channel;			/* ��ǰͨ�� */	
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



