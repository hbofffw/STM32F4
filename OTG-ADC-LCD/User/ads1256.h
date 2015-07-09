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

void SPI_Init1(void);

extern u8  results1;
extern u8  results2;
extern u8  results3;
void SPI_ADS1256_Init(void);
void ADS1256_GPIO_init(void);
void ADS1256_Init(void);
void ADS_sum(unsigned char road);
u8 SPI_SendByte(u8 byte);
int32_t ADS1256ReadData(void);
void ADS1256WREG(unsigned char regaddr, unsigned char databyte);

void ADS1256_SetChannal(uint8_t _ch);
void ADS1256_SetDiffChannal(uint8_t _ch);

void ADS1256_SetDiffChannal2(uint8_t _ch);

#endif



