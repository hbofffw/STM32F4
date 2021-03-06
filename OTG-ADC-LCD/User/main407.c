/**
*	USB VCP for STM32F4xx example.
*
*	@author		Tilen Majerle
*	@email		tilen@majerle.eu
*	@website	http://stm32f4-discovery.com
*	@ide		Keil uVision
*	@packs		STM32F4xx Keil packs version 2.2.0 or greater required
*	@stdperiph	STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
*
* Add line below to use this example with F429 Discovery board (in defines.h file)
*
* #define USE_USB_OTG_HS
*
* Before compile in Keil, select your target, I made some settings for different targets
*/

#include "defines.h"

/* In stdio.h file is everything related to output stream */
#include "stm32f4xx.h"
#include "tm_stm32f4_usb_vcp.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_spi.h"
#include "ads1256.h"

#include <stdio.h>
/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {

	int dummy;
};


/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
FILE __stdout;

int fputc(int ch, FILE *f) {
	/* Do your stuff here */
	/* Send your custom byte */
	/* Send byte to USART */
	//TM_USART_Putc(USART1, ch);
	TM_USB_VCP_Putc((uint8_t)ch);
	/* If everything is OK, you have to return character written */
	return ch;
	/* If character is not correct, you can return EOF (-1) to stop writing */
	//return -1;
}

int main(void) {
	uint8_t c;
	uint8_t s;
	int32_t adc[8];
	int32_t volt[8];
	uint8_t i;
	SystemInit();

	/* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
	TM_DISCO_LedInit();
	TM_DISCO_ButtonInit();
	/*Delay init*/
	TM_DELAY_Init();
	TM_USART_Init(USART1, TM_USART_PinsPack_2, 9600);
	//TM_SPI_Init(SPI1, TM_SPI_PinsPack_Custom);
	/* Initialize USB VCP */
	TM_USB_VCP_Init();
	if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
		/*USB OK, drivers OK*/
		TM_DISCO_LedOn(LED_GREEN);
	}
	else {
		/* USB not OK */
		TM_DISCO_LedOff(LED_GREEN);
	}
	/* Initialize ADC1 on channel 0, this is pin PA0 */
	//TM_ADC_Init(ADC1, ADC_Channel_4);

	/* Initialize ADC1 on channel 3, this is pin PA3 */
	//TM_ADC_Init(ADC1, ADC_Channel_3);
	Delayms(500); /* 等待上电稳定，等基准电压电路稳定, bsp_InitADS1256() 内部会进行自校准 */
	InitADS1256(); /* 初始化配置ADS1256.  PGA=1, DRATE=30KSPS, BUFEN=1, 输入正负5V */
	/* 打印芯片ID (通过读ID可以判断硬件接口是否正常) , 正常时状态寄存器的高4bit = 3 */
	{
		uint8_t id;
		id = ADS1256_ReadChipID();
		if (id != 3)
		{
			printf("Error, ADS1256 Chip ID = 0x%X\r\n", id);
		}
		else
		{
			printf("Ok, ADS1256 Chip ID = 0x%X\r\n", id);
		}
	}
	ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS); /* 配置ADC参数： 增益1:1, 数据输出速率 1KHz */
	ADS1256_StartScan(); /* 启动中断扫描模式, 轮流采集8个通道的ADC数据. 通过 ADS1256_GetAdc() 函数来读取这些数据 */

	while (1) {
		if (TM_DISCO_ButtonOnReleased())
		{
			//printf("ADC channel 4/3: %4d/%4d \n", TM_ADC_Read(ADC1, ADC_Channel_4), TM_ADC_Read(ADC1, ADC_Channel_3));
			Delayms(50);
		}
		/* USB configured OK, drivers OK */
		if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
			/*USB OK, drivers OK*/
			TM_DISCO_LedOn(LED_GREEN);
		}
		else {
			/* USB not OK */
			TM_DISCO_LedOff(LED_GREEN);
		}

		if (TM_USB_VCP_Getc(&s) == TM_USB_VCP_DATA_OK){
			//printf("rdy");
			if (s == 's')
			{
				for (i = 0; i < 8; i++)
				{
					/* 从全局缓冲区读取采样结果。 采样结果是在中断服务程序中读取的。*/
					adc[i] = ADS1256_GetAdc(i);

					/* 4194303 = 2.5V , 这是理论值，实际可以根据2.5V基准的实际值进行公式矫正 */
					volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* 计算实际电压值（近似估算的），如需准确，请进行校准 */
				}
				/* 打印采集数据 */
				{
					int32_t iTemp;

					for (i = 0; i < 8; i++)
					{
						iTemp = volt[i];	/* 余数，uV  */
						if (iTemp < 0)
						{
							iTemp = -iTemp;
							printf("%d=%6d,(-%d.%03d %03d V) ", i, adc[i], iTemp / 1000000, (iTemp % 1000000) / 1000, iTemp % 1000);
						}
						else
						{
							printf("%d=%6d,( %d.%03d %03d V) ", i, adc[i], iTemp / 1000000, (iTemp % 1000000) / 1000, iTemp % 1000);
						}
						printf("\r\n");
					}
					Delayms(300);
				}
			}
			//TM_USART_Putc(USART1, s);
		}
		c = TM_USART_Getc(USART1);
		if (c)
		{
			printf("%c", c);
		}
	}
}


/* Custom pins callback for SPI */
//void TM_SPI_InitCustomPinsCallback(SPI_TypeDef* SPIx) {
//	/* Check for SPI1 */
//	if (SPIx == SPI1) {
//		TM_DISCO_LedOn(LED_ORANGE);
//		/* Init custom pins */
//		/* MOSI = PA7, MISO = PA6 */
//		TM_GPIO_InitAlternate(GPIOA, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Medium, GPIO_AF_SPI1);
//	}
//}

