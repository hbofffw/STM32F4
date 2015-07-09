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
//#include "tm_stm32f4_usb_vcp.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_spi.h"
#include "ADS1256.h"

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
	TM_USART_Putc(USART2, ch);
	//TM_USB_VCP_Putc((uint8_t)ch);
	/* If everything is OK, you have to return character written */
	return ch;
	/* If character is not correct, you can return EOF (-1) to stop writing */
	//return -1;
}

int main(void) {
	uint8_t c;
	uint8_t s;
	int32_t adc[8];
	int32_t adct[1000];
	int32_t adctest;
	int32_t volt[8];
	uint8_t i;
	int count;
	int number;
	SystemInit();

	/* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
	TM_DISCO_LedInit();
	//TM_DISCO_ButtonInit();
	/*Delay init*/
	TM_DELAY_Init();
	//link wifi modual
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 115200);
	//usb
	TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);
	//TM_SPI_Init(SPI1, TM_SPI_PinsPack_Custom);

	/* Initialize ADC1 on channel 3, this is pin PA3 */
	//TM_ADC_Init(ADC1, ADC_Channel_3);
	Delayms(100); /* 等待上电稳定，等基准电压电路稳定, bsp_InitADS1256() 内部会进行自校准 */
	//InitADS1256(); /* 初始化配置ADS1256.  PGA=1, DRATE=30KSPS, BUFEN=1, 输入正负5V */
	Init_ADS1256_GPIO();
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	ADS1256_Init();
	/* 打印芯片ID (通过读ID可以判断硬件接口是否正常) , 正常时状态寄存器的高4bit = 3 */
	//{
	//	uint8_t id;
	//	id = ADS1256_ReadChipID();
	//	if (id != 3)
	//	{
	//		printf("Error, ADS1256 Chip ID = 0x%X\r\n", id);
	//	}
	//	else
	//	{
	//		printf("Ok, ADS1256 Chip ID = 0x%X\r\n", id);
	//	}
	//}
	//ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_30000SPS); /* 配置ADC参数： 增益1:1, 数据输出速率 2KHz */
	//ADS1256_StartScan(); 
	//ADS1256_SetChannal(0);
	while (1) {
		if (TM_DISCO_ButtonOnReleased())
		{
			//printf("ADC channel 4/3: %4d/%4d \n", TM_ADC_Read(ADC1, ADC_Channel_4), TM_ADC_Read(ADC1, ADC_Channel_3));
			Delayms(50);
		}
		s = TM_USART_Getc(USART1);

		if (s)
		{
			printf("%c", s);
		}
		c = TM_USART_Getc(USART2);
		if (c) {
			TM_USART_Putc(USART1, c);
			/* If anything received, put it back to terminal */
			TM_USART_Putc(USART1, c);
			//printf("rdy");
			if (c == 's')
			{
				for (i = 0; i < 8; i++)
				{
					/* 从全局缓冲区读取采样结果。 采样结果是在中断服务程序中读取的。*/
					//adc[i] = ADS1256_GetAdc(0);
					//adc[i] = ADS1256_GetAdc();

					/* 4194303 = 2.5V , 这是理论值，实际可以根据2.5V-基准的实际值进行公式矫正 */
					//volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* 计算实际电压值（近似估算的），如需准确，请进行校准 */
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
							printf("%d=%6d ", i, adc[i]);
						}
						else
						{
							printf("%d=%6d", i, adc[i]);
						}
						printf("\r\n");
					}
					Delayms(300);
				}
			}
			if (c == 'c')
			{
				printf("counting...\r\n");
				count = 0;
				number = 0;
				TM_DELAY_SetTime(0);
				do 
				{
					count++;
					Delay(1000);
				} while (TM_DELAY_Time()<=1000);
				
				//while (1)
				//{
				//	TM_DELAY_SetTime(0);
				//	//TM_DISCO_LedToggle(LED_GREEN);
				//	++count;
				//	if (count>=1000)
				//	{
				//		break;
				//	/}
				//	while (TM_DELAY_Time() < 1);
				//	//{
				//		/*TM_DELAY_SetTime2(0);
				//		while (1){
				//		if (TM_DELAY_Time2() >= 1)
				//		{
				//		count++;
				//		break;
				//		}
				//		}*/
				//		/*if (++count == 433)
				//		{
				//		number++;
				//		count = 0;
				//		}*/
				//		
				//	//}
				//}
				
				printf("timer test: %d\r\n", count);
			}
			if (c == 't')
			{
				count = 0;
				TM_DELAY_SetTime(0);
				do
				{
					adctest = ADS_sum((i << 4) | ADS1256_MUXN_AIN0);
					count++;
					//Delay(1000);
				} while (TM_DELAY_Time() <= 1000);
				printf("there are %d samples\r\n", count);
			}
			if (c == 'r')
			{
				adctest = ADS_sum((i << 4) | ADS1256_MUXN_AIN0);
				printf("data sampled: %d\r\n", adctest);
			}
			//if (c=='l')
			//{
			//	TM_USART_Puts(USART1,"Hi sir!\r\n");
			//}
			//TM_USART_Putc(USART1, s);
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

