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
#include "tm_stm32f4_general.h"
#include "ads1256.h"
#include "tm_stm32f4_gpio.h"
#include "stm32f4xx_gpio.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_tim.h"
#include "tm_stm32f4_stdio.h"
/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */

int32_t adcsamples[100][200];

void convert_and_print(int32_t value);
//struct __FILE {
//
//	int dummy;
//};


/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
//FILE __stdout;
//
//int fputc(int ch, FILE *f) {
//	/* Do your stuff here */
//	/* Send your custom byte */
//	/* Send byte to USART */
//	TM_USART_Putc(USART2, ch);
//	//TM_USB_VCP_Putc((uint8_t)ch);
//	/* If everything is OK, you have to return character written */
//	return ch;
//	/* If character is not correct, you can return EOF (-1) to stop writing */
//	//return -1;
//}

FILE USART1_Stream;
/* Output stream for USART1 function references */
int USART1_Stream_OutputFunction(int ch, FILE* f);
/* Input stream for USART1 function references */
int USART1_Stream_InputFunction(FILE* f);


/* Handle stdout actions = default name and can not be changed */
int TM_STDIO_StdoutHandler(int ch, FILE* f) {
	/* Send data to USART1 */
	TM_USART_Putc(USART2, (char)ch);

	/* Return ch, it means OK */
	return ch;
	/* If you want to return error, then you have to send EOF (-1) */
	//return -1;
}

/* Handle stdin actions */
int TM_STDIO_StdinHandler(FILE* f) {
	/* If any data at USART, return them */
	/* Do your custom implementation here when string ends */
	if (!TM_USART_BufferEmpty(USART2)) {
		return (int)TM_USART_Getc(USART2);
	}

	/* End of data, string is valid */
	/* You have to send -1 at the end of string */
	return -1;
}

int main(void) {
	uint8_t c;
	uint8_t s;
	int32_t adc[8];
	uint32_t val;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef ShortDelayTimer;
	int32_t adctest;
	int32_t volt[8];
	uint8_t i;
	uint8_t j;
	int count;
	int number;
	SystemInit();


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	ShortDelayTimer.TIM_Prescaler = 1;
	ShortDelayTimer.TIM_CounterMode = TIM_CounterMode_Up;
	ShortDelayTimer.TIM_Period = 0xFFFFFFFF;
	ShortDelayTimer.TIM_ClockDivision = TIM_CKD_DIV1;
	ShortDelayTimer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &ShortDelayTimer);
	TIM_Cmd(TIM2, ENABLE);


	/* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
	//TM_DISCO_LedInit();
	//TM_DISCO_ButtonInit();
	/*Delay init*/
	TM_DELAY_Init();
	//link wifi modual
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 115200);
	//usb
	TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);

	//usart output function
	TM_STDIO_SetOutputFunction(&USART1_Stream, USART1_Stream_OutputFunction);
	//input function
	TM_STDIO_SetInputFunction(&USART1_Stream, USART1_Stream_InputFunction);
	/*memset(adcsample1, 0x00, sizeof(adcsample1));
	memset(adcsample2, 0x00 + sizeof(adcsample1), sizeof(adcsample2));*/

	for (j = 0; j < 100; j++)
	{
		memset(adcsamples[j], 0x00 + sizeof(adcsamples[j])*j, sizeof(adcsamples[j]));
	}

	/* Get system reset source and clear flags after read */
	printf("System reset source: %d\r\n", (uint8_t)TM_GENERAL_GetResetSource(1));

	/* Get system reset source and clear flags after read */
	/* You should see number which corresponds to "None", because we cleared flags in statement above */
	printf("System reset source: %d\r\n", (uint8_t)TM_GENERAL_GetResetSource(1));

	/* Get system core and PCLK1 (Peripheral Clock 1, APB1) clocks */
	printf("System core clock: %u Hz; PCLK1 clock: %u Hz\r\n",
		TM_GENERAL_GetClockSpeed(TM_GENERAL_Clock_SYSCLK),
		TM_GENERAL_GetClockSpeed(TM_GENERAL_Clock_PCLK1)
		);
	val = TM_GENERAL_DWTCounterEnable();
	if (val != 0)
	{
		printf("DWT counter started! \r\n");
	}
	else
	{
		printf("DWT counter not started! \r\n");
	}
	//TM_SPI_Init(SPI1, TM_SPI_PinsPack_Custom);

	/* Initialize ADC1 on channel 3, this is pin PA3 */
	//TM_ADC_Init(ADC1, ADC_Channel_3);
	Delayms(100); /* 等待上电稳定，等基准电压电路稳定, bsp_InitADS1256() 内部会进行自校准 */
	//InitADS1256(); /* 初始化配置ADS1256.  PGA=1, DRATE=30KSPS, BUFEN=1, 输入正负5V */
	//SPI_Init2();
	InitADS1256();
	//ADS1256_Init();
	/*ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_30000SPS);
	Delay(10);*/
	//ADS1256_StartScan();
	//ADS1256_SetChannal(0);
	/* 打印芯片ID (通过读ID可以判断硬件接口是否正常) , 正常时状态寄存器的高4bit = 3 */
	/*{
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
		}*/
	ADS1256_CfgADC(ADS1256_GAIN_64, ADS1256_15000SPS); /* 配置ADC参数： 增益1:1, 数据输出速率 2KHz */
	//ADS1256_StartScan(); 
	//ADS1256_SetChannal(0);
	while (1) {
		
		adctest = 0;
		s = TM_USART_Getc(USART1);

		if (s)
		{
			printf("%c", s);
		}
		c = TM_USART_Getc(USART2);
		if (c) {
			//TM_USART_Putc(USART1, c);
			/* If anything received, put it back to terminal */
			//TM_USART_Putc(USART1, c);
			//printf("rdy");
			if (c == 's')
			{
				c = 0;
				ADS1256_StartScan();
				for (i = 0; i < 8; i++)
				{
					/* 从全局缓冲区读取采样结果。 采样结果是在中断服务程序中读取的。*/
					adc[i] = ADS1256_ReadAdc();// ADS1256_GetAdc(i);
					//adc[i] = ADS1256_GetAdc();

					/* 4194303 = 2.5V , 这是理论值，实际可以根据2.5V-基准的实际值进行公式矫正 */
					volt[i] = ((int64_t)adc[i] * 2500000) / 4194303;	/* 计算实际电压值（近似估算的），如需准确，请进行校准 */
				}
				ADS1256_StopScan();
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
			if (c == 't')
			{
				printf("start sampling...\r\n");
				ADS1256_StartScan();
				c = 0;
				count = 0;
				//TM_DELAY_SetTime(0);
				for (j = 0; j < 5; j++)
				{
					for (i = 0; i < 200; i++)
					{
						//TM_DELAY_SetTime(0);
						TM_GENERAL_DWTCounterSetValue(0);
						adcsamples[j][i] = ADS1256_ReadAdc();
						while (TM_GENERAL_DWTCounterGetValue() < 84000);
						count++;
						//if (TM_DELAY_Time()>=1000)
						//	break;
						//if (TM_GENERAL_DWTCounterGetValue() < 84000);   //心跳改为 1/64 ms
					}
				}
				ADS1256_StopScan();
				printf("\r\nthere are %d samples\r\n", count);
				//fprintf(&USART1_Stream, "\r\nthere are %d samples\r\n", count);
				//Delay(1000000);
				for (j = 0; j < 5; j++)
				{
					for (i = 0; i < 200; i++)
					{
						//printf("%d ", adcsamples[j][i]);
						convert_and_print(adcsamples[j][i]);
						/*fprintf(&USART1_Stream, "%d, ", adcsamples[j][i]);*/
					}
				}
				printf("\r\n");
				//fprintf(&USART1_Stream, "\r\n");
			}
			if (c == 'c')
			{
				printf("counting...\r\n");
				count = 0;
				number = 0;
				TM_DELAY_SetTime(0);
				/*TIM_SetCounter(TIM2, 0);
				Delay(1000000);
				printf("Now tim counter is %d\r\n", TIM_GetCounter(TIM2));*/
				do
				{
					TM_GENERAL_DWTCounterSetValue(0);
					TM_DISCO_LedToggle(TM_DISCO_LED_PINS);
					/*if (count == 0)
					{
					GPIO_SetBits(GPIOB, GPIO_Pin_9);
					count = 1;
					}
					else
					{
					GPIO_ResetBits(GPIOB, GPIO_Pin_9);
					count = 0;
					}*/
					count++;
					//Delay(1000);
					//while (TM_GENERAL_DWTCounterGetValue() < 84000);
				} while (TM_DELAY_Time() < 1000);

				printf("timer test: %d\r\n", count);
			}
			if (c == '1')
			{
				printf("start sampling...\r\n");
				ADS1256_StartScan();
				c = 0;
				count = 0;
				for (j = 0; j < 5; j++)
				{
					for (i = 0; i < 200; i++)
					{
						//TM_DELAY_SetTime(0);
						TM_GENERAL_DWTCounterSetValue(0);
						adcsamples[j][i] = ADS1256_ReadAdc();
						count++;
						while (TM_GENERAL_DWTCounterGetValue() < 84000);   //心跳改为 1/64 ms
					}
				}
				ADS1256_StopScan();
				printf("\r\nthere are %d samples\r\n", count);
				//fprintf(&USART1_Stream, "\r\nthere are %d samples\r\n", count);
				//Delay(1000000);
				for (j = 0; j < 5; j++)
				{
					for (i = 0; i < 200; i++)
					{
						printf("%d ", adcsamples[j][i]);
						/*fprintf(&USART1_Stream, "%d, ", adcsamples[j][i]);*/
					}
				}
				printf("\r\n");
				//fprintf(&USART1_Stream, "\r\n");
			}
			if (c == '2')
			{
				printf("start sampling...\r\n");
				ADS1256_StartScan();
				c = 0;
				count = 0;

				for (j = 0; j < 10; j++)
				{
					for (i = 0; i < 200; i++)
					{
						//TM_DELAY_SetTime(0);
						TM_GENERAL_DWTCounterSetValue(0);
						adcsamples[j][i] = ADS1256_ReadAdc();
						count++;
						while (TM_GENERAL_DWTCounterGetValue() < 42000);   //1/2ms
					}
				}

				ADS1256_StopScan();
				printf("\r\nthere are %d samples\r\n", count);
				//fprintf(&USART1_Stream, "\r\nthere are %d samples\r\n", count);
				//Delay(1000000);
				for (j = 0; j < 10; j++)
				{
					for (i = 0; i < 200; i++)
					{
						printf("%d ", adcsamples[j][i]);
						//fprintf(&USART1_Stream, "%d, ", adcsamples[j][i]);
					}
				}
				printf("\r\n");
				//fprintf(&USART1_Stream, "\r\n");
			}
			if (c == '4')
			{
				printf("start sampling...\r\n");
				ADS1256_StartScan();
				count = 0;

				for (j = 0; j < 20; j++)
				{
					for (i = 0; i < 200; i++)
					{
						//TM_DELAY_SetTime(0);
						TM_GENERAL_DWTCounterSetValue(0);
						adcsamples[j][i] = ADS1256_ReadAdc();
						count++;
						while (TM_GENERAL_DWTCounterGetValue() < 21000);   //1/2ms
					}
				}

				ADS1256_StopScan();
				printf("\r\nthere are %d samples\r\n", count);
				//fprintf(&USART1_Stream, "\r\nthere are %d samples\r\n", count);
				//Delay(1000000);
				for (j = 0; j < 20; j++)
				{
					for (i = 0; i < 200; i++)
					{
						printf("%d ", adcsamples[j][i]);
						//fprintf(&USART1_Stream, "%d, ", adcsamples[j][i]);
					}
				}
				printf("\r\n");
				//fprintf(&USART1_Stream, "\r\n");
			}
			if (c == '8')
			{
				printf("start sampling...\r\n");
				ADS1256_StartScan();
				count = 0;

				for (j = 0; j < 40; j++)
				{
					for (i = 0; i < 200; i++)
					{
						//TM_DELAY_SetTime(0);
						TM_GENERAL_DWTCounterSetValue(0);
						adcsamples[j][i] = ADS1256_ReadAdc();
						count++;
						while (TM_GENERAL_DWTCounterGetValue() < 10500);   //1/2ms
					}
				}

				ADS1256_StopScan();
				printf("\r\nthere are %d samples\r\n", count);
				//fprintf(&USART1_Stream, "\r\nthere are %d samples\r\n", count);
				//Delay(1000000);
				for (j = 0; j < 40; j++)
				{
					for (i = 0; i < 200; i++)
					{
						printf("%d ", adcsamples[j][i]);
						//fprintf(&USART1_Stream, "%d, ", adcsamples[j][i]);
					}
				}
				printf("\r\n");
				//fprintf(&USART1_Stream, "\r\n");
			}
			if (c == 'l')
			{
				printf("start sampling...\r\n");
				ADS1256_StartScan();
				TM_DELAY_SetTime(0);
				do
				{
					adctest = ADS1256_ReadAdc();
					count++;
				} while (TM_DELAY_Time() <= 32000);
				ADS1256_StopScan();
				printf("there are %d samples\r\n", count);
				printf("data sampled: %d\r\n", adctest);
			}
			if (c == 'r')
			{
				fprintf(&USART1_Stream, "I find u %c\r\n", c);
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
/* USART1 output stream handler = custom function name, linked with USART2 stream in the beginning of main() function */
int USART1_Stream_OutputFunction(int ch, FILE* f) {
	/* Send char via USART2 */
	TM_USART_Putc(USART1, (char)ch);

	/* Return ch, it means OK */
	return ch;
	/* If you want to return error, then you have to send EOF (-1) */
	//return -1;
}
/* Handle USART1 stream input = custom function name, linked with USART2 stream in the beginning of main() function */
int USART1_Stream_InputFunction(FILE* f) {
	/* If any data at USART, return them */
	/* Do your custom implementation here when string ends */
	if (!TM_USART_BufferEmpty(USART1)) {
		return (int)TM_USART_Getc(USART1);
	}

	/* End of data, string is valid */
	/* You have to send -1 at the end of string */
	return -1;
}

void convert_and_print(int32_t value)
{
	int32_t volt;
	int32_t iTemp;
	int i;
	volt = ((int64_t)value * 2500000) / 4194303;
	iTemp = volt;	/* 余数，uV  */
	if (iTemp < 0)
	{
		iTemp = -iTemp;
		printf("-%d.%03d%03d", iTemp / 1000000, (iTemp % 1000000) / 1000, iTemp % 1000);
	}
	else
	{
		printf("%d.%03d%03d", iTemp / 1000000, (iTemp % 1000000) / 1000, iTemp % 1000);
	}	
	printf(" ");
}