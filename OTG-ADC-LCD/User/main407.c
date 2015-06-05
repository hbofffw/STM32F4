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
	long ulResult;
	double ldVolutage;
	uint8_t i;
	SystemInit();

	/* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
	TM_DISCO_LedInit();
	TM_DISCO_ButtonInit();
	/*Delay init*/
	TM_DELAY_Init();
	TM_USART_Init(USART1, TM_USART_PinsPack_2, 9600);

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
	Init_ADS1256_GPIO(); //初始化ADS1256 GPIO管脚
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	Delay(0xFF);
	ADS1256_Init();


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
					ulResult = ADS_sum((i << 4) | ADS1256_MUXN_AINCOM);
					//ulResult = ADS_sum( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);	
					if (ulResult & 0x800000)
					{
						ulResult = ~(unsigned long)ulResult;
						ulResult &= 0x7fffff;
						ulResult += 1;
						ulResult = -ulResult;
					}

					ldVolutage = (long double)ulResult*0.59604644775390625;

					printf("第%d通道:", (i & 0x07) ? (i & 0x07) - 1 : 7);
					printf("%f", ldVolutage); 	//double
					printf("uV\r\n");

					//printf("%x",(unsigned long)ulResult);//16
					Delay(0x3fFFF);
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
