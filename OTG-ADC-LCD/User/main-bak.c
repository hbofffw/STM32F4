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
#include "button_back.h"
#include "tm_stm32f4_ili9341.h"
//#include "tm_stm32f4_stmpe811.h"
//#include "tm_stm32f4_ili9341_button.h"
#include "tm_stm32f4_usart.h"
//#include "tm_stm32f4_stdio.h"
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
	int32_t	adc[8];
	//	float i = 10.10;
	//LCD and TOUCHPAD seeting///////////////////
	/* TM_STMPE811_TouchData instance */
	//TM_STMPE811_TouchData touchData;
	/* TM_ILI9341_Button_t instance */
	//TM_ILI9341_Button_t button;
	//int8_t buttonPressed, button1, button2;
	//char str[30];
	/////////////////////////////////////////	
	SystemInit();
	//USART initialization
	TM_USART_Init(USART1, TM_USART_PinsPack_2, 9600);
	
	/* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
	TM_DISCO_LedInit();
	TM_DISCO_ButtonInit();
	/* Initialize USB VCP */
	TM_USB_VCP_Init();
	/* Initialize ADC1 on channel 4, this is pin PA4 */
	TM_ADC_Init(ADC1, ADC_Channel_4);
	/* Initialize ADC1 on channel 3, this is pin PA3 */
	TM_ADC_Init(ADC1, ADC_Channel_3);
	/*Delay init*/
	TM_DELAY_Init();
	Delayms(100); /* 等待上电稳定，等基准电压电路稳定, bsp_InitADS1256() 内部会进行自校准 */
	bsp_InitADS1256(); /* 初始化配置ADS1256.  PGA=1, DRATE=30KSPS, BUFEN=1, 输入正负5V */
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
	//LCD and TOUCHPAD seeting///////////////////	
	/* Initialize LCD */ 
	//TM_ILI9341_Init();
	/* Initialize Touch */
	//TM_STMPE811_Init();
	/* Fill LCD with gray color */
	//TM_ILI9341_Fill(ILI9341_COLOR_GRAY);
	/* Select orientation */
	//TM_ILI9341_Rotate(TM_ILI9341_Orientation_Portrait_2);

	///* Select touch screen orientation */
	//touchData.orientation = TM_STMPE811_Orientation_Portrait_2;

	//
	///* Button 1, default configuration */
	///* Red with black border and black font 11x18 */
	//button.x = 10;	/* X location */
	//button.y = 50;	/* Y location */
	//button.width = 219;
	//button.height = 50;
	//button.background = ILI9341_COLOR_RED;
	//button.borderColor = ILI9341_COLOR_BLACK;
	//button.label = "LCD";
	//button.color = ILI9341_COLOR_BLACK;
	//button.font = &TM_Font_11x18;
	///* Add button */
	//button1 = TM_ILI9341_Button_Add(&button);

	///* Button with custom background and without label */
	//button.x = 10;
	//button.y = 110;
	//button.width = 219;
	//button.height = 50;
	//button.background = ILI9341_COLOR_RED;
	//button.borderColor = ILI9341_COLOR_BLACK;
	//button.label = "UART";
	///* Use background image and no label */
	////button.flags = TM_BUTTON_FLAG_NOLABEL | TM_BUTTON_FLAG_IMAGE;
	//button.color = ILI9341_COLOR_BLACK;
	//button.font = &TM_Font_11x18;
	//button.image = buttonBackground; /* Variable stored in  */
	///* Add button */
	//button2 = TM_ILI9341_Button_Add(&button);
	///* Draw buttons */
	//TM_ILI9341_Button_DrawAll();

	///* Draw some strings */
	////TM_ILI9341_Puts(45, 245, "prepared         reserved", &TM_Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);

	/////////////////////////////////////////////end of lcd and touchpad setting //////////////////////////
	//TM_ILI9341_Puts(10, 20, "ADC Use channel 4/3, PA4/PA3", &TM_Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
	//TM_ILI9341_DisplayOff();
	while (1) {
		/* USB configured OK, drivers OK */
		if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
			/*USB OK, drivers OK*/
			TM_DISCO_LedOn(LED_GREEN);
		}
		else {
			/* USB not OK */
			TM_DISCO_LedOff(LED_GREEN);
		}
		//button
		if (TM_DISCO_ButtonPressed()) {
			
			/* Turn on leds */
			Delayms(500);
			//sprintf(str, "ADC 4/3:%4d/%4d\n", TM_ADC_Read(ADC1, ADC_Channel_4), TM_ADC_Read(ADC1, ADC_Channel_3));
			//TM_USART_Puts(USART1, "Hello world\n");
			TM_USART_Puts(USART1, "AT\r\n");
			//TM_ILI9341_Puts(10, 190, str, &TM_Font_11x18, ILI9341_COLOR_GREEN, ILI9341_COLOR_GRAY);
			//Delayms(100);
			
		}
		/* Get character from internal buffer */
		c = TM_USART_Getc(USART1);
		if (c) {
			/* If anything received, put it back to terminal */
			//TM_USART_Putc(USART1, c);
			printf("%c", c); 
		}
		if (TM_USB_VCP_Getc(&s) == TM_USB_VCP_DATA_OK)
		{
			//printf("data\r\n");
			TM_USART_Putc(USART1, s);
		}
		//LCD
		//if (TM_STMPE811_ReadTouch(&touchData) == TM_STMPE811_State_Pressed) {
		//	buttonPressed = TM_ILI9341_Button_Touch(&touchData);
		//	if (buttonPressed == button1) {

		//		sprintf(str, "ADC 4/3:%4d/%4d\n", TM_ADC_Read(ADC1, ADC_Channel_4), TM_ADC_Read(ADC1, ADC_Channel_3));

		//		TM_ILI9341_Puts(10, 220, "LCD pressed              \n", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
		//		Delayms(100);
		//	}
		//	else if (buttonPressed == button2) {

		//		if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
		//			/*USB OK, drivers OK*/
		//			TM_DISCO_LedOn(LED_GREEN);
		//			TM_ILI9341_Puts(10, 220, "UART pressed             \n", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
		//			printf("ADC channel 0/3: %4d/%4d \n", TM_ADC_Read(ADC1, ADC_Channel_0), TM_ADC_Read(ADC1, ADC_Channel_3));
		//		}
		//		else {
		//			/* USB not OK */
		//			TM_DISCO_LedOff(LED_GREEN);
		//			TM_ILI9341_Puts(10, 220, "USB NOT CONNECTED         \n", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
		//			sprintf(str, "ADC 0/3:%4d/%4d\n", TM_ADC_Read(ADC1, ADC_Channel_0), TM_ADC_Read(ADC1, ADC_Channel_3));
		//		}


		//		Delayms(100);
		//	}
		//	TM_ILI9341_Puts(10, 190, str, &TM_Font_11x18, ILI9341_COLOR_GREEN, ILI9341_COLOR_GRAY);
		//}

	}
}
