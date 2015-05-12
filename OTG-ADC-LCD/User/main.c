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
#include "tm_stm32f4_stmpe811.h"
#include "tm_stm32f4_ili9341_button.h"
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
//	float i = 10.10;
//LCD and TOUCHPAD seeting///////////////////
	/* TM_STMPE811_TouchData instance */
	TM_STMPE811_TouchData touchData;
	/* TM_ILI9341_Button_t instance */
	TM_ILI9341_Button_t button;
	int8_t buttonPressed, button1, button2;
	char str[30];
/////////////////////////////////////////	
    SystemInit();
    
    /* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
    TM_DISCO_LedInit();
	
	
//LCD and TOUCHPAD seeting///////////////////	
	/* Initialize LCD */
	TM_ILI9341_Init();
	/* Fill LCD with gray color */
	TM_ILI9341_Fill(ILI9341_COLOR_GRAY);
	/* Select orientation */
	TM_ILI9341_Rotate(TM_ILI9341_Orientation_Portrait_2);
	
	/* Select touch screen orientation */
	touchData.orientation = TM_STMPE811_Orientation_Portrait_2;
	
	/* Initialize Touch */
	TM_STMPE811_Init();
  /* Button 1, default configuration */
	/* Red with black border and black font 11x18 */
	button.x = 10;	/* X location */
	button.y = 30;	/* Y location */
	button.width = 219;
	button.height = 50;
	button.background = ILI9341_COLOR_RED;
	button.borderColor = ILI9341_COLOR_BLACK;
	button.label = "Button 1";
	button.color = ILI9341_COLOR_BLACK;
	button.font = &TM_Font_11x18;
	/* Add button */
	button1 = TM_ILI9341_Button_Add(&button);
	
	/* Button with custom background and without label */
	button.x = 10;
	button.y = 90;
	button.width = 219;
	button.height = 50;
	button.background = ILI9341_COLOR_RED;
	button.borderColor = ILI9341_COLOR_BLACK;
	button.label = "Button 2";
	/* Use background image and no label */
	//button.flags = TM_BUTTON_FLAG_NOLABEL | TM_BUTTON_FLAG_IMAGE;
	button.color = ILI9341_COLOR_BLACK;
	button.font = &TM_Font_11x18;
	button.image = buttonBackground; /* Variable stored in  */
	/* Add button */
	button2 = TM_ILI9341_Button_Add(&button);
	
	
	if (!TM_DISCO_LedIsOn(LED_RED)) {
		/* If led res is turned off, disable buttons 2 and 3 */
		TM_ILI9341_Button_Disable(button2);
		TM_ILI9341_Puts(25, 220, "Buttons disabled!", &TM_Font_11x18, ILI9341_COLOR_RED, ILI9341_COLOR_GRAY);
	}
	/* Draw buttons */
	TM_ILI9341_Button_DrawAll();
	
	/* Draw some strings */
	TM_ILI9341_Puts(45, 245, "prepared         reserved", &TM_Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
	
///////////////////////////////////////////end of lcd and touchpad setting //////////////////////////
	
    /* Initialize USB VCP */    
    TM_USB_VCP_Init();
    /* Initialize ADC1 on channel 0, this is pin PA0 */
	TM_ADC_Init(ADC1, ADC_Channel_0);
	
	/* Initialize ADC1 on channel 3, this is pin PA3 */
	TM_ADC_Init(ADC1, ADC_Channel_3);
	
    while (1) {
			if (TM_STMPE811_ReadTouch(&touchData) == TM_STMPE811_State_Pressed) {
			buttonPressed = TM_ILI9341_Button_Touch(&touchData);
			if (buttonPressed >= 0) {
				/* Any button pressed */
				sprintf(str, "Pressed: Button %d", (buttonPressed + 1));
			} else {
				sprintf(str, "Press the button ");
			}
			if (buttonPressed == button1) {
				/* Red button 1 is pressed, toggle led */
				TM_DISCO_LedToggle(LED_RED);
				
				if (TM_DISCO_LedIsOn(LED_RED)) {
					/* If led is turned on, enable button 2 and button 3 */
					//TM_ILI9341_Button_Enable(button2);
					TM_ILI9341_Puts(25, 220, "Buttons enabled! ", &TM_Font_11x18, ILI9341_COLOR_GREEN, ILI9341_COLOR_GRAY);
				} else {
					/* otherwise disable both */
					//TM_ILI9341_Button_Disable(button2);
					TM_ILI9341_Puts(25, 220, "Buttons disabled!", &TM_Font_11x18, ILI9341_COLOR_RED, ILI9341_COLOR_GRAY);
				}
			} else if (buttonPressed == button2) {
				/* If button 2 is pressed, turn green led on */
				//TM_DISCO_LedOn(LED_GREEN);
				TM_ILI9341_Puts(10, 100, "button 2 pressed", &TM_Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
			}
		}
		TM_ILI9341_Puts(10, 5, str, &TM_Font_11x18, ILI9341_COLOR_GREEN, ILI9341_COLOR_GRAY);
        /* USB configured OK, drivers OK */
        if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
						/*USB OK, drivers OK*/
            TM_DISCO_LedOn(LED_GREEN);
            /* If something arrived at VCP */
            if (TM_USB_VCP_Getc(&c) == TM_USB_VCP_DATA_OK) {
                /* Return data back */
                //TM_USB_VCP_Putc(c);
							//printf("%c%f",c,i);
							printf("%4d: %4d\n\r", TM_ADC_Read(ADC1, ADC_Channel_0), TM_ADC_Read(ADC1, ADC_Channel_3));
            }
        } else {
            /* USB not OK */
					//TM_ILI9341_Puts(10, 100, "Bottom buttons work\nonly if red led is turned on.\nYou can toggle red\nled with Button 1.\nUSB cofigured not OK", &TM_Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_GRAY);
            TM_DISCO_LedOff(LED_GREEN);
        }
		//Delayms(1000);
    }
}
