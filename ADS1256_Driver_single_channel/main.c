/**
 ******************************************************************************
 * @file    USART/Printf/main.c 
 * @author  MCD Application Team
 * @version V3.3.0
 * @date    04/16/2010
 * @brief   Main program body
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "flash.h" 
#include "delay.h"
#include "ADS1256.h"
#include <stdio.h>

/** @addtogroup STM32F10x_StdPeriph_Examples
 * @{
 */

/** @addtogroup USART_Printf
 * @{
 */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
/* Values magic to the Board keys */
#define  NOKEY  0
#define  KEY1   1
#define  KEY2   2
#define  KEY3   3
#define  KEY4   4


unsigned char data[16];




/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private functions ---------------------------------------------------------*/

void Init_UART2()
{
    USART_InitTypeDef USART_InitStructure;
    /* USARTx configured as follow:
       - BaudRate = 115200 baud  
       - Word Length = 8 Bits
       - One Stop Bit
       - No parity
       - Hardware flow control disabled (RTS and CTS signals)
       - Receive and transmit enabled
       */	
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    STM_EVAL_COMInit(COM2, &USART_InitStructure);
}


/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
    unsigned char i=0;
    long ulResult;
    long double ldVolutage;

    Init_UART2();

    Init_ADS1256_GPIO(); //初始化ADS1256 GPIO管脚 
    GPIO_SetBits(GPIOC, GPIO_Pin_10 );  
    Delay(0xFF);

    ADS1256_Init();

    while(1)
    {	
        for(i = 0;i < 8;i++)
        {
            ulResult = ADS_sum( (i << 4) | ADS1256_MUXN_AINCOM);	
            //ulResult = ADS_sum( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);	
            if( ulResult & 0x800000 )
            {
                ulResult = ~(unsigned long)ulResult;
                ulResult &= 0x7fffff;
                ulResult += 1;
                ulResult = -ulResult;
            }

            ldVolutage = (long double)ulResult*0.59604644775390625;

            printf("第%d通道:",(i & 0x07)?(i & 0x07) - 1:7);
            printf("%lf",ldVolutage); 	//double
            printf("uV\r\n");

            //printf("%x",(unsigned long)ulResult);//16
            Delay(0x3fFFF);
        }
    }
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM2, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM2, USART_FLAG_TC) == RESET)
    {}

    return ch;
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
 * @}
 */ 

/**
 * @}
 */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
