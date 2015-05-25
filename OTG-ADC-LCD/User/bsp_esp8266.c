/*
*********************************************************************************************************
*
*	ģ������ : ESP8266 ����WIFIģ����������
*	�ļ����� : bsp_esp8266.c
*	��    �� : V1.0
*	˵    �� : ��װ ESP8266 ģ����ص�AT����
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2014-11-29 armfly  ��ʽ����
*
*	Copyright (C), 2014-2015, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/* ESP8266 ģ�����ͼ
	ESP8266ģ��    STM32-V5������


		UTXD   ---  PG14/USART6_TX
		GND    ---  GND
		CH_PD  ---  PI0  (��3.3V �� IO����ģ����磩
		GPIO2
		GPIO16 ---  PB7   (wifi Ӳ����λ)
		GPIO0
		VCC    ---  3.3  (����)
		URXD   ---  PC7/USART6_RX


	ģ��ȱʡ������ 9600;  ֧�ֵķ�Χ��110~460800bps
	�ڰ����ϵ��ʼ��boot rom��һ��log����Ҫ�� 74880 �Ĳ�������������ӡ�������Ǵ�ӡ����������.

	----------- PD = 1 ֮�� 74880bps ��ӡ�������� ------

	 ets Jan  8 2013,rst cause:1, boot mode:(3,6)

	load 0x40100000, len 25052, room 16
	tail 12
	chksum 0x0b
	ho 0 tail 12 room 4
	load 0x3ffe8000, len 3312, room 12
	tail 4
	chksum 0x53
	load 0x3ffe8cf0, len 6576, room 4
	tail 12
	chksum 0x0d
	csum 0x0d

	----------- ֮���� 9600bps ��ӡ ---------------

	[Vendor:www.ai-thinker.com Version:0.9.2.4]

	ready


	ʹ�ô��ڳ����ն����ʱ����Ҫ���� �ն� - ���� - ģʽ ҳ�湴ѡ������ģʽ��.


	���޸Ĳ����ʡ�
	AT+CIOBAUD=?     ---- ��ѯ�������
	+CIOBAUD:(9600-921600)

	OK

	AT+CIOBAUD=115200
	BAUD->115200

	��ѡ�� WIFI Ӧ��ģʽ ��
	AT+CWMODE=1
		1   Station ģʽ
		2   AP ģʽ
		3   AP �� Station ģʽ

	���г���ǰ���� AP��
	AT+CWLAP=<ssid>,< mac >,<ch>
	AT+CWLAP

	��AT+CWJAP���� AP��
	AT+CWJAP=<ssid>,< pwd >

*/

#define AT_CR		'\r'
#define AT_LF		'\n'

#define RCC_CH_PD 		RCC_AHB1Periph_GPIOI
#define PORT_CH_PD		GPIOI
#define PIN_CH_PD		GPIO_Pin_0

#define RCC_RESET 		RCC_AHB1Periph_GPIOB
#define PORT_RESET		GPIOB
#define PIN_RESET		GPIO_Pin_7

/* Ӳ������������� -- �� 3.3V ��ʼ����  */
#define ESP_CH_PD_0()	GPIO_ResetBits(PORT_CH_PD, PIN_CH_PD);
#define ESP_CH_PD_1()	GPIO_SetBits(PORT_CH_PD, PIN_CH_PD);

/* Ӳ����λ���� -- ���Բ��� */
#define ESP_RESET_1()	GPIO_ResetBits(PORT_RESET, PIN_RESET);
#define ESP_RESET_0()	GPIO_SetBits(PORT_RESET, PIN_RESET);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitESP8266
*	����˵��: ��������ģ����ص�GPIO,  �ú����� bsp_Init() ���á�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitESP8266(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_RESET | RCC_CH_PD, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Pin = PIN_CH_PD;
	GPIO_Init(PORT_CH_PD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_RESET;
	GPIO_Init(PORT_RESET, &GPIO_InitStructure);

	/* CPU�Ĵ��������Ѿ��� bsp_uart_fifo.c �е� bsp_InitUart() ���� */
	ESP_CH_PD_0();
	//ESP8266_Reset();

	bsp_InitUart6(115200, 0);	/* 1��ʾӲ������CRS RTS��Ч;  0��ʾ����Ӳ������ */
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_PrintRxData
*	����˵��: ��ӡSTM32��ESP8266�յ������ݵ�COM1���ڣ���Ҫ���ڸ��ٵ���
*	��    ��: _ch : �յ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ESP8266_PrintRxData(uint8_t _ch)
{
	#ifdef ESP8266_TO_COM1_EN
		comSendChar(COM1, _ch);		/* �����յ����ݴ�ӡ�����Դ���1 */
	#endif
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_PowerOn
*	����˵��: ��ESP8266ģ���ϵ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ESP8266_PowerOn(void)
{
	//comClearRxFifo(COM_ESP8266);	/* ���㴮�ڽ��ջ����� */

	ESP_CH_PD_0();

	bsp_InitUart6(74880, 0);	/* 1��ʾӲ������CRS RTS��Ч;  0��ʾ����Ӳ������ */

	ESP_CH_PD_1();

	ESP8266_Reset();


	/* �ȴ�ģ������ϵ磬�ж��Ƿ���յ� ready */
	ESP8266_WaitResponse("csum", 2000);

	bsp_InitUart6(115200, 0);	/* 1��ʾӲ������CRS RTS��Ч;  0��ʾ����Ӳ������ */

	/* �ȴ�ģ������ϵ磬�ж��Ƿ���յ� ready */
	ESP8266_WaitResponse("ready", 5000);


}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_PowerOff
*	����˵��: ����ESP8266ģ��ػ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ESP8266_PowerOff(void)
{
	ESP_CH_PD_0();
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_PowerOn
*	����˵��: ��ESP8266ģ���ϵ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ESP8266_Reset(void)
{
	ESP_RESET_0();
	bsp_DelayMS(20);
	ESP_RESET_1();

	bsp_DelayMS(10);
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_WaitResponse
*	����˵��: �ȴ�ESP8266����ָ����Ӧ���ַ���. ����ȴ� OK
*	��    ��: _pAckStr : Ӧ����ַ����� ���Ȳ��ó���255
*			 _usTimeOut : ����ִ�г�ʱ��0��ʾһֱ�ȴ�. >����ʾ��ʱʱ�䣬��λ1ms
*	�� �� ֵ: 1 ��ʾ�ɹ�  0 ��ʾʧ��
*********************************************************************************************************
*/
uint8_t ESP8266_WaitResponse(char *_pAckStr, uint16_t _usTimeOut)
{
	uint8_t ucData;
	uint8_t ucRxBuf[256];
	uint16_t pos = 0;
	uint32_t len;
	uint8_t ret;

	len = strlen(_pAckStr);
	if (len > 255)
	{
		return 0;
	}

	/* _usTimeOut == 0 ��ʾ���޵ȴ� */
	if (_usTimeOut > 0)
	{
		bsp_StartTimer(ESP8266_TMR_ID, _usTimeOut);		/* ʹ�������ʱ��3����Ϊ��ʱ���� */
	}
	while (1)
	{
		bsp_Idle();				/* CPU����ִ�еĲ����� �� bsp.c �� bsp.h �ļ� */

		if (_usTimeOut > 0)
		{
			if (bsp_CheckTimer(ESP8266_TMR_ID))
			{
				ret = 0;	/* ��ʱ */
				break;
			}
		}

		if (comGetChar(COM_ESP8266, &ucData))
		{
			ESP8266_PrintRxData(ucData);		/* �����յ����ݴ�ӡ�����Դ���1 */

			if (ucData == '\n')
			{
				if (pos > 0)	/* ��2���յ��س����� */
				{
					if (memcmp(ucRxBuf, _pAckStr,  len) == 0)
					{
						ret = 1;	/* �յ�ָ����Ӧ�����ݣ����سɹ� */
						break;
					}
					else
					{
						pos = 0;
					}
				}
				else
				{
					pos = 0;
				}
			}
			else
			{
				if (pos < sizeof(ucRxBuf))
				{
					/* ֻ����ɼ��ַ� */
					if (ucData >= ' ')
					{
						ucRxBuf[pos++] = ucData;
					}
				}
			}
		}
	}
	return ret;
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_ReadResponse
*	����˵��: ��ȡESP8266����Ӧ���ַ������ú��������ַ��䳬ʱ�жϽ����� ��������Ҫ����AT����ͺ�����
*	��    ��: _pBuf : ���ģ�鷵�ص������ַ���
*			  _usBufSize : ��������󳤶�
*			 _usTimeOut : ����ִ�г�ʱ��0��ʾһֱ�ȴ�. >0 ��ʾ��ʱʱ�䣬��λ1ms
*	�� �� ֵ: 0 ��ʾ���󣨳�ʱ��  > 0 ��ʾӦ������ݳ���
*********************************************************************************************************
*/
uint16_t ESP8266_ReadResponse(char *_pBuf, uint16_t _usBufSize, uint16_t _usTimeOut)
{
	uint8_t ucData;
	uint16_t pos = 0;
	uint8_t ret;
	uint8_t status = 0;		/* ����״̬ */

	/* _usTimeOut == 0 ��ʾ���޵ȴ� */
	if (_usTimeOut > 0)
	{
		bsp_StartTimer(ESP8266_TMR_ID, _usTimeOut);		/* ʹ�������ʱ����Ϊ��ʱ���� */
	}
	while (1)
	{
		bsp_Idle();				/* CPU����ִ�еĲ����� �� bsp.c �� bsp.h �ļ� */

		if (status == 2)		/* ���ڽ�����ЧӦ��׶Σ�ͨ���ַ��䳬ʱ�ж����ݽ������ */
		{
			if (bsp_CheckTimer(ESP8266_TMR_ID))
			{
				_pBuf[pos]	 = 0;	/* ��β��0�� ���ں���������ʶ���ַ������� */
				ret = pos;		/* �ɹ��� �������ݳ��� */
				break;
			}
		}
		else
		{
			if (_usTimeOut > 0)
			{
				if (bsp_CheckTimer(ESP8266_TMR_ID))
				{
					ret = 0;	/* ��ʱ */
					break;
				}
			}
		}

		if (comGetChar(COM_ESP8266, &ucData))
		{
			ESP8266_PrintRxData(ucData);		/* �����յ����ݴ�ӡ�����Դ���1 */

			switch (status)
			{
				case 0:			/* ���ַ� */
					if (ucData == AT_CR)		/* ������ַ��ǻس�����ʾ AT������� */
					{
						_pBuf[pos++] = ucData;		/* ������յ������� */
						status = 2;	 /* ��Ϊ�յ�ģ��Ӧ���� */
					}
					else	/* ���ַ��� A ��ʾ AT������� */
					{
						status = 1;	 /* �����������͵�AT�����ַ�����������Ӧ�����ݣ�ֱ������ CR�ַ� */
					}
					break;

				case 1:			/* AT������Խ׶�, ����������. �����ȴ� */
					if (ucData == AT_CR)
					{
						status = 2;
					}
					break;

				case 2:			/* ��ʼ����ģ��Ӧ���� */
					/* ֻҪ�յ�ģ���Ӧ���ַ���������ַ��䳬ʱ�жϽ�������ʱ�����ܳ�ʱ�������� */
					bsp_StartTimer(ESP8266_TMR_ID, 5);
					if (pos < _usBufSize - 1)
					{
						_pBuf[pos++] = ucData;		/* ������յ������� */
					}
					break;
			}
		}
	}
	return ret;
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_SendAT
*	����˵��: ��ģ�鷢��AT��� �������Զ���AT�ַ���������<CR>�ַ�
*	��    ��: _Str : AT�����ַ�����������ĩβ�Ļس�<CR>. ���ַ�0����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ESP8266_SendAT(char *_Cmd)
{
	comSendBuf(COM_ESP8266, (uint8_t *)_Cmd, strlen(_Cmd));
	comSendBuf(COM_ESP8266, "\r\n", 2);
}

/*
*********************************************************************************************************
*	�� �� ��: ESP8266_JoinAP
*	����˵��: ����AP
*	��    ��: _ssid : AP�����ַ���
*			  _pwd : �����ַ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ESP8266_JoinAP(char *_ssid, char *_pwd)
{
	char buf[128];

	sprintf(buf, "AT+CWJAP=\"%s\",\"%s\"", _ssid, _pwd);
	ESP8266_SendAT(buf);
}



/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
