
#include "ads1256.h"

u8 tabb[2];


#define RCC_SCK 	RCC_AHB1Periph_GPIOB
#define PORT_SCK	GPIOB
#define PIN_SCK		GPIO_Pin_10

#define RCC_DIN 	RCC_AHB1Periph_GPIOC
#define PORT_DIN	GPIOC
#define PIN_DIN		GPIO_Pin_2

#define RCC_CS 		RCC_AHB1Periph_GPIOB
#define PORT_CS		GPIOB
#define PIN_CS		GPIO_Pin_12

#define RCC_RESET 	RCC_AHB1Periph_GPIOC
#define PORT_RESET	GPIOC
#define PIN_RESET	GPIO_Pin_4

//#define RCC_PWDN 	RCC_AHB1Periph_GPIOA
//#define PORT_PWDN	GPIOA
//#define PIN_PWDN	GPIO_Pin_0

#define RCC_DRDY 	RCC_AHB1Periph_GPIOC
#define PORT_DRDY	GPIOC
#define PIN_DRDY	GPIO_Pin_9

#define RCC_DOUT 	RCC_AHB1Periph_GPIOC
#define PORT_DOUT	GPIOC
#define PIN_DOUT	GPIO_Pin_3



//PE6=CS PE7=RESET PE8=PWDN   //PE9=DRSY
#define ADS_CS_LOW()        GPIO_ResetBits(PORT_CS, PIN_CS)
#define ADS_CS_HIGH()       GPIO_SetBits(PORT_CS, PIN_CS)
//#define PWDN_LOW()	    GPIO_ResetBits(GPIOE, GPIO_Pin_8)
//#define PWDN_HIGH()	    GPIO_SetBits(GPIOE, GPIO_Pin_8)
#define SCK_LOW()		GPIO_ResetBits(PORT_SCK, PIN_SCK)
#define SCK_HIGH()		GPIO_SetBits(PORT_SCK,	PIN_SCK)
#define DI_LOW()		GPIO_ResetBits(PORT_DIN, PIN_DIN)
#define DI_HIGH()		GPIO_SetBits(PORT_DIN, PIN_DIN)
#define RESET_LOW()	    GPIO_ResetBits(PORT_RESET, PIN_RESET)
#define RESET_HIGH()	    GPIO_SetBits(PORT_RESET, PIN_RESET)
#define DO_IS_HIGH()	(GPIO_ReadInputDataBit(PORT_DOUT, PIN_DOUT) == Bit_SET)
#define DRDY_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DRDY, PIN_DRDY) == Bit_RESET)
//#define ADS_DRDY	    GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)


/******************ads1248�Ĵ�����ַ*******************/
// define commands 
#define ADS1256_CMD_WAKEUP   0x00     //���SYNC���˳�����ģʽ
#define ADS1256_CMD_RDATA    0x01     //������
#define ADS1256_CMD_RDATAC   0x03     //����������
#define ADS1256_CMD_SDATAC   0x0f     //ֹͣ���������� 
#define ADS1256_CMD_RREG     0x10     //�ӼĴ��������� 
#define ADS1256_CMD_WREG     0x50     //��Ĵ���д���� 
#define ADS1256_CMD_SELFCAL  0xf0     //ƫ�ƺ������Զ�У׼
#define ADS1256_CMD_SELFOCAL 0xf1     //ƫ���Զ�У׼ 
#define ADS1256_CMD_SELFGCAL 0xf2     //�����Զ�У׼ 
#define ADS1256_CMD_SYSOCAL  0xf3     //ϵͳʧ��У׼ 
#define ADS1256_CMD_SYSGCAL  0xf4     //ϵͳ����У׼ 
#define ADS1256_CMD_SYNC     0xfc     //ͬ��ADת�� 
#define ADS1256_CMD_STANDBY  0xfd     //����ģʽ��ʼ 
#define ADS1256_CMD_REST    0xfe      //��λ

// define the ADS1256 register values 
#define ADS1256_STATUS       0x00   
#define ADS1256_MUX          0x01   
#define ADS1256_ADCON        0x02   
#define ADS1256_DRATE        0x03   
#define ADS1256_IO           0x04   
#define ADS1256_OFC0         0x05   
#define ADS1256_OFC1         0x06   
#define ADS1256_OFC2         0x07   
#define ADS1256_FSC0         0x08   
#define ADS1256_FSC1         0x09   
#define ADS1256_FSC2         0x0A 


// define multiplexer codes 
#define ADS1256_MUXP_AIN0   0x00 
#define ADS1256_MUXP_AIN1   0x10 
#define ADS1256_MUXP_AIN2   0x20 
#define ADS1256_MUXP_AIN3   0x30 
#define ADS1256_MUXP_AIN4   0x40 
#define ADS1256_MUXP_AIN5   0x50 
#define ADS1256_MUXP_AIN6   0x60 
#define ADS1256_MUXP_AIN7   0x70 
#define ADS1256_MUXP_AINCOM 0x80 

#define ADS1256_MUXN_AIN0   0x00 
#define ADS1256_MUXN_AIN1   0x01 
#define ADS1256_MUXN_AIN2   0x02 
#define ADS1256_MUXN_AIN3   0x03 
#define ADS1256_MUXN_AIN4   0x04 
#define ADS1256_MUXN_AIN5   0x05 
#define ADS1256_MUXN_AIN6   0x06 
#define ADS1256_MUXN_AIN7   0x07 
#define ADS1256_MUXN_AINCOM 0x08   


//// define gain codes 
//#define ADS1256_GAIN_1      0x00 
//#define ADS1256_GAIN_2      0x01 
//#define ADS1256_GAIN_4      0x02 
//#define ADS1256_GAIN_8      0x03 
//#define ADS1256_GAIN_16     0x04 
//#define ADS1256_GAIN_32     0x05 
//#define ADS1256_GAIN_64     0x06 
////#define ADS1256_GAIN_64     0x07 

//define drate codes 
#define ADS1256_DRATE_30000SPS   0xF0 
#define ADS1256_DRATE_15000SPS   0xE0 
#define ADS1256_DRATE_7500SPS   0xD0 
#define ADS1256_DRATE_3750SPS   0xC0 
#define ADS1256_DRATE_2000SPS   0xB0 
#define ADS1256_DRATE_1000SPS   0xA1 
#define ADS1256_DRATE_500SPS    0x92 
#define ADS1256_DRATE_100SPS    0x82 
#define ADS1256_DRATE_60SPS     0x72 
#define ADS1256_DRATE_50SPS     0x63 
#define ADS1256_DRATE_30SPS     0x53 
#define ADS1256_DRATE_25SPS     0x43 
#define ADS1256_DRATE_15SPS     0x33 
#define ADS1256_DRATE_10SPS     0x23 
#define ADS1256_DRATE_5SPS      0x13 
#define ADS1256_DRATE_2_5SPS    0x03

ADS1256_VAR_T g_tADS1256;
static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/* ��λʱȱʡֵ */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};

static void ADS1256_WaitDRDY(void);
static void ADS1256_Send8Bit(uint8_t _data);
static uint8_t ADS1256_Recive8Bit(void);

void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	//ADS1256_StopScan();			/* ��ͣCPU�ж� */

	ADS1256_ResetHard();		/* Ӳ����λ */

	ADS1256_WaitDRDY();

	ADS1256_Send8Bit(ADS1256_CMD_SELFGCAL);
	Delay(100);

	{
		//uint8_t buf[4];		/* �ݴ�ADS1256 �Ĵ������ò�����֮������д4���Ĵ��� */

		/* ״̬�Ĵ�������
		Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

		Bit 3 ORDER: Data Output Bit Order
		0 = Most Significant Bit First (default)
		1 = Least Significant Bit First
		Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
		byte first. The ORDER bit only controls the bit order of the output data within the byte.

		Bit 2 ACAL : Auto-Calibration
		0 = Auto-Calibration Disabled (default)
		1 = Auto-Calibration Enabled
		When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
		the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
		values.

		Bit 1 BUFEN: Analog Input Buffer Enable
		0 = Buffer Disabled (default)
		1 = Buffer Enabled

		Bit 0 DRDY :  Data Ready (Read Only)
		This bit duplicates the state of the DRDY pin.

		ACAL=1ʹ����У׼���ܡ��� PGA��BUFEEN, DRATE�ı�ʱ��������У׼
		*/
		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		//buf[1] = 0x08;	/* ����λ0��ʾAINP�� AIN0,  ����λ8��ʾ AINN �̶��� AINCOM */

		/*	ADCON: A/D Control Register (Address 02h)
		Bit 7 Reserved, always 0 (Read Only)
		Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
		00 = Clock Out OFF
		01 = Clock Out Frequency = fCLKIN (default)
		10 = Clock Out Frequency = fCLKIN/2
		11 = Clock Out Frequency = fCLKIN/4
		When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

		Bits 4-2 SDCS1, SCDS0: Sensor Detect Current Sources
		00 = Sensor Detect OFF (default)
		01 = Sensor Detect Current = 0.5 �� A
		10 = Sensor Detect Current = 2 �� A
		11 = Sensor Detect Current = 10�� A
		The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
		ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

		Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
		000 = 1 (default)
		001 = 2
		010 = 4
		011 = 8
		100 = 16
		101 = 32
		110 = 64
		111 = 64
		*/
		//buf[2] = (0 << 5) | (0 << 2) | (_gain << 1);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/* ѡ��1;1����, ��������5V */

		/* ��Ϊ�л�ͨ���Ͷ����ݺ�ʱ 123uS, ���ɨ���ж�ģʽ����ʱ��������� = DRATE_1000SPS */
		//buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* ѡ������������� */

		//ADS_CS_LOW();	/* SPIƬѡ = 0 */
		//ADS1256_Send8Bit(ADS1256_CMD_WREG | 0);	/* д�Ĵ���������, �����ͼĴ�����ַ */
		//ADS1256_Send8Bit(0x03);			/* �Ĵ������� - 1, �˴�3��ʾд4���Ĵ��� */

		//ADS1256_Send8Bit(buf[0]);	/* ����״̬�Ĵ��� */
		//ADS1256_Send8Bit(buf[1]);	/* ��������ͨ������ */
		//ADS1256_Send8Bit(buf[2]);	/* ����ADCON���ƼĴ��������� */
		//ADS1256_Send8Bit(buf[3]);	/* ��������������� */

		//ADS_CS_HIGH();	/* SPIƬѡ = 1 */

		ADS1256WREG(ADS1256_STATUS, 0x04);   	//��ʼ��STATUS reg  IDλΪf,���������λ����,�Զ�У׼��,���뻺��ر�,DRDY�ߵ�ƽ
		// 	ADS1256_WriteReg(ADS1256_STATUS,0x06);  			 		//buff on	,ģ�������ѹ����<AVDD-2V,�����������ZHP 20131104
		ADS1256WREG(ADS1256_MUX, ADS1256_MUXP_AIN0 + ADS1256_MUXN_AINCOM); //��ʼ��MUX    reg  ��������AIN0-AINCOM		ADS1256WREG(ADS1256_ADCON, ADS1256_GAIN_1);
		ADS1256WREG(ADS1256_DRATE, ADS1256_DRATE_30000SPS);
		ADS1256_Send8Bit(ADS1256_CMD_SELFGCAL);
		Delay(100);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Recive8Bit
*	����˵��: ��SPI���߽���8��bit���ݡ� ����CS���ơ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t i;
	uint8_t read = 0;

	Delay(10);
	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
	for (i = 0; i < 8; i++)
	{
		SCK_HIGH();
		Delay(10);
		read = read << 1;
		SCK_LOW();
		if (DO_IS_HIGH())
		{
			read++;
		}
		Delay(10);
	}
	return read;
}

static void ADS1256_Send8Bit(uint8_t _data)
{
	uint8_t i;

	/* �������Ͷ���ֽ�ʱ����Ҫ�ӳ�һ�� */
	Delay(20);

	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
	for (i = 0; i < 8; i++)
	{
		if (_data & 0x80)
		{
			DI_HIGH();
		}
		else
		{
			DI_LOW();
		}
		SCK_HIGH();
		Delay(5);
		_data <<= 1;
		SCK_LOW();			/* <----  ADS1256 ����SCK�½��ز���DIN����, ���ݱ���ά�� 50nS */
		Delay(5);
	}
}
static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 40000000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 40000000)
	{
		//printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* �������. �����Ŵ� */
	}
}

void SPI_Init2(void) //PE5��ʼ
{
	//u8 read_tab[5];
	//SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RESET_HIGH();
	ADS_CS_HIGH();
	SCK_HIGH();
	DI_HIGH();

	RCC_AHB1PeriphClockCmd(RCC_SCK | RCC_DIN | RCC_DOUT | RCC_CS | RCC_DRDY | RCC_RESET, ENABLE);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //PE3=1
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_Init(GPIOE, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOE, GPIO_Pin_3);

	/*GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);*/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  //PC9=DRSY
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;

	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DOUT;
	GPIO_Init(PORT_DOUT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Pin = PIN_DIN;
	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_CS;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	GPIO_Init(PORT_SCK, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_RESET;
	GPIO_Init(PORT_RESET, &GPIO_InitStructure);
	//SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	//SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	//SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	//SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	//SPI_InitStructure.SPI_CRCPolynomial = 7;
	//SPI_Cmd(SPI2, ENABLE);
	//SPI_Init(SPI2, &SPI_InitStructure);
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE); //ʹ��	SPI���ͻ�����ж�

	//ADS_CS_HIGH();

	//      ADS1256_ResetHard();		/* Ӳ����λ */                        
	//      ADS1256_WaitDRDY();
	//      read_tab[0] = ADS1256_ReadReg(0);   
	//      ADS1256_WaitDRDY();        
	//      read_tab[1] = ADS1256_ReadReg(1);  
	//      ADS1256_WaitDRDY();        
	//      read_tab[2] = ADS1256_ReadReg(2); 
	//      ADS1256_WaitDRDY();        
	//      read_tab[3] = ADS1256_ReadReg(3); 
	//      ADS1256_WaitDRDY();        
	//      read_tab[4] = ADS1256_ReadReg(4);   
}

//u8 results1, results2, results3;
void ADS1256_ResetHard(void)
{
	RESET_LOW();
	Delay(100);
	RESET_HIGH();
	Delay(100);
}
/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
//u8 SPI_SendByte(u8 byte)
//{
//	/* Loop while DR register in not emplty */
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//	/* Send byte through the SPI2 peripheral */
//	SPI_I2S_SendData(SPI2, byte);
//	/* Wait to receive a byte */
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
//	/* Return the byte read from the SPI bus */
//	return SPI_I2S_ReceiveData(SPI2);
//}
//-----------------------------------------------------------------//
//	��    �ܣ�ADS1256 д����
//	��ڲ���: /
//	���ڲ���: /
//	ȫ�ֱ���: /
//	��    ע: ��ADS1256�е�ַΪregaddr�ļĴ���д��һ���ֽ�databyte
//-----------------------------------------------------------------//
void ADS1256WREG(unsigned char regaddr, unsigned char databyte)
{
	ADS_CS_LOW();
	//while (ADS_DRDY);//��ADS1256_DRDYΪ��ʱ����д�Ĵ���
	ADS1256_Send8Bit(ADS1256_CMD_WREG | regaddr );//��Ĵ���д�����ݵ�ַ
	ADS1256_Send8Bit(0x00);//д�����ݵĸ���n-1
	ADS1256_Send8Bit(databyte);//��regaddr��ַָ��ļĴ���д������databyte
	ADS_CS_HIGH();
}
//-----------------------------------------------------------------//
//	��    �ܣ�ADS1256 ���Ĵ�������
//	��ڲ���: /
//	���ڲ���: /
//	ȫ�ֱ���: /
//	��    ע: ��ADS1256�е�ַΪregaddr�ļĴ�������һ���ֽ�databyte
//-----------------------------------------------------------------//
unsigned char ADS1256RREG(unsigned char regaddr)
{
	//��ADS1256�е�ַΪregaddr�ļĴ�������һ���ֽ�
	unsigned char r = 0;
	ADS_CS_LOW();
	//while (ADS_DRDY);//��ADS1256_DRDYΪ��ʱ����д�Ĵ���
	//SPI_SendByte(ADS1256_CMD_RREG | (regaddr & 0xF));
	ADS1256_Send8Bit(ADS1256_CMD_RREG | regaddr);
	ADS1256_Send8Bit(0x00);//д���ȡ���ݵĸ���n-1
	Delay(10); //min=50*(1/fCLKIN)=50*(1/7.68MHZ)=6500ns;max=whatever                   
	//r = SPI_SendByte(0); //����regaddr��ַָ��ļĴ���������
	r = ADS1256_Recive8Bit();
	ADS_CS_HIGH();
	return r;//��������
}
//-----------------------------------------------------------------//
//	��    �ܣ�ADS1256��ʼ���ӳ���
//	��ڲ���: /
//	���ڲ���: /
//	ȫ�ֱ���: /
//	��    ע: /
//-----------------------------------------------------------------//
//void ADS1256_Init(void)
//{
//	//unsigned char tab1[5];
//	SPI_Init2();
//	ADS1256_ResetHard();
//	
//	/*tab1[0] = ADS1256RREG(0);
//	tab1[1] = ADS1256RREG(1);
//	tab1[2] = ADS1256RREG(2);
//	tab1[3] = ADS1256RREG(3);*/
//
//	//ADS1256WREG(0x00, 0x31 );      
//	//ADS1256WREG(0x01, 0x23 );         
//	//ADS1256WREG(0x02, 0x20 );         
//	//ADS1256WREG(0x03, 0xf0 ); 
//
//
//
//
//
//
//	
//	//while (ADS_DRDY);
//	//SPI_SendByte(ADS1256_CMD_WREG | ADS1256_STATUS);//����д��4���Ĵ���
//	//SPI_SendByte(3);
//	//SPI_SendByte(0x31);
//	//SPI_SendByte(0x23);
//	//SPI_SendByte(0x20);
//	//SPI_SendByte(0x03);  
//	//ADS_CS_LOW();
//	ADS1256WREG(ADS1256_STATUS, 0x00);               // ��λ��ǰ��У׼��ʹ�û���
//	Delay(10);
//	//	ADS1256WREG(ADS1256_MUX,0x08);                  // ��ʼ���˿�A0Ϊ��+����AINCOMλ��-��
//	ADS1256WREG(ADS1256_ADCON, 0x00);                // �Ŵ���1
//	Delay(10);
//	ADS1256WREG(ADS1256_DRATE, ADS1256_DRATE_30000SPS);  // ����5sps
//	Delay(10);
//	ADS1256WREG(ADS1256_IO, 0x00);
//	Delay(10);
//	SPI_I2S_SendData(SPI2, ADS1256_CMD_SELFCAL);
//	while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9));
//	SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_TXE);	 //������ͻ���Ϊ���жϱ�ʶ
//	//ADS_CS_HIGH();
//
//
//	Delay(50);
//
//	/*Delay(100);
//	tab1[1] = ADS1256RREG(1);
//	tab1[2] = ADS1256RREG(2);
//	tab1[3] = ADS1256RREG(3);
//	tab1[4] = ADS1256RREG(4);
//	tab1[5] = ADS1256RREG(5);*/
//
//}

int32_t ADS1256_ReadData(void)
{
	//unsigned char i = 0;
	uint32_t sum = 0;
	//uint32_t r = 0;

	
	//while (DRDY_IS_LOW());	/* �ȴ� DRDY �� */
	//while (!DRDY_IS_LOW());	/* �ȴ� DRDY �� */

	//ADS1256_Send8Bit(ADS1256_CMD_SYNC);
	//Delay(5);
	//ADS1256_Send8Bit(ADS1256_CMD_WAKEUP);
	//Delay(5);
	//ADS_CS_LOW();
	ADS1256_Send8Bit(ADS1256_CMD_RDATA);
	Delay(7);               //min=50*(1/fCLKIN)=50*(1/7.68MHZ)=6500ns;max=whatever
	/*for (i = 0; i < 3; i++)
	{
	sum = sum << 8;
	r = ADS1256_Send8Bit(0);
	sum |= r;
	}*/
	//while (DRDY_IS_LOW());	/* �ȴ� DRDY �� */
	//while (!DRDY_IS_LOW());	/* �ȴ� DRDY �� */
	//ADS1256_WaitDRDY();
	sum = ADS1256_Recive8Bit() << 16;
	sum += ADS1256_Recive8Bit() << 8;
	sum += ADS1256_Recive8Bit() << 0;
	//ADS_CS_HIGH();
	/* ����������չ��24λ�з�������չΪ32λ�з����� */
	if (sum & 0x800000)
	{
		sum += 0xFF000000;
		//sum = sum * 2;
	}
	return (int32_t)sum;
}
//-----------------------------------------------------------------//
//	��    �ܣ���ȡADS1256��·����
//	��ڲ���: /
//	���ڲ���: /
//	ȫ�ֱ���: /
//	��    ע: /
//-----------------------------------------------------------------//
//void ADS_sum(void)
//{
//	unsigned long results = 0;
//	//ADS1256WREG(ADS1256_MUX,road);		//����ͨ��
//	results = ADS1256ReadData();        //��ȡADֵ������24λ���ݡ�
//	results1 = (results >> 16) & 0x0000ff; //�������λ	
//	results2 = (results >> 8) & 0x0000ff;  //�����м�λ
//	results3 = results & 0x0000ff;		   //���͵�λ
//}


/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetChannal
*	����˵��: ����ͨ���š���·���á�AIN- �̶��ӵأ�ACOM).
*	��    ��: _ch : ͨ���ţ� 0-7
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_SetChannal(uint8_t _ch)
{
	if (_ch > 7)
	{
		return;
	}
	ADS1256WREG(ADS1256_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN �̶��� AINCOM */
	ADS1256_WaitDRDY();
	ADS1256_Send8Bit(ADS1256_CMD_SYNC);
	ADS1256_Send8Bit(ADS1256_CMD_WAKEUP);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetDiffChannal
*	����˵��: ���ò��ͨ���š���·���á�
*	��    ��: _ch : ͨ����,0-3����4��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
*********************************************************************************************************
*/
//void ADS1256_SetDiffChannal(uint8_t _ch)
//{
//
//	if (_ch == 0)
//	{
//		ADS1256WREG(ADS1256_MUX, (0 << 4) | 1);	//������� AIN0�� AIN1 
//	}
//	else if (_ch == 1)
//	{
//
//		ADS1256WREG(ADS1256_MUX, (2 << 4) | 3);	//������� AIN2�� AIN3 
//	}
//	else if (_ch == 2)
//	{
//		ADS1256WREG(ADS1256_MUX, (4 << 4) | 5);	//������� AIN4�� AIN5 
//	}
//	else if (_ch == 3)
//	{
//		ADS1256WREG(ADS1256_MUX, (6 << 4) | 7);	//������� AIN6�� AIN7
//	}
//
//
//	tabb[0] = ADS1256RREG(ADS1256_MUX);
//	tabb[0] = ADS1256RREG(ADS1256_MUX);
//}


//void ADS1256_SetDiffChannal2(uint8_t _ch)
//{
//	ADS_CS_LOW();
//	while (ADS_DRDY);
//	SPI_SendByte(ADS1256_CMD_WREG | 0x01);
//	SPI_SendByte(0);
//	Delay(100);
//	if (_ch == 0)
//	{
//		SPI_SendByte(0x01);
//	}
//	else if (_ch == 1)
//	{
//
//		SPI_SendByte(0x23);
//	}
//	else if (_ch == 2)
//	{
//		SPI_SendByte(0x45);
//	}
//	else if (_ch == 3)
//	{
//		SPI_SendByte(0x67);
//	}
//
//	//      ADS_CS_HIGH(); 
//
//
//}
