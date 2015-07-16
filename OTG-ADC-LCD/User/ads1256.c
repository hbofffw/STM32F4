/*
*********************************************************************************************************
*
*	ģ������ : ADS1256 ����ģ��(8ͨ����PGA��24λADC)
*	�ļ����� : bsp_ads1256.c
*	��    �� : V1.0
*	˵    �� : ADS1256ģ���CPU֮�����SPI�ӿڡ�����������֧��Ӳ��SPI�ӿں����SPI�ӿڡ�
*			  ͨ�����л���
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2014-01-01  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/


#include "ads1256.h"
//#include "tm_stm32f4_usb_vcp.h"
#include "tm_stm32f4_spi.h"
#include "tm_stm32f4_general.h"


//#define SOFT_SPI		/* ������б�ʾʹ��GPIOģ��SPI�ӿ� */
//#define HARD_SPI		/* ������б�ʾʹ��CPU��Ӳ��SPI�ӿ� */

/*
ADS1256ģ�����ֱ�Ӳ嵽STM32-V5������CN26��ĸ(2*6P 2.54mm)�ӿ���.

//FOR F4DISCOVERY
+5V   <------  5.0V      5V����
GND   -------  GND       ��
DRDY  ------>  PE9       ׼������
CS    <------  PB0       SPI_CS
DIN   <------  PA7       SPI_MOSI
DOUT  ------>  PA6       SPI_MISO
SCLK  <------  PA5       SPIʱ��
GND   -------  GND       ��
//-PDWN  <------  PB0       �������
RST   <------  PC5       ��λ�ź�
NC   �ս�
NC   �ս�


//FOR F401
+5V <------  5.0V      5V����
GND	------ - GND       ��
DRDY------>  PC9       ׼������
CS    <------PB0       SPI_CS
DIN   <------PC2       SPI_MISO
DOUT------>  PC3       SPI_MOSI
SCLK  <------PB10      SPIʱ��
GND  ------- GND       ��
//-PDWN  <------  PB2       �������
RST   <------PC4       ��λ�ź�
*/

/*
ADS1256��������:
1��ģ�ⲿ�ֹ���5V;
2��SPI���ֽӿڵ�ƽ��3.3V
3��PGA���÷�Χ�� 1��2��4��8��16��32��64��
4���ο���ѹ2.5V (�Ƽ�ȱʡ�ģ����õģ�
5�������ѹ��Χ��PGA = 1 ʱ, ����������5V
6. �Զ�У׼ ����������PGA,BUFʹ�ܡ����ݲ�����ʱ����������У׼)
7. ����Ļ��������������ú͹رգ�һ��ѡ���ã�


�ⲿ����Ƶ�� = 7.68MHz,
ʱ��Ƶ�� tCLK = 1/7.68M = 0.13uS
����������� tDATA =  1 / 30K = 0.033mS  (��30Ksps����)

��SPI��ʱ���ٶ�Ҫ��: (ads1256.pdf page 6)
��� 4��tCLK = 0.52uS
���� 10��tDATA = 0.3mS (�� 30Ksps ����)

SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns

RREG, WREG, RDATA ����֮����Ҫ�ӳ� 4 * tCLK = 0.52uS;
RDATAC, RESET, SYNC ����֮����Ҫ�ӳ� 24 * tCLK = 3.12uS;

ʵ�ʲ��ԣ���3.3V�ϵ��, ��ʱ�����κ����ã�ADS125��DRDY ���߼���ʼ��������źţ�2.6us��,33.4�ͣ�Ƶ��30KHz��
*/

/*
���Լ�¼
(1) ���üĴ���ʱ��SCK���쵼��оƬ����ÿ�ζ��յ����ݡ�ԭ��: ���͵����ڵ��ֽ�֮����Ҫ�ӳ�һС��ʱ��.
(2) ������λCPUʱ��ż������оƬ����������쳣��
*/

//#if !defined(SOFT_SPI) && !defined(HARD_SPI)
// 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
//#endif
//
//#ifdef SOFT_SPI		/* ���SPI */
/* ����GPIO�˿� */
#if defined (TM_DISCO_NUCLEO_F401)
#define RCC_SCK 	RCC_AHB1Periph_GPIOB
#define PORT_SCK	GPIOB	
#define PIN_SCK		GPIO_Pin_10

#define RCC_DIN 	RCC_AHB1Periph_GPIOC		
#define PORT_DIN	GPIOC	
#define PIN_DIN		GPIO_Pin_2	

#define RCC_CS 		RCC_AHB1Periph_GPIOB
#define PORT_CS		GPIOB	
#define PIN_CS		GPIO_Pin_0

#define RCC_RESET 	RCC_AHB1Periph_GPIOC
#define PORT_RESET	GPIOC
#define PIN_RESET	GPIO_Pin_4

#define RCC_PWDN 	RCC_AHB1Periph_GPIOB
#define PORT_PWDN	GPIOB
#define PIN_PWDN	GPIO_Pin_2

#define RCC_DRDY 	RCC_AHB1Periph_GPIOC
#define PORT_DRDY	GPIOC
#define PIN_DRDY	GPIO_Pin_9

#define RCC_DOUT 	RCC_AHB1Periph_GPIOC		
#define PORT_DOUT	GPIOC	
#define PIN_DOUT	GPIO_Pin_3

#endif

#if defined (TM_DISCO_STM32F4_DISCOVERY)
#define RCC_SCK 	RCC_AHB1Periph_GPIOA
#define PORT_SCK	GPIOA
#define PIN_SCK		GPIO_Pin_5

#define RCC_DIN 	RCC_AHB1Periph_GPIOA
#define PORT_DIN	GPIOA
#define PIN_DIN		GPIO_Pin_7

#define RCC_CS 		RCC_AHB1Periph_GPIOB
#define PORT_CS		GPIOB
#define PIN_CS		GPIO_Pin_0

#define RCC_RESET 	RCC_AHB1Periph_GPIOC
#define PORT_RESET	GPIOC
#define PIN_RESET	GPIO_Pin_5

#define RCC_PWDN 	RCC_AHB1Periph_GPIOB
#define PORT_PWDN	GPIOB 
#define PIN_PWDN	GPIO_Pin_2

#define RCC_DRDY 	RCC_AHB1Periph_GPIOE
#define PORT_DRDY	GPIOE
#define PIN_DRDY	GPIO_Pin_9

#define RCC_DOUT 	RCC_AHB1Periph_GPIOA
#define PORT_DOUT	GPIOA
#define PIN_DOUT	GPIO_Pin_6
#endif
/* ���������0����1�ĺ� */
#define PWDN_0()	GPIO_ResetBits(PORT_PWDN, PIN_PWDN)
#define PWDN_1()	GPIO_SetBits(PORT_PWDN, PIN_PWDN)

#define RESET_0()	GPIO_ResetBits(PORT_RESET, PIN_RESET)
#define RESET_1()	GPIO_SetBits(PORT_RESET, PIN_RESET)

#define CS_0()		GPIO_ResetBits(PORT_CS, PIN_CS)
#define CS_1()		GPIO_SetBits(PORT_CS, PIN_CS)

#define SCK_0()		GPIO_ResetBits(PORT_SCK, PIN_SCK)
#define SCK_1()		GPIO_SetBits(PORT_SCK, PIN_SCK)

#define DI_0()		GPIO_ResetBits(PORT_DIN, PIN_DIN)
#define DI_1()		GPIO_SetBits(PORT_DIN, PIN_DIN)

//added by dongbo
//#define SCK_IS_HIGH()	(GPIO_ReadInputDataBit(PORT_SCK, PIN_SCK) == Bit_SET)
//#define SCK_IS_LOW()	(GPIO_ReadInputDataBit(PORT_SCK, PIN_SCK) == Bit_RESET)

#define DO_IS_HIGH()	(GPIO_ReadInputDataBit(PORT_DOUT, PIN_DOUT) == Bit_SET)

#define DRDY_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DRDY, PIN_DRDY) == Bit_RESET)

//
//#ifdef HARD_SPI		/* Ӳ��SPI */
//	;
//#endif= Bit_RESET)
//

/* �Ĵ������壺 Table 23. Register Map --- ADS1256�����ֲ��30ҳ */
enum
{
	/* �Ĵ�����ַ�� �����Ǹ�λ��ȱʡֵ */
	REG_STATUS = 0,	// x1H
	REG_MUX = 1, // 01H
	REG_ADCON = 2, // 20H
	REG_DRATE = 3, // F0H
	REG_IO = 4, // E0H
	REG_OFC0 = 5, // xxH
	REG_OFC1 = 6, // xxH
	REG_OFC2 = 7, // xxH
	REG_FSC0 = 8, // xxH
	REG_FSC1 = 9, // xxH
	REG_FSC2 = 10, // xxH
};

/* ����壺 TTable 24. Command Definitions --- ADS1256�����ֲ��34ҳ */
enum
{
	CMD_WAKEUP = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

static void ADS1256_Send8Bit(uint8_t _data);
static uint8_t ADS1256_Recive8Bit(void);
static void ADS1256_WaitDRDY(void);
static void ADS1256_ResetHard(void);
static void ADS1256_DelaySCLK(void);
static void ADS1256_DelayDATA(void);

static void ADS1256_WriteCmd(uint8_t _cmd);
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
static uint8_t ADS1256_ReadReg(uint8_t _RegID);
static int32_t ADS1256_ReadData(void);
static void ADS1256_SetChannal(uint8_t _ch);
//static void ADS1256_SetDiffChannal(uint8_t _ch);

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

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitADS1256
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� ADS1256
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InitADS1256(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef HARD_SPI
	SPI_InitTypeDef  SPI_InitStructure;
#endif
	RESET_1();
	PWDN_1();
	CS_1();
	SCK_0();		/* SPI���߿���ʱ�������ǵ͵�ƽ */
	DI_1();


	/* ��GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_CS | RCC_DIN | RCC_DOUT | RCC_SCK | RCC_DRDY | RCC_RESET | RCC_PWDN, ENABLE);
#ifdef HARD_SPI
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	GPIO_PinAFConfig(PORT_DIN, PIN_DIN, GPIO_AF_SPI2);
	GPIO_PinAFConfig(PORT_DOUT, PIN_DOUT, GPIO_AF_SPI2);
	GPIO_PinAFConfig(PORT_SCK, PIN_SCK, GPIO_AF_SPI2);
#endif

	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Pin = PIN_CS;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_PWDN;
	GPIO_Init(PORT_PWDN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_RESET;
	GPIO_Init(PORT_RESET, &GPIO_InitStructure);
#ifdef HARD_SPI
	/* ���ü����������IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */
	GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	GPIO_Init(PORT_SCK, &GPIO_InitStructure);
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		/* ��Ϊ��������� */
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* ���������費ʹ�� */
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	//GPIO_InitStructure.GPIO_Pin = PIN_DOUT;
	//GPIO_Init(PORT_DOUT, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = PIN_DIN;
	//GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	//GPIO_Init(PORT_SCK, &GPIO_InitStructure);

	/* ����GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* �������������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */





	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);


	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI2, &SPI_InitStructure);

	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	
	SPI_Cmd(SPI2, ENABLE);
#else
	GPIO_InitStructure.GPIO_Pin = PIN_DIN;
	GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	GPIO_Init(PORT_SCK, &GPIO_InitStructure);

	/* ����GPIOΪ��������ģʽ(ʵ����CPU��λ���������״̬) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* ��Ϊ����ģʽ */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* �������������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Pin = PIN_DOUT;
	GPIO_Init(PORT_DOUT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
#endif
	

	//ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);	/* ����ADC������ ����1:1, ����������� 1KHz */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_CfgADC
*	����˵��: ����ADC����������������������
*	��    ��: _gain : ����
*			  _drate : �����������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	//ADS1256_StopScan();			/* ��ͣCPU�ж� */

	ADS1256_ResetHard();		/* Ӳ����λ */

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* �ݴ�ADS1256 �Ĵ������ò�����֮������д4���Ĵ��� */

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
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		//buf[0] = (0 << 3) | (1 << 1);
		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	/* ����λ0��ʾAINP�� AIN0,  ����λ8��ʾ AINN �̶��� AINCOM */

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
		buf[2] = (1 << 5) | (0 << 2) | (_gain << 1);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/* ѡ��1;1����, ��������5V */

		/* ��Ϊ�л�ͨ���Ͷ����ݺ�ʱ 123uS, ���ɨ���ж�ģʽ����ʱ��������� = DRATE_1000SPS */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* ѡ������������� */

		CS_0();	/* SPIƬѡ = 0 */
#ifdef HARD_SPI
		SPI_SendByte(CMD_WREG | 0);
		SPI_SendByte(0x03);
		SPI_SendByte(buf[0]);
		SPI_SendByte(buf[1]);
		SPI_SendByte(buf[2]);
		SPI_SendByte(buf[3]);
#else
		ADS1256_Send8Bit(CMD_WREG | 0);	/* д�Ĵ���������, �����ͼĴ�����ַ */
		ADS1256_Send8Bit(0x03);			/* �Ĵ������� - 1, �˴�3��ʾд4���Ĵ��� */

		ADS1256_Send8Bit(buf[0]);	/* ����״̬�Ĵ��� */
		ADS1256_Send8Bit(buf[1]);	/* ��������ͨ������ */
		ADS1256_Send8Bit(buf[2]);	/* ����ADCON���ƼĴ��������� */
		ADS1256_Send8Bit(buf[3]);	/* ��������������� */
#endif		
		//ADS1256_Send8Bit(CMD_SELFCAL);
		//Delay(100);
		CS_1();	/* SPIƬѡ = 1 */
	}
	Delay(50);
	//bsp_DelayUS(50);

}


uint8_t SPI_SendByte(uint8_t byte)
{
	/* Loop while DR register in not emplty */
	SPI_I2S_SendData(SPI2, byte);
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	/* Send byte through the SPI2 peripheral */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2);
}


/*
*********************************************************************************************************
*	�� �� ��: ADS1256_DelaySCLK
*	����˵��: CLK֮����ӳ٣�ʱ���ӳ�. ����STM32F407  168M��Ƶ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_DelaySCLK(void)
{
	//uint16_t i;

	/*
	ȡ 5 ʱ��ʵ��ߵ�ƽ200ns, �͵�ƽ250ns <-- ���ȶ�
	ȡ 10 ���ϣ��������������� �͵�ƽ400ns �߶�400ns <--- �ȶ�
	*/
	//for (i = 0; i < 10; i++);
	Delay(50);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_DelayDATA
*	����˵��: ��ȡDOUT֮ǰ���ӳ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
	Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
	��С 50 ��tCLK = 50 * 0.13uS = 6.5uS
	*/
	//bsp_DelayUS(10);	/* ��С�ӳ� 6.5uS, �˴�ȡ10us */
	Delay(10);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ResetHard
*	����˵��: Ӳ����λ ADS1256оƬ.�͵�ƽ��λ�����4��ʱ�ӣ�Ҳ���� 4x0.13uS = 0.52uS
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_ResetHard(void)
{
	/* ADS1256�����ֲ��7ҳ */
	RESET_0();			/* ��λ */
	//bsp_DelayUS(5);
	Delay(5);
	RESET_1();

	PWDN_0();			/* ������� ͬ��*/
	//Delay(2);
	//bsp_DelayUS(2);	
	PWDN_1();			/* �˳����� */

	//bsp_DelayUS(5);
	Delay(5);
	ADS1256_WaitDRDY();	/* �ȴ� DRDY��Ϊ0, �˹���ʵ��: 630us */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Send8Bit
*	����˵��: ��SPI���߷���8��bit���ݡ� ����CS���ơ�
*	��    ��: _data : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{
	uint8_t i;

	/* �������Ͷ���ֽ�ʱ����Ҫ�ӳ�һ�� */
	ADS1256_DelaySCLK();  //50us
	ADS1256_DelaySCLK();

	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
	for (i = 0; i < 8; i++)
	{
		if (_data & 0x80)
		{
			DI_1();
		}
		else
		{
			DI_0();
		}
		SCK_1();
		ADS1256_DelaySCLK();
		_data <<= 1;
		SCK_0();			/* <----  ADS1256 ����SCK�½��ز���DIN����, ���ݱ���ά�� 50nS */
		ADS1256_DelaySCLK();
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
	uint8_t j;
	uint8_t i;
	uint8_t read = 0;

	//ADS1256_DelaySCLK();
	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
	//SCK_1();
	//Delay(10);
	for (i = 0; i < 8; i++)
	{
		//TM_GENERAL_DWTCounterSetValue(0);
		SCK_1();
		//while (TM_GENERAL_DWTCounterGetValue() < 84);
		Delay(20);
		//while (SCK_IS_LOW());
		read = read << 1;
		//TM_GENERAL_DWTCounterSetValue(0);
		SCK_0();
		if (DO_IS_HIGH())
		{
			read++;
		}
		//while (TM_GENERAL_DWTCounterGetValue() < 84);
		Delay(20);
		//while (SCK_IS_HIGH());
	}
	//SCK_0();
	//Delay(10);
	return read;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WriteReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPIƬѡ = 0 */
#ifdef HARD_SPI
	SPI_SendByte(CMD_WREG | _RegID);
	SPI_SendByte(0x00);
	SPI_SendByte(_RegValue);
#else
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);		/* �Ĵ������� - 1, �˴�д1���Ĵ��� */

	ADS1256_Send8Bit(_RegValue);	/* ���ͼĴ���ֵ */
#endif
	CS_1();	/* SPIƬѡ = 1 */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ��
*	�� �� ֵ: �����ļĴ���ֵ��
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPIƬѡ = 0 */
#ifdef HARD_SPI
	SPI_SendByte(CMD_RREG | _RegID);
	SPI_SendByte(0x00);
	Delay(10);
	read = SPI_SendByte(0x00);
#else
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);	/* �Ĵ������� - 1, �˴���1���Ĵ��� */

	ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */

	read = ADS1256_Recive8Bit();	/* ���Ĵ���ֵ */
#endif
	
	CS_1();	/* SPIƬѡ = 1 */

	return read;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WriteCmd
*	����˵��: ���͵��ֽ�����
*	��    ��:  _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPIƬѡ = 1 */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadChipID
*	����˵��: ��оƬID, ��״̬�Ĵ����еĸ�4bit
*	��    ��: ��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetChannal
*	����˵��: ����ͨ���š���·���á�AIN- �̶��ӵأ�ACOM).
*	��    ��: _ch : ͨ���ţ� 0-7
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
	0000 = AIN0 (default)
	0001 = AIN1
	0010 = AIN2 (ADS1256 only)
	0011 = AIN3 (ADS1256 only)
	0100 = AIN4 (ADS1256 only)
	0101 = AIN5 (ADS1256 only)
	0110 = AIN6 (ADS1256 only)
	0111 = AIN7 (ADS1256 only)
	1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

	NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
	0000 = AIN0
	0001 = AIN1 (default)
	0010 = AIN2 (ADS1256 only)
	0011 = AIN3 (ADS1256 only)
	0100 = AIN4 (ADS1256 only)
	0101 = AIN5 (ADS1256 only)
	0110 = AIN6 (ADS1256 only)
	0111 = AIN7 (ADS1256 only)
	1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN �̶��� AINCOM */
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetDiffChannal
*	����˵��: ���ò��ͨ���š���·���á�
*	��    ��: _ch : ͨ����,0-3����4��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
*********************************************************************************************************
*/
#if 0
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
	0000 = AIN0 (default)
	0001 = AIN1
	0010 = AIN2 (ADS1256 only)
	0011 = AIN3 (ADS1256 only)
	0100 = AIN4 (ADS1256 only)
	0101 = AIN5 (ADS1256 only)
	0110 = AIN6 (ADS1256 only)
	0111 = AIN7 (ADS1256 only)
	1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

	NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
	0000 = AIN0
	0001 = AIN1 (default)
	0010 = AIN2 (ADS1256 only)
	0011 = AIN3 (ADS1256 only)
	0100 = AIN4 (ADS1256 only)
	0101 = AIN5 (ADS1256 only)
	0110 = AIN6 (ADS1256 only)
	0111 = AIN7 (ADS1256 only)
	1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* ������� AIN0�� AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* ������� AIN2�� AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* ������� AIN4�� AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* ������� AIN6�� AIN7 */
	}
}
#endif

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_WaitDRDY
*	����˵��: �ȴ��ڲ�������ɡ� ��У׼ʱ��ϳ�����Ҫ�ȴ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 40000000; i++)
	{
		if (DRDY_IS_LOW())
		{
#if defined (TM_DISCO_STM32F4_DISCOVERY)
			TM_DISCO_LedOn(LED_RED);
#endif
			break;
		}
	}
	if (i >= 40000000)
	{
		//printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* �������. �����Ŵ� */
		//TM_USB_VCP_Puts("ADS1256_WaitDRDY() Time Out ...\r\n");
#if defined (TM_DISCO_STM32F4_DISCOVERY)
		TM_DISCO_LedOn(LED_BLUE);
#endif
	}
	else
	{
#if defined (TM_DISCO_STM32F4_DISCOVERY)
		TM_DISCO_LedOff(LED_BLUE);
#endif
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadData
*	����˵��: ��ADC����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
	uint32_t r = 0;
	int i = 0;

	

	//ADS1256_Send8Bit(CMD_RDATA);	/* �����ݵ����� */

	//ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */

	/* �����������3���ֽڣ����ֽ���ǰ */
#ifdef HARD_SPI
	//CS_0();	/* SPIƬѡ = 0 */
	//SPI_SendByte(CMD_RDATA);
	////ADS1256_DelayDATA();	/* �����ӳٲ��ܶ�ȡоƬ�������� */
	//Delay(10);
	for (i = 0; i < 3; i++)
	{
		read = read << 8;
		r = SPI_SendByte(0);
		read |= r;
	}
	Delay(10);
	//CS_1();
#else

	read = ADS1256_Recive8Bit() << 16;
	read += ADS1256_Recive8Bit() << 8;
	read += ADS1256_Recive8Bit() << 0;
#endif
	
	//CS_1();	/* SPIƬѡ = 1 */

	/* ����������չ��24λ�з�������չΪ32λ�з����� */
	if (read & 0x800000)
	{
		read += 0xFF000000;
	}

	return (int32_t)read;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ReadAdc
*	����˵��: ��ָ��ͨ����ADC����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t ADS1256_ReadAdc(void)
{
	/* ADS1256 �����ֲ��21ҳ */

	//#if 0	/* ����30Ksps �������� */
	int32_t read;

	//while (!DRDY_IS_LOW());	/*����ת��ʱ�� �ȴ� DRDY �ߣ� ����ת��ʱ���ȴ� DRDY �� */
	//while (!DRDY_IS_LOW());	/* �ȴ� DRDY �� */

	//ADS1256_SetChannal(_ch);	/* �л�ģ��ͨ�� */
	//Delay(5);

	//ADS1256_WriteCmd(CMD_SYNC);
	//Delay(5);

	//ADS1256_WriteCmd(CMD_WAKEUP);  /* ��������£����ʱ�� DRDY �Ѿ�Ϊ�� */
	//Delay(25);

	read = (int32_t)ADS1256_ReadData();

	//while (DRDY_IS_LOW());	/* �ȴ� DRDY �� */
	//while (!DRDY_IS_LOW());	/* �ȴ� DRDY �� */

	//read = (int32_t)ADS1256_ReadData();

	return read;
	//#else	
	//	//while (DRDY_IS_LOW());
	//
	//	/* ADS1256 �����ֲ��21ҳ */
	//	ADS1256_WaitDRDY();		/* �ȴ� DRDY = 0 */
	//
	//	//ADS1256_SetChannal(_ch);	/* �л�ģ��ͨ�� */
	//	//Delay(5);
	//
	//	//ADS1256_WriteCmd(CMD_SYNC);
	//	//Delay(5);
	//
	//	//ADS1256_WriteCmd(CMD_WAKEUP);
	//	//Delay(25);
	//
	//	//ADS1256_WaitDRDY();		/* �ȴ� DRDY = 0 */
	//
	//	return (int32_t)ADS1256_ReadData();
	//#endif	
}

/*
*********************************************************************************************************
*	����ĺ�������DRDY�жϹ���ģʽ
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_StartScan
*	����˵��: �� DRDY���� ��PH9 �����ó��ⲿ�жϴ�����ʽ�� �жϷ��������ɨ��8��ͨ�������ݡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
//need to be edited
void ADS1256_StartScan(void)
{
	CS_0();
	//while (!DRDY_IS_LOW());
#ifdef HARD_SPI	
	SPI_SendByte(CMD_RDATAC);
#else
	ADS1256_Send8Bit(CMD_RDATAC);	/* �����ݵ����� */
#endif
	Delay(10);
	//Delay(50);
	/* SPIƬѡ = 0 */
	//	EXTI_InitTypeDef   EXTI_InitStructure;
	//	NVIC_InitTypeDef   NVIC_InitStructure;
	//
	//	/* ʹ��SYSCFGʱ�� */
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//
	//	/* ���� EXTI Line9 �� PE9 ���� */
	//#if defined (TM_DISCO_STM32F4_DISCOVERY)
	//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);
	//#elif defined (TM_DISCO_NUCLEO_F401)
	//	/* ���� EXTI Line9 �� PC9 ���� */
	//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	//#endif
	//	/* ���� EXTI LineXXX */
	//	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	/* �½���(�ȴ� DRDY ��1��0��ʱ��) */
	//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	//	EXTI_Init(&EXTI_InitStructure);
	//
	//	/* ����NVIC���ȼ�����ΪGroup2��0-3��ռʽ���ȼ���0-3����Ӧʽ���ȼ� */
	//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//
	//	/* �ж����ȼ����� ������ȼ� ����һ��Ҫ�ֿ��������жϣ����ܹ��ϲ���һ���������� */
	//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitStructure);

	/* ��ʼɨ��ǰ, ������������ */
	/*{
		uint8_t i;

		g_tADS1256.Channel = 0;

		for (i = 0; i < 8; i++)
		{
		g_tADS1256.AdcNow[i] = 0;
		}
		}*/
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_StopScan
*	����˵��: ֹͣ DRDY �ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_StopScan(void)
{
	while (!DRDY_IS_LOW());
#ifdef HARD_SPI
	SPI_SendByte(CMD_SDATAC);
#else
	ADS1256_Send8Bit(CMD_SDATAC);
#endif
	CS_1();
	//	EXTI_InitTypeDef   EXTI_InitStructure;
	//	//	NVIC_InitTypeDef   NVIC_InitStructure;
	//
	//	/* ���� EXTI LineXXX */
	//	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	/* �½���(�ȴ� DRDY ��1��0��ʱ��) */
	//	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		/* ��ֹ */
	//	EXTI_Init(&EXTI_InitStructure);
	//
	//#if 0			
	//	/* �ж����ȼ����� ������ȼ� ����һ��Ҫ�ֿ��������жϣ����ܹ��ϲ���һ���������� */
	//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;		/* ��ֹ */
	//	NVIC_Init(&NVIC_InitStructure);
	//#endif
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_GetAdc
*	����˵��: �ӻ�������ȡADC��������������ṹ�����жϷ���������ġ�
*	��    ��: _ch ͨ���� (0 - 7)
*	�� �� ֵ: ADC�ɼ�������з�������
*********************************************************************************************************
*/
//need to be edited
int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	/*if (_ch > 7)
	{
	return 0;
	}*/

	//DISABLE_INT();  			/* ���ж� */
	//__disable_irq();

	iTemp = g_tADS1256.AdcNow[_ch];

	//ENABLE_INT();  				/* ���ж� */
	//__enable_irq();

	return iTemp;
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_ISR
*	����˵��: ��ʱ�ɼ��жϷ������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	/* ��ȡ�ɼ��ṹ��������ȫ�ֱ��� */
	//ADS1256_SetChannal(g_tADS1256.Channel);	/* �л�ģ��ͨ�� */
	//bsp_DelayUS(5);
	//Delay(5);

	//ADS1256_WriteCmd(CMD_SYNC);
	//bsp_DelayUS(5);
	//Delay(5);

	//ADS1256_WriteCmd(CMD_WAKEUP);
	//bsp_DelayUS(25);
	//Delay(25);
	g_tADS1256.AdcNow[0] = ADS1256_ReadData();
	//if (g_tADS1256.Channel == 0)
	//{
	//	g_tADS1256.AdcNow[7] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
	//}
	//else
	//{
	//	g_tADS1256.AdcNow[g_tADS1256.Channel - 1] = ADS1256_ReadData();	/* ע�Ᵽ�������һ��ͨ�������� */
	//}

	//if (++g_tADS1256.Channel >= 8)
	//{
	//	g_tADS1256.Channel = 0;
	//}
}


/*
*********************************************************************************************************
*	�� �� ��: EXTI9_5_IRQHandler
*	����˵��: �ⲿ�жϷ������.  �˳���ִ��ʱ��Լ 123uS
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/

//need to be edited
//#ifndef EXTI9_5_ISR_MOVE_OUT		/* bsp.h �ж�����У���ʾ�������Ƶ� stam32f4xx_it.c�� �����ظ����� */
//void EXTI9_5_IRQHandler(void)
//{
//	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line9);		/* ����жϱ�־λ */
//
//		ADS1256_ISR();
//
//		/* ִ������Ĵ�����Ϻ��ٴ������жϱ�־ */
//		EXTI_ClearITPendingBit(EXTI_Line9);		/* ����жϱ�־λ */
//	}
//}
//#endif





/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
