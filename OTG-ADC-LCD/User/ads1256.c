/*
*********************************************************************************************************
*
*	模块名称 : ADS1256 驱动模块(8通道带PGA的24位ADC)
*	文件名称 : bsp_ads1256.c
*	版    本 : V1.0
*	说    明 : ADS1256模块和CPU之间采用SPI接口。本驱动程序支持硬件SPI接口和软件SPI接口。
*			  通过宏切换。
*
*	修改记录 :
*		版本号  日期         作者     说明
*		V1.0    2014-01-01  armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/


#include "ads1256.h"
//#include "tm_stm32f4_usb_vcp.h"
#include "tm_stm32f4_spi.h"
#include "tm_stm32f4_general.h"


//#define SOFT_SPI		/* 定义此行表示使用GPIO模拟SPI接口 */
//#define HARD_SPI		/* 定义此行表示使用CPU的硬件SPI接口 */

/*
ADS1256模块可以直接插到STM32-V5开发板CN26排母(2*6P 2.54mm)接口上.

//FOR F4DISCOVERY
+5V   <------  5.0V      5V供电
GND   -------  GND       地
DRDY  ------>  PE9       准备就绪
CS    <------  PB0       SPI_CS
DIN   <------  PA7       SPI_MOSI
DOUT  ------>  PA6       SPI_MISO
SCLK  <------  PA5       SPI时钟
GND   -------  GND       地
//-PDWN  <------  PB0       掉电控制
RST   <------  PC5       复位信号
NC   空脚
NC   空脚


//FOR F401
+5V <------  5.0V      5V供电
GND	------ - GND       地
DRDY------>  PC9       准备就绪
CS    <------PB0       SPI_CS
DIN   <------PC2       SPI_MISO
DOUT------>  PC3       SPI_MOSI
SCLK  <------PB10      SPI时钟
GND  ------- GND       地
//-PDWN  <------  PB2       掉电控制
RST   <------PC4       复位信号
*/

/*
ADS1256基本特性:
1、模拟部分供电5V;
2、SPI数字接口电平：3.3V
3、PGA设置范围： 1、2、4、8、16、32、64、
4、参考电压2.5V (推荐缺省的，外置的）
5、输入电压范围：PGA = 1 时, 可输入正负5V
6. 自动校准 （当设置了PGA,BUF使能、数据采样率时，会启动自校准)
7. 输入的缓冲器可设置启用和关闭（一般选启用）


外部晶振频率 = 7.68MHz,
时钟频率 tCLK = 1/7.68M = 0.13uS
输出数据周期 tDATA =  1 / 30K = 0.033mS  (按30Ksps计算)

对SPI的时钟速度要求: (ads1256.pdf page 6)
最快 4个tCLK = 0.52uS
最慢 10个tDATA = 0.3mS (按 30Ksps 计算)

SCL高电平和低电平持续时间最小 200ns

RREG, WREG, RDATA 命令之后，需要延迟 4 * tCLK = 0.52uS;
RDATAC, RESET, SYNC 命令之后，需要延迟 24 * tCLK = 3.12uS;

实际测试，在3.3V上电后, 及时不做任何配置，ADS125的DRDY 口线即开始输出脉冲信号（2.6us高,33.4低，频率30KHz）
*/

/*
调试记录
(1) 设置寄存器时，SCK过快导致芯片不能每次都收到数据。原因: 发送的相邻的字节之间需要延迟一小段时间.
(2) 连续复位CPU时，偶尔出现芯片输出采样率异常。
*/

//#if !defined(SOFT_SPI) && !defined(HARD_SPI)
// 	#error "Please define SPI Interface mode : SOFT_SPI or HARD_SPI"
//#endif
//
//#ifdef SOFT_SPI		/* 软件SPI */
/* 定义GPIO端口 */
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
/* 定义口线置0和置1的宏 */
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
//#ifdef HARD_SPI		/* 硬件SPI */
//	;
//#endif= Bit_RESET)
//

/* 寄存器定义： Table 23. Register Map --- ADS1256数据手册第30页 */
enum
{
	/* 寄存器地址， 后面是复位后缺省值 */
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

/* 命令定义： TTable 24. Command Definitions --- ADS1256数据手册第34页 */
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
	0xF0,		/* 复位时缺省值 */
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
*	函 数 名: bsp_InitADS1256
*	功能说明: 配置STM32的GPIO和SPI接口，用于连接 ADS1256
*	形    参: 无
*	返 回 值: 无
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
	SCK_0();		/* SPI总线空闲时，钟线是低电平 */
	DI_1();


	/* 打开GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_CS | RCC_DIN | RCC_DOUT | RCC_SCK | RCC_DRDY | RCC_RESET | RCC_PWDN, ENABLE);
#ifdef HARD_SPI
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	GPIO_PinAFConfig(PORT_DIN, PIN_DIN, GPIO_AF_SPI2);
	GPIO_PinAFConfig(PORT_DOUT, PIN_DOUT, GPIO_AF_SPI2);
	GPIO_PinAFConfig(PORT_SCK, PIN_SCK, GPIO_AF_SPI2);
#endif

	/* 配置几个推完输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Pin = PIN_CS;
	GPIO_Init(PORT_CS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_PWDN;
	GPIO_Init(PORT_PWDN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_RESET;
	GPIO_Init(PORT_RESET, &GPIO_InitStructure);
#ifdef HARD_SPI
	/* 配置几个推完输出IO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */
	GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	GPIO_Init(PORT_SCK, &GPIO_InitStructure);
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		/* 设为输入输出口 */
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 上下拉电阻不使能 */
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	//GPIO_InitStructure.GPIO_Pin = PIN_DOUT;
	//GPIO_Init(PORT_DOUT, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = PIN_DIN;
	//GPIO_Init(PORT_DIN, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = PIN_SCK;
	//GPIO_Init(PORT_SCK, &GPIO_InitStructure);

	/* 配置GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* 设为输入口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 无需上下拉电阻 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */





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

	/* 配置GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		/* 设为输入口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽模式 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 无需上下拉电阻 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO口最大速度 */

	GPIO_InitStructure.GPIO_Pin = PIN_DOUT;
	GPIO_Init(PORT_DOUT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
#endif
	

	//ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);	/* 配置ADC参数： 增益1:1, 数据输出速率 1KHz */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_CfgADC
*	功能说明: 配置ADC参数，增益和数据输出速率
*	形    参: _gain : 增益
*			  _drate : 数据输出速率
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	//ADS1256_StopScan();			/* 暂停CPU中断 */

	ADS1256_ResetHard();		/* 硬件复位 */

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* 暂存ADS1256 寄存器配置参数，之后连续写4个寄存器 */

		/* 状态寄存器定义
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

		ACAL=1使能自校准功能。当 PGA，BUFEEN, DRATE改变时会启动自校准
		*/
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		//buf[0] = (0 << 3) | (1 << 1);
		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	/* 高四位0表示AINP接 AIN0,  低四位8表示 AINN 固定接 AINCOM */

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
		01 = Sensor Detect Current = 0.5 μ A
		10 = Sensor Detect Current = 2 μ A
		11 = Sensor Detect Current = 10μ A
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
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/* 选择1;1增益, 输入正负5V */

		/* 因为切换通道和读数据耗时 123uS, 因此扫描中断模式工作时，最大速率 = DRATE_1000SPS */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* 选择数据输出速率 */

		CS_0();	/* SPI片选 = 0 */
#ifdef HARD_SPI
		SPI_SendByte(CMD_WREG | 0);
		SPI_SendByte(0x03);
		SPI_SendByte(buf[0]);
		SPI_SendByte(buf[1]);
		SPI_SendByte(buf[2]);
		SPI_SendByte(buf[3]);
#else
		ADS1256_Send8Bit(CMD_WREG | 0);	/* 写寄存器的命令, 并发送寄存器地址 */
		ADS1256_Send8Bit(0x03);			/* 寄存器个数 - 1, 此处3表示写4个寄存器 */

		ADS1256_Send8Bit(buf[0]);	/* 设置状态寄存器 */
		ADS1256_Send8Bit(buf[1]);	/* 设置输入通道参数 */
		ADS1256_Send8Bit(buf[2]);	/* 设置ADCON控制寄存器，增益 */
		ADS1256_Send8Bit(buf[3]);	/* 设置输出数据速率 */
#endif		
		//ADS1256_Send8Bit(CMD_SELFCAL);
		//Delay(100);
		CS_1();	/* SPI片选 = 1 */
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
*	函 数 名: ADS1256_DelaySCLK
*	功能说明: CLK之间的延迟，时序延迟. 用于STM32F407  168M主频
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelaySCLK(void)
{
	//uint16_t i;

	/*
	取 5 时，实测高电平200ns, 低电平250ns <-- 不稳定
	取 10 以上，可以正常工作， 低电平400ns 高定400ns <--- 稳定
	*/
	//for (i = 0; i < 10; i++);
	Delay(50);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_DelayDATA
*	功能说明: 读取DOUT之前的延迟
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
	Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
	最小 50 个tCLK = 50 * 0.13uS = 6.5uS
	*/
	//bsp_DelayUS(10);	/* 最小延迟 6.5uS, 此处取10us */
	Delay(10);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ResetHard
*	功能说明: 硬件复位 ADS1256芯片.低电平复位。最快4个时钟，也就是 4x0.13uS = 0.52uS
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_ResetHard(void)
{
	/* ADS1256数据手册第7页 */
	RESET_0();			/* 复位 */
	//bsp_DelayUS(5);
	Delay(5);
	RESET_1();

	PWDN_0();			/* 进入掉电 同步*/
	//Delay(2);
	//bsp_DelayUS(2);	
	PWDN_1();			/* 退出掉电 */

	//bsp_DelayUS(5);
	Delay(5);
	ADS1256_WaitDRDY();	/* 等待 DRDY变为0, 此过程实测: 630us */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_Send8Bit
*	功能说明: 向SPI总线发送8个bit数据。 不带CS控制。
*	形    参: _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{
	uint8_t i;

	/* 连续发送多个字节时，需要延迟一下 */
	ADS1256_DelaySCLK();  //50us
	ADS1256_DelaySCLK();

	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
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
		SCK_0();			/* <----  ADS1256 是在SCK下降沿采样DIN数据, 数据必须维持 50nS */
		ADS1256_DelaySCLK();
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_Recive8Bit
*	功能说明: 从SPI总线接收8个bit数据。 不带CS控制。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t j;
	uint8_t i;
	uint8_t read = 0;

	//ADS1256_DelaySCLK();
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
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
*	函 数 名: ADS1256_WriteReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPI片选 = 0 */
#ifdef HARD_SPI
	SPI_SendByte(CMD_WREG | _RegID);
	SPI_SendByte(0x00);
	SPI_SendByte(_RegValue);
#else
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);		/* 寄存器个数 - 1, 此处写1个寄存器 */

	ADS1256_Send8Bit(_RegValue);	/* 发送寄存器值 */
#endif
	CS_1();	/* SPI片选 = 1 */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值。
*	返 回 值: 读到的寄存器值。
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPI片选 = 0 */
#ifdef HARD_SPI
	SPI_SendByte(CMD_RREG | _RegID);
	SPI_SendByte(0x00);
	Delay(10);
	read = SPI_SendByte(0x00);
#else
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);	/* 寄存器个数 - 1, 此处读1个寄存器 */

	ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */

	read = ADS1256_Recive8Bit();	/* 读寄存器值 */
#endif
	
	CS_1();	/* SPI片选 = 1 */

	return read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WriteCmd
*	功能说明: 发送单字节命令
*	形    参:  _cmd : 命令
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI片选 = 1 */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadChipID
*	功能说明: 读芯片ID, 读状态寄存器中的高4bit
*	形    参: 无
*	返 回 值: 8bit状态寄存器值的高4位
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
*	函 数 名: ADS1256_SetChannal
*	功能说明: 配置通道号。多路复用。AIN- 固定接地（ACOM).
*	形    参: _ch : 通道号， 0-7
*	返 回 值: 无
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
	1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

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
	1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN 固定接 AINCOM */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetDiffChannal
*	功能说明: 配置差分通道号。多路复用。
*	形    参: _ch : 通道号,0-3；共4对
*	返 回 值: 8bit状态寄存器值的高4位
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
	1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

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
	1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* 差分输入 AIN0， AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* 差分输入 AIN2， AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* 差分输入 AIN4， AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* 差分输入 AIN6， AIN7 */
	}
}
#endif

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WaitDRDY
*	功能说明: 等待内部操作完成。 自校准时间较长，需要等待。
*	形    参: 无
*	返 回 值: 无
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
		//printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* 调试语句. 用语排错 */
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
*	函 数 名: ADS1256_ReadData
*	功能说明: 读ADC数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
	uint32_t r = 0;
	int i = 0;

	

	//ADS1256_Send8Bit(CMD_RDATA);	/* 读数据的命令 */

	//ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */

	/* 读采样结果，3个字节，高字节在前 */
#ifdef HARD_SPI
	//CS_0();	/* SPI片选 = 0 */
	//SPI_SendByte(CMD_RDATA);
	////ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */
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
	
	//CS_1();	/* SPI片选 = 1 */

	/* 负数进行扩展。24位有符号数扩展为32位有符号数 */
	if (read & 0x800000)
	{
		read += 0xFF000000;
	}

	return (int32_t)read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadAdc
*	功能说明: 读指定通道的ADC数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t ADS1256_ReadAdc(void)
{
	/* ADS1256 数据手册第21页 */

	//#if 0	/* 对于30Ksps 采样速率 */
	int32_t read;

	//while (!DRDY_IS_LOW());	/*单次转换时， 等待 DRDY 高； 连续转换时，等待 DRDY 低 */
	//while (!DRDY_IS_LOW());	/* 等待 DRDY 低 */

	//ADS1256_SetChannal(_ch);	/* 切换模拟通道 */
	//Delay(5);

	//ADS1256_WriteCmd(CMD_SYNC);
	//Delay(5);

	//ADS1256_WriteCmd(CMD_WAKEUP);  /* 正常情况下，这个时候 DRDY 已经为高 */
	//Delay(25);

	read = (int32_t)ADS1256_ReadData();

	//while (DRDY_IS_LOW());	/* 等待 DRDY 高 */
	//while (!DRDY_IS_LOW());	/* 等待 DRDY 低 */

	//read = (int32_t)ADS1256_ReadData();

	return read;
	//#else	
	//	//while (DRDY_IS_LOW());
	//
	//	/* ADS1256 数据手册第21页 */
	//	ADS1256_WaitDRDY();		/* 等待 DRDY = 0 */
	//
	//	//ADS1256_SetChannal(_ch);	/* 切换模拟通道 */
	//	//Delay(5);
	//
	//	//ADS1256_WriteCmd(CMD_SYNC);
	//	//Delay(5);
	//
	//	//ADS1256_WriteCmd(CMD_WAKEUP);
	//	//Delay(25);
	//
	//	//ADS1256_WaitDRDY();		/* 等待 DRDY = 0 */
	//
	//	return (int32_t)ADS1256_ReadData();
	//#endif	
}

/*
*********************************************************************************************************
*	下面的函数用于DRDY中断工作模式
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StartScan
*	功能说明: 将 DRDY引脚 （PH9 ）配置成外部中断触发方式， 中断服务程序中扫描8个通道的数据。
*	形    参: 无
*	返 回 值: 无
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
	ADS1256_Send8Bit(CMD_RDATAC);	/* 读数据的命令 */
#endif
	Delay(10);
	//Delay(50);
	/* SPI片选 = 0 */
	//	EXTI_InitTypeDef   EXTI_InitStructure;
	//	NVIC_InitTypeDef   NVIC_InitStructure;
	//
	//	/* 使能SYSCFG时钟 */
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//
	//	/* 连接 EXTI Line9 到 PE9 引脚 */
	//#if defined (TM_DISCO_STM32F4_DISCOVERY)
	//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);
	//#elif defined (TM_DISCO_NUCLEO_F401)
	//	/* 连接 EXTI Line9 到 PC9 引脚 */
	//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	//#endif
	//	/* 配置 EXTI LineXXX */
	//	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	/* 下降沿(等待 DRDY 由1变0的时刻) */
	//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	//	EXTI_Init(&EXTI_InitStructure);
	//
	//	/* 设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级 */
	//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//
	//	/* 中断优先级配置 最低优先级 这里一定要分开的设置中断，不能够合并到一个里面设置 */
	//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitStructure);

	/* 开始扫描前, 清零结果缓冲区 */
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
*	函 数 名: ADS1256_StopScan
*	功能说明: 停止 DRDY 中断
*	形    参: 无
*	返 回 值: 无
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
	//	/* 配置 EXTI LineXXX */
	//	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	/* 下降沿(等待 DRDY 由1变0的时刻) */
	//	EXTI_InitStructure.EXTI_LineCmd = DISABLE;		/* 禁止 */
	//	EXTI_Init(&EXTI_InitStructure);
	//
	//#if 0			
	//	/* 中断优先级配置 最低优先级 这里一定要分开的设置中断，不能够合并到一个里面设置 */
	//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;		/* 禁止 */
	//	NVIC_Init(&NVIC_InitStructure);
	//#endif
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_GetAdc
*	功能说明: 从缓冲区读取ADC采样结果。采样结构是由中断服务程序填充的。
*	形    参: _ch 通道号 (0 - 7)
*	返 回 值: ADC采集结果（有符号数）
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

	//DISABLE_INT();  			/* 关中断 */
	//__disable_irq();

	iTemp = g_tADS1256.AdcNow[_ch];

	//ENABLE_INT();  				/* 开中断 */
	//__enable_irq();

	return iTemp;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ISR
*	功能说明: 定时采集中断服务程序
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	/* 读取采集结构，保存在全局变量 */
	//ADS1256_SetChannal(g_tADS1256.Channel);	/* 切换模拟通道 */
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
	//	g_tADS1256.AdcNow[7] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
	//}
	//else
	//{
	//	g_tADS1256.AdcNow[g_tADS1256.Channel - 1] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
	//}

	//if (++g_tADS1256.Channel >= 8)
	//{
	//	g_tADS1256.Channel = 0;
	//}
}


/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序.  此程序执行时间约 123uS
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/

//need to be edited
//#ifndef EXTI9_5_ISR_MOVE_OUT		/* bsp.h 中定义此行，表示本函数移到 stam32f4xx_it.c。 避免重复定义 */
//void EXTI9_5_IRQHandler(void)
//{
//	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line9);		/* 清除中断标志位 */
//
//		ADS1256_ISR();
//
//		/* 执行上面的代码完毕后，再次清零中断标志 */
//		EXTI_ClearITPendingBit(EXTI_Line9);		/* 清除中断标志位 */
//	}
//}
//#endif





/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
