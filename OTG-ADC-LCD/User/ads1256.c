
#include "ads1256.h"

u8 tabb[2];

//PE6=CS PE7=RESET PE8=PWDN   //PE9=DRSY
#define ADS_CS_LOW()        GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define ADS_CS_HIGH()       GPIO_SetBits(GPIOB, GPIO_Pin_12)
//#define PWDN_LOW()	    GPIO_ResetBits(GPIOE, GPIO_Pin_8)
//#define PWDN_HIGH()	    GPIO_SetBits(GPIOE, GPIO_Pin_8)
#define RESET_LOW()	    GPIO_ResetBits(GPIOC, GPIO_Pin_4)
#define RESET_HIGH()	    GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define ADS_DRDY	    GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)


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


// define gain codes 
#define ADS1256_GAIN_1      0x00 
#define ADS1256_GAIN_2      0x01 
#define ADS1256_GAIN_4      0x02 
#define ADS1256_GAIN_8      0x03 
#define ADS1256_GAIN_16     0x04 
#define ADS1256_GAIN_32     0x05 
#define ADS1256_GAIN_64     0x06 
//#define ADS1256_GAIN_64     0x07 

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

void SPI_Init2(void) //PE5��ʼ
{
	u8 read_tab[5];
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //PE3=1
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_Init(GPIOE, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOE, GPIO_Pin_3);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  //PC9=DRSY
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;    //PB12=CS 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;    //PC4=RESET 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_12);  //CS=1 
	GPIO_SetBits(GPIOC, GPIO_Pin_4);  //RESET=1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //sck
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //MISO
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //mosi
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Cmd(SPI2, ENABLE);
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE); //ʹ��	SPI���ͻ�����ж�

	ADS_CS_HIGH();

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

u8 results1, results2, results3;
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
u8 SPI_SendByte(u8 byte)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI2, byte);
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2);
}
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
	while (ADS_DRDY);//��ADS1256_DRDYΪ��ʱ����д�Ĵ���
	SPI_SendByte(ADS1256_CMD_WREG | (regaddr & 0xF));//��Ĵ���д�����ݵ�ַ
	SPI_SendByte(0);//д�����ݵĸ���n-1
	Delay(100);
	SPI_SendByte(databyte);//��regaddr��ַָ��ļĴ���д������databyte
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
	while (ADS_DRDY);//��ADS1256_DRDYΪ��ʱ����д�Ĵ���
	SPI_SendByte(ADS1256_CMD_RREG | (regaddr & 0xF));
	SPI_SendByte(0);//д���ȡ���ݵĸ���n-1
	Delay(10); //min=50*(1/fCLKIN)=50*(1/7.68MHZ)=6500ns;max=whatever                   
	r = SPI_SendByte(0); //����regaddr��ַָ��ļĴ���������
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
void ADS1256_Init(void)
{
	//unsigned char tab1[5];
	SPI_Init2();
	ADS1256_ResetHard();
	
	/*tab1[0] = ADS1256RREG(0);
	tab1[1] = ADS1256RREG(1);
	tab1[2] = ADS1256RREG(2);
	tab1[3] = ADS1256RREG(3);*/

	//ADS1256WREG(0x00, 0x31 );      
	//ADS1256WREG(0x01, 0x23 );         
	//ADS1256WREG(0x02, 0x20 );         
	//ADS1256WREG(0x03, 0xf0 ); 






	
	//while (ADS_DRDY);
	//SPI_SendByte(ADS1256_CMD_WREG | ADS1256_STATUS);//����д��4���Ĵ���
	//SPI_SendByte(3);
	//SPI_SendByte(0x31);
	//SPI_SendByte(0x23);
	//SPI_SendByte(0x20);
	//SPI_SendByte(0x03);  
	//ADS_CS_LOW();
	ADS1256WREG(ADS1256_STATUS, 0x00);               // ��λ��ǰ��У׼��ʹ�û���
	Delay(10);
	//	ADS1256WREG(ADS1256_MUX,0x08);                  // ��ʼ���˿�A0Ϊ��+����AINCOMλ��-��
	ADS1256WREG(ADS1256_ADCON, 0x00);                // �Ŵ���1
	Delay(10);
	ADS1256WREG(ADS1256_DRATE, ADS1256_DRATE_30000SPS);  // ����5sps
	Delay(10);
	ADS1256WREG(ADS1256_IO, 0x00);
	Delay(10);
	SPI_I2S_SendData(SPI2, ADS1256_CMD_SELFCAL);
	while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9));
	SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_TXE);	 //������ͻ���Ϊ���жϱ�ʶ
	//ADS_CS_HIGH();


	Delay(50);

	/*Delay(100);
	tab1[1] = ADS1256RREG(1);
	tab1[2] = ADS1256RREG(2);
	tab1[3] = ADS1256RREG(3);
	tab1[4] = ADS1256RREG(4);
	tab1[5] = ADS1256RREG(5);*/

}

int32_t ADS1256ReadData(void)
{
	unsigned char i = 0;
	uint32_t sum = 0;
	uint32_t r = 0;
	Delay(5);
	//    ADS_CS_LOW();
	while (ADS_DRDY);               //��ADS1256_DRDYΪ��ʱ����д�Ĵ��� 	
	SPI_SendByte(ADS1256_CMD_SYNC);
	Delay(5);
	SPI_SendByte(ADS1256_CMD_WAKEUP);
	Delay(5);
	SPI_SendByte(ADS1256_CMD_RDATA);
	Delay(10);               //min=50*(1/fCLKIN)=50*(1/7.68MHZ)=6500ns;max=whatever
	for (i = 0; i < 3; i++)
	{
		sum = sum << 8;
		r = SPI_SendByte(0);
		sum |= r;
	}
	ADS_CS_HIGH();
	/* ����������չ��24λ�з�������չΪ32λ�з����� */
	if (sum & 0x800000)
	{
		sum += 0xFF000000;
		sum = sum * 2;
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
void ADS_sum(void)
{
	unsigned long results = 0;
	//ADS1256WREG(ADS1256_MUX,road);		//����ͨ��
	results = ADS1256ReadData();        //��ȡADֵ������24λ���ݡ�
	results1 = (results >> 16) & 0x0000ff; //�������λ	
	results2 = (results >> 8) & 0x0000ff;  //�����м�λ
	results3 = results & 0x0000ff;		   //���͵�λ
}


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
}

/*
*********************************************************************************************************
*	�� �� ��: ADS1256_SetDiffChannal
*	����˵��: ���ò��ͨ���š���·���á�
*	��    ��: _ch : ͨ����,0-3����4��
*	�� �� ֵ: 8bit״̬�Ĵ���ֵ�ĸ�4λ
*********************************************************************************************************
*/
void ADS1256_SetDiffChannal(uint8_t _ch)
{

	if (_ch == 0)
	{
		ADS1256WREG(ADS1256_MUX, (0 << 4) | 1);	//������� AIN0�� AIN1 
	}
	else if (_ch == 1)
	{

		ADS1256WREG(ADS1256_MUX, (2 << 4) | 3);	//������� AIN2�� AIN3 
	}
	else if (_ch == 2)
	{
		ADS1256WREG(ADS1256_MUX, (4 << 4) | 5);	//������� AIN4�� AIN5 
	}
	else if (_ch == 3)
	{
		ADS1256WREG(ADS1256_MUX, (6 << 4) | 7);	//������� AIN6�� AIN7
	}


	tabb[0] = ADS1256RREG(ADS1256_MUX);
	tabb[0] = ADS1256RREG(ADS1256_MUX);
}


void ADS1256_SetDiffChannal2(uint8_t _ch)
{
	ADS_CS_LOW();
	while (ADS_DRDY);
	SPI_SendByte(ADS1256_CMD_WREG | 0x01);
	SPI_SendByte(0);
	Delay(100);
	if (_ch == 0)
	{
		SPI_SendByte(0x01);
	}
	else if (_ch == 1)
	{

		SPI_SendByte(0x23);
	}
	else if (_ch == 2)
	{
		SPI_SendByte(0x45);
	}
	else if (_ch == 3)
	{
		SPI_SendByte(0x67);
	}

	//      ADS_CS_HIGH(); 


}
