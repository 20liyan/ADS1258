#include "ads1258.h"
//#include "systick.h"
#include "string.h"
#include "stm32f10x_spi.h"
#include "delay.h"
#include "stdio.h"
//#include "spi.h"


/**************************************************************
 * IO ������
 * CS----->PB10
 * START---->PB11
 * DRDY----->PB12(SPI_NSS)
 * SCLK----->PB13(SPI2_SCK)
 * DOUT---->PB14(SPI2_MISO)
 * DIN---->PB15(SPIN2_MOSI)
 * CLKSET----->PB9
 * RESET----->PB8
 * PWDN------>PB7
 **************************************************************/
 
 #define ADS1258_CS_PIN          PBout(10)

#define ADS1258_START_PIN       PBout(11)
/* SPI */
#define ADS1258_DRDY_PIN        PBin(12)
/* SPIʱ���ź� */
#define ADS1258_SCLK_PIN        PBout(13)
/* SPI����ӿڣ���ӦSPI MISO */
#define ADS1258_DOUT_PIN        PBout(14)
/* SPI-> ����ӿ� ��ӦSPI MOSI*/
#define ADS1258_DIN_PIN         PBout(15)
/* ʱ��ѡ�����ţ��͵�ƽ�������ⲿʱ�Ӿ���32.768KHz */
#define ADS1258_CLKSEL_PIN      PBout(9)
// ��λ����
#define ADS1258_RESET_PIN       PBout(8)
// 
#define ADS1258_PWDN_PIN        PBout(7)


#define DATA_READY       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
uint32_t adsvolt[30];


float ft;

float gain;
float wendu;
float vcc;
uint32_t ADIvcc;

uint32_t ADIref;
uint32_t ADIgain;
uint32_t ADItemp;
uint32_t ADIoffset;

uint8_t Status_byte = 0x00;
uint8_t channel = 0x00;

uint8_t HSB_byte = 0x00;
uint8_t MSB_byte = 0x00;
uint8_t LSB_byte = 0x00;

void ads1258_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );

    /* ��ʼ��IO �� ��SPI IO����SPI�Ѿ����� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //PB13/14/15����������� 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB

    ADS1258_CS_PIN = 1;
    ADS1258_CLKSEL_PIN = 0;
    ADS1258_RESET_PIN = 1;
    ADS1258_PWDN_PIN = 1;
    ADS1258_START_PIN = 0;
}

void ads1258_spi_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTBʱ��ʹ�� 
		RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2ʱ��ʹ�� 	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15����������� 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB

		GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15����

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ

    // moautumn...
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ

    // moautumn...
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����

    // moautumn.....
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	//SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
}

void ads1258_hw_reset(void)
{
    ADS1258_RESET_PIN = 0;
    delay_ms(1);
    ADS1258_RESET_PIN = 1;
    delay_ms(1);
}

void ads1258_init(void)
{
    ads1258_gpio_config();
    ads1258_spi_config();
		//ads1258_exit_config();
    ads1258_hw_reset();
	  //ads1258_getchipid();
}

void ads1258_getchipid(void)
{
	uint8_t cmd_readID2[2] = {0x49, 0x00}; //�����ID�Ĵ���
	uint8_t back_readID2[2] = {0x00, 0x00};

	ADS1258_CS_PIN        = 0;
	delay_us(100);
	Ads1258_TransmitReceive(cmd_readID2, back_readID2, 2);
	delay_us(100);
	ADS1258_CS_PIN        = 1;
	delay_ms(10);
	printf("id = %x\n",back_readID2[1]);
}


void ads_config(void)
{
	#if 0
	uint8_t i = 0, j = 0;
	
	#ifdef AUTOSCAN
	//uint8_t txdata[11] = {0x70, 0x06, 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan   neibu
	//uint8_t txdata[11] = {0x70, 0x16, 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan
	//uint8_t txdata[11] = {0x70, 0x16, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan
	//uint8_t txdata[11] = {0x70, 0x06, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan   �ڲ�ֱ��
	#ifdef DIFF
	//uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan   �ڲ�ֱ��  ��׼����
	uint8_t txdata[11] = {0x70, 0x12, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan   �ⲿֱ�� ��� ��׼����
	#else
	//uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x00, 0xFF, 0x03, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan   �ڲ�ֱ��  ��׼����   ��������뱣����Ŀ
	uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x00, 0x1C, 0x00, 0X3C, 0xFF, 0x00, 0x8B}; //normal-autoscan   �ڲ�ֱ��  ��׼����   ������ͨ��
	#endif
	#endif
	#ifdef SINGLE
	uint8_t txdata[11] = {0x70, 0x36, 0x01, 0x76, 0x00, 0xc0, 0x00, 0x00, 0xFF, 0x00, 0x8B}; //normal-fixed-01
	//uint8_t txdata[11] = {0x70, 0x36, 0x01, 0x10, 0x00, 0x03, 0x00, 0x00, 0xFF, 0x00, 0x8B}; //normal-fixed-01
	#endif
  //
	//uint8_t txdata[11] = {0x70, 0x36, 0x01, 0x76, 0x00, 0xc0, 0x00, 0x00, 0xFF, 0x00, 0x8B};
	//uint8_t txdata[11] = {0x70, 0x16, 0x01, 0x00, 0x00, 0x28, 0x3c, 0x00, 0xFF, 0x00, 0x8B}; //normal
	//uint8_t txdata[11] = {0x70, 0x12, 0x01, 0x00, 0x00, 0x28, 0x3c, 0x3c, 0xFF, 0x00, 0x9B};//config ref channel for test 
	uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan
	uint8_t readcmd[11] = {0}; //��00h��09h �����мĴ���
 
	uint8_t rxdata[11] = {0}; //�ض�10���Ĵ��������ݣ��ж������Ƿ�ɹ�
 
	//����10���Ĵ���
	ADS1258_CS_PIN = 0;//ADS1258_CS_LOW;
	delay_us(20);
	Ads1258_TransmitReceive(txdata, rxdata, 11);
	delay_us(20);
	ADS1258_CS_PIN = 1;
	delay_ms(10);
 
	//�ض�10���Ĵ���
	readcmd[0] = 0x50; //��00h��09h �����мĴ���
 
	ADS1258_CS_PIN = 0;
	delay_us(10);
	Ads1258_TransmitReceive(readcmd, rxdata, 11);
	delay_us(10);
	ADS1258_CS_PIN = 1;
	delay_ms(10);
	
	printf("tx data is %x %x %x %x %x %x %x %x %x %x\r\n",txdata[1],txdata[2],txdata[3],txdata[4],txdata[5],txdata[6],txdata[7],txdata[8],txdata[9],txdata[10]);
	printf("rx data is %x %x %x %x %x %x %x %x %x %x\r\n",rxdata[1],rxdata[2],rxdata[3],rxdata[4],rxdata[5],rxdata[6],rxdata[7],rxdata[8],rxdata[9],rxdata[10]);
	for (i = 1; i < 9; i++)
	{
		if (txdata[i] == rxdata[i])
			j++;
	}
	if (j == 8)
	{
		//����ok
		printf("ads1258 set up ok\r\n");
	}
	#endif
	//д����Ĵ���������ʱ���Զ�ɨ�衢�������ʡ����ù̶������ò�֡����õ�ͨ����
	uint8_t txdata[8] = {0x70, 0x02, 0xF0, 0x00, 0x00, 0xFF,0xFF,0x3D};
	uint8_t rxdata[8] = {0x00};
	ADS1258_START_PIN = 0;
	delay_us(10);
	Ads1258_TransmitReceive(txdata, rxdata, 8);
	delay_us(10);
	ADS1258_START_PIN = 1;
}

u16 dataChannelFlag = 0;
float vTemp;

u8 REG_readdata(void)
{
 
	u8 ret = 0;
	double x[16];
	double DIFF[6];
	//int32_t adccode = 0;
	//uint8_t REG_cmddata[5] = {0x30, 0x00, 0x00, 0x00,0x00}; //ʹ�üĴ�����ʽ ��ȡͨ������
	//uint8_t REG_recdata[5] = {0x00, 0x00, 0x00, 0x00,0x00}; //ʹ�üĴ�����ʽ ��ȡͨ������
	uint8_t REG_cmddata[4] = {0x00,0x00, 0x00, 0x00}; //ʹ�üĴ�����ʽ ��ȡͨ������
	uint8_t REG_recdata[4] = {0x00, 0x00, 0x00,0x00}; //ʹ�üĴ�����ʽ ��ȡͨ������
 
	ADS1258_CS_PIN = 0;
	delay_us(1);
	
	Ads1258_TransmitReceive(REG_cmddata, REG_recdata,4);
 
	delay_us(1);
	ADS1258_CS_PIN = 1;

	
	Status_byte = REG_recdata[0];
	channel = Status_byte & 0x1f;
	//if(channel<24)
		//channel = channel - 0x08;
	//printf("channel = %d\n",channel);
	
	HSB_byte = REG_recdata[1];
	MSB_byte = REG_recdata[2];
	LSB_byte = REG_recdata[3];
 
 
	switch(channel)
	{
		case 0:
		adsvolt[0] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 1:
		adsvolt[1] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 2:
		adsvolt[2] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 3:
		adsvolt[3] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 4:
		adsvolt[4] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 5:
		adsvolt[5] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 6:
		adsvolt[6] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 7:
		adsvolt[7] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 8:
		adsvolt[8] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 9:
		adsvolt[9] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[0] = 2.5/0x780000*adsvolt[9];
		break;
		case 10:
		adsvolt[10] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[1] = 2.5/0x780000*adsvolt[10];
		DIFF[0] = x[1]-x[0];
		printf("DIFF[0] = %f\n",DIFF[0]);
		break;
		case 11:
		adsvolt[11] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[2] = 2.5/0x780000*adsvolt[11];
		break;
		case 12:
		adsvolt[12] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[3] = 2.5/0x780000*adsvolt[12];
		DIFF[1] = x[3]-x[2];
		printf("DIFF[1] = %f\n",DIFF[1]);
		break;
		case 13:
		adsvolt[13] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[4] = 2.5/0x780000*adsvolt[13];
		break;
		case 14:
		adsvolt[14] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[5] = 2.5/0x780000*adsvolt[14];
		DIFF[2] = x[5]-x[4];
		printf("DIFF[2] = %f\n",DIFF[2]);
		break;
		case 15:
		adsvolt[15] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[6] = 2.5/0x780000*adsvolt[15];
		break;
		case 16:
		adsvolt[16] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 17:
		adsvolt[17] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[7] = 2.5/0x780000*adsvolt[17];
		DIFF[3] = x[7]-x[6];
		printf("DIFF[3] = %f\n",DIFF[3]);
		break;
		case 18:
		adsvolt[18] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[8] = 2.5/0x780000*adsvolt[18];
		break;
		case 19:
		adsvolt[19] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[9] = 2.5/0x780000*adsvolt[19];
		DIFF[4] = x[9]-x[8];
		printf("DIFF[4] = %f\n",DIFF[4]);
		break;
		case 20:
		adsvolt[20] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[10] = 2.5/0x780000*adsvolt[20];
		break;
		case 21:
		adsvolt[21] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		x[11] = 2.5/0x780000*adsvolt[21];
		DIFF[5] = x[11]-x[10];
		printf("DIFF[5] = %f\n",DIFF[5]);
		break;
		case 22:
		adsvolt[22] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 23:
		adsvolt[23] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 24:
		adsvolt[24] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 25:
		adsvolt[25] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 26:
		adsvolt[26] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 27:
		adsvolt[27] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 28:
		adsvolt[28] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 29:
		adsvolt[29] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		#if 0//
		case 24:
			ADIoffset = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
			break;
		case 26:
			ADIvcc = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		  vcc = ADIvcc/786432.0;
		  vcc = vcc + BASE;
			break;
		case 27:
			ADItemp = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		  if(ADItemp & 0x800000)
			{
				wendu = ((float)(0xffffff - ADItemp)/(float)0x780000)*ft;
				wendu = -vTemp;
			}
			else
			{
				wendu = ((float)ADItemp/(float)0x780000)*ft;
			}
			wendu = (wendu * 1000000 - 168000) / 563 + 25;
			break;
		case 28:
			ADIgain = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		  gain = ADIgain/7864320.0;
			break;
		case 29:
			ADIref = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		  ft = ADIref/786432.0;
			break;
			#endif
		default:
			break;
	}
	#if 0
	if(channel < 9)
	{
		if(adsvolt[channel] & 0x800000)
		{
			vTemp = ((float)(0xffffff - adsvolt[channel])/(float)0x780000)*ft/gain;
			vTemp = -vTemp;
		}
		else
		{
			vTemp = ((float)adsvolt[channel]/(float)0x780000)*ft/gain;
		}
		
 #if 0
		if(4 == channel)
			dataInfoCan.voltCanSend = vTemp * 50;//V*10
		else if(2 == channel)
		{
			//vTemp = vTemp - 0.613;
		  if(vTemp<0)
				vTemp = -vTemp;
			dataInfoCan.currentLCanSend = vTemp/0.006;//100MA
			if(dataInfoCan.currentLCanSend<4)
				dataInfoCan.currentLCanSend = 0;
		}
		else if(3 == channel)
		{
			//vTemp = vTemp - 0.613;
		  if(vTemp<0)
				vTemp = -vTemp;
			
			dataInfoCan.currentRCanSend = vTemp/0.006;//100MA
			if(dataInfoCan.currentRCanSend<4)
				dataInfoCan.currentRCanSend = 0;
		}
 
#endif		
		#ifdef PRINTF
		printf("%d:%f  ",channel,vTemp);
		#endif
	}
 
	//printf("channel %d is %x:%f\r\n",channel,adsvolt[channel],vTemp);
	if(channel ==4)
	{
		#if 1//#ifdef PRINTF
		printf("ref:%f  ",ft);
		printf("temp:%f  ",wendu);
		printf("vcc:%f  ",vcc);
		printf("gain:%f    ",gain);
		//printf("offset:%d  ",ADIoffset);
		//printf("totol time is %.2f ms",(10000-timestamp1)*0.1);
		printf("\r\n\r\n");
		#endif
		return 1;
	}
 #endif
}

void ads_Pulse(void)
{
	ADS1258_CS_PIN = 0;
	ADS1258_START_PIN = 1;
	delay_us(5);
	ADS1258_START_PIN = 0;
	ADS1258_CS_PIN = 1;
}
void readADS(void)
{
//		if(_loop_count == 0)
//		{
			 int timeout = 10000;
//			 timestamp1 = 10000;
//			 _loop_count = LOOP_DELAY;
			 ads_Pulse();
			 
			 while(timeout)
			 {
					if (DATA_READY == 0)
					{
						if (REG_readdata() == 1) 
						{
						//	timeout = 0;
							break;
						}
						ads_Pulse();
					}
					timeout--;
			 }
		//}
}
		
//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����					    
}

void Ads1258_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	unsigned int i = 0;
	for (i = 0; i < Size; i++)
	{
		pRxData[i] = SPI2_ReadWriteByte(pTxData[i]);
	}
}
