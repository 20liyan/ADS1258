#include "ads1258.h"
#include "string.h"
#include "stm32f10x_spi.h"
#include "delay.h"
#include "stdio.h"

/**************************************************************
 * IO 口配置
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
/* SPI时钟信号 */
#define ADS1258_SCLK_PIN        PBout(13)
/* SPI输出接口：对应SPI MISO */
#define ADS1258_DOUT_PIN        PBout(14)
/* SPI-> 输入接口 对应SPI MOSI*/
#define ADS1258_DIN_PIN         PBout(15)
/* 时钟选择引脚，低电平：采用外部时钟晶振32.768KHz */
#define ADS1258_CLKSEL_PIN      PBout(9)
// 复位引脚
#define ADS1258_RESET_PIN       PBout(8)
// 
#define ADS1258_PWDN_PIN        PBout(7)


#define DATA_READY       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
uint32_t adsvolt[30];
uint8_t txdata[8];

uint8_t Status_byte = 0x00;
uint8_t channel = 0x00;
uint8_t HSB_byte = 0x00;
uint8_t MSB_byte = 0x00;
uint8_t LSB_byte = 0x00;

void ads1258_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );

    /* 初始化IO 口 ，SPI IO口在SPI已经定义 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //PB13/14/15复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

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

    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTB时钟使能 
		RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2时钟使能 	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB
	// MISO 单独配置为输入浮空（或上拉）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 或 GPIO_Mode_IPU if you want pull-up
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15上拉
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 若板上没有外部上拉，建议使用输入上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构

    // moautumn...
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平

    // moautumn...
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制

    // moautumn.....
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	//SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
 
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
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
	uint8_t cmd_readID2[2] = {0x49, 0x00}; //命令读ID寄存器
	uint8_t back_readID2[2] = {0x00, 0x00};

	ADS1258_CS_PIN        = 0;
	delay_us(100);
	Ads1258_TransmitReceive(cmd_readID2, back_readID2, 2);
	delay_us(100);
	ADS1258_CS_PIN        = 1;
	delay_ms(10);
	printf("id = %x\n",back_readID2[1]);
}


void ads_config(uint8_t mux0, uint8_t mux1)
{
	#if 0
	uint8_t i = 0, j = 0;
	
	#ifdef AUTOSCAN
	//uint8_t txdata[11] = {0x70, 0x06, 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan   neibu
	//uint8_t txdata[11] = {0x70, 0x16, 0x01, 0x00, 0x00, 0xFF, 0x03, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan
	//uint8_t txdata[11] = {0x70, 0x16, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan
	//uint8_t txdata[11] = {0x70, 0x06, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x8B}; //normal-autoscan   内部直连
	#ifdef DIFF
	//uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan   内部直连  基准计算
	uint8_t txdata[11] = {0x70, 0x12, 0x01, 0x00, 0x1f, 0x00, 0x00, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan   外部直连 差分 基准计算
	#else
	//uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x00, 0xFF, 0x03, 0x3d, 0xFF, 0x00, 0x8B}; //normal-autoscan   内部直连  基准计算   用在天津与保定项目
	uint8_t txdata[11] = {0x70, 0x02, 0x01, 0x00, 0x00, 0x1C, 0x00, 0X3C, 0xFF, 0x00, 0x8B}; //normal-autoscan   内部直连  基准计算   用三个通道
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
	}
	#endif
//写7个寄存器，设置时钟自动扫描、设置速率、禁用固定、禁用差分、启用单通道、
	uint8_t txdata[8] = {0x70, 0x02, 0xF0, 0x00, 0x00, mux0,mux1,0x00}; 
	uint8_t rxdata[8] = {0};
//配置7个寄存器
	ADS1258_CS_PIN = 0;//ADS1258_CS_LOW;
	delay_us(20);
	Ads1258_TransmitReceive(txdata, rxdata, 8);
	delay_us(20);
	ADS1258_CS_PIN = 1;
	delay_ms(10);
	ADS1258_START_PIN = 1;
 
//	回读7个寄存器
//	uint8_t readcmd[8] = {0}; //回读7个寄存器的数据，判断设置是否成功
//	readcmd[0] = 0x50; //从00h到06h 读所有寄存器
//	ADS1258_CS_PIN = 0;
//	delay_us(10);
//	Ads1258_TransmitReceive(readcmd, rxdata, 8);
//	delay_us(10);
//	ADS1258_CS_PIN = 1;
//	delay_ms(10);	
//	printf("tx data is %x %x %x %x %x %x %x \r\n",txdata[1],txdata[2],txdata[3],txdata[4],txdata[5],txdata[6],txdata[7]);
//	printf("rx data is %x %x %x %x %x %x %x \r\n",rxdata[1],rxdata[2],rxdata[3],rxdata[4],rxdata[5],rxdata[6],rxdata[7]);
}

void REG_readdata(void)
{
	uint8_t REG_cmddata[4] = {0x00,0x00,0x00,0x00}; //使用寄存器方式 读取通道数据
	uint8_t REG_recdata[4] = {0x00,0x00,0x00,0x00}; //使用寄存器方式 读取通道数据
	ADS1258_CS_PIN = 0;
	delay_us(2);	
	Ads1258_TransmitReceive(REG_cmddata, REG_recdata,4);
	delay_us(2);
	ADS1258_CS_PIN = 1;
	
	Status_byte = REG_recdata[0];
	channel = Status_byte & 0x1f;	
	HSB_byte = REG_recdata[1];
	MSB_byte = REG_recdata[2];
	LSB_byte = REG_recdata[3];
  
	switch(channel)
	{
		case 8:
		adsvolt[8] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 9:
		adsvolt[9] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 10:
		adsvolt[10] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 11:
		adsvolt[11] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 12:
		adsvolt[12] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 13:
		adsvolt[13] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 14:
		adsvolt[14] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 15:
		adsvolt[15] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 16:
		adsvolt[16] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 17:
		adsvolt[17] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 18:
		adsvolt[18] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 19:
		adsvolt[19] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 20:
		adsvolt[20] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
		case 21:
		adsvolt[21] = (HSB_byte<<16) | (MSB_byte<<8) | LSB_byte;
		break;
	}
}

void readADS(void)
{
	int timeout = 10000;
    while (timeout--) {
        if (DATA_READY == 0) { // 等待DRDY拉低（数据就绪）
            REG_readdata();
            break;
        }
        delay_us(1);
    }
    if (timeout == 0) {
        printf("DRDY timeout!\r\n"); // 调试：判断是否通信错误
    }
}
	
static inline int32_t signext24(uint32_t v24)
{
    if (v24 & 0x800000) v24 |= 0xFF000000; // 24-bit two's complement -> 32-bit
    return (int32_t)v24;
}

void ads_send_one_row_csv(void)
{
    // 目标通道顺序（与 Excel AIN0..AIN11 对应）
    uint8_t channels[] = {1,2,3,4,5,6,7,9,10,11,12,13};
    int32_t row[12];
    for (int i = 0; i < 12; i++) row[i] = 0; // 可按需设初值
    for (int i = 0; i < 12; i++)
    {
        uint8_t ch = channels[i];
        uint8_t mux0 = 0, mux1 = 0;
        if (ch < 8) mux0 = (1u << ch);         // AIN1..AIN7
        else        mux1 = (1u << (ch - 8));   // AIN9..AIN13
        // 配置该单端通道
        ads_config(mux0, mux1);
        // 等待 DRDY 并读出 1 组数据（会更新全局：Status_byte/HSB/MSB/LSB/channel）
        readADS();
        // 只接收我们关心的 12 个通道
        int idx = -1;
        if (channel >= 9 && channel <= 15) {          // AIN1..AIN7
            idx = (int)(channel - 9);                 // 0..6
        } else if (channel >= 17 && channel <= 21) {  // AIN9..AIN13
            idx = 7 + (int)(channel - 17);            // 7..11
        } else {
            // 跳过非目标通道（通常不会发生）
            continue;
        }
        // 24-bit 组装并符号扩展
        uint32_t v24 = ((uint32_t)adsvolt[channel]); 
        row[idx] = signext24(v24);
    }
    // 以 CSV 形式发出一整行（12 列，逗号分隔，行尾 \r\n）
    printf("%ld", (long)row[0]);
    for (int i = 1; i < 12; i++) {
        printf(",%ld", (long)row[i]);
    }
    printf("\r\n");
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据					    
}

void Ads1258_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	unsigned int i = 0;
	for (i = 0; i < Size; i++)
	{
		pRxData[i] = SPI2_ReadWriteByte(pTxData[i]);
	}
}
