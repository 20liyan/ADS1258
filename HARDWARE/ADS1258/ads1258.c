#include "ads1258.h"
#include "string.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "delay.h"
#include "stdio.h"
#include <stdint.h>
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
/* 时钟选择引脚，低电平：采用外部时钟晶振32.768KHz 2^15 */
#define ADS1258_CLKSEL_PIN      PBout(9)
// 复位引脚
#define ADS1258_RESET_PIN       PBout(8)
#define ADS1258_PWDN_PIN        PBout(7)
#define DATA_READY       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
uint32_t adsvolt[12];
uint8_t txdata[10];
static uint8_t adc_got[12];
static uint8_t adc_collect_count = 0;
//ADC原始数据临时帧
static uint8_t adc_frame_data[36];
#define UART_FRAME_LENGTH       38
#define UART_TX_BUFFER_NUM      2
static uint8_t uart_tx_buffer[UART_TX_BUFFER_NUM][UART_FRAME_LENGTH];//UART DMA双缓冲
static volatile uint8_t uart_tx_ready[UART_TX_BUFFER_NUM];//UART DMA缓冲区状态,0空闲,1已经准备好
static volatile uint8_t uart_dma_buffer_index = 0;//DMA当前正在发送哪个buffer
static volatile uint8_t uart_dma_busy = 0;//DMA是否正在发送
volatile uint32_t uart_frame_drop_count = 0;//当前ADC采集完成后，统计丢帧次数,正常情况下应该一直为0
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB
    ADS1258_CS_PIN = 1;//默认不选中
    ADS1258_CLKSEL_PIN = 0;//使用 ADC 内部时钟
    ADS1258_RESET_PIN = 1;
    ADS1258_PWDN_PIN = 1;//退出掉电，上电工作
    ADS1258_START_PIN = 0;//默认不启动 ADC 转换
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
	// MISO 单独配置为浮空输入（或上拉）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 或 GPIO_Mode_IPU
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15上拉	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 若板上没有外部上拉，建议使用输入上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为256
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
	ADS1258_CS_PIN = 0;
	delay_us(100);
	Ads1258_TransmitReceive(cmd_readID2, back_readID2, 2);
	delay_us(100);
	ADS1258_CS_PIN = 1;
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
	/*ADS1258寄存器配置
     * 0x70：写寄存器命令
     * 后面7个字节对应ADS1258寄存器配置。
     * 当前采用Auto-Scan模式。*/
    uint8_t txdata[10] =
	{
		0x70,   // Command: Write Multiple, start at 00h
		0x06,   // 00h CONFIG0
		0x00,   // 01h CONFIG1
		0x00,   // 02h MUXSCH
		0x00,   // 03h MUXDIF
		0xFE,   // 04h MUXSG0
		0x3E,   // 05h MUXSG1
		0x00,   // 06h SYSRED
		0xFF,   // 07h GPIOC
		0x00    // 08h GPIOD	
	};
    uint8_t rxdata[10] = {0};
    //CS拉低
    ADS1258_CS_PIN = 0;
    delay_us(20);
    //写入ADS1258寄存器
    Ads1258_TransmitReceive(txdata, rxdata, 10);
    //CS拉高
    delay_us(20);
    ADS1258_CS_PIN = 1;
    //等待寄存器配置完成
    delay_ms(10);
    //START拉高,使ADS1258开始转换/扫描
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
/**************************************************************
 * SPI2读写一个字节
 * TxData：要发送的数据
 * 返回值：从SPI2读取到的数据
 **************************************************************/
u8 SPI2_ReadWriteByte(u8 TxData)
{
    uint16_t retry = 0;
    //等待发送缓冲区空
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }
    //发送数据
    SPI_I2S_SendData(SPI2, TxData);
    retry = 0;
    // 等待接收数据
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }
    //返回接收到的数据
    return SPI_I2S_ReceiveData(SPI2);
}
//SPI2连续发送并接收
void Ads1258_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    uint16_t i = 0;
    for (i = 0; i < Size; i++)
    {
        pRxData[i] = SPI2_ReadWriteByte(pTxData[i]);
    }
}
/**********************************************************************
 * 读取ADS1258当前转换数据
 * ADS1258返回：
 * Byte0 = Status
 * Byte1 = ADC[23:16]
 * Byte2 = ADC[15:8]
 * Byte3 = ADC[7:0]
 **********************************************************************/
uint8_t REG_readdata(void)
{
    uint8_t REG_recdata[4];
    uint32_t raw24;
    uint8_t channel_index;
    ADS1258_CS_PIN = 0;//CS拉低
    delay_us(2);
    //连续读取4 Byte,发送0x00作为dummy
    REG_recdata[0] = SPI2_ReadWriteByte(0x00);
    REG_recdata[1] = SPI2_ReadWriteByte(0x00);
    REG_recdata[2] = SPI2_ReadWriteByte(0x00);
    REG_recdata[3] = SPI2_ReadWriteByte(0x00);
    delay_us(2);
    ADS1258_CS_PIN = 1;
    Status_byte = REG_recdata[0];//解析Status
    channel = Status_byte & 0x1F;
    //ADC 24bit数据
    HSB_byte = REG_recdata[1];
    MSB_byte = REG_recdata[2];
    LSB_byte = REG_recdata[3];
    raw24 = ((uint32_t)HSB_byte << 16) | ((uint32_t)MSB_byte << 8) | ((uint32_t)LSB_byte);
    /***********************
     * channel → 12通道index
     * channel 9  → AIN1  → 0
     * channel 10 → AIN2  → 1...
     * channel 15 → AIN7  → 6
     * channel 17 → AIN9  → 7...
     * channel 21 → AIN13 → 11
     ***********************/
    channel_index = 0xFF;
    if (channel >= 9 && channel <= 15)
    {
        channel_index = channel - 9;
    }
    else if (channel >= 17 && channel <= 21)
    {
        channel_index = 7 + (channel - 17);
    }
    if (channel_index < 12)//如果是目标通道
    {
        adsvolt[channel_index] = raw24;
        if (adc_got[channel_index] == 0)//当前组还没有采集过
        {
            //写入当前36 Byte ADC帧
            adc_frame_data[channel_index * 3 + 0] = (uint8_t)((raw24 >> 16) & 0xFF);
            adc_frame_data[channel_index * 3 + 1] = (uint8_t)((raw24 >> 8) & 0xFF);
            adc_frame_data[channel_index * 3 + 2] = (uint8_t)(raw24 & 0xFF);
            adc_got[channel_index] = 1;//标记该通道已经采集
            adc_collect_count++;//已完成通道数量 +1
        }
    }
    return 1;
}
/**********************************************************************
 * UART DMA：寻找空闲buffer
 * 返回：
 * 0 / 1 -> 可以使用
 * 0xFF  -> 没有空闲buffer
 **********************************************************************/
static uint8_t uart_dma_get_free_buffer(void)
{
    uint8_t i;
    for (i = 0; i < UART_TX_BUFFER_NUM; i++)
    {
        /* 不能使用：
         * 1. DMA当前正在发送
         * 2. buffer已经处于ready状态*/
        if (i != uart_dma_buffer_index || uart_dma_busy == 0)
        {
            if (uart_tx_ready[i] == 0)
            {
                return i;
            }
        }
    }
    return 0xFF;
}
/**********************************************************************
 * 启动UART DMA发送
 * USART1_TX
 * DMA1_Channel4
 **********************************************************************/
static void uart_dma_start(uint8_t buffer_index)
{
    if (uart_dma_busy)//防止DMA正在工作时重新启动
    {
        return;
    }
    uart_dma_buffer_index = buffer_index;//记录当前DMA使用哪个buffer
    uart_dma_busy = 1;//标记DMA忙
    DMA_Cmd(DMA1_Channel4, DISABLE);//关闭DMA
    DMA_ClearFlag(DMA1_FLAG_GL4 | DMA1_FLAG_TC4 | DMA1_FLAG_HT4 | DMA1_FLAG_TE4);//清除DMA标志
    DMA1_Channel4->CMAR = (uint32_t) &uart_tx_buffer[buffer_index][0];//设置内存地址
    DMA1_Channel4->CNDTR = UART_FRAME_LENGTH;//设置发送长度
    uart_tx_ready[buffer_index] = 0;//该buffer已经被DMA占用
    DMA_Cmd(DMA1_Channel4, ENABLE);//开启DMA
}
//如果DMA空闲，则寻找ready buffer并启动发送
static void uart_dma_try_start(void)
{
    uint8_t i;
    if (uart_dma_busy)
    {
        return;
    }
    for (i = 0; i < UART_TX_BUFFER_NUM; i++)
    {
        if (uart_tx_ready[i])
        {
            uart_dma_start(i);

            return;
        }
    }
}
/**********************************************************************
 * UART1 TX DMA初始化
 * USART1_TX -> DMA1_Channel4
 **********************************************************************/
void ads1258_uart_dma_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//开启DMA1时钟
    DMA_DeInit(DMA1_Channel4);//DMA1 Channel4复位
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;//DMA配置
    //初始内存地址,实际发送时会重新修改CMAR
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&uart_tx_buffer[0][0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//内存 → 外设
    DMA_InitStructure.DMA_BufferSize = UART_FRAME_LENGTH;//初始发送38 Byte
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//USART DR地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据宽度：8bit
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据宽度：8bit
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//ormal模式,发送38 Byte后自动停止
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//不使用Memory-to-Memory
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);//DMA传输完成中断
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//USART1允许TX DMA请求
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;//NVIC
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    uart_tx_ready[0] = 0;//初始化状态
    uart_tx_ready[1] = 0;
    uart_dma_busy = 0;
    uart_dma_buffer_index = 0;
    uart_frame_drop_count = 0;
}
/**********************************************************************
 * DMA1 Channel4中断
 * USART1_TX DMA发送完成
 **********************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);//清除TC标志
        DMA_Cmd(DMA1_Channel4, DISABLE);//关闭DMA
        uart_dma_busy = 0;//DMA发送完成
        uart_dma_try_start();//查看是否还有另一个buffer,等待发送
    }
}
/**********************************************************************
 * 将当前36 Byte ADC数据复制到UART空闲buffer
 * 返回：
 * 1 = 成功
 * 0 = 没有空闲buffer
 **********************************************************************/
static uint8_t uart_prepare_new_frame(void)
{
    uint8_t buffer_index;
    buffer_index = uart_dma_get_free_buffer();//找空闲buffer
    if (buffer_index == 0xFF)
    {
        //两个buffer都被占用,正常情况下不会发生。
        uart_frame_drop_count++;
        return 0;
    }
    uart_tx_buffer[buffer_index][0] = 0xAA;//包头
    uart_tx_buffer[buffer_index][1] = 0x55;
    memcpy(&uart_tx_buffer[buffer_index][2], adc_frame_data, 36);//复制36 Byte ADC数据
    uart_tx_ready[buffer_index] = 1;//标记该buffer准备好了
    uart_dma_try_start();//如果DMA空闲,立即开始发送
    return 1;
}
//ADC采集状态复位
static void adc_frame_reset(void)
{
    uint8_t i;
    adc_collect_count = 0;
    for (i = 0; i < 12; i++)
    {
        adc_got[i] = 0;
    }
}
//初始化ADC采集帧
static void adc_frame_init(void)
{
    memset(adc_frame_data, 0, sizeof(adc_frame_data));
    adc_frame_reset();
}
/**********************************************************************
 * ADS1258数据采集任务
 * 每次调用只检查一次DRDY。
 * 因此主循环可以不断调用：
 * CPU可以在ADC采集间隙处理其他事情。
 **********************************************************************/
void ads1258_task(void)
{
    if (DATA_READY != 0)//DRDY还没有拉低,直接返回,不阻塞CPU
    {
        return;
    }
    //读取一次ADC数据,REG_readdata()内部：,Status + 24bit ADC
    REG_readdata();
    if (adc_collect_count >= 12)//如果12个目标通道全部采集完成
    {
        uart_prepare_new_frame();//将当前36 Byte数据,放入UART空闲buffer
        adc_frame_reset();//开始下一组12通道采集
    }
}
/**********************************************************************
 * 24bit ADC数据符号扩展
 * ADS1258：
 * bit23 = 1 -> 负数
 * bit23 = 0 -> 正数
 **********************************************************************/
static inline int32_t signext24(uint32_t v24)
{
    if (v24 & 0x800000)
    {
        v24 |= 0xFF000000;
    }
    return (int32_t)v24;
}
