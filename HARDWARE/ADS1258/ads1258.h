#ifndef __ADS1258_H
#define __ADS1258_H
#include "sys.h"
#define BASE 0.06
void ads1258_gpio_config(void);
void ads1258_spi_config(void);
void ads1258_hw_reset(void);
void ads1258_init(void);
void ads1258_getchipid(void);
void ads_config(void);
u8 SPI2_ReadWriteByte(u8 TxData);
void Ads1258_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
uint8_t REG_readdata(void);
static uint8_t uart_dma_get_free_buffer(void);
static void uart_dma_start(uint8_t buffer_index);
static void uart_dma_try_start(void);
void ads1258_uart_dma_init(void);
void DMA1_Channel4_IRQHandler(void);
static uint8_t uart_prepare_new_frame(void);
static void adc_frame_reset(void);
static void adc_frame_init(void);
void ads1258_task(void);
static inline int32_t signext24(uint32_t v24);
#endif
