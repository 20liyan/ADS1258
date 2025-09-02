#ifndef __ADS1258_H
#define __ADS1258_H
#include "sys.h"
#define BASE 0.06
void ads1258_gpio_config(void);
void ads1258_spi_config(void);
void ads1258_hw_reset(void);
void ads1258_init(void);
void ads1258_getchipid(void);
void ads_config(uint8_t mux0, uint8_t mux1);
u8 SPI2_ReadWriteByte(u8 TxData);
void Ads1258_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void readADS(void);
void REG_readdata(void);
static inline int32_t signext24(uint32_t v24);
void ads_send_one_row_csv(void);

#endif
