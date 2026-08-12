#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd_init.h"
#include "lcd.h"
#include "ads1258.h"
extern uint32_t adsvolt[12];
int main(void)
 {		
	delay_init();	    	  
	uart_init(576000);	 
	LCD_Init();
	LCD_Fill(0,0,LCD_W,LCD_H,GRED);
	ads1258_init();
	delay_ms(100);
	ads_config();
	delay_ms(100);
	ads1258_uart_dma_init();
 	while(1)
	{
		ads1258_task();
	}	 
 }

