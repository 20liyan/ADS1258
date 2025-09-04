#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd_init.h"
#include "lcd.h"
#include "ads1258.h"
extern uint32_t adsvolt[30];
 int main(void)
 {		
	delay_init();	    	  
	uart_init(115200);	 
	LCD_Init();
	LCD_Fill(0,0,LCD_W,LCD_H,GRED);
	ads1258_init();
	delay_ms(100);
	ads_config(0x00, 0x00);
	delay_ms(100);
 	while(1)
	{
		ads_send_one_row_csv();
		delay_ms(100);
	}	 
 }

