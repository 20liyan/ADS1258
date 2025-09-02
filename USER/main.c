#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "beep.h"
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
#include "ads1258.h"
extern uint32_t adsvolt[30];
 int main(void)
 {		
	delay_init();	    	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	uart_init(115200);	 
 	LED_Init();			    
	KEY_Init();  
	BEEP_Init();
	 LCD_Init();   //LCD≥ı ºªØ
	LCD_Fill(0,0,LCD_W,LCD_H,GRED);
	ads1258_init();
	delay_ms(100);
	ads1258_getchipid();
	printf("ads1258_init...\n");

	delay_ms(100);
	ads_config();
	delay_ms(100);
	//printf("%d\n",REG_readdata());
 	while(1)
	{
//		LCD_ShowString(0,0,"AIN0:",BLUE,GRED,12,0);
		readADS();
//		delay_ms(500);
	}	 
 }

