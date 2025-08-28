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
		LCD_ShowString(0,0,"AIN0:",BLUE,GRED,12,0);
		LCD_ShowString(0,12,"AIN1:",BLUE,GRED,12,0);
		LCD_ShowString(0,24,"AIN2:",BLUE,GRED,12,0);
		LCD_ShowString(0,36,"AIN3:",BLUE,GRED,12,0);
		LCD_ShowString(0,48,"AIN4:",BLUE,GRED,12,0);
		LCD_ShowString(0,60,"AIN5:",BLUE,GRED,12,0);
		LCD_ShowString(0,72,"AIN6:",BLUE,GRED,12,0);
		LCD_ShowString(0,84,"AIN7:",BLUE,GRED,12,0);
		LCD_ShowString(0,96,"AIN8:",BLUE,GRED,12,0);
		LCD_ShowString(0,108,"AIN9:",BLUE,GRED,12,0);
		LCD_ShowString(0,120,"AIN10:",BLUE,GRED,12,0);
		LCD_ShowString(0,132,"AIN11:",BLUE,GRED,12,0);
		LCD_ShowString(0,144,"AIN12:",BLUE,GRED,12,0);
		readADS();
		LCD_ShowFloatNum1(45,0,2.5/0x780000*adsvolt[8],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,12,2.5/0x780000*adsvolt[9],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,24,2.5/0x780000*adsvolt[10],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,36,2.5/0x780000*adsvolt[11],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,48,2.5/0x780000*adsvolt[12],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,60,2.5/0x780000*adsvolt[13],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,72,2.5/0x780000*adsvolt[14],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,84,2.5/0x780000*adsvolt[15],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,96,2.5/0x780000*adsvolt[16],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,108,2.5/0x780000*adsvolt[17],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,120,2.5/0x780000*adsvolt[18],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,132,2.5/0x780000*adsvolt[19],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,144,2.5/0x780000*adsvolt[20],7,BLUE,GRED,12);
		delay_ms(500);
		LCD_Fill(0,0,LCD_W,LCD_H,GRED);
		LCD_ShowString(0,0,"AIN13:",BLUE,GRED,12,0);
		LCD_ShowString(0,12,"AIN14:",BLUE,GRED,12,0);
		LCD_ShowString(0,24,"AIN15:",BLUE,GRED,12,0);
		LCD_ShowFloatNum1(45,0,2.5/0x780000*adsvolt[21],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,12,2.5/0x780000*adsvolt[22],7,BLUE,GRED,12);
		LCD_ShowFloatNum1(45,24,2.5/0x780000*adsvolt[23],7,BLUE,GRED,12);
		delay_ms(500);
		LCD_Fill(0,0,LCD_W,LCD_H,GRED);
	}	 
 }

