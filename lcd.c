////=========================================================
///		Code by: Muneeb Abid
//		Dated: 27 Feb 2015
///		Description: 4-bit operation of 16x2 Alphanumeric LCD 
////=========================================================


#include <stdint.h>
#include "inc/tm4c123gh6pge.h"
#include "inc/lcd.h"
#include "inc/SysTick.h"

void lcd_putch(char c)
{
  LCD_RS(1); 
  
  delay_ms(2);       
  
  LCD_DATA = (LCD_DATA&0x0F)|(c & 0xF0);
  LCD_EN(1);
	delay_ms(2);
	LCD_EN(0);
  LCD_DATA =  (LCD_DATA&0x0F)|((c & 0x0F) << 4);
  LCD_EN(1);
	delay_ms(2);
	LCD_EN(0);
}

void lcd_command(unsigned char c)
{
  LCD_RS(0);
  
  delay_ms(2);      
  
  LCD_DATA = (LCD_DATA&0x0F)|(c & 0xF0);
  LCD_EN(1);
	delay_ms(2);
	LCD_EN(0);
	delay_ms(2);
  LCD_DATA =  (LCD_DATA&0x0F)|((c & 0x0F) << 4);
  LCD_EN(1);
	delay_ms(2);
	LCD_EN(0);
	delay_ms(2);
}

void lcd_cls(void)
{  
  lcd_command(0x01);
  delay_ms(10);
}

void lcd_puts(const char* s)
{ 
  while(*s)
    lcd_putch(*s++);
}

void lcd_goto(char x, char y)
{ 
  if(x==1)
    lcd_command(0x80+((y-1)%16));
  else
    lcd_command(0xC0+((y-1)%16));
}
	
void lcd_init()
{
	// PORTF Init
  SYSCTL_RCGC2_R |= 0x00000020;
	delay_ms(5);      
  GPIO_PORTF_LOCK_R = 0X4C4F434B;
	GPIO_PORTF_CR_R = 0xFF;
	GPIO_PORTF_AMSEL_R = 0x00;
	GPIO_PORTF_PCTL_R = 0x00000000;
	GPIO_PORTF_AFSEL_R = 0x00;
	GPIO_PORTF_DEN_R = 0xFF;
	GPIO_PORTF_DIR_R = 0xFF;
	
	delay_ms(15);
	
  LCD_RS(0);
  LCD_RW(0);
	LCD_EN(0);

  lcd_command(0x28);  
  lcd_command(0x0C);
  lcd_command(0x06);  
  lcd_command(0x80);  
  lcd_command(0x28);
  lcd_cls();
	
}
