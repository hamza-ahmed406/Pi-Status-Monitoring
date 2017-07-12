////============================================
///	Code by: Muneeb Abid
//	Dated: 28 Feb 2015
///	Details: Alphanumeric LCD interface
////============================================

/*

RS -> PF0
RW -> PF1
EN -> PF2

D4 -> PF4
D5 -> PF5
D6 -> PF6
D7 -> PF7

*/

#define delay_us(x)     SysTick_delay_us(x)
#define delay_ms(x)     SysTick_delay_ms(x)

#define LCD_DATA        GPIO_PORTF_DATA_R
#define LCD_CONTROL     GPIO_PORTF_DATA_R

#define	LCD_RS(x)       ( (x) ? (LCD_CONTROL |= 0x01) : (LCD_CONTROL &= ~0x01) )
#define	LCD_RW(x)       ( (x) ? (LCD_CONTROL |= 0x02) : (LCD_CONTROL &= ~0x02) )
#define LCD_EN(x)       ( (x) ? (LCD_CONTROL |= 0x04) : (LCD_CONTROL &= ~0x04) )


void lcd_command  (unsigned char); 
void lcd_cls (void);         
void lcd_puts   (const char*);
void lcd_goto   (char,char);  
void lcd_init   (void);       
void lcd_putch  (char);
