#include <LPC21xx.h>
#include "types.h"
#include "lcd_defines.h"
#include "defines.h"
#include "delay.h"
#include "lcd.h"
void writeLCD(u8 byte)
{
   IOCLR0=1<<LCD_RW;
   WRITEBYTE(IOPIN0,LCD_DATA,byte);
   IOSET0=1<<LCD_EN;
   delay_us(1);
   IOCLR0=1<<LCD_EN;
   delay_ms(2);
}
void cmdLCD(u8 cmd)
{
  IOCLR0=1<<LCD_RS;
  writeLCD(cmd);
}
void InitLCD(void)
{
   WRITEBYTE(IODIR0, LCD_DATA, 0XFF);
   SETBIT(IODIR0,LCD_RW);
   SETBIT(IODIR0,LCD_EN);
   SETBIT(IODIR0,LCD_RW);
   delay_ms(15);
   cmdLCD(0x30);
   delay_ms(4);
   delay_us(100);
   cmdLCD(0x30);
   delay_us(100);
   cmdLCD(0x30);
   cmdLCD(MODE_8BIT_2LINE);
   cmdLCD(DSP_ON_CUR_OFF);
   cmdLCD(CLEAR_LCD);
   cmdLCD(SHIFT_CUR_RIGHT);
}
void charLCD(u8 asciival)
{
        IOSET0=1<<LCD_RS;
        writeLCD(asciival);
}
void setcursor(u8 lineNo, u8 pos)
{
        if(lineNo==1)
                cmdLCD(GOTO_LINE1_POS0+pos);
        else if(lineNo==2)
                cmdLCD(GOTO_LINE2_POS0+pos);
}
void strLCD(s8 *str)
{
        while(*str) charLCD(*str++);
}
void u32LCD(u32 num)
{
        u8 a[10];
        s32 i=0;
        if(num==0)
                charLCD('0');
                else
                {
				                        while(num!=0)
                        {
                                a[i++]=(num%10)+48;
                                num=num/10;
                        }
                        for(--i;i>=0;i--)
                                charLCD(a[i]);
                }
}
void s32LCD(s32 num)
{
        if(num<0)
        {
                charLCD('-');
                num=-num;
        }
        u32LCD(num);
}
void f32LCD(f32 fnum, u32 nDP)
{
        s32 inum;
        if(fnum<0.0)
        {
                charLCD('-');
                fnum=fnum;
        }
        inum=fnum;
        u32LCD(inum);
        charLCD('.');
        while(nDP)
        {
                fnum=(fnum-inum)*10;
                inum=fnum;
                charLCD(inum+48);
                nDP--;
        }
}/*
void BuildCGRAM(u8 *p)
{
        u32 i;
        cmdLCD(GOTO_CGRAM_START);
        for(i=0;i<=7;i++)
                charLCD(p[i]);
        cmdLCD(GOTO_LINE1_POS0);
}  */
