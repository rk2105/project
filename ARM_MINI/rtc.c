#include <LPC214x.H>
#include "rtc.h"
#include "kpm.h"
#include "types.h"
#include "lcd.h"
#include "lcd_defines.h"
#include "delay.h"

#define FOSC 12000000
#define CCLK (5*FOSC)
#define PCLK (CCLK/4)
#define PREINT_Val ((PCLK/32768)-1)
#define PREFRAC_Val (PCLK-((PREINT + 1) * 32768))

s8 week[][4] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};

void printMenu(void)
{
        CmdLCD(GOTO_LINE1_POS0);
        StrLCD("1.H 2.M 3.S 4.DY");
        CmdLCD(GOTO_LINE2_POS0);
        StrLCD("5.DT 6.MN 7.YEAR");
}
void RTC_Init(void)
{
        //for LPC2129 CPU need to update PREINT
        PREINT = PREINT_Val;
        PREFRAC = PREFRAC_Val;
        CCR = 0x11;
}
//set time time values (hours, mins, secs)
void Set_Time(u8 hour,u8 mins,u8 sec)
{
        HOUR = hour;
        MIN = mins;
        SEC = sec;
}
//set the date values (date, month, year)
void Set_Date(u8 date,u8 month,u32 year)
{
        DOM = date;
        MONTH = month;
        YEAR = year;
}
//set week value
void Set_Week(u8 week)
{
        DOW = week;
}

//display hours, mins, secs
void Display_Time_week_date(void)
{
//display hour,min,sec
        CmdLCD(GOTO_LINE1_POS0);
        CharLCD((HOUR/10)+48);
        CharLCD((HOUR%10)+48);
        CharLCD(':');
        CharLCD((MIN/10)+0x30);
        CharLCD((MIN%10)+0x30);
        CharLCD(':');
        CharLCD((SEC/10)+'0');
		    CharLCD((SEC%10)+'0');

//display week
        CmdLCD(GOTO_LINE1_POS0+11);
        StrLCD(week[DOW]);

//display date, month, year

        CmdLCD(GOTO_LINE2_POS0);
        CharLCD((DOM/10)+48);
        CharLCD((DOM%10)+48);
        CharLCD('/');
        CharLCD((MONTH/10)+0x30);
        CharLCD((MONTH%10)+0x30);
        CharLCD('/');
        U32LCD(YEAR);
}
u32 GetFunction(void)
{
        s32 val;
        s32 sum=0,count=0;
        while(count<2)
        {
                count++;
                val=keyscan();
                delay_ms(200);
                if((val>=0)&&(val<=9))
                {
                                sum=sum*10+val;
                                U32LCD(sum);
                                CmdLCD(CLEAR_LCD);
                }
        }
        return sum;
}
//editing hours
u8 GetHour(void)
{
        s32 res;
        StrLCD("HH");
hour:res=GetFunction();
        if((res>=0)&&(res<=23))
        {
                return res;
        }
        else
        {
                StrLCD("INVAILD HOUR");
                goto hour;
        }
}
//editing mins
u8 GetMin(void)
{
        s32 res;
        StrLCD("MM");
min:res=GetFunction();
        if((res>=0)&&(res<=59))
        {
                return res;
        }
        else
        {
                StrLCD("INVAILD MIN");
                goto min;
        }
}
//editing sec
u8 GetSec(void)
{
        s32 res;
        StrLCD("SS");
sec:res=GetFunction();
        if((res>=0)&&(res<=59))
        {
                return res;
        }
        else
        {
                StrLCD("INVAILD SEC");
                goto sec;
        }
}
//editing day
u8 GetWeek(void)
{
        s32 val;
week:val=keyscan();
                if((val>=0)&&(val<=6))
                {
                        return val;
                }
                else
                {
                        StrLCD("INVALID WEEK");
                        goto week;
                }
}
//editing date
u8 GetDate(void)
{
        s32 res;
        StrLCD("DD");
date:res=GetFunction();
        if((res>=0)&&(res<=31))
        {
                return res;
        }
        else
        {
                StrLCD("INVAILD DATE");
                goto date;
        }
}
//editing month
u8 GetMonth(void)
{
        s32 res;
        StrLCD("MM");
month:res=GetFunction();
        if((res>=0)&&(res<=12))
        {
                return res;
        }
        else
        {
                StrLCD("INVAILD MONTH");
                goto month;
        }
}
//editing year
u32 GetYear(void)
{
        s32 val;
        s32 sum=0,count=0;
        StrLCD("YYYY");
        while(count<4)
        {
                count++;
                val=keyscan();
                if((val>=0)&&(val<=9))
                {
                        sum=sum*10+val;
                }
        }
        return sum;
}
