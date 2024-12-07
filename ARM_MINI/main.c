#include "LPC21xx.h"
#include "lcd_defines.h"
#include "keypad_defines.h"
#include "kpm.h"
#include "rtc.h"
#include "lcd.h"
#include "defines.h"
u32 choice;
main()
{
        RTC_Init();
        InitLCD();
        init_keypad();
        Set_Time(17,40,00);
        Set_Date(06,12,2024);
        Set_Week(5);
        while(1)
        {
                Display_Time_week_date();
                if(((IOPIN0>>KEY)&1)==0)
                {
                        while(((IOPIN0>>KEY)&1)==0);
                        printMenu();
                        choice = keyscan();
                        switch(choice)
                        {
                                case 1:HOUR=GetHour();
                                             break;
                                case 2:MIN=GetMin();
                                                break;
                                case 3:SEC=GetSec();
                                                break;
                                case 4:Set_Week(GetWeek());
                                                break;
                                case 5:DOM=GetDate();
                                                break;
                                case 6:MONTH=GetMonth();
                                                break;
                                case 7:YEAR=GetYear();
                                                break;
                                default:StrLCD("Thank you");
                                                break;
                        }
                }
                CmdLCD(CLEAR_LCD);
        }

}
