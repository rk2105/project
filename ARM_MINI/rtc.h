#ifndef _RTC_H
#define _RTC_H

#include "types.h"
void printMenu(void);
void RTC_Init(void);
void Set_Time(u8,u8,u8);
void Set_Date(u8,u8,u32);
void Set_Week(u8);
void Display_Time_week_date(void);

u8 GetHour(void);
u8 GetMin(void);
u8 GetSec(void);
u8 GetWeek(void);
u8 GetDate(void);
u8 GetMonth(void);
u32 GetYear(void);

u32 getfuction(void);

#endif
