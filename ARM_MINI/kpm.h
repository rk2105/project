//keypad.h
#ifndef _KEYPAD_H
#define _KEYPAD_H
#include "types.h"
void init_keypad(void);
u32 colscan(void);
u32 rowcheck(void);
u32 colcheck(void);
u32 keyscan(void);
//u32 readnum(void);
#endif
