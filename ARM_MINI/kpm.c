#include <LPC214x.h>
#include "keypad_defines.h"
#include "defines.h"
#include "types.h"
u32 keypadLUT[4][4]={{1,2,3,'A'},{4,5,6,'B'},{7,8,9,'C'},{'*',0,'#','D'}};
void init_keypad(void)
{
        //making row as output oins and col as input pin
        WRITENIBBLE(IODIR1,ROW0,15);
}
u32 colscan(void)
{
        return(READNIBBLE(IOPIN1,COL0)<15)?0:1;
}
u32 rowcheck(void)
{
        u32 rNo;
        for(rNo=0;rNo<=3;rNo++)
        {
                WRITENIBBLE(IOPIN1,ROW0,~(1<<rNo));
                if(colscan()==0)
                {
                break;
                }
        }
        WRITENIBBLE(IOPIN1,ROW0,0);
        return rNo;
}
u32 colcheck(void)
{
        u32 cNo;
        for(cNo=0;cNo<=3;cNo++)
        {
                if(READBIT(IOPIN1,COL0+cNo)==0)
                {
                        break;
                }
        }
        return cNo;
}
u32 keyscan(void)
{
        u32 key,rNo,cNo;
        while(colscan());
        rNo=rowcheck();
        cNo=colcheck();
        while(!colscan());
        key=keypadLUT[rNo][cNo];
        return key;
}
