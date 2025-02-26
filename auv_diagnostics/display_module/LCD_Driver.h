#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include "DEV_Config.h"

#define LCD_WIDTH   320 //LCD width
#define LCD_HEIGHT  240 //LCD height

 
void LCD_WriteData_Word(UWORD da);

void LCD_SetCursor(UWORD X, UWORD Y);
void LCD_SetWindow(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD  Yend);
void LCD_DrawPaint(UWORD x, UWORD y, UWORD Color);

void LCD_Init(void);
void LCD_SetBackLight(UWORD Value);

void LCD_Clear(UWORD Color);
void LCD_ClearWindow(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend,UWORD color);

#endif
