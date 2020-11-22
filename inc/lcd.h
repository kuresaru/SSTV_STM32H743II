#ifndef __LCD_H
#define __LCD_H
#include "sys.h"

typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint8_t  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
}_lcd_dev; 	

extern _lcd_dev lcddev;	//管理LCD重要参数

#define LCD_W 320
#define LCD_H 480

typedef struct
{
	volatile uint16_t LCD_REG;
	volatile uint16_t LCD_RAM;	
} LCD_TypeDef;

#define LCD_BASE        ((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

#define LCD_FMC_BCR FMC_Bank1_R->BTCR[0]
#define LCD_FMC_BTR FMC_Bank1_R->BTCR[1]
#define LCD_FMC_BWTR FMC_Bank1E_R->BWTR[0]

#define LCD_WriteRAM_Prepare() lcd_write_reg(0x2C)
#define LCD_ReadRAM_Prepare() lcd_write_reg(0x2E)
#define RGB565_R(x) (((uint16_t)x & 0xF8) << 8)
#define RGB565_G(x) (((uint16_t)x & 0xFC) << 3)
#define RGB565_B(x) (((uint16_t)x & 0xF8) >> 3)
#define RGB(r, g, b) (RGB565_R(r) | RGB565_G(g) | RGB565_B(b))
#define RGB888(color) (RGB((color >> 16), (color >> 8), (color)))

#define LCD_BGCOLOR RGB888(0x000000)

void lcd_init();
uint16_t lcd_read(void);
void lcd_write_reg(uint16_t data);
void lcd_set_window(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd);
u16 lcd_readpixel(u16 x, u16 y);

#endif