#include "lcd.h"
#include "delay.h"

extern u8 Font_ASC8X16[];
const u8 UTF8TOUNICODEMASK[] = {0x1F, 0x0F, 0x07, 0x03, 0x01};
_lcd_dev lcddev;

uint16_t lcd_read(void)
{
    volatile uint16_t data; //防止被优化
    data = LCD->LCD_RAM;
    return data;
}

void lcd_write_reg(uint16_t data)
{
    data = data;         //使用-O2优化的时候,必须插入的延时
    LCD->LCD_REG = data; //写入要写的寄存器序号
}

void lcd_write_data(uint16_t data)
{
    data = data; //使用-O2优化的时候,必须插入的延时
    LCD->LCD_RAM = data;
}

uint16_t lcd_read_data(void)
{
    return lcd_read();
}

void lcd_set_reg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
    LCD->LCD_REG = LCD_Reg;      //写入要写的寄存器序号
    LCD->LCD_RAM = LCD_RegValue; //写入数据
}

void lcd_read_reg(uint16_t LCD_Reg, uint8_t *Rval, int n)
{
    lcd_write_reg(LCD_Reg);
    while (n--)
    {
        *(Rval++) = lcd_read_data();
        delay_us(5);
    }
}

void lcd_set_window(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{
    lcd_write_reg(0x2A);
    lcd_write_data(xStar >> 8);
    lcd_write_data(0x00FF & xStar);
    lcd_write_data(xEnd >> 8);
    lcd_write_data(0x00FF & xEnd);

    lcd_write_reg(0x2B);
    lcd_write_data(yStar >> 8);
    lcd_write_data(0x00FF & yStar);
    lcd_write_data(yEnd >> 8);
    lcd_write_data(0x00FF & yEnd);
}

u16 lcd_readpixel(u16 x, u16 y)
{
    u16 r, g, b;
    lcd_set_window(x, y, x, y);
    LCD_ReadRAM_Prepare();
    if (LCD->LCD_RAM)
        r = 0; // dummy
    // opt_delay(2);
    r = LCD->LCD_RAM;
    // opt_delay(2);
    b = LCD->LCD_RAM;
    g = r & 0XFF;
    g <<= 8;
    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));
}

void lcd_clear(uint16_t Color)
{
    unsigned int i;
    uint32_t total_point = lcddev.width * lcddev.height;
    lcd_set_window(0, 0, lcddev.width - 1, lcddev.height - 1);
    LCD_WriteRAM_Prepare();
    for (i = 0; i < total_point; i++)
    {
        LCD->LCD_RAM = Color; //写十六位颜色值
    }
}

void lcd_set_direction(uint8_t direction)
{
    lcddev.dir = direction % 4;
    switch (lcddev.dir)
    {
    case 0: // 竖屏
        lcddev.width = LCD_W;
        lcddev.height = LCD_H;
        lcd_set_reg(0x36, (1 << 3));
        break;
    case 1: // 横屏
        lcddev.width = LCD_H;
        lcddev.height = LCD_W;
        lcd_set_reg(0x36, (1 << 3) | (0b011 << 5));
        break;
    }
}

void lcd_init()
{
    RCC->AHB4ENR |= 1 << 1;  //使能PORTB时钟
    RCC->AHB4ENR |= 3 << 3;  //使能PD,PE
    RCC->AHB3ENR |= 1 << 12; //使能FMC时钟

    GPIO_Set(GPIOB, PIN5, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_MID, GPIO_PUPD_PU);                                        //PB5 推挽输出,控制背光
    GPIO_Set(GPIOD, (3 << 0) | (3 << 4) | (15 << 7) | (7 << 13), GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU); //PD0,1,4,5,7,8,9,10,13,14,15 AF OUT
    GPIO_Set(GPIOE, (0X1FF << 7), GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU);                                //PE7~15,AF OUT
    GPIO_AF_Set(GPIOD, 0, 12);                                                                                                //PD0,AF12
    GPIO_AF_Set(GPIOD, 1, 12);                                                                                                //PD1,AF12
    GPIO_AF_Set(GPIOD, 4, 12);                                                                                                //PD4,AF12
    GPIO_AF_Set(GPIOD, 5, 12);                                                                                                //PD5,AF12
    GPIO_AF_Set(GPIOD, 7, 12);                                                                                                //PD7,AF12
    GPIO_AF_Set(GPIOD, 8, 12);                                                                                                //PD8,AF12
    GPIO_AF_Set(GPIOD, 9, 12);                                                                                                //PD9,AF12
    GPIO_AF_Set(GPIOD, 10, 12);                                                                                               //PD10,AF12
    GPIO_AF_Set(GPIOD, 13, 12);                                                                                               //PD13,AF12
    GPIO_AF_Set(GPIOD, 14, 12);                                                                                               //PD14,AF12
    GPIO_AF_Set(GPIOD, 15, 12);                                                                                               //PD15,AF12

    GPIO_AF_Set(GPIOE, 7, 12);  //PE7,AF12
    GPIO_AF_Set(GPIOE, 8, 12);  //PE8,AF12
    GPIO_AF_Set(GPIOE, 9, 12);  //PE9,AF12
    GPIO_AF_Set(GPIOE, 10, 12); //PE10,AF12
    GPIO_AF_Set(GPIOE, 11, 12); //PE11,AF12
    GPIO_AF_Set(GPIOE, 12, 12); //PE12,AF12
    GPIO_AF_Set(GPIOE, 13, 12); //PE13,AF12
    GPIO_AF_Set(GPIOE, 14, 12); //PE14,AF12
    GPIO_AF_Set(GPIOE, 15, 12); //PE15,AF12

    // PLL2R 220MHz 4.55ns

    LCD_FMC_BCR = 0;
    LCD_FMC_BTR = 0;
    LCD_FMC_BWTR = 0;

    // set timing
    LCD_FMC_BCR |= FMC_BCRx_WREN;          //存储器写使能
    LCD_FMC_BCR |= 1 << FMC_BCRx_MWID_Pos; //存储器数据宽度为16bit  0=8b 1=16b 2=32b
    LCD_FMC_BCR |= FMC_BCRx_EXTMOD;        // 读写不同时序

    LCD_FMC_BTR |= 0 << FMC_BTRx_ACCMOD_Pos; // mode A
    // R ADDSET typ=0ns
    LCD_FMC_BTR |= 0 << FMC_BTRx_ADDSET_Pos;
    // R DATAST
    LCD_FMC_BTR |= 98 << FMC_BTRx_DATAST_Pos;

    LCD_FMC_BWTR |= 0 << FMC_BWTRx_ACCMOD_Pos; // mode A
    // W ADDSET typ=0ns
    LCD_FMC_BWTR |= 0 << FMC_BWTRx_ADDSET_Pos;
    // W DATAST typ=10ns 10/4.55=2.2
    LCD_FMC_BWTR |= 3 << FMC_BWTRx_DATAST_Pos;

    // enable fmc
    LCD_FMC_BCR |= FMC_BCRx_MBKEN;
    LCD_FMC_BCR |= FMC_BCR1_FMCEN;

    delay_ms(50);

    //************* ILI9488初始化**********//
    lcd_write_reg(0XF7);
    lcd_write_data(0xA9);
    lcd_write_data(0x51);
    lcd_write_data(0x2C);
    lcd_write_data(0x82);

    lcd_write_reg(0xC0);
    lcd_write_data(0x11);
    lcd_write_data(0x09);

    lcd_write_reg(0xC1);
    lcd_write_data(0x41);

    lcd_write_reg(0XC5);
    lcd_write_data(0x00);
    lcd_write_data(0x0A);
    lcd_write_data(0x80);

    lcd_write_reg(0xB1);
    lcd_write_data(0xB0);
    lcd_write_data(0x11);

    lcd_write_reg(0xB4);
    lcd_write_data(0x02);

    lcd_write_reg(0xB6);
    lcd_write_data(0x02);
    lcd_write_data(0x22);

    lcd_write_reg(0xB7);
    lcd_write_data(0xc6);

    lcd_write_reg(0xBE);
    lcd_write_data(0x00);
    lcd_write_data(0x04);

    lcd_write_reg(0xE9);
    lcd_write_data(0x00);

    lcd_write_reg(0x36);
    lcd_write_data(0x08);

    lcd_write_reg(0x3A);
    lcd_write_data(0x55);

    lcd_write_reg(0xE0);
    lcd_write_data(0x00);
    lcd_write_data(0x07);
    lcd_write_data(0x10);
    lcd_write_data(0x09);
    lcd_write_data(0x17);
    lcd_write_data(0x0B);
    lcd_write_data(0x41);
    lcd_write_data(0x89);
    lcd_write_data(0x4B);
    lcd_write_data(0x0A);
    lcd_write_data(0x0C);
    lcd_write_data(0x0E);
    lcd_write_data(0x18);
    lcd_write_data(0x1B);
    lcd_write_data(0x0F);

    lcd_write_reg(0XE1);
    lcd_write_data(0x00);
    lcd_write_data(0x17);
    lcd_write_data(0x1A);
    lcd_write_data(0x04);
    lcd_write_data(0x0E);
    lcd_write_data(0x06);
    lcd_write_data(0x2F);
    lcd_write_data(0x45);
    lcd_write_data(0x43);
    lcd_write_data(0x02);
    lcd_write_data(0x0A);
    lcd_write_data(0x09);
    lcd_write_data(0x32);
    lcd_write_data(0x36);
    lcd_write_data(0x0F);

    lcd_write_reg(0x11);
    delay_ms(120);
    lcd_write_reg(0x29);

    lcd_set_direction(0);
    lcd_clear(RGB888(0x000000));

    delay_ms(100);
    GPIOB->BSRR = 1 << 5; // bl on
}