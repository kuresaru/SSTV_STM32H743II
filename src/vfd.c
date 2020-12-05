#include "vfd.h"
#include "delay.h"
#include <stdio.h>

/**
 * SDA PA7
 * SCL PG11
 * CS  PC4
 * RST PC1
 * PEN PG13
 */

uint8_t VFD_DCRAM[] = "\x07SSTV-Line  0"; //DCRAM
const uint8_t VFD_CGRAM[8][5] = {
    //CGRAM
    {0x08, 0x1c, 0x3e, 0x00, 0x3e}, //left arrow
    {0x3e, 0x00, 0x3e, 0x1c, 0x08}, //right arrow
    {0x28, 0x2c, 0x2e, 0x2c, 0x28}, //up arrow
    {0x0a, 0x1a, 0x2a, 0x1a, 0x0a}, //down arrow
    {0x00, 0x00, 0x00, 0x00, 0x00}, //
    {0x00, 0x00, 0x00, 0x00, 0x00}, //
    {0x00, 0x00, 0x00, 0x00, 0x00}, //
    {
        0x00, // x x (480i)p 576     S9    S4 TV   POWER
        0x00, // x x x       (576)i  S10   S5 FILE SAT
        0x00, // x x x       (576i)p 1080p S6 S1   REC
        0x00, // x x x       480     1080i S7 S2   RADIO
        0x00, // x x x       (480)i  720p  S8 S3   TER
    },
};

const uint8_t VFD_ADRAM[] = {
    0x00, //
    0x00, // TIME
    0x00, // SHIFT
    0x00, // CLOCK
    0x00, // HD
    0x00, // USB
    0x00, // LOCK :1
    0x00, // DOLBY :1
    0x00, // MUTE :2
    0x00, // TU1 :2
    0x00, // TU2 :3
    0x00, // MP3 :3
    0x00, // LOOP
};        //ADRAM

void vfd_write_data(u8 data)
{
    SPI1->CR1 |= SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_CSTART;
    while (!(SPI1->SR & SPI_SR_TXP))
        ;
    *((volatile uint8_t *)&SPI1->TXDR) = data;
    while (!(SPI1->SR & SPI_SR_RXP))
        ;
    SPI1->IFCR |= SPI_IFCR_EOTC;
    SPI1->CR1 &= ~SPI_CR1_SPE;
}

void vfd_init()
{
    uint32_t tmp;

    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIOGEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    VFD_PEN_HI();
    VFD_RST_LO();

    GPIO_Set(GPIOA, 1 << 7, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU);     // SDA
    GPIO_Set(GPIOG, 1 << 11, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_PU);    // SCL
    GPIO_Set(GPIOC, 1 << 4, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);  // CS
    GPIO_Set(GPIOC, 1 << 1, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE);  // RST
    GPIO_Set(GPIOG, 1 << 13, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_HIGH, GPIO_PUPD_NONE); // PEN

    GPIO_AF_Set(GPIOA, 7, 5);
    GPIO_AF_Set(GPIOG, 11, 5);

    RCC->D2CCIP1R &= ~(7 << 12);
    RCC->D2CCIP1R |= 0 << 12; // pll1q 200MHz

    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    SPI1->CR1 |= SPI_CR1_SSI;

    SPI1->CFG1 |= 0b010 << SPI_CFG1_MBR_Pos; // 200/4=50MHz
    SPI1->CFG1 |= 0b00111;                   // 8bit

    tmp = SPI_CFG2_AFCNTR;           // 始终控制io
    tmp |= SPI_CFG2_SSM;             // 软件nss
    tmp |= SPI_CFG2_CPOL;            // clock high
    tmp |= SPI_CFG2_CPHA;            // 2Edge
    tmp |= SPI_CFG2_LSBFRST;
    tmp |= SPI_CFG2_MASTER;
    SPI1->CFG2 = tmp;

    SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

    vfd_write_data(0xFF);

    VFD_CS_HI();
    delay_ms(5);
    VFD_RST_HI();
    delay_ms(5);

    u8 i, j;

    VFD_CS_LO();
    vfd_write_data(VFD_CMD_SET_DISPLAY_TIMING); //Set the display timing
    vfd_write_data(0x0d);                       //
    VFD_CS_HI();

    VFD_CS_LO();
    vfd_write_data(VFD_CMD_DIMMING_SET); //Set the dimming data
    vfd_write_data(200);
    VFD_CS_HI();

    VFD_CS_LO();
    vfd_write_data(VFD_CMD_CGRAM_DATA_WRITE); //write CGRAM
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 5; j++)
        {
            vfd_write_data(VFD_CGRAM[i][j]); //CGRAM
        }
    }
    VFD_CS_HI();

    VFD_CS_LO();
    vfd_write_data(VFD_CMD_DCRAM_DATA_WRITE); //write DCRAM
    for (i = 0; i < 13; i++)
    {
        vfd_write_data(VFD_DCRAM[i]); //DCRAM
    }
    VFD_CS_HI();

    VFD_CS_LO();
    vfd_write_data(VFD_CMD_ADRAM_DATA_WRITE); //write ADRAM
    for (i = 0; i < 13; i++)
    {
        vfd_write_data(VFD_ADRAM[i]); //ADRAM
    }
    VFD_CS_HI();

    VFD_CS_LO();
    vfd_write_data(VFD_CMD_DISPLAY_LIGHT_ON_OFF); //all display on
    VFD_CS_HI();

    VFD_PEN_LO();
}

void vfd_update_num(uint16_t num)
{
    u8 i;
    sprintf((char *)(VFD_DCRAM + 10), "%3u", num);
    VFD_CS_LO();
    vfd_write_data(VFD_CMD_DCRAM_DATA_WRITE + 10); //write DCRAM
    for (i = 0; i < 3; i++)
    {
        vfd_write_data(VFD_DCRAM[i + 10]); //DCRAM
    }
    VFD_CS_HI();
}