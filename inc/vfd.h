#ifndef __VFD_H
#define __VFD_H
#include "sys.h"

#define VFD_CMD_DISPLAY_MODE         0x00
#define VFD_CMD_DCRAM_DATA_WRITE     0x20
#define VFD_CMD_CGRAM_DATA_WRITE     0x40
#define VFD_CMD_ADRAM_DATA_WRITE     0x60
#define VFD_CMD_URAM_DATA_WRITE      0x80
#define VFD_GRAY_LEVEL_DATA          0xA0
#define VFD_GRAY_LEVEL_ON_OFF_SET    0xC0
#define VFD_CMD_SET_DISPLAY_TIMING   0xE0
#define VFD_CMD_DIMMING_SET          0xE4
#define VFD_CMD_DISPLAY_LIGHT_ON_OFF 0xE8
#define VFD_STANDBY_MODE_SET         0xEC

#define VFD_CS_LO() GPIOC->BSRR |= 1 << (4 + 16)
#define VFD_CS_HI() GPIOC->BSRR |= 1 << (4)
#define VFD_PEN_LO() GPIOG->BSRR |= 1 << (13 + 16)
#define VFD_PEN_HI() GPIOG->BSRR |= 1 << (13)
#define VFD_RST_LO() GPIOC->BSRR |= 1 << (1 + 16)
#define VFD_RST_HI() GPIOC->BSRR |= 1 << (1)


void vfd_init();
void vfd_update_num(uint16_t num);

#endif