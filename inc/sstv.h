#ifndef __ROBOT36_H
#define __ROBOT36_H
#include "sys.h"
#include "arm_math.h"

#define SSTV_MODE_INFO_COUNT 7

struct sstv_mode_info
{
    u8 code;
    char *name;
    u8 flag1;
};

int sstv(void);

#endif