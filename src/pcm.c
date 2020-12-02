#include <stdlib.h>
#include <stdio.h>
#include "pcm.h"
#include "sai.h"

static uint8_t bufidx = 0;                    // 当前在使用的缓冲区
static uint16_t bufpos = SAI_RX_DMA_BUF_SIZE; // 当前缓冲区已读的字节数 (数据偏移)

u16 read_pcm(int16_t *buff, uint16_t frames)
{
    // TODO 未知原因在切换DMA buffer的时候前后数据对不上
    u16 frames_remain = frames;
    u16 i;
    int16_t *cpsrcbuf;

    u16 changed = 0;

    while (frames_remain)
    {
        while ((bufpos == SAI_RX_DMA_BUF_SIZE) && (bufidx == sairecbuf_curvalid)) // 一个buf读取完成后等待下一个buf传输完成
        {
        }
        if (bufidx != sairecbuf_curvalid) // 切过buf
        {
            if (bufpos != SAI_RX_DMA_BUF_SIZE)
            {
                printf("ovf\r\n");
            }
            bufidx = sairecbuf_curvalid;
            bufpos = 0;
            changed = frames_remain;
        }

        u32 curbufremain = SAI_RX_DMA_BUF_SIZE - bufpos; // 当前缓冲区还剩下多少
        u32 translen;                                    // 这一次能读到多少
        if ((curbufremain / 2) > frames_remain)          // 缓冲区剩余比请求剩余多
        {
            translen = frames_remain;
        }
        else
        {
            translen = (curbufremain / 2);
        }
        // 复制数据或直接跳过
        if (buff)
        {
            cpsrcbuf = (int16_t *)((bufidx == 1 ? sairecbuf1 : sairecbuf2)) + bufpos; // 一会要从哪个地址开始读
            for (i = 0; i < translen; i++)
            {
                *(buff + i) = (*(cpsrcbuf + (i * 2) + 1)) + 140; // 采集到的数据大概有-140的直流量
            }
        }

        bufpos += translen * 2;
        frames_remain -= translen;
    }
    return changed;
}