#include "sstv.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "arm_math.h"
#include "pcm.h"
#include "delay.h"
#include "lcd.h"

#define SAMPLE_RATE 16000
#define SAMPLE_10MS_CNT 160
#define BUF40MS_SIZE (SAMPLE_10MS_CNT * 4)
#define NPT 1024                 // 16 64 256 1024 4096
// #define SAMPLE_FREQ_RES 7.8125
#define SAMPLE_FREQ_RES 15.625
#define ifreq2freq(x) ((x) * SAMPLE_FREQ_RES)

#define hann(n, N) (0.5 * (1.0 - arm_cos_f32(2.0 * PI * (n) / ((N)-1.0))))
#define freq2lum(f) (round(((f) - 1500.0) / 3.1372549))

const struct sstv_mode_info mode_infos[SSTV_MODE_INFO_COUNT] = {
    {8, "Robot 36", 0},
    {12, "Robot 72", 0},
    {40, "Martin 2", 0},
    {44, "Martin 1", 1},
    {56, "Scottie 2", 0},
    {60, "Scottie 1", 0},
    {76, "Scottie DX", 0},
};

arm_rfft_fast_instance_f32 S;
float32_t sInput[NPT];
float32_t sOutput[NPT];
float32_t fftOutput[NPT];

int16_t buf40ms[BUF40MS_SIZE];

uint16_t line_rgb_buf1[320] __attribute__((section(".memd1base")));

// 找一个频率范围里音量最大的频率
static void find_fft_peak_ranged(u16 min, u16 max, u16 *ifreq, float32_t *vol)
{
    u16 i;
    *ifreq = 0;
    *vol = 0;

    for (i = min; i <= max; i++)
    {
        if (fftOutput[i] > *vol)
        {
            *ifreq = i;
            *vol = fftOutput[i];
        }
    }
}

static uint64_t samples_read = 0;
static uint16_t last_samples_cnt = BUF40MS_SIZE;
static void read_samples(uint64_t pos_sa, uint16_t samples_cnt)
{
    int8_t start_offset_sa = pos_sa - samples_read; // 偏移量 正数需要跳过数据 负数需要倒回数据
    uint8_t newread_sa;
    uint8_t newread_offset;
    if (start_offset_sa >= 0)
    {
        // 跳过采样
        read_pcm(0, start_offset_sa);
        samples_read += start_offset_sa;
        newread_sa = samples_cnt;
        newread_offset = 0;
    }
    else
    {
        // start_offset_sa是负数
        // 使用上一次最后几个采样
        for (uint16_t i16 = 0; i16 < (-start_offset_sa); i16++)
        {
            buf40ms[i16] = buf40ms[last_samples_cnt + start_offset_sa + i16];
        }
        newread_sa = samples_cnt + start_offset_sa;
        newread_offset = (-start_offset_sa);
    }
    read_pcm(buf40ms + newread_offset, newread_sa);
    samples_read += newread_sa;
    last_samples_cnt = samples_cnt;
}

static void fill_and_fft(uint16_t len)
{
    uint16_t i16;
    // 填充采样
    for (i16 = 0; i16 < len; i16++)
    {
        sInput[i16] = hann(i16, len) * buf40ms[i16];
    }
    for (i16 = len; i16 < NPT; i16++)
    {
        sInput[i16] = 0;
    }
    // fft计算
    arm_rfft_fast_f32(&S, sInput, sOutput, 0);
    arm_cmplx_mag_f32(sOutput, fftOutput, NPT);

}

// 找10ms音量最高的频率
static void read_10ms_fft_peak_freq(float32_t *freq, float32_t *vol)
{
    const u16 minth = 600 / SAMPLE_FREQ_RES;
    const u16 maxth = (NPT / 2) - 1;
    u16 ifreq;
    // 读10ms采样
    read_pcm(buf40ms, SAMPLE_10MS_CNT);
    // 填充10ms采样 fft计算
    fill_and_fft(SAMPLE_10MS_CNT);
    // 找音量最高的频率
    find_fft_peak_ranged(minth, maxth, &ifreq, vol);
    *freq = ifreq2freq(ifreq);
}

// 找信号头
static void wait_header()
{
    float32_t freq;
    float32_t vol;

    u8 leader1_10cnt, break_10cnt, leader2_10cnt, vis_start_10cnt;

    leader1_10cnt = break_10cnt = leader2_10cnt = vis_start_10cnt = 0;

    for (;;)
    {
        read_10ms_fft_peak_freq(&freq, &vol);

        // 判断频率
        if ((abs((int)freq - 1900) < 80)) // 1900Hz
        {
            if (break_10cnt)
            {
                leader2_10cnt++;
            }
            else
            {
                leader1_10cnt++;
            }
        }
        else if ((abs((int)freq - 1200) < 80)) // 1200Hz
        {
            if (leader2_10cnt)
            {
                vis_start_10cnt++;
            }
            else
            {
                break_10cnt++;
            }
        }
        else // 其它无效
        {

            leader1_10cnt = break_10cnt = leader2_10cnt = vis_start_10cnt = 0;
        }
        // 判断是否接收到头
        if ((leader1_10cnt > 5)              // 标准值 30
            && (abs(break_10cnt - 1) < 2)    // 标准值 1
            && (abs(leader2_10cnt - 30) < 5) // 标准值 30
            && (vis_start_10cnt == 3)        // 标准值 3
        )
        {
            printf("found header: %d,%d,%d,%d\r\n", leader1_10cnt, break_10cnt, leader2_10cnt, vis_start_10cnt);
            return;
        }
    }
}

// 读编码格式
static u8 read_mode()
{
    float32_t freq;
    float32_t vol;

    u8 i, f1cnt;
    u8 tmp;

    tmp = f1cnt = 0;
    // 1300Hz=0 1100Hz=1
    for (i = 1; i <= 24; i++) // 一共8位 一位30ms
    {
        read_10ms_fft_peak_freq(&freq, &vol);

        if (abs((int)freq - 1100) < 100)
        {
            f1cnt++; // 如果这10ms是1100Hz 变量+1 最后如果是1100Hz 变量肯定是2或者3 否则就是1300Hz
        }
        else if (abs((int)freq - 1300) > 100)
        {
            printf("read mode error: %dHz\r\n", (int)round(freq));
            return 0;
        }

        if ((i % 3) == 0) // 已经分析了30ms 判断结果
        {
            tmp >>= 1;     // LSB
            if (f1cnt > 1) // 1100Hz是1
            {
                tmp |= 0x80;
            }
            // 清空计数下次再用
            f1cnt = 0;
        }
    }
    // 最高位是奇偶校验 7位数据里有奇数个1就是1 偶数个1就是0
    if ((__builtin_popcount(tmp & 0x7F) % 2) != ((tmp >> 7) & 0x1))
    {
        printf("mode parity check error\r\n");
        return 0;
    }
    // 接收成功 跳过30ms
    read_samples(0, SAMPLE_10MS_CNT * 3);
    // 返回结果
    return tmp & 0x7F;
}

static void decode_matin1()
{
    const uint16_t LINE_WIDTH = 320;
    const uint16_t LINE_COUNT = 256;
    const float32_t SCAN_TIME = 0.146432;
    const float32_t SYNC_PULSE = 0.004862;
    const float32_t SYNC_PORCH = 0.000572;
    const float32_t SEP_PULSE = 0.000572;
    const uint16_t CHAN_COUNT = 3;
    const float32_t CHAN_TIME = SEP_PULSE + SCAN_TIME;
    const float32_t LINE_TIME = SYNC_PULSE + SYNC_PORCH + 3 * CHAN_TIME;
    const float32_t PIXEL_TIME = SCAN_TIME / LINE_WIDTH;
    const float32_t WINDOW_FACTOR = 2.34;
    const float32_t centre_window_time = (PIXEL_TIME * WINDOW_FACTOR) / 2;
    const float32_t CHAN_OFFSETS[] = {
        SYNC_PULSE + SYNC_PORCH,
        SYNC_PULSE + SYNC_PORCH + CHAN_TIME,
        SYNC_PULSE + SYNC_PORCH + CHAN_TIME + CHAN_TIME,
    };

    uint8_t pixel_window_sa = (uint8_t)round(centre_window_time * 2 * SAMPLE_RATE);

    uint64_t seq_start_sa = 0;
    uint16_t line, chan, px;
    uint16_t i16, ifreq;
    float32_t freq, vol;

    // 屏幕准备写入
    lcd_set_window(0, 0, LINE_WIDTH - 1, LINE_COUNT - 1);
    LCD_WriteRAM_Prepare();

    // 开始解码
    read_samples(samples_read, SAMPLE_10MS_CNT * 3);
    // seq_start_sa = samples_read;
    // float32_t seq_start_fsa = seq_start_sa;
    for (line = 0; line < LINE_COUNT; line++)
    {
        printf("l%d\r\n", line);
        for (chan = 0; chan < CHAN_COUNT; chan++)
        {
            if (0 == chan)
            {
                // set start
                if (line > 0)
                {
                    seq_start_sa += (uint64_t)round(LINE_TIME * SAMPLE_RATE);
                }
                // TODO hsync
            }
            for (px = 0; px < LINE_WIDTH; px++)
            {
                uint64_t px_pos_sa = (uint64_t)round(seq_start_sa + (CHAN_OFFSETS[chan] + px * PIXEL_TIME - centre_window_time) * SAMPLE_RATE);
                // printf("l%dc%dp%d %d-%d\n", line, chan, px, (int)px_pos_sa, (int)px_pos_sa + pixel_window_sa);

                read_samples(px_pos_sa, pixel_window_sa);
                fill_and_fft(pixel_window_sa);
                find_fft_peak_ranged(96, 147, &ifreq, &vol); // 在这限制频率为1500Hz~2296.875Hz
                freq = ifreq2freq(ifreq);

                // 计算颜色值并写入缓冲区
                // uint16_t *line_rgb_buf = (line_rgb_bufidx ? line_rgb_buf2 : line_rgb_buf1);
                uint16_t *line_rgb_buf = line_rgb_buf1;
                uint8_t lum = freq2lum(freq);
                if (chan == 0)
                {
                    line_rgb_buf[px] = RGB565_G(lum);
                }
                else if (chan == 1)
                {
                    line_rgb_buf[px] |= RGB565_B(lum);
                }
                else
                {
                    line_rgb_buf[px] |= RGB565_R(lum);
                }
            }
        }
        // if (line == 1) { return; }
        // 当前行颜色写入屏幕
        for (i16 = 0; i16 < LINE_WIDTH; i16++)
        {
            LCD->LCD_RAM = line_rgb_buf1[i16];
        }
    }
}

int sstv()
{
    u8 mode, i;
    const struct sstv_mode_info *modeinfo = 0;

    printf("\r\n\r\nstart MCUSSTV Decoder\r\n");
    arm_rfft_fast_init_f32(&S, NPT);

    //直接测试
    // decode_matin1();

    wait_header();
    samples_read = 0;
    mode = read_mode();

    for (i = 0; i < SSTV_MODE_INFO_COUNT; i++)
    {
        if (mode_infos[i].code == mode)
        {
            modeinfo = &mode_infos[i];
            break;
        }
    }

    if (!modeinfo)
    {
        printf("Unknown mode: %d\r\n", mode);
        return 1;
    }
    printf("mode=%d, name=%s\r\n", modeinfo->code, modeinfo->name);

    if (!modeinfo->flag1)
    {
        printf("Unsupported yet\r\n");
        return 1;
    }

    if (mode == 44)
    {
        decode_matin1();
    }

    return 0;
}