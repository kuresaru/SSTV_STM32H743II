#include "sys.h"
#include "mpu.h"
#include "usart.h"
#include "delay.h"
#include "wm8978.h"
#include "sai.h"
#include "lcd.h"
#include <stdio.h>
#include "sstv.h"

// 好像内存必须在d1区域里才能连上dma
u16 saiplaybuf[2] __attribute__((section(".memd1base"))) = {0X0000, 0X0000};

int main()
{
    Stm32_Clock_Init();
    SCB->CPACR |= 0b1111 << 20; // enable fpu
    delay_init(400);
    uart_init(100, 115200);
    MPU_Memory_Protection();
    lcd_init();

    RCC->AHB4ENR |= 1 << 1;
    GPIO_Set(GPIOB, PIN0 | PIN1, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_MID, GPIO_PUPD_PU);
    GPIOB->BSRR |= 0b11;

    WM8978_Init();
    WM8978_HPvol_Set(40, 40);
    WM8978_SPKvol_Set(50);

    WM8978_ADDA_Cfg(0, 1);     //开启ADC
    WM8978_Input_Cfg(1, 1, 0); //开启输入通道(MIC&LINE IN)
    WM8978_Output_Cfg(0, 1);   //开启BYPASS输出
    WM8978_MIC_Gain(20);       //MIC增益设置
    WM8978_SPKvol_Set(0);      //关闭喇叭.
    WM8978_I2S_Cfg(2, 0);      //飞利浦标准,16位数据长度

    SAIA_Init(0, 1, 4);                                                 //SAI1 Block A,主发送,16位数据
    SAIB_Init(3, 1, 4);                                                 //SAI1 Block B从模式接收,16位
    SAIA_SampleRate_Set(16000);                                          //设置采样率
    SAIA_TX_DMA_Init((u8 *)&saiplaybuf[0], (u8 *)&saiplaybuf[1], 1, 1); //配置TX DMA,16位
    SAIB_RX_DMA_Init();   //配置RX DMA
    SAI_Play_Start();                                                   //开始SAI数据发送(主机)
    SAI_Rec_Start();                                                    //开始SAI数据接收(从机)

    while (1)
    {
        sstv();
    }
}

void HardFault_Handler()
{
    while (1)
    {
    }
}