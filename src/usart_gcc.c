#include "sys.h"
#include "usart.h"

int _write(int fd, char *ptr, int len)
{
    int i;
    (void)fd;
    for (i = 0; i < len; i++)
    {
        while (!(USART1->ISR & USART_ISR_TC))
        {
        }
        USART1->TDR = *((u8 *)ptr + i);
    }
    return i;
}

void uart_init(u32 pclk2, u32 bound)
{
    u32 temp;
    temp = (pclk2 * 1000000 + bound / 2) / bound;
    RCC->AHB4ENR |= 1 << 0;
    RCC->APB2ENR |= 1 << 4;
    GPIO_Set(GPIOA, PIN9 | PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_MID, GPIO_PUPD_PU);
    GPIO_AF_Set(GPIOA, 9, 7);
    GPIO_AF_Set(GPIOA, 10, 7);
    USART1->BRR = temp;
    USART1->CR1 = 0;
    USART1->CR1 |= 0 << 28;
    USART1->CR1 |= 0 << 12;
    USART1->CR1 |= 0 << 15;
    USART1->CR1 |= 1 << 3;
#if EN_USART1_RX
    USART1->CR1 |= 1 << 2;
    USART1->CR1 |= 1 << 5;
    MY_NVIC_Init(3, 3, USART1_IRQn, 2);
#endif
    USART1->CR1 |= 1 << 0;
}
