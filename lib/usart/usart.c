#include "sys.h"
#include "usart.h"
#include <stdio.h>  
#include <errno.h>
#include "led.h"    
#include <string.h>
//-------------------------------------------------------------------------------
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 	
u16 FalshSave[32];
//--------------------------------------------------------
uint8_t UART2_DMA_TX_BUF[UART2_DMA_TX_BUF_SIZE];
//----------------------------------------------------------
//
//----------------------------------------------------------
void uart2_init(uint32_t bound) {
    // 1. 时钟使能 (G4 必须开启 DMAMUX)
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    // 2. GPIO 配置 PB3=TX, PB4=RX (直接操作寄存器最稳)
    GPIOB->MODER   &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER   |= (2U << GPIO_MODER_MODE3_Pos) | (2U << GPIO_MODER_MODE4_Pos); // AF模式

    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4);
    GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED3_Pos) | (3U << GPIO_OSPEEDR_OSPEED4_Pos); // 高速模式
    
    // PB3 -> AF7, PB4 -> AF7 (AFRL 寄存器)
    GPIOB->AFR[0] &= ~(0xF << (3 * 4) | 0xF << (4 * 4));
    GPIOB->AFR[0] |=  (7 << (3 * 4) | 7 << (4 * 4));

    // 3. USART2 配置
    // 注意：请务必确认系统频率！如果是 170MHz，用 170000000，如果是 16MHz，用 16000000
    USART2->BRR = 170000000 / bound; 

    USART2->CR1 = 0;  // 先清
    USART2->CR1 = USART_CR1_TE 
                | USART_CR1_RE
                | USART_CR1_UE;   // UE 最后使能

    USART2->CR3 |= USART_CR3_DMAT; // 使能 DMA 发送请求

    // 4. DMAMUX 配置 (USART2_TX ID=27)
    DMAMUX1_Channel0->CCR = 27U; 

    // 5. DMA1_Channel1 配置
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&USART2->TDR;
    
    DMA1_Channel1->CCR = DMA_CCR_DIR        // 1 = 内存 → 外设
                        | DMA_CCR_MINC;      // 内存地址递增


    // NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));  // 抢占优先级 2
    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

//----------------------------------------------------------
// USART2 中断服务程序
//----------------------------------------------------------
// void USART2_IRQHandler(void)
// {
//     if(USART2->ISR & USART_ISR_RXNE) // 接收到数据
//     {
//         uint8_t res = USART2->RDR;   // 读取数据清除标志位
//         // 可以在这里处理接收
//     }
// }
//-----------------------------------------------------------
//
//----------------------------------------------------------
void uart2_dma_send(uint16_t len)
{
    // 等待之前的 DMA 发送完成（如果上次没发完）
    while(DMA1_Channel1->CNDTR != 0); 
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;          // 临时关闭通道
    DMA1_Channel1->CMAR = (uint32_t)UART2_DMA_TX_BUF; // 设置内存缓冲区地址
    DMA1_Channel1->CNDTR = len;                 // 设置发送长度
    
    DMA1->IFCR |= DMA_IFCR_CTCIF1;              // 清除通道1完成标志
    DMA1_Channel1->CCR |= DMA_CCR_EN;           // 重新开启 DMA
}
//--------------------------------------------------------------------------
// @brief  串口1发送一个字节
//---------------------------------------------------------------------------
void uart2_send_byte(u8 data)
{
    while (!(USART2->ISR & USART_ISR_TXE)); 
    USART2->TDR = data;
}
// --------------------------------------------------------------------------
// @brief  重定向 printf 到串口 2
// --------------------------------------------------------------------------
int _write(int file, char *ptr, int len)
{
    // 1. [关键] 检查并强制清除串口硬件错误标志
    // 如果不清除 ORE(过载)、NE(噪声)、FE(帧错误)，串口会彻底罢工
    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE)) {
        USART2->ICR |= (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF);
    }

    // 2. [关键] 增加超时保护，防止 DMA 计数器停滞导致死循环
    uint32_t timeout = 50000; 
    while(DMA1_Channel1->CNDTR != 0 && timeout--); 

    if(timeout == 0) {
        // 如果 DMA 卡住了，强行关闭通道并重置，放弃本次打印
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel1->CNDTR = 0; 
        return 0; 
    }

    // 3. 正常 DMA 拷贝与发送
    int send_len = (len < 256) ? len : 256;
    memcpy(UART2_DMA_TX_BUF, ptr, send_len);
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CMAR = (uint32_t)UART2_DMA_TX_BUF;
    DMA1_Channel1->CNDTR = send_len;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    return len;
}


