#include "sys.h"
#include "usart.h"
#include <stdio.h>  
#include <errno.h>
#include <sys/unistd.h>
#include <string.h>
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	   
//
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 	
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 	
u16 FalshSave[32];
//--------------------------------------------------------

extern volatile uint32_t sys_ms_ticks;   // 全局毫秒时间戳
//////////////////////////////////////////////////////////////////
u8  USART1_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
u8  USART1_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
u8  USART2_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
u8  USART2_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.
u8  USART3_RX_BUF[USART_TRANS_LEN];     //接收缓冲,最大128个字节.
u8  USART3_TX_BUF[USART_TRANS_LEN];     //发送缓冲,最大128个字节.

uint8_t UART2_DMA_TX_BUF[UART2_DMA_TX_BUF_SIZE];

u8  u8TxD1Busy,TxD1Num,TxD1pt,RxD1pt,RxD1Buf;
u8  u8TxD2Busy,TxD2Num,TxD2pt,RxD2pt,RxD2Buf;
u8  u8TxD3Busy,TxD3Num,TxD3pt,RxD3pt,RxD3Buf;
u8  u8ErrCode;
u8	UART1_RX_Flag;

u8  u8test1,u8test2,u8test3,u8test4,u8test5,u8test6,u8test7,u8test8;

u8  u8Uart2_flag,u8Uart2_flag_test;

int32_t recv_uart2_M1_val,recv_uart2_M2_val,recv_uart2_M3_val;

int32_t temp_val;
u8  rData1Temp,rData2Temp;
//----------------------------------------------------------
//
//----------------------------------------------------------
void uart2_init(u32 bound)
{
    // 1. 时钟使能
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;     // 使能 DMA1 时钟
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;  // 使能 DMAMUX 时钟

    // 2. GPIO 配置 (PB3=TX, PB4=RX)
    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER |= (GPIO_MODE_AF << GPIO_MODER_MODE3_Pos) | 
                    (GPIO_MODE_AF << GPIO_MODER_MODE4_Pos);
    GPIO_AF_Set(GPIOB, 3, 7);
    GPIO_AF_Set(GPIOB, 4, 7);

    // 3. USART2 基本参数配置
    USART2->BRR = 170000000 / bound;
    USART2->CR3 |= USART_CR3_DMAT;          // 【关键】使能串口 DMA 发送请求
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // 4. DMAMUX 配置 (将 DMA1 Channel 1 映射到 USART2_TX)
    // G431 的 DMAMUX 请求 ID: USART2_TX 是 27
    DMAMUX1_Channel0->CCR = (27U << DMAMUX_CxCR_DMAREQ_ID_Pos); 

    // 5. DMA1_Channel1 配置
    DMA1_Channel1->CPAR = (uint32_t)&USART2->TDR; // 外设地址
    DMA1_Channel1->CCR = 0;                       // 复位
    DMA1_Channel1->CCR |= DMA_CCR_DIR;            // 从内存到外设
    DMA1_Channel1->CCR |= DMA_CCR_MINC;           // 内存地址递增
    DMA1_Channel1->CCR |= (1U << DMA_CCR_PL_Pos); // 中等优先级
}

//----------------------------------------------------------
// USART2 中断服务程序
//----------------------------------------------------------
void USART2_IRQHandler(void)
{
    if(USART2->ISR & USART_ISR_RXNE) // 接收到数据
    {
        uint8_t res = USART2->RDR;   // 读取数据清除标志位
        // 可以在这里处理接收
    }
}
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
//--------------------------------------------------------------------------
// @brief  重定向 printf 到串口 2
//--------------------------------------------------------------------------
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

