#include "sys.h"
#include "usart.h"
#include <stdio.h>  
#include <errno.h>
#include <sys/unistd.h>
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
    // 1. 使能 GPIOB 和 USART2 时钟 (注意是 USART2)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // USART2 在 APB1 线上

    // 2. 配置 PB3 (TX) 和 PB4 (RX) 为复用模式 (AF7)
    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOB->MODER |= (GPIO_MODE_AF << GPIO_MODER_MODE3_Pos) | 
                     (GPIO_MODE_AF << GPIO_MODER_MODE4_Pos);

    // 设置复用功能编号为 7 (针对 G431，USART2 的 PB3/PB4 也是 AF7)
    GPIO_AF_Set(GPIOB, 3, 7);
    GPIO_AF_Set(GPIOB, 4, 7);

    // 3. 配置波特率 (假设 PCLK1 = 170MHz)
    USART2->BRR = 170000000 / bound;

    // 4. 使能
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // 6. (可选) 开启接收中断
    //USART2->CR1 |= USART_CR1_RXNEIE;
    //MY_NVIC_Init(3, 3, USART2_IRQn, 2); // 优先级设为中等
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
//--------------------------------------------------------------------------
// @brief  串口1发送一个字节
//---------------------------------------------------------------------------
void uart2_send_byte(u8 data)
{
    while (!(USART2->ISR & USART_ISR_TXE)); 
    USART2->TDR = data;
}
//--------------------------------------------------------------------------
// @brief  重定向 printf 到串口 1
//--------------------------------------------------------------------------
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        uart2_send_byte((uint8_t)ptr[i]);
    }
    return len;
}


