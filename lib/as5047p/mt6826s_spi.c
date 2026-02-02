#include "mt6826s_spi.h"
#include <stdio.h>
#include "delay.h"
//===============================================
//
//===============================================
/**
 * @brief 简单的微秒级延迟 (针对 170MHz 频率优化)
 */
static void SPI_Delay(void) {
    for (volatile int i = 0; i < 200; i++); 
}
/**
 * @brief 初始化模拟 SPI 引脚
 */
void MT6826S_SPI_Init(void) {
    // 1. 开启 GPIOA, GPIOB, GPIOC 时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // 2. 配置 NSS (PC10 输出推挽高速)
    GPIOC->MODER   &= ~(GPIO_MODER_MODE10_Msk);
    GPIOC->MODER   |=  (GPIO_MODER_MODE10_0);  // 输出模式
    GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT10);     // 推挽
    GPIOC->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0);  // 高速

    // 3. 配置 SCK (PA11 输出推挽高速)
    GPIOA->MODER   &= ~(GPIO_MODER_MODE11_Msk);
    GPIOA->MODER   |=  (GPIO_MODER_MODE11_0);
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT11);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED11_0);

    // 4. 配置 MOSI (PB3 输出推挽高速)
    GPIOB->MODER   &= ~(GPIO_MODER_MODE3_Msk);
    GPIOB->MODER   |=  (GPIO_MODER_MODE3_0);
    GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT3);
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED3_0);

    // // 4. 配置 MOSI (PA10 输出推挽高速)
    // GPIOA->MODER   &= ~(GPIO_MODER_MODE10_Msk);
    // GPIOA->MODER   |=  (GPIO_MODER_MODE10_0);
    // GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT10);
    // GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0);
    
    // 5. 配置 MISO (PA15 输入 + 上拉)
    GPIOA->MODER   &= ~(GPIO_MODER_MODE15_Msk);               // 输入模式 (00)
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD15_Msk);
    GPIOA->PUPDR   |=  (GPIO_PUPDR_PUPD15_0);                 // 上拉 (01)

    // 初始状态
    AS_NSS_H;
    AS_SCK_L;
    AS_MOSI_H;

    delay_ms(10);  // 上电稳定时间
}

/**
 * @brief 模拟 SPI 16位数据传输
 */
uint16_t SPI_Transfer(uint16_t data_out)
{
    uint16_t data_in = 0;

    AS_NSS_L;
    for (volatile int d=0; d<20; d++) __NOP(); 

    for (int i = 15; i >= 0; i--) {
        // 1. SCK 上升沿：此时传感器开始切换 MISO 电平
        AS_SCK_H; 
        
        // 2. 在 SCK 高电平期间设置 MOSI
        if (data_out & (1U << i)) AS_MOSI_H; else AS_MOSI_L;

        for (volatile int d=0; d<40; d++) __NOP(); // 保持高电平宽度

        // 3. SCK 下降沿：核心时刻！
        // 此时：传感器采样 MOSI，同时 MISO 电平早已稳定，MCU 采样 MISO
        AS_SCK_L; 
        
        for (volatile int d=0; d<10; d++) __NOP(); // 短暂等待下降沿稳定
        if (AS_MISO_IN) data_in |= (1U << i);

        for (volatile int d=0; d<30; d++) __NOP(); // 保持低电平宽度
    }

    AS_NSS_H;
    for (volatile int d=0; d<100; d++) __NOP(); 

    return data_in;
}
// ====================== 辅助函数：计算 Even Parity ======================
static uint8_t CalcEvenParity(uint16_t value) {
    value &= 0x7FFF;  // 只计算低15位
    uint8_t parity = 0;
    while (value) {
        parity ^= (value & 1);
        value >>= 1;
    }
    return parity;  // 返回 0=偶数个1, 1=奇数个1
}

// ====================== 写寄存器（修正版）======================
void MT6826S_WriteReg(uint16_t addr, uint16_t data)
{
    uint16_t cmd_frame;
    uint16_t data_frame;
    uint8_t parity;

    // --- 第一帧：写命令帧 ---
    // Bit[15]: Parity (待计算)
    // Bit[14]: R/W = 0 (写操作)
    // Bit[13:0]: 寄存器地址
    cmd_frame = (addr & 0x3FFF);  // bit14=0 自动满足写操作
    
    parity = CalcEvenParity(cmd_frame);
    if (parity) cmd_frame |= 0x8000;  // 如果低15位有奇数个1，则bit15=1

    SPI_Transfer(cmd_frame);
    for (volatile int d=0; d<150; d++) __NOP();  // 帧间延迟 >350ns

    // --- 第二帧：数据帧 ---
    // Bit[15]: Parity
    // Bit[14]: PARC (上一帧的奇偶校验错误标志，写入时忽略)
    // Bit[13:0]: 数据
    data_frame = (data & 0x3FFF);  // 只取低14位数据
    
    parity = CalcEvenParity(data_frame);
    if (parity) data_frame |= 0x8000;

    SPI_Transfer(data_frame);
    for (volatile int d=0; d<150; d++) __NOP();
}

// ====================== 读寄存器（修正版）======================
uint16_t MT6826S_ReadReg(uint16_t addr)
{
    uint16_t cmd_frame;
    uint16_t result;
    uint8_t parity;

    // --- 第一帧：读命令 ---
    // Bit[15]: Parity
    // Bit[14]: R/W = 1 (读操作)
    // Bit[13:0]: 地址
    cmd_frame = (addr & 0x3FFF) | 0x4000;  // bit14 = 1
    
    parity = CalcEvenParity(cmd_frame);
    if (parity) cmd_frame |= 0x8000;

    SPI_Transfer(cmd_frame);
    for (volatile int d=0; d<150; d++) __NOP();

    // --- 第二帧：读取数据 ---
    result = SPI_Transfer(0xC000);  // 发送NOP命令（地址0x3FFF的读命令）
    
    // 解析返回帧
    uint8_t ef = (result >> 14) & 1;           // Error Flag
    uint8_t received_parity = (result >> 15);  // Parity bit
    
    // 校验奇偶性
    uint8_t calc_parity = CalcEvenParity(result & 0x7FFF);
    
    // if (ef) {
    //     printf("AS5047P Read Error: EF=1, addr=0x%04X\r\n", addr);
    //     return 0xFFFF;
    // }
    
    // if (calc_parity != received_parity) {
    //     printf("AS5047P Parity Error: addr=0x%04X\r\n", addr);
    //     return 0xFFFF;
    // }

    return (result & 0x3FFF);  // 返回14位数据
}

