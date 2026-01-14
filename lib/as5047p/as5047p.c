#include "as5047p.h"
//===============================================
//
//===============================================
// 预先计算好的读取 ANGLECOM 寄存器的指令 (包含校验位和读位)
// 地址 0x3FFF (ANGLECOM), 读位 0x4000, 校验位计算后 Bit15=1 -> 0xFFFF
#define READ_ANGLE_CMD 0xFFFF
//------------------------------------------------------------
/**
 * @brief 极短延迟以适配 170MHz 主频
 */
static void SPI_Delay(void) {
    for(volatile int d = 0; d < 5; d++); 
}

/**
 * @brief 计算 AS5047P 的偶校验位
 * @param value: 包含读写位和地址的 15 位数据 (Bit 0-14)
 * @return 返回 0 或 1，用于设置第 15 位
 */
static uint16_t CalcEvenParity(uint16_t value) {
    uint16_t count = 0;
    // 只计算低 15 位 (Bit 0 ~ 14)，因为第 15 位是我们要设置的校验位
    for (int i = 0; i < 15; i++) {
        if (value & (1U << i)) {
            count++;
        }
    }
    // 如果 1 的个数是奇数，则返回 1，凑成偶数；如果是偶数，则返回 0
    return (count % 2); 
}

/**
 * @brief 初始化 AS5047P 模拟 SPI 引脚
 * 引脚映射参考 UM2516 Table 4 
 */
void AS5047P_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // PA15 (NSS) - 推挽输出 
    GPIOA->MODER &= ~(GPIO_MODER_MODE15);
    GPIOA->MODER |= (1U << GPIO_MODER_MODE15_Pos);
    
    // PB8 (SCK), PB6 (MOSI) - 推挽输出 
    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE6);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE8_Pos) | (1U << GPIO_MODER_MODE6_Pos);

    // PB7 (MISO) - 关键修改：暂时去掉内部上拉，测试是否变为 0x0000
    // 如果去掉上拉后读到 0x0000，说明 MISO 没接通；如果依然是 0xFFFF，说明 MISO 外部短路到 3.3V
    GPIOB->MODER &= ~(GPIO_MODER_MODE7);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7); // 设为 None (无上下拉)
    GPIOB->PUPDR |= (2U << GPIO_PUPDR_PUPD7_Pos); // 开启内部下拉 (Pull-down)

    AS_NSS_H;
    AS_SCK_L;
}

/**
 * @brief 模拟 SPI 16位数据传输
 */
uint16_t SPI_Transfer(uint16_t data) {
    uint16_t read_val = 0;
    AS_SCK_L; // 确保起始为低
    AS_NSS_L;
    for(volatile int d = 0; d < 200; d++); // 给芯片一点反应时间

    for (int i = 15; i >= 0; i--) {
        if (data & (1U << i)) AS_MOSI_H; else AS_MOSI_L;
        SPI_Delay();

        AS_SCK_H; // 芯片在此边缘采样 MOSI
        SPI_Delay();

        AS_SCK_L; // 芯片在此边缘更新 MISO
        SPI_Delay();

        read_val <<= 1;
        if (AS_MISO_IN) read_val |= 0x01; // 下降沿后采样
    }
    AS_NSS_H;
    return read_val;
}
/**
 * @brief 获取 14 位绝对角度
 * @return 0-16383 的数值，0xFFFF 表示错误
 */
uint16_t AS5047P_GetAngle(void) {
    uint16_t cmd = 0x4000 | AS5047P_REG_ANGLECOM; 
    if (CalcEvenParity(cmd)) cmd |= 0x8000;

    SPI_Transfer(cmd); 
    uint16_t raw = SPI_Transfer(cmd); 

    // --- 关键修改：检查错误位并屏蔽高2位 ---
    if (raw & 0x4000) { // Bit 14 是错误标志
        // 可以记录一个错误计数器
        return 0xFFFF; 
    }
    return (raw & 0x3FFF); // 只取低 14 位
}

uint16_t AS5047P_ReadRegister(uint16_t addr) {
    // 1. 合成基础指令：Bit 14 = 1 (读操作), Bit 13:0 = 地址
    uint16_t command = 0x4000 | (addr & 0x3FFF); 
    
    // 2. 重新核对校验计算逻辑
    if (CalcEvenParity(command)) {
        command |= 0x8000; // 如果 1 的个数是奇数，Bit 15 置 1 凑成偶数
    }
    
    // 3. 必须发送两次：第一次发送地址，第二次获取数据
    SPI_Transfer(command); 
    return SPI_Transfer(command);
}

/**
 * @brief 极速版模拟 SPI，专门为 15kHz FOC 优化
 * 耗时控制在 5-8us 左右
 */
__attribute__((always_inline)) inline uint16_t AS5047P_Transfer_Fast(uint16_t data) {
    uint16_t read_val = 0;
    
    AS_NSS_L;
    // 去掉原来的 200 次 delay，AS5047P 的 tCSN 只需要 10ns
    for(volatile int d = 0; d < 5; d++); 

    for (int i = 15; i >= 0; i--) {
        // SCK 初始为低 (Mode 1: CPOL=0, CPHA=1)
        if (data & (1U << i)) AS_MOSI_H; else AS_MOSI_L;
        
        AS_SCK_H; // 芯片在上升沿采样 MOSI
        // 缩短延时，G431 170MHz 下 3-5 次循环足以满足 10MHz 速率
        for(volatile int d = 0; d < 3; d++); 

        read_val <<= 1;
        if (AS_MISO_IN) read_val |= 0x01; 

        AS_SCK_L; // 芯片在下降沿更新 MISO
        for(volatile int d = 0; d < 3; d++); 
    }
    
    AS_NSS_H;
    return read_val;
}

/**
 * @brief 在 FOC 中断中调用的函数
 */
uint16_t FOC_ReadAngle_Optimized(void) {
    // 每次中断只读写一次
    // 返回的是上一次中断时发送 READ_ANGLE_CMD 后产生的角度数据
    uint16_t raw = AS5047P_Transfer_Fast(READ_ANGLE_CMD);
    return (raw & 0x3FFF); 
}


