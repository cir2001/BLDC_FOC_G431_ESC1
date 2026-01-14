#include "adc_foc.h"
#include "delay.h"
#include <stdio.h>

// 全局变量定义
float offset_u = 0.0f, offset_v = 0.0f, offset_w = 0.0f;
float i_u = 0, i_v = 0, i_w = 0;

/**
 * @brief ADC1/2 寄存器初始化 (注入组 + TIM1触发)
 */
void ADC_Init_Registers(void) {
    printf("\r\n[ADC System] Starting Final Robust Initialization...\r\n");

    // 1. 开启时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    
    // 强制复位 ADC，确保所有内部标志位清零
    RCC->AHB2RSTR |= (1U << 13);
    delay_ms(10);
    RCC->AHB2RSTR &= ~(1U << 13);

    // 2. --- 核心修改：切换到同步时钟模式 ---
    // 放弃 RCC_CCIPR 的异步配置，直接使用总线同步时钟
    // 设置 CKMODE = 01 (HCLK/1)，即 ADC 运行在 170MHz
    // 同时设置 PRESC = 0010 (即 4 分频，170/4 = 42.5MHz，非常理想的频率)
    ADC12_COMMON->CCR = 0; // 先清空
    ADC12_COMMON->CCR |= (1U << 16); // CKMODE = 01 (Sync HCLK/1)
    ADC12_COMMON->CCR |= (2U << 18); // PRESC = 0010 (4分频)
    
    printf("[ADC System] 1. Clock set to Synchronous Mode (HCLK/4).\r\n");

    // 3. 开启调节器
    ADC1->CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1->CR |= (1U << 28);  // ADVREGEN = 1
    ADC2->CR &= ~(1U << 29);
    ADC2->CR |= (1U << 28);
    delay_ms(10); 
    printf("[ADC System] 2. Voltage Regulators enabled.\r\n");

    // 4. ADC1 校准 (增加手动复位确认)
    printf("[ADC System] 3. Calibrating ADC1...\r\n");
    ADC1->CR &= ~ADC_CR_ADEN; // 确保 ADEN 为 0，否则校准必卡死
    ADC1->CR |= ADC_CR_ADCAL;
    
    uint32_t timeout = 2000000; // 足够长的手动超时
    while((ADC1->CR & ADC_CR_ADCAL) && timeout--); 
    
    if(timeout == 0) {
        printf("[ADC System] ERROR: ADC1 Calibration STUCK! Check PLL settings.\r\n");
        return; 
    }
    printf("[ADC System]    ADC1 Done.\r\n");

    // 5. ADC2 校准
    printf("[ADC System] 4. Calibrating ADC2...\r\n");
    ADC2->CR &= ~ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADCAL;
    timeout = 2000000;
    while((ADC2->CR & ADC_CR_ADCAL) && timeout--); 
    printf("[ADC System]    ADC2 Done.\r\n");

    // 6. 开启 ADC 并等待就绪 (ADRDY)
    ADC1->ISR |= ADC_ISR_ADRDY; // 写 1 清除标志
    ADC1->CR  |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    
    ADC2->ISR |= ADC_ISR_ADRDY;
    ADC2->CR  |= ADC_CR_ADEN;
    while(!(ADC2->ISR & ADC_ISR_ADRDY));
    printf("[ADC System] 6. Both ADCs are READY.\r\n");

    // --- 采样时间配置 ---
    // ADC1: 通道 13 (Phase U)
    ADC1->SMPR2 &= ~(7U << 9); 
    ADC1->SMPR2 |= (6U << 9);     // 92.5 周期

    // ADC2: 通道 16 (Phase V) 和 通道 18 (Phase W)
    ADC2->SMPR2 &= ~((7U << 18) | (7U << 24));
    ADC2->SMPR2 |= (6U << 18);    // 通道 16 -> 92.5 周期
    ADC2->SMPR2 |= (6U << 24);    // 通道 18 -> 92.5 周期 (必须添加！)

    // --- 注入序列配置 ---

    // ADC1 负责 Phase U (OPAMP1)
    ADC1->JSQR = (0U << 0)         // JL = 0 (总共 1 个转换)
            | (8U << 2)         // JEXTSEL = 8 (TIM1_TRGO2)
            | (1U << 7)         // 允许外部触发
            | (13U << 9);       // JSQ1 = 通道 13 (OPAMP1 -> Phase U)

    // --- ADC2 配置 (V相 + W相) ---
    ADC2->JSQR = (1U << 0)         // JL = 1 (总共 2 个转换)
            | (8U << 2)         // JEXTSEL = 8 (TIM1_TRGO2)
            | (1U << 7)         // 允许外部触发
            | (16U << 9)        // JSQ1 = 通道 16 (OPAMP2 -> Phase V)
            | (18U << 15);      // JSQ2 = 通道 18 (OPAMP3 -> Phase W)

    printf("[ADC System] 7. Injected sequences FIXED. READY FOR FOC.\r\n\r\n");

}

/**
 * @brief 电流零位校准 (需要在电机静止、PWM未输出时调用)
 * 
    运放放大后的 1.65V（对应 0A）在每个板子上都有微小差异。
    如果在 main 里不校准，直接在中断里减去固定的 2048，
    算出来的力矩电流 I_q 会有一个很大的初始偏置，电机静止时也会嗡嗡响
 */
void Calibrate_Current_Offset(void) {
    // 临时关断 PWM 主输出，确保功率级完全断开
    uint16_t backup_bdtr = TIM1->BDTR;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    const int samples = 1024;

    printf("[Calib] Starting Current Offset Calibration...\r\n");

    // 关键：校准时必须临时关闭硬件触发 (JEXTEN = 0)，否则 JADSTART 无效
    ADC1->JSQR &= ~ADC_JSQR_JEXTEN;
    ADC2->JSQR &= ~ADC_JSQR_JEXTEN;

    for(int i = 0; i < samples; i++) {
        ADC1->CR |= ADC_CR_JADSTART;
        ADC2->CR |= ADC_CR_JADSTART;
        
        while(!(ADC2->ISR & ADC_ISR_JEOC)); // 等待转换完成
        
        sum_u += ADC1->JDR1;
        sum_v += ADC2->JDR1;
        sum_w += ADC2->JDR2;

        ADC1->ISR |= ADC_ISR_JEOC; // 清除标志
        ADC2->ISR |= ADC_ISR_JEOC;
    }

    offset_u = (float)sum_u / (float)samples;
    offset_v = (float)sum_v / (float)samples;
    offset_w = (float)sum_w / (float)samples;

    // 恢复硬件触发模式 (TIM1_TRGO2 上升沿)
    ADC1->JSQR |= (1U << ADC_JSQR_JEXTEN_Pos);
    ADC2->JSQR |= (1U << ADC_JSQR_JEXTEN_Pos);

    // 将原来的 float 打印改为整数打印，排除浮点打印补丁的问题
    printf("[Calib] Done. Offsets: U:%d, V:%d, W:%d\r\n", (int)offset_u, (int)offset_v, (int)offset_w);

    // 校准完还原
    TIM1->BDTR = backup_bdtr;
}

/**
 * @brief 在 FOC 中断中调用的电流读取函数
 */
void Get_Current_Phases(void) {
    // 直接从数据寄存器读取
    // 注意：在 TIM1 中断中，由于是硬件同步触发，进入中断时转换通常已完成
    i_u = ((float)ADC1->JDR1 - offset_u) * I_COEFF;
    i_v = ((float)ADC2->JDR1 - offset_v) * I_COEFF;
    i_w = ((float)ADC1->JDR2 - offset_w) * I_COEFF;
}



