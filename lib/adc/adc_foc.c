#include "adc_foc.h"
#include "delay.h"
#include <stdio.h>
#include "timer.h"

// 全局变量定义
float offset_u = 0.0f, offset_v = 0.0f, offset_w = 0.0f;
float i_u = 0, i_v = 0, i_w = 0;

/**
 * @brief ADC1/2 寄存器初始化 (注入组 + TIM1触发)
 */
void ADC_Init_Registers(void) 
{
    // 1. 开启时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    
    // 强制复位 ADC，确保所有内部标志位清零
    RCC->AHB2RSTR |= (1U << 13);
    delay_ms(10);
    RCC->AHB2RSTR &= ~(1U << 13);

    // 2. --- 切换到同步时钟模式 ---
    // 放弃 RCC_CCIPR 的异步配置，直接使用总线同步时钟
    // 设置 CKMODE = 01 (HCLK/1)，即 ADC 运行在 170MHz
    // 同时设置 PRESC = 0010 (即 4 分频，170/4 = 42.5MHz，非常理想的频率)
    ADC12_COMMON->CCR = 0; // 先清空
    ADC12_COMMON->CCR |= (3U << 16); // CKMODE = 01 (Sync HCLK/1)
    ADC12_COMMON->CCR |= (2U << 18); // PRESC = 0010 (4分频)
    
    // 3. 开启调节器
    ADC1->CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1->CR |= (1U << 28);  // ADVREGEN = 1
    ADC2->CR &= ~(1U << 29);
    ADC2->CR |= (1U << 28);
    delay_ms(10); 

    // 4. ADC1 校准 (增加手动复位确认)
    ADC1->CR &= ~ADC_CR_ADEN; // 确保 ADEN 为 0，否则校准必卡死
    ADC1->CR |= ADC_CR_ADCAL;
    
    uint32_t timeout = 2000000; // 足够长的手动超时
    while((ADC1->CR & ADC_CR_ADCAL) && timeout--); 

    // 5. ADC2 校准
    ADC2->CR &= ~ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADCAL;
    timeout = 2000000;
    while((ADC2->CR & ADC_CR_ADCAL) && timeout--); 

    // 6. 开启 ADC 并等待就绪 (ADRDY)
    ADC1->ISR |= ADC_ISR_ADRDY; // 写 1 清除标志
    ADC1->CR  |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    
    ADC2->ISR |= ADC_ISR_ADRDY;
    ADC2->CR  |= ADC_CR_ADEN;
    while(!(ADC2->ISR & ADC_ISR_ADRDY));

    // --- 新增：硬件过采样配置 (针对注入组) ---
    // 目标：通过 8 倍过采样减小噪声，保持 12 位分辨率
    // ADC1 配置
    ADC1->CFGR2 &= ~(ADC_CFGR2_OVSR | ADC_CFGR2_OVSS | ADC_CFGR2_ROVSE | ADC_CFGR2_JOVSE); // 清零
    ADC1->CFGR2 |= (2U << ADC_CFGR2_OVSR_Pos);   // OVSR = 010: 8倍过采样 (Ratio = 8x)
    ADC1->CFGR2 |= (3U << ADC_CFGR2_OVSS_Pos);   // OVSS = 0011: 右移 3 位 (Shift = 3)
    ADC1->CFGR2 |= ADC_CFGR2_JOVSE;              // JOVSE = 1: 开启注入组过采样

    // ADC2 配置
    ADC2->CFGR2 &= ~(ADC_CFGR2_OVSR | ADC_CFGR2_OVSS | ADC_CFGR2_ROVSE | ADC_CFGR2_JOVSE); // 清零
    ADC2->CFGR2 |= (2U << ADC_CFGR2_OVSR_Pos);   // 8倍过采样
    ADC2->CFGR2 |= (3U << ADC_CFGR2_OVSS_Pos);   // 右移 3 位
    ADC2->CFGR2 |= ADC_CFGR2_JOVSE;              // 开启注入组过采样

    // --- 采样时间优化配置 ---
    // 目标：将采样时间压缩到 0.2us ~ 0.5us 左右
    // 2.5周期有时会导致采样阻抗不匹配，建议改为 6.5 周期 (010)
    uint32_t smp_time = 3U; // 6.5 cycles
    ADC1->SMPR2 &= ~(7U << 9); 
    ADC1->SMPR2 |= (smp_time << 9); 
    ADC2->SMPR2 &= ~((7U << 18) | (7U << 24));
    ADC2->SMPR2 |= (smp_time << 18);
    ADC2->SMPR2 |= (smp_time << 24);

    //----FGR 配置（最合适的位置）----
    // 禁用注入队列模式（我们只用单次注入，不需要队列）
    ADC1->CFGR |= ADC_CFGR_JQDIS;
    ADC2->CFGR |= ADC_CFGR_JQDIS;

    // 关闭连续转换模式（注入组是触发式，不是连续）
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    ADC2->CFGR &= ~ADC_CFGR_CONT;

    // --- 注入序列配置 ---
    // ADC1 负责 Phase U (通道 13 = OPAMP1)
    ADC1->JSQR = (0U << ADC_JSQR_JL_Pos)          // JL = 0 (1个转换)
            | (8U << ADC_JSQR_JEXTSEL_Pos)    // JEXTSEL = 36 → TIM1_TRGO2 (查 RM0440 Table 88)
            | (1U << ADC_JSQR_JEXTEN_Pos)      // JEXTEN = 01 → 上升沿触发（最稳）
            | (13U << ADC_JSQR_JSQ1_Pos);      // JSQ1 = 通道 13 (Phase U)

    // ADC2 负责 Phase V + W (通道 16=OPAMP2, 18=OPAMP3)
    ADC2->JSQR = (1U << ADC_JSQR_JL_Pos)          // JL = 1 (2个转换)
            | (8U << ADC_JSQR_JEXTSEL_Pos)    // 同上，TRGO2
            | (1U << ADC_JSQR_JEXTEN_Pos)      // 上升沿触发
            | (16U << ADC_JSQR_JSQ1_Pos)       // JSQ1 = 通道 16 (Phase V)
            | (18U << ADC_JSQR_JSQ2_Pos);      // JSQ2 = 通道 18 (Phase W)

    // --- 清除所有初始标志位 ---
    ADC1->ISR |= (ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_EOC | ADC_ISR_OVR);
    ADC2->ISR |= (ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_EOC | ADC_ISR_OVR);

}

/**
 * @brief 电流零位校准 (需要在电机静止、PWM未输出时调用)
 * 
    运放放大后的 1.65V（对应 0A）在每个板子上都有微小差异。
    如果在 main 里不校准，直接在中断里减去固定的 2048，
    算出来的力矩电流 I_q 会有一个很大的初始偏置，电机静止时也会嗡嗡响
 */
void Calibrate_Current_Offset(void) 
{
    printf("[System] Starting Dynamic Current Calibration...\r\n");

    // 1. 设置三相占空比为 50%（中值），并开启功率输出
    // 此时电角度虽然在旋转，但三相电压相等，电机电流理论为 0
    // 这样校准可以包含功率管开关带来的地弹噪声和电磁干扰
    TIM1->CCR1 = PWM_ARR / 2;
    TIM1->CCR2 = PWM_ARR / 2;
    TIM1->CCR3 = PWM_ARR / 2;
    TIM1->BDTR |= TIM_BDTR_MOE; 
    
    // 给功率级和自举电容足够的稳定时间
    delay_ms(500); 

    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    const int samples = 10000;

    // 2. 暂时关闭硬件触发，切换到软件手动触发采样
    ADC1->JSQR &= ~ADC_JSQR_JEXTEN;
    ADC2->JSQR &= ~ADC_JSQR_JEXTEN;

    for(int i = 0; i < samples; i++) {
        // 手动启动注入组转换
        ADC1->CR |= ADC_CR_JADSTART;
        ADC2->CR |= ADC_CR_JADSTART;
        
        // 等待转换完成 (使用 ADC2 作为同步参考)
        uint32_t wait_timeout = 2000;
        while(!(ADC2->ISR & ADC_ISR_JEOC) && wait_timeout--);

        sum_u += ADC1->JDR1;
        sum_v += ADC2->JDR1;
        sum_w += ADC2->JDR2;

        // 清除转换完成标志
        ADC1->ISR |= ADC_ISR_JEOC; 
        ADC2->ISR |= ADC_ISR_JEOC;
    }

    // 3. 计算最终偏移量
    offset_u = (float)sum_u / (float)samples;
    offset_v = (float)sum_v / (float)samples;
    offset_w = (float)sum_w / (float)samples;

    // 4. 恢复硬件触发模式 (准备进入 FOC 15kHz 中断逻辑)
    // 假设你使用的是 TIM1_TRGO2 触发，Pos 通常根据 CubeMX 配置确定
    ADC1->JSQR |= (1U << ADC_JSQR_JEXTEN_Pos);
    ADC2->JSQR |= (1U << ADC_JSQR_JEXTEN_Pos);

    // 5. 重要：校准完成后立即关闭 MOE 
    // 防止在进入 FOC 逻辑前产生不必要的静态发热
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    printf("[Calib] Dynamic Done. U:%d, V:%d, W:%d\r\n", (int)offset_u, (int)offset_v, (int)offset_w);

    // 6. 检查硬件一致性
    // 如果偏差过大，说明某相的运放电路或采样电阻可能存在硬件误差
    if (fabs(offset_u - offset_v) > 30 || fabs(offset_u - offset_w) > 30) 
    {
        printf("WARNING: Phase Offset Imbalance! U-V:%.1f, U-W:%.1f\r\n", 
                fabs(offset_u - offset_v), fabs(offset_u - offset_w));
    }
}



