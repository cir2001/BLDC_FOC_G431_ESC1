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

    ADC1->CFGR2 &= ~ADC_CFGR2_JOVSE; // 关闭注入组过采样
    ADC2->CFGR2 &= ~ADC_CFGR2_JOVSE;

    // --- 采样时间优化配置 ---
    // 目标：将采样时间压缩到 0.2us ~ 0.5us 左右
    uint32_t smp_time = 3U; 
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
// void Calibrate_Current_Offset(void) 
// {
//     printf("[System] Starting Synchronous Dynamic Calibration...\r\n");

//     // 1. 预设占空比为 50%
//     TIM1->CCR1 = PWM_ARR / 2;
//     TIM1->CCR2 = PWM_ARR / 2;
//     TIM1->CCR3 = PWM_ARR / 2;
//     // 确保采样点在波谷（根据之前的调试，建议设在 ARR - 50 或 -100）
//     TIM1->CCR4 = PWM_ARR - 100; 

//     // 2. 强制产生一次更新事件，使 CCR 寄存器立即生效
//     TIM1->EGR |= TIM_EGR_UG;

//     // 3. 开启功率输出 (MOE) 并 启动定时器 (CEN)
//     // 只有 CEN=1 后，定时器才开始计数，TRGO2 才会开始按频率产生
//     TIM1->BDTR |= TIM_BDTR_MOE; 
//     TIM1->CR1  |= TIM_CR1_CEN;

//     // 4. 给自举电容充电和运放电路稳定留出时间
//     delay_ms(200); 

//     uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
//     const int samples = 2000; 

//     // 5. 进入同步采样循环
//     for(int i = 0; i < samples; i++) {
//         // 等待硬件触发导致的注入组转换完成 (JEOS)
//         // 注意：这里不需要 ADC_CR_JADSTART，硬件会自动触发
//         uint32_t timeout = 100000;
//         while(!(ADC2->ISR & ADC_ISR_JEOS) && timeout--);

//         if(timeout == 0) {
//             printf("Error: ADC Hardware Trigger Timeout! Check TRGO2 settings.\r\n");
//             break;
//         }

//         sum_u += ADC1->JDR1;
//         sum_v += ADC2->JDR1;
//         sum_w += ADC2->JDR2;

//         // 清除标志位，等待下一个 PWM 周期的自动触发
//         ADC1->ISR |= ADC_ISR_JEOS;
//         ADC2->ISR |= ADC_ISR_JEOS;
//     }

//     // 6. 计算平均偏移
//     offset_u = (float)sum_u / samples;
//     offset_v = (float)sum_v / samples;
//     offset_w = (float)sum_w / samples;

//     // 7. 校准结束，关闭输出（或者保持开启直接进入对齐阶段）
//     TIM1->BDTR &= ~TIM_BDTR_MOE;
//     // 如果后续紧接着对齐逻辑，可以不关 CEN，只改 CCR 即可

//     printf("[Calib] Sync Done. U:%d, V:%d, W:%d\r\n", (int)offset_u, (int)offset_v, (int)offset_w);
// }

// 

void Calibrate_Current_Offset(void) 
{
    printf("[System] Starting Automatic Synchronous Calibration...\r\n");

   // 1. 强制占空比 50%，开启定时器产生触发信号
    TIM1->CCR1 = PWM_ARR / 2;
    TIM1->CCR2 = PWM_ARR / 2;
    TIM1->CCR3 = PWM_ARR / 2;
    TIM1->CCR4 = PWM_ARR - 50; // 预留 100 ticks 确保在下桥臂导通窗口内

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1  |= TIM_CR1_CEN;
    delay_ms(100);

    // 2. 【最重要的一步】启动 ADC 的硬件触发监听模式
    // 只有执行了这一步，ADC 才会开始等待 CCR4 的上升沿
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    int success_count = 0;

    for(int i = 0; i < 2000; i++) 
    {
        uint32_t timeout = 200000;
        // 3. 等待硬件触发完成 (JEOS 代表整个序列转换完成)
        while(!(ADC2->ISR & ADC_ISR_JEOS) && timeout--);

        if(timeout > 0) {
            sum_u += ADC1->JDR1;
            sum_v += ADC2->JDR1;
            sum_w += ADC2->JDR2;
            success_count++;
            
            // 4. 清除标志位，ADC 会自动等待下一个 PWM 周期的 CCR4 信号
            ADC1->ISR |= ADC_ISR_JEOS;
            ADC2->ISR |= ADC_ISR_JEOS;
        }
    }

    // 5. 停止监听（正式跑 FOC 时再开启）
    // 注意：在正式 FOC 循环开始前，需再次执行 JADSTART 开启监听

    if(success_count > 0) {
        offset_u = (float)sum_u / success_count;
        offset_v = (float)sum_v / success_count;
        offset_w = (float)sum_w / success_count;
        printf("[Calib] CCR4 Success! U:%.1f, V:%.1f, W:%.1f\r\n", offset_u, offset_v, offset_w);
    } else {
        printf("[Calib] FAILED: No Hardware Triggers detected.\r\n");
    }

    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

