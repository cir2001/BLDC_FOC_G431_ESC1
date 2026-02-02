#include "mt6826s.h"
//----------------------------------------------------------------
// 使用TIM4
// PB6------A
// PB7------B
// PB8------Z
//----------------------------------------------------------------
float mechanical_offset = 0.0f; // 通过校准获得的偏差值

volatile uint32_t edge_count_A = 0;
volatile uint32_t edge_count_B = 0;

volatile uint32_t A_rise = 0, A_fall = 0;
volatile uint32_t B_rise = 0, B_fall = 0;
//===============================================================
void MT6826S_Init(void) {
    // 1. 开启时钟
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;

    // 2. GPIO 配置 (PB6=A, PB7=B, PB8=Z)
    GPIO_Set(GPIOB, PIN6 | PIN7 | PIN8, GPIO_MODE_AF, GPIO_MODE_AF, GPIO_SPEED_170M, GPIO_PUPD_PU); 

    GPIO_AF_Set(GPIOB, 6, 2); 
    GPIO_AF_Set(GPIOB, 7, 2); 
    GPIO_AF_Set(GPIOB, 8, 2); 

    // 2. 复位 TIM4 确保干净状态
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_TIM4RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM4RST;

    // 3. 输入捕获配置：A/B 用最常用滤波，Index 滤波适中
    TIM4->CCMR1 = (TIM4->CCMR1 & ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_CC2S_Msk | TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk))
                | (0x1 << TIM_CCMR1_CC1S_Pos)   // CH1 → TI1
                | (0x1 << TIM_CCMR1_CC2S_Pos)   // CH2 → TI2
                | (0x1 << TIM_CCMR1_IC1F_Pos)   // 滤波 f_DTS/16 ≈ 125kHz 适合 AS5047P
                | (0x1 << TIM_CCMR1_IC2F_Pos);

    TIM4->CCMR2 = (TIM4->CCMR2 & ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_IC3F_Msk))
                | (0x1 << TIM_CCMR2_CC3S_Pos)   // CH3 → TI3 (Index)
                | (0x6 << TIM_CCMR2_IC3F_Pos);  // Index 滤波稍松一点，避免误触发

    // 4. Encoder 模式：Mode 3（双边沿计数 + 方向检测）
    TIM4->SMCR &= ~TIM_SMCR_SMS_Msk;
    // TIM4->SMCR |= 0x3 << TIM_SMCR_SMS_Pos;   // SMS = 011 → Encoder mode 3
    TIM4->SMCR |= 0x0003;   // SMS = 011 → Encoder mode 3

    // 5. 关键：启用 Index 硬件复位（TIMx_ECR）
    TIM4->ECR = 0;  // 先清零
    TIM4->ECR |= (1U << TIM_ECR_IE_Pos);          // IE = 1：启用 Index 功能
    TIM4->ECR |= (3U << TIM_ECR_IPOS_Pos);           // IPOS 
    // TIM4->ECR &= ~TIM_ECR_IPOS_Msk;        // 确保 IPOS=00 (Index 信号为高时触发复位)
    
    // 7. 极性配置 - 改为 both edges for CH1 and CH2
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | 
                    TIM_CCER_CC2P | TIM_CCER_CC2NP);


    // 启用通道
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

    // 8. ARR 设置
    // 暂时设为 4000 以上。如果 Index 触发成功，你会看到 CNT 在 4000 左右自动跳回 0
    TIM4->PSC = 0;
    TIM4->ARR = 16383; 
    TIM4->CNT = 0;
    TIM4->EGR |= TIM_EGR_UG; // 强制触发更新事件，将 PSC=0 立即装载到硬件逻辑中

    TIM4->CR1 |= TIM_CR1_CEN;
}
/**
 * @brief TIM4 中断服务函数
 * 当 Z 相 (Index) 信号出现时，将计数器清零
 */
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_CC3IF) {
        TIM4->SR &= ~TIM_SR_CC3IF; // 清除标志位
        TIM4->CNT = 0;             // 机械零位对齐：将计数值归零
    }
}

/**
 * @brief 获取原始脉冲计数值
 * @return 0 到 3999 之间的整数 (假设 1000线编码器)
 */
uint16_t Encoder_Get_Raw_CNT(void) {
    return (uint16_t)(TIM4->CNT);
}

/**
 * @brief 获取旋转方向
 * @return 0: 递增 (通常为顺时针), 1: 递减 (通常为逆时针)
 */
uint8_t Encoder_Get_Direction(void) {
    // CR1 寄存器的位 4 (DIR) 记录了当前的计数方向
    return (uint8_t)((TIM4->CR1 & TIM_CR1_DIR) >> TIM_CR1_DIR_Pos);
}

/**
 * @brief 获取当前机械角度 (0.0 - 359.91 度)
 */
float Encoder_Get_Mechanical_Angle(void) {
    // 4000.0f 是 1000线 * 4倍频 得到的结果
    return ((float)TIM4->CNT / 4000.0f) * 360.0f;
}
/**
 * @brief 获取校正后的角度
 */
float Get_Corrected_Angle(void) {
    float raw_angle = Encoder_Get_Mechanical_Angle();
    float corrected_angle = raw_angle - mechanical_offset;
    
    // 范围限幅到 0~360
    if (corrected_angle < 0) corrected_angle += 360.0f;
    if (corrected_angle >= 360.0f) corrected_angle -= 360.0f;
    
    return corrected_angle;
}

// PC10--引脚 拉高，让as5047的CS为高电平，支持ABI模式
void GPIO_CS_High_Init(void) {
    // 1. 开启 GPIOC 时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // 2. 配置 PC10 为输出模式 (01)
    GPIOC->MODER &= ~GPIO_MODER_MODE10;
    GPIOC->MODER |= (1 << GPIO_MODER_MODE10_Pos);

    // 3. 配置为推挽输出 (0), 低速即可 (00)
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT10;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10;

    // 4. 关键：立即置高电平
    // 使用 BSRR 寄存器，位 10 置 1
    GPIOC->BSRR = GPIO_BSRR_BS10; 
}


/**
 * @brief 配置 EXTI 监控 PB6 和 PB7 的所有边沿
 */
void Encoder_Diagnosis_EXTI_Init(void) {
    // 1. 确保 GPIOB 时钟和 SYSCFG 时钟开启
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // 2. 将 EXTI 6/7 线路映射到 GPIOB (PB6, PB7)
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI6_Msk | SYSCFG_EXTICR2_EXTI7_Msk);
    SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PB | SYSCFG_EXTICR2_EXTI7_PB);

    // 3. 配置为双边沿触发 (Rising & Falling)
    EXTI->RTSR1 |= (EXTI_RTSR1_RT6 | EXTI_RTSR1_RT7);
    EXTI->FTSR1 |= (EXTI_FTSR1_FT6 | EXTI_FTSR1_FT7);

    // 4. 开启中断屏蔽位
    EXTI->IMR1 |= (EXTI_IMR1_IM6 | EXTI_IMR1_IM7);

    // 5. 设置 NVIC (注意：EXTI9_5 是同一个中断向量)
    NVIC_SetPriority(EXTI9_5_IRQn, 0); 
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief EXTI 中断服务函数
 */
void EXTI9_5_IRQHandler(void) {
// 诊断 A 相 (PB6)
    if (EXTI->PR1 & EXTI_PR1_PIF6) {
        if (GPIOB->IDR & GPIO_IDR_ID6) {
            A_rise++;
        } else {
            A_fall++;
        }
        EXTI->PR1 = EXTI_PR1_PIF6;
    }

    // 诊断 B 相 (PB7)
    if (EXTI->PR1 & EXTI_PR1_PIF7) {
        if (GPIOB->IDR & GPIO_IDR_ID7) {
            B_rise++;
        } else {
            B_fall++;
        }
        EXTI->PR1 = EXTI_PR1_PIF7;
    }
}

void Encoder_Diagnosis_Init(void) {
    // 1. 确保 PB6, PB7 为输入模式
    GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
    
    // 2. 配置 EXTI 映射
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI6_Msk | SYSCFG_EXTICR2_EXTI7_Msk);
    SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PB | SYSCFG_EXTICR2_EXTI7_PB);

    // 3. 关键：同时开启上升沿和下降沿触发
    EXTI->RTSR1 |= (EXTI_RTSR1_RT6 | EXTI_RTSR1_RT7); // 上升沿
    EXTI->FTSR1 |= (EXTI_FTSR1_FT6 | EXTI_FTSR1_FT7); // 下降沿

    EXTI->IMR1  |= (EXTI_IMR1_IM6 | EXTI_IMR1_IM7);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}




