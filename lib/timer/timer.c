#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 	
//-----------------------------------------------
// 外部函数声明
//-----------------------------------------------

//-----------------------------------------------
//外部变量声明
//-----------------------------------------------
extern u16 oled_tick;
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
volatile int Target_Speed_M1 = 0; 
volatile int Target_Speed_M2 = 0;
volatile int Target_Speed_M3 = 0;

volatile uint32_t g_system_tick = 0; // 全局毫秒计数

int16_t cnt;
int16_t iMotorA_Encoder,iMotorB_Encoder,iMotorC_Encoder;
long int iMotorAPulseTotle,iMotorBPulseTotle,iMotorCPulseTotle;

u8 timer2_Counter;
//==============================================
// TIM1 更新中断服务函数
//==============================================
void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM1->SR&0X0001) // 检查更新中断标志位
    {
		//LED_U1=!LED_U1;
        // 在这里执行您的代码
    }
	TIM1->SR&=~(1<<0); // 清除更新中断标志位
}
/**
 * @brief  初始化 TIM1 产生中心对齐 PWM (避坑版)
 * @param  arr: 自动重装载值 (170MHz下, 5666对应15kHz)
 * @param  psc: 分频系数 (通常为0)
 */
void TIM1_PWM_Init(u16 arr)
{
    // 1. 开启 GPIOA, GPIOB, GPIOC 和 TIM1 时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. 配置引脚复用 (根据 UM2516 Table 4 定义)
    // PA8, PA9, PA10 -> AF6 (TIM1_CH1/2/3)
    GPIO_AF_Set(GPIOA, 8, 6); 
    GPIO_AF_Set(GPIOA, 9, 6); 
    GPIO_AF_Set(GPIOA, 10, 6);
    
    // PC13 -> AF2 (TIM1_CH1N) [手册规定映射]
    GPIO_AF_Set(GPIOC, 13, 2); 
    // PA12 -> AF6 (TIM1_CH2N) [手册规定映射]
    GPIO_AF_Set(GPIOA, 12, 6); 
    // PB15 -> AF4 (TIM1_CH3N) [手册规定映射]
    GPIO_AF_Set(GPIOB, 15, 4);

    // 4. 定时器基础设置
    TIM1->ARR = arr;
    TIM1->PSC = 0;

    // 5. 设置中心对齐模式 1 (向上/向下计数均在达到比较值时翻转)
    // 这种模式下采样最准
    TIM1->CR1 &= ~TIM_CR1_CMS;
    TIM1->CR1 |= (1 << TIM_CR1_CMS_Pos); 

    // 6. 配置通道比较模式 (PWM模式1)
    // CCMR1 对应 CH1, CH2; CCMR2 对应 CH3
    // CCMR1 寄存器：控制 CH1 (低8位) 和 CH2 (高8位)
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; // CH1
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // CH2 对应 PA11

	// CCMR2 寄存器：控制 CH3 (低8位) 和 CH4 (高8位)
	TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; // CH3 对应 PA12

    // 7. 使能互补输出与极性
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

    // 8. 死区时间配置 (Dead-time)
    // G431 170MHz 主频，设置 150 约等于 880ns，足以保护大部分 MOSFET
    TIM1->BDTR &= ~TIM_BDTR_DTG;
    TIM1->BDTR |= 150; 

    // 9. 开启主输出 (MOE) 并启动定时器
    TIM1->BDTR |= TIM_BDTR_MOE; 
    TIM1->CR1 |= TIM_CR1_CEN;
    
    // 10. 触发信号：Update事件触发 TRGO (用于 ADC 同步采样)
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |= (2 << TIM_CR2_MMS_Pos);
}


