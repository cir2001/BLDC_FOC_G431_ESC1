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
void TIM1_PWM_Init(u16 arr, u16 psc)
{
    // 1. 使能时钟
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // 2. 配置 PA8, PA11, PA12 为复用功能 (AF6)
    // 这种配置确保了不触碰 PA9 和 PA10
    GPIO_Set(GPIOA, PIN8 | PIN11 | PIN12, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_NONE);
    GPIO_AF_Set(GPIOA, 8, 6);
    GPIO_AF_Set(GPIOA, 11, 6);
    GPIO_AF_Set(GPIOA, 12, 6);
    
    // 3. 配置互补通道 PB13, PB14, PB15 (AF6)
    GPIO_Set(GPIOB, PIN13 | PIN14 | PIN15, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_170M, GPIO_PUPD_NONE);
    GPIO_AF_Set(GPIOB, 13, 6);
    GPIO_AF_Set(GPIOB, 14, 6);
    GPIO_AF_Set(GPIOB, 15, 6);

    // 4. 定时器基础设置
    TIM1->ARR = arr;
    TIM1->PSC = psc;

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


