#include "timer.h"
#include "as5047p.h"
#include "led.h"
#include <math.h>
#include "align.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 	
#define PWM_ARR 5666.0f
#define SQRT3 1.73205081f

#define POLE_PAIRS 7
#define M_PI 3.1415926535f
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
extern uint16_t my_zero_offset;

u16 Timer1_Counter = 0;

uint16_t current_raw;
float elec_angle;
float open_loop_angle = 0.0f;
float test_theta = 0.0f;
//==============================================
// TIM1 更新中断服务函数
//==============================================
void TIM1_UP_TIM16_IRQHandler(void)
{
    if(TIM1->SR & TIM_SR_UIF) // 检查更新中断标志位
    {
        // 清除中断标志位，防止重复进入中断
        TIM1->SR = ~TIM_SR_UIF;
        Timer1_Counter++;
        if (Timer1_Counter >= 1500) // 每500ms翻转一次LED0
        {
            Timer1_Counter = 0;
            LED0_TOGGLE();      // 翻转LED0
        }
//--------- 开环测试-------------------------
        // static float ol_angle = 0.0f;
        // ol_angle += 0.003f; // 极低速步进
        // if(ol_angle > 2.0f * 3.14159f) ol_angle = 0;

        // float U_max = 0.2f; // 占空比幅值 (20%)
        
        // // 直接计算三相占空比，跳过 SVPWM 扇区逻辑
        // float duty_u = 0.5f + U_max * sinf(ol_angle);
        // float duty_v = 0.5f + U_max * sinf(ol_angle - 2.0f/3.0f*3.14159f);
        // float duty_w = 0.5f + U_max * sinf(ol_angle + 2.0f/3.0f*3.14159f);

        // // 写入寄存器 (假设 ARR = 5666)
        // TIM1->CCR1 = (uint16_t)(duty_u * 5666.0f);
        // TIM1->CCR2 = (uint16_t)(duty_v * 5666.0f);
        // TIM1->CCR3 = (uint16_t)(duty_w * 5666.0f);
//------- 开环测试 end -----------------------
//------- FOC 测试 -----------------------
//--- 不需要再进行 2*PI - elec_angle 的取反操作
        //1. 获取编码器角度
        uint16_t raw = AS5047P_GetAngle();
        if(raw == 0xFFFF) return;

        // 2. 获取电角度 (保持成功的 Update_Electrical_Angle 逻辑)
        float elec_angle = Update_Electrical_Angle(raw, my_zero_offset);

        // 3. 设定转矩 (Vq)
        // 注意：如果你之前 SPWM 顺时针转，target_Vq 设为正值通常也是顺时针
        float target_Vq = -0.3f; 
        float target_Vd = 0.0f;

        // 4. 修正后的反 Park 变换
        // 我们给角度补偿 1.5708 rad (90度)，以匹配你 SPWM 的坐标系
        float s = sinf(elec_angle);
        float c = cosf(elec_angle);

        // 这是 FOC 的灵魂公式：将 DQ 轴转换回 Alpha/Beta 轴
        // float Valpha = target_Vd * c - target_Vq * s;
        // float Vbeta  = target_Vd * s + target_Vq * c;
        
        float Valpha = -target_Vq*s; 
        float Vbeta = target_Vq*c;

        SVPWM_Output_Standard(Valpha, Vbeta);
//------- FOC 测试 end -----------------------
//------- 闭环测试 -----------------------
//--- 需要进行 2*PI - elec_angle 的取反操作
        // 1. 读取编码器
        // uint16_t raw = AS5047P_GetAngle();
        // if(raw == 0xFFFF) return;

        // // 2. 获取当前电角度（已取反）
        // float elec_angle = Update_Electrical_Angle(raw, my_zero_offset);

        // // 3. 闭环控制：在当前角度基础上超前 90 度 (1.5708 rad)
        // // 这是产生最大转矩 (Q轴) 的位置
        // float theta_feedback = elec_angle + 1.5708f; 
        
        // float V_mag = 0.25f; // 先给一个小一点的电压，防止剧烈震动
        // float duty_u = 0.5f + V_mag * sinf(theta_feedback);
        // float duty_v = 0.5f + V_mag * sinf(theta_feedback - 2.0944f);
        // float duty_w = 0.5f + V_mag * sinf(theta_feedback + 2.0944f);

        // // 4. 写入寄存器 (ARR=5666)
        // TIM1->CCR1 = (uint16_t)(duty_u * 5666.0f);
        // TIM1->CCR2 = (uint16_t)(duty_v * 5666.0f);
        // TIM1->CCR3 = (uint16_t)(duty_w * 5666.0f);
//------- 闭环测试 end-----------------------
//------- 标准 SVPWM 开环测试-----------------
        // test_theta += 0.0005f; // 步进
        // if(test_theta > 6.28318f) test_theta -= 6.28318f;

        // float V_mag = 0.2;
        // // 开环直接生成 Alpha/Beta
        // float Valpha = V_mag * cosf(test_theta);
        // float Vbeta  = V_mag * sinf(test_theta);

        // SVPWM_Output_Standard(Valpha, Vbeta);
//------- 标准 SVPWM 开环测试 end -----------------


//--------------------------------------------
        TIM1->BDTR |= TIM_BDTR_MOE; // 强制开启主输出使能
    }
}
//==============================================
// @brief  初始化 TIM1 产生中心对齐 PWM (避坑版)
// @param  arr: 自动重装载值 (170MHz下, 5666对应15kHz)
// @param  psc: 分频系数 (通常为0)
//==============================================
void TIM1_PWM_Init(u16 arr)
{
    // 1. 开启 GPIOA, GPIOB, GPIOC 和 TIM1 时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIOA->OSPEEDR |= (3U << (8 * 2)) | (3U << (9 * 2)) | (3U << (10 * 2)) | (3U << (12 * 2));
    GPIOB->OSPEEDR |= (3U << (15 * 2));
    GPIOC->OSPEEDR |= (3U << (13 * 2));

    // 2. 配置引脚复用 (根据 UM2516 Table 4 定义)
    // PA8, PA9, PA10 -> AF6 (TIM1_CH1/2/3)
    GPIO_AF_Set(GPIOA, 8, 6); 
    GPIO_AF_Set(GPIOA, 9, 6); 
    GPIO_AF_Set(GPIOA, 10, 6);
    
    // PC13 -> AF4 (TIM1_CH1N) [手册规定映射]
    GPIO_AF_Set(GPIOC, 13, 4); 
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
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);//先清除模式位再设置
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; // CH1
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // CH2 对应 PA11

	// CCMR2 寄存器：控制 CH3 (低8位) 和 CH4 (高8位)
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;//先清除模式位再设置
	TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; // CH3 对应 PA12

    // 7. 使能互补输出与极性
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

    // 8. 死区时间配置 (Dead-time)
    // G431 170MHz 主频，设置 150 约等于 880ns，足以保护大部分 MOSFET
    TIM1->BDTR &= ~TIM_BDTR_DTG;
    TIM1->BDTR |= 100; 

    // 9. 开启主输出 (MOE) 并启动定时器
    TIM1->BDTR |= TIM_BDTR_MOE; 
    TIM1->CR1 |= TIM_CR1_CEN;
    
    // 10. 触发信号：Update事件触发 TRGO (用于 ADC 同步采样)
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |= (2 << TIM_CR2_MMS_Pos);

    // 在 TIM1_PWM_Init 函数末尾添加
    //TIM1->DIER |= TIM_DIER_UIE; // 开启更新中断
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0); // 设置最高优先级
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}
//-----------------------------------------------
//  * @brief 反 Park 变换
//  * 将旋转坐标系的 Vd, Vq 转换为静止坐标系的 V_alpha, V_beta
//-----------------------------------------------
void Inverse_Park(float Vd, float Vq, float angle, float *Valpha, float *Vbeta) 
{
    float s = sinf(angle);
    float c = cosf(angle);
    *Valpha = Vd * c - Vq * s;
    *Vbeta  = Vd * s + Vq * c;
}

//-----------------------------------------------
//  * @brief 简易 SVPWM 实现 (注入零序分量以优化电压利用率)
//-----------------------------------------------
void SVPWM_Output(float Valpha, float Vbeta) 
{
    int sector;
    float T1, T2, T0;
    // 初始化为 0.5f，确保 sector 为 0 时输出 50% 占空比（零力矩）
    float Ta = 0.5f, Tb = 0.5f, Tc = 0.5f; 

    float v_ref1 = Vbeta;
    float v_ref2 = (SQRT3 * Valpha - Vbeta) * 0.5f;
    float v_ref3 = (-SQRT3 * Valpha - Vbeta) * 0.5f;

    sector = (v_ref1 > 0 ? 1 : 0) + (v_ref2 > 0 ? 2 : 0) + (v_ref3 > 0 ? 4 : 0);

    // 此时 T1, T2 是基于 v_ref 的投影
    switch (sector) {
        case 3: T1 = v_ref2;  T2 = v_ref1;  break; // Sector 1
        case 1: T1 = -v_ref2; T2 = -v_ref3; break; // Sector 2
        case 5: T1 = v_ref3;  T2 = v_ref2;  break; // Sector 3
        case 4: T1 = -v_ref1; T2 = -v_ref2; break; // Sector 4
        case 6: T1 = v_ref1;  T2 = v_ref3;  break; // Sector 5
        case 2: T1 = -v_ref3; T2 = -v_ref1; break; // Sector 6
        default: T1 = 0; T2 = 0; break;
    }

    float sum = T1 + T2;
    if (sum > 1.0f) { // 过调制处理
        T1 /= sum;
        T2 /= sum;
    }
    T0 = (1.0f - T1 - T2) * 0.5f;

    switch (sector) {
        case 3: Ta = T0 + T1 + T2; Tb = T0 + T2;      Tc = T0;           break;
        case 1: Ta = T0 + T1;      Tb = T0 + T1 + T2; Tc = T0;           break;
        case 5: Ta = T0;           Tb = T0 + T1 + T2; Tc = T0 + T2;      break;
        case 4: Ta = T0;           Tb = T0 + T1;      Tc = T0 + T1 + T2; break;
        case 6: Ta = T0 + T2;      Tb = T0;           Tc = T0 + T1 + T2; break;
        case 2: Ta = T0 + T1 + T2; Tb = T0;           Tc = T0 + T1;      break;
        // 即使进入 default，Ta,Tb,Tc 也会保持 0.5f
    }

    // 写入寄存器。注意：确认 ARR 是否真的是 18888
    TIM1->CCR1 = (uint16_t)(Ta * 5666.0f);
    TIM1->CCR2 = (uint16_t)(Tb * 5666.0f);
    TIM1->CCR3 = (uint16_t)(Tc * 5666.0f);
}

void SVPWM_Output_Standard(float Valpha, float Vbeta) 
{
    // 幅度限幅：防止 Valpha 和 Vbeta 的矢量模值超过母线电压利用极限 (1.0)
    float v_mod = sqrtf(Valpha*Valpha + Vbeta*Vbeta);
    if (v_mod > 1.0f) {
        Valpha /= v_mod;
        Vbeta /= v_mod;
    }
    // 1. 反克拉克变换 (Inverse Clarke) - 直接得到三相电压
    float U_u = Valpha;
    float U_v = -0.5f * Valpha - 0.8660254f * Vbeta; // 注意这里改成了减号
    float U_w = -0.5f * Valpha + 0.8660254f * Vbeta; // 注意这里改成了加号

    // 2. 计算最小值和最大值，用于中心对齐（SVPWM 的核心）
    float max_v = U_u;
    if (U_v > max_v) max_v = U_v;
    if (U_w > max_v) max_v = U_w;

    float min_v = U_u;
    if (U_v < min_v) min_v = U_v;
    if (U_w < min_v) min_v = U_w;

    // 3. 注入零序分量，实现 SVPWM
    float V_offset = (max_v + min_v) * 0.5f;
    
    float Ta = (U_u - V_offset) + 0.5f;
    float Tb = (U_v - V_offset) + 0.5f;
    float Tc = (U_w - V_offset) + 0.5f;

    // 4. 写入寄存器 (ARR = 5666)
    TIM1->CCR1 = (uint16_t)(Ta * 5666.0f);
    TIM1->CCR2 = (uint16_t)(Tb * 5666.0f);
    TIM1->CCR3 = (uint16_t)(Tc * 5666.0f);
}

