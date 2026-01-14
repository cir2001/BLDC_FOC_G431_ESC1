#include "timer.h"
#include "as5047p.h"
#include "led.h"
#include <math.h>
#include "align.h"
#include "control.h"
#include "math.h"
#include "adc_foc.h"

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


u16 Timer1_Counter = 0;

uint16_t current_raw;
float elec_angle;
float open_loop_angle = 0.0f;
float test_theta = 0.0f;

extern PID_Controller vel_pid;
extern uint16_t my_zero_offset;

float offset_u, offset_v, offset_w;
// 用于调试监控的变量
float last_Iq = 0.0f;
float last_Id = 0.0f;
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
// --- 1. 获取转子位置 (角度) ---
        // uint16_t raw_angle = AS5047P_GetAngle();
        // if (raw_angle == 0xFFFF) return; // SPI通信错误处理

        // // 计算电角度 (elec_angle)
        // float elec_angle = Update_Electrical_Angle(raw_angle, my_zero_offset);
        // float s = sinf(elec_angle);
        // float c = cosf(elec_angle);

        // // --- 2. 获取相电流反馈 (力矩感知) ---
        // // 注意：由于 RCR=1，进入此中断时 ADC 转换已在波谷完成
        // float iu = ((float)ADC1->JDR1 - offset_u) * I_COEFF;
        // float iv = ((float)ADC2->JDR1 - offset_v) * I_COEFF;
        // // float iw = ((float)ADC1->JDR2 - offset_w) * I_COEFF; // 可选读 W 相

        // // --- 3. 电流正向变换 (得到实时力矩 Iq) ---
        // // 克拉克变换 (Clarke)
        // float i_alpha = iu;
        // float i_beta  = 0.5773503f * (iu + 2.0f * iv);

        // // 派克变换 (Park) -> 得到实时的 Id 和 Iq (即力矩)
        // last_Id = i_alpha * c + i_beta * s;
        // last_Iq = -i_alpha * s + i_beta * c; 

        // // --- 4. 速度环 PID 控制 (产生 Vq) ---
        // // 计算机械角度用于速度反馈
        // float mech_angle = (float)raw_angle * (2.0f * 3.14159265f / 16384.0f);
        // float speed_now = calculate_speed_rad_s(mech_angle);
        
        // // PID 计算：输入速度误差 -> 输出目标 Vq 电压
        // float target_Vq = PID_Compute(&vel_pid, speed_now);
        // float target_Vd = 0.0f; // D 轴电压通常设为 0

        // // --- 5. 电压反向变换 (从 Vd/Vq 到 Valpha/Vbeta) ---
        // // 反派克变换 (Inverse Park)
        // float Valpha = target_Vd * c - target_Vq * s;
        // float Vbeta  = target_Vd * s + target_Vq * c;

        // // --- 6. SVPWM 最终输出 ---
        // // 使用你之前跑通的 Min-Max 标准 SVPWM 函数
        // SVPWM_Output_Standard(Valpha, Vbeta);
//------- 标准 SVPWM 开环测试 end -----------------
//---- 速度 PID ----------------------------------
        // // 1. 传感器获取
        // uint16_t raw = AS5047P_GetAngle();
        // float mech_angle = (float)raw * (2.0f * 3.14159265f / 16384.0f); // 机械角度
        // float elec_angle = Update_Electrical_Angle(raw, my_zero_offset);

        // // 2. 计算实时滤波速度
        // float speed_now = calculate_speed_rad_s(mech_angle);

        // // 3. PID 计算得出 Vq
        // // 假设你想要电机以 10 rad/s 的速度旋转
        // vel_pid.target = 50.0f; 
        // float target_Vq = PID_Compute(&vel_pid, speed_now);
        // float target_Vd = 0.0f;

        // // 4. FOC 变换
        // float s = sinf(elec_angle);
        // float c = cosf(elec_angle);
        // float Valpha = target_Vd * c - target_Vq * s;
        // float Vbeta  = target_Vd * s + target_Vq * c;

        // // 5. 输出 SVPWM
        // SVPWM_Output_Standard(Valpha, Vbeta);

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
    // 1. 时钟开启
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. 引脚速度
    GPIOA->OSPEEDR |= (3U << (8 * 2)) | (3U << (9 * 2)) | (3U << (10 * 2)) | (3U << (12 * 2));
    GPIOB->OSPEEDR |= (3U << (15 * 2));
    GPIOC->OSPEEDR |= (3U << (13 * 2));

    // 3. 复位 AF 映射 (AF6 为 TIM1)
    GPIO_AF_Set(GPIOA, 8, 6); 
    GPIO_AF_Set(GPIOA, 9, 6); 
    GPIO_AF_Set(GPIOA, 10, 6);
    GPIO_AF_Set(GPIOC, 13, 4); 
    GPIO_AF_Set(GPIOA, 12, 6); 
    GPIO_AF_Set(GPIOB, 15, 4);

    // 4. 基础设置
    TIM1->ARR = arr;
    TIM1->PSC = 0;
    
    // [关键] 5. 重复计数器设为 1，确保只在波谷产生 Update 事件
    TIM1->RCR = 1; 

    // 6. 中心对齐模式 1
    TIM1->CR1 &= ~TIM_CR1_CMS;
    TIM1->CR1 |= (1 << TIM_CR1_CMS_Pos); 

    // 7. PWM 模式 1 配置
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    // 8. 使能输出
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);

    // 9. 死区与主输出
    TIM1->BDTR &= ~TIM_BDTR_DTG;
    TIM1->BDTR |= 100; 
    TIM1->BDTR |= TIM_BDTR_MOE; 

    // [关键] 10. 触发信号映射：Update -> TRGO2
    // 用于触发 ADC 注入组 (JEXTSEL=36)
    TIM1->CR2 &= ~TIM_CR2_MMS2;
    TIM1->CR2 |= (2U << 20); // MMS2 = 0010: 仅在 Underflow (波谷) 产生 TRGO2

    // 11. 启动
    TIM1->EGR |= TIM_EGR_UG;   // 更新寄存器
    TIM1->CR1 |= TIM_CR1_CEN;
    
    // 12. 中断配置
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0); 
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    // 开启 TIM1 的更新中断 (Update Interrupt Enable)
    TIM1->DIER |= TIM_DIER_UIE;
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

