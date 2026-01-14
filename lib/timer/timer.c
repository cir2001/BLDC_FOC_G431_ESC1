#include "timer.h"
#include "as5047p.h"
#include "led.h"
#include <math.h>
#include "align.h"
// #include "control.h"
#include "math.h"
#include "adc_foc.h"

//////////////////////////////////////////////////////////////////////////////////	 
//
////////////////////////////////////////////////////////////////////////////////// 	
//-----------------------------------------------
//-----------------------------------------------
// --- 宏定义与常量 ---
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SQRT3 0.577350269f

#define POLE_PAIRS 7

//-----------------------------------------------
// 外部函数声明
//-----------------------------------------------

//-----------------------------------------------
//外部变量声明
//-----------------------------------------------
extern PID_Controller vel_pid;
extern uint16_t my_zero_offset;// 14位机械偏置 (0-16383)
extern float offset_u, offset_v, offset_w;

// 用于主循环打印的调试变量
extern volatile float debug_theta;
extern volatile float debug_id;
extern volatile float debug_iq;

extern volatile uint16_t interrupt_flag_count;

extern PID_Controller pid_id;
extern PID_Controller pid_iq;
extern float target_iq;
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
u16 Timer1_Counter = 0;

float theta_elec = 0.0f;

// 用于调试监控的变量
float last_Iq = 0.0f;
float last_Id = 0.0f;
//--------------------------------------------
float i_alpha, i_beta;      // Clark 变换结果
float i_d, i_q;            // Park 变换结果
float theta_elec;          // 电角度 (弧度)
float sin_t, cos_t;        // 角度的正余弦值

float elec_angle;

int16_t raw_diff;
//==============================================
// TIM1 更新中断服务函数
//==============================================
void TIM1_UP_TIM16_IRQHandler(void)
{
    if(TIM1->SR & TIM_SR_UIF)
    {
        // 1. 立即清除中断标志
        TIM1->SR &= ~TIM_SR_UIF;
        Timer1_Counter++;
        interrupt_flag_count++;
        // 2. 角度获取 (流水线读取)
        uint16_t angle_raw = FOC_ReadAngle_Optimized();

        static float last_elec_angle = 0;
        if (!(angle_raw & 0x4000) && angle_raw != 0x3FFF)
        {
            // --- A. 减去零位偏移量 (关键：必须先处理偏置) ---
            raw_diff = (int16_t)(angle_raw & 0x3FFF) - (int16_t)my_zero_offset;
            if (raw_diff < 0) raw_diff += 16384; // 处理 14 位翻转

            // --- B. 计算机械角度与电角度 ---
            float mech_angle = ((float)raw_diff / 16384.0f) * 6.2831853f;
            elec_angle = mech_angle * 7.0f; // 7为极对数

            // --- D. 补偿 90 度相位差 (解决“吸住不转”的关键) ---
            // FOC 中，磁场需领先转子 90 度才能产生转矩
            // 修改这行补偿值，每次烧录一个，看看哪个能让电机“丝滑”转动

            // elec_angle += 1.57f;   // 尝试 90 度
            // elec_angle -= 1.57f;   // 尝试 90 度

            // --- E. 快速约束 0 ~ 2*PI ---
            while(elec_angle >= 6.2831853f) elec_angle -= 6.2831853f;
            while(elec_angle < 0)           elec_angle += 6.2831853f;

            last_elec_angle = elec_angle;
        }
        else
        {
            elec_angle = last_elec_angle;
        }
        // 3. 等待 ADC 转换完成 (此时 ADC 应该已经转完了)
        while(!(ADC2->ISR & ADC_ISR_JEOC));

        // 4. 读取电流并去偏置 (iu + iv + iw = 0)
        // float iu = -((float)ADC1->JDR1 - offset_u);
        // float iv = -((float)ADC2->JDR1 - offset_v);
        // float iw = -((float)ADC2->JDR2 - offset_w);

        // float iu = ((float)ADC1->JDR1 - offset_u);
        // float iv = ((float)ADC2->JDR1 - offset_v);
        // float iw = ((float)ADC2->JDR2 - offset_w);
        
        // 1. 读取并去偏置
        float raw_u = (float)ADC1->JDR1 - offset_u;
        float raw_v = (float)ADC2->JDR1 - offset_v;
        float raw_w = (float)ADC2->JDR2 - offset_w;

        // 2. 核心修正：缩放到安培量级
        // B-G431 的放大倍数下，大约 200 个单位对应 1A
        // 我们先除以 200.0f，让 i_q 回到 1.0 左右的正常范围
        float iu = raw_u / 200.0f; 
        float iv = raw_v / 200.0f;
        float iw = raw_w / 200.0f;

        // 5. 三角函数 (G4 系列建议未来使用 CORDIC 硬件加速)
        sin_t = sinf(elec_angle);
        cos_t = cosf(elec_angle);

        // 6. Clark 变换 (等幅值)
        // 注意：0.57735f 是 1/sqrt(3)
        i_alpha = iu;
        i_beta = (iv - iw) * 0.5773503f;

        // 7. Park 变换
        i_d =  i_alpha * cos_t + i_beta * sin_t;
        i_q = -i_alpha * sin_t + i_beta * cos_t;

        // 定义两个静态变量做滤波
        static float i_d_f = 0.0f;
        static float i_q_f = 0.0f;
        float lpf_alpha = 0.05f;// 滤波系数，越小越平滑，但响应越慢

        // 在 Park 变换后
        i_d_f = (i_d * lpf_alpha) + (i_d_f * (1.0f - lpf_alpha));
        i_q_f = (i_q * lpf_alpha) + (i_q_f * (1.0f - lpf_alpha));

        // PID 计算时使用滤波后的值
        float Vd = PID_Calc(&pid_id, 0.0f, i_d_f);
        float Vq = PID_Calc(&pid_iq, target_iq, i_q_f);

        // float Vd = 0.0f;
        // float Vq = 1.5f;

        // 9. 反 Park 与 SVPWM
        float Valpha = Vd * cos_t - Vq * sin_t;
        float Vbeta  = Vd * sin_t + Vq * cos_t;
        SVPWM_Output_Standard(Valpha, Vbeta);

        // 10. 准备下一次触发
        ADC1->ISR |= ADC_ISR_JEOC;
        ADC2->ISR |= ADC_ISR_JEOC;

        // 11. 调试赋值
        debug_theta = elec_angle;
        debug_id = i_d_f;
        debug_iq = i_q_f;

        // LED 翻转逻辑
        if (Timer1_Counter >= 7500) { // 15kHz下，7500次是500ms
            Timer1_Counter = 0;
            LED0_TOGGLE();
        }
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

void SVPWM_Output_Standard(float Valpha, float Vbeta)
{
    // 1. 平方限幅 (比 sqrtf 快很多)
    float v_sq = Valpha*Valpha + Vbeta*Vbeta;
    if (v_sq > 1.0f) {
        float inv_mod = 1.0f / sqrtf(v_sq); // 仅在需要时开方
        Valpha *= inv_mod;
        Vbeta *= inv_mod;
    }

    // 2. 反克拉克
    float U_u = Valpha;
    float U_v = -0.5f * Valpha - 0.8660254f * Vbeta;
    float U_w = -0.5f * Valpha + 0.8660254f * Vbeta;

    // 3. 寻找极值 (用 if 比用库函数快)
    float max_v = U_u, min_v = U_u;
    if (U_v > max_v) max_v = U_v; if (U_v < min_v) min_v = U_v;
    if (U_w > max_v) max_v = U_w; if (U_w < min_v) min_v = U_w;

    // 4. 注入零序分量
    float V_offset = (max_v + min_v) * 0.5f;

    // 映射到 0~1.0
    float Ta = (U_u - V_offset) + 0.5f;
    float Tb = (U_v - V_offset) + 0.5f;
    float Tc = (U_w - V_offset) + 0.5f;

    // 5. 写入寄存器 (ARR = 5666)
    TIM1->CCR1 = (uint16_t)(Ta * (float)TIM1->ARR);
    TIM1->CCR2 = (uint16_t)(Tb * (float)TIM1->ARR);
    TIM1->CCR3 = (uint16_t)(Tc * (float)TIM1->ARR);
}

float PID_Calc(PID_Controller* pid, float target, float current) {
    float error = target - current;
    
    // 增加一个微小的时间常数或直接减小步长，防止积分过快
    pid->integral += error * 0.01f; 

    // 积分限幅建议不要太大，根据你的输出 limit (2.0~5.0) 来看，积分限幅 1.0~2.0 足够了
    if (pid->integral > 1.0f) pid->integral = 1.0f;
    if (pid->integral < -1.0f) pid->integral = -1.0f;

    float out = (pid->kp * error) + (pid->ki * pid->integral);

    if (out > pid->output_limit) out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;
    
    return out;
}



