#include "timer.h"
#include "as5047p.h"
#include "led.h"
#include <math.h>
#include "align.h"
// #include "control.h"
#include "math.h"
#include "adc_foc.h"
#include <stdio.h>
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
extern volatile float debug_Vq;
extern volatile float debug_Vd;

extern volatile float debug_iu;
extern volatile float debug_iv;
extern volatile float debug_iw;
extern volatile float debug_mech_angle;

volatile float debug_raw_u;
volatile float debug_raw_v;
volatile float debug_raw_w;

volatile float debug_iu_filt;
volatile float debug_iv_filt;
volatile float debug_iw_filt;

extern PID_Controller pid_id;
extern PID_Controller pid_iq;

extern  volatile uint8_t run_foc_flag;
//-----------------------------------------------
// 变量声明
//-----------------------------------------------
u16 Timer1_Counter = 0;
//--------------------------------------------
float i_alpha, i_beta;      // Clark 变换结果
float i_d, i_q;            // Park 变换结果
float theta_elec = 0.0f;          // 电角度 (弧度)
float sin_t, cos_t;        // 角度的正余弦值

float elec_angle;

int16_t raw_diff;

// --- 变量定义 ---
float target_speed = 0.0f;      // 目标速度 (单位: rad/s)
float actual_speed_filt = 0.0f; // 实际速度 (滤波后)
float target_iq = 0.0f;         // 速度环的输出，作为电流环的输入
float target_id = 0.0f;      

static float last_mech_angle = 0.0f;
static float i_d_f = 0.0f, i_q_f = 0.0f; // 电流滤波变量
static float iu_filt = 0.0f, iv_filt = 0.0f, iw_filt = 0.0f; // 电流

extern PID_Controller pid_id, pid_iq, pid_speed;
extern uint16_t my_zero_offset;
extern float offset_u, offset_v, offset_w;

// 用于调试打印
volatile float debug_speed_act = 0;
volatile float debug_iq_target = 0;

volatile float Vd = 0.0f;
volatile float Vq = 0.0f;
volatile float force_angle = 0.0f; // 强制输出时的电角度
//==============================================
// TIM1 更新中断服务函数
//==============================================
void TIM1_UP_TIM16_IRQHandler(void)
{
    if(TIM1->SR & TIM_SR_UIF)
    {
        // 立即清除中断标志
        TIM1->SR &= ~TIM_SR_UIF;
        Timer1_Counter++;

        // 1. 获取机械角度与电角度
        uint16_t angle_raw = FOC_ReadAngle_Optimized();
        if ((angle_raw & 0x4000) || angle_raw == 0x3FFF) return; 

        int16_t raw_diff = (int16_t)(angle_raw & 0x3FFF) - (int16_t)my_zero_offset;
        if (raw_diff < 0) raw_diff += 16384;
        
        float mech_angle = ((float)raw_diff / 16384.0f) * 6.2831853f;
        float elec_angle = (mech_angle * 7.0f); // 极对数 7

        debug_mech_angle = mech_angle;

        while(elec_angle >= 6.2831853f) elec_angle -= 6.2831853f;
        while(elec_angle < 0)           elec_angle += 6.2831853f;

        // 2. 速度环逻辑：15分频 (运行频率 1kHz)
        // static uint16_t speed_cnt = 0;
        // speed_cnt++;
        // if (speed_cnt >= 15) 
        // {
        //     speed_cnt = 0;
            
        //     // 计算角度差 (处理过零点)
        //     float delta_angle = mech_angle - last_mech_angle;
        //     if (delta_angle > 3.14159f)  delta_angle -= 6.28318f;
        //     if (delta_angle < -3.14159f) delta_angle += 6.28318f;
            
        //     // 计算瞬时速度 (rad/s) -> 1kHz 频率，所以乘以 1000
        //     float instant_speed = delta_angle * 1000.0f; 
            
        //     // 速度低通滤波 
        //     actual_speed_filt = (instant_speed * 0.03f) + (actual_speed_filt * 0.97f);
        //     last_mech_angle = mech_angle;

        //     float base_iq = PID_Calc_Speed(&pid_speed, target_speed, actual_speed_filt);

        //     // target_iq = base_iq;
        //     // target_iq = 0.3f;
        //     // target_id = 0.0f;
        // }
        target_iq = 0.0f;
        target_id = 0.5f;

        // 每次中断都要确保启动注入组转换
        ADC1->CR |= ADC_CR_JADSTART;
        ADC2->CR |= ADC_CR_JADSTART;

        // 3. 电流环逻辑 (15kHz)
        // 1. 计算原始偏差（ADC值 - 你的静态偏置）
        float raw_u = (float)ADC1->JDR1 - offset_u;
        float raw_v = (float)ADC2->JDR1 - offset_v;
        float raw_w = (float)ADC2->JDR2 - offset_w;

        debug_raw_u = raw_u;
        debug_raw_v = raw_v;
        debug_raw_w = raw_w;

        // 1. 计算原始三相和（用于补偿）
        float sum_raw = raw_u + raw_v + raw_w;

        // 2. 低通滤波三相和（更稳定，避免单次跳变影响补偿）
        static float sum_filt = 0.0f;
        sum_filt = sum_filt * 0.95f + sum_raw * 0.05f;

        // 3. 均值偏差（滤波后更平滑）
        float sum_error = sum_filt * 0.3333333f;

        // 4. 补偿
        float iu = (raw_u - sum_error) / -10.0f;
        float iv = (raw_v - sum_error) / -10.0f;
        float iw = (raw_w - sum_error) / -10.0f;

        // 后续滤波、Clark/Park 不变
        iu_filt = iu_filt*0.85f + iu*0.15f;
        iv_filt = iv_filt*0.85f + iv*0.15f;
        iw_filt = iw_filt*0.85f + iw*0.15f;

        debug_iu = iu;
        debug_iv = iv;
        debug_iw = iw;

        debug_iu_filt = iu_filt;
        debug_iv_filt = iv_filt;
        debug_iw_filt = iw_filt;

        // Clark & Park 变换
        // float i_alpha = iu;
        // float i_beta = (iv - iw) * 0.5773503f;
        float i_alpha = iu_filt;
        float i_beta = (iv_filt - iw_filt) * 0.5773503f;

        float sin_t = sinf(elec_angle);
        float cos_t = cosf(elec_angle);

        float i_d =  i_alpha * cos_t + i_beta * sin_t;
        float i_q = -i_alpha * sin_t + i_beta * cos_t;

        // 电流低通滤波 (系数 0.1)
        // i_d_f = (i_d * 0.05f) + (i_d_f * 0.95f);
        // i_q_f = (i_q * 0.05f) + (i_q_f * 0.95f);

        i_d_f = i_d;
        i_q_f = i_q;

        debug_iq = i_q_f;
        debug_id = i_d_f;

        // --- 4. 控制模式分流 (核心修改) ---
        float out_angle;
        if (run_foc_flag == 1) 
        {
            // PID 计算 Vd, Vq
            Vd = PID_Calc(&pid_id, target_id, i_d_f);
            Vq = PID_Calc(&pid_iq, target_iq, i_q_f);
            out_angle = elec_angle;
        }
        else 
        {
            // ======= 强制模式：预对齐 =======
            // Vd, Vq 此时保持由 FOC_PreAlign 函数传入的固定值
            out_angle = 0.0f; // 强行指向 0 度（U相轴线）
            Vd = Vd; // 保持 PreAlign 函数中给的值
            Vq = 0.0f;
        }

        // --- SVPWM 输出 ---
        debug_Vq = Vq;
        debug_Vd = Vd;

        // Vd = 0.0f; 
        // Vq = 0.0f;

        float final_s = sinf(out_angle);
        float final_c = cosf(out_angle);

        // 逆 Park 变换
        float Valpha = Vd * final_c - Vq * final_s;
        float Vbeta  = Vd * final_s + Vq * final_c;
        
        // 调用你之前的 Min-Max SVPWM 函数
        SVPWM_Output_Standard(Valpha, Vbeta);
        
        debug_speed_act = actual_speed_filt;
        debug_iq_target = target_iq;

        ADC1->ISR |= ADC_ISR_JEOC;
        ADC2->ISR |= ADC_ISR_JEOC;
        
        // LED 翻转逻辑
        if (Timer1_Counter >= 1500) 
        { // 15kHz下，7500次是500ms
            Timer1_Counter = 0;
            LED0_TOGGLE();
            // printf("IU:%.2f, IV:%.2f, IW:%.2f,raw_u:%.2f,raw_v:%.2f,raw_w:%.2f\r\n", 
            //         debug_iu, debug_iv,debug_iw, debug_raw_u, debug_raw_v, debug_raw_w);
            
            // printf("IU:%.2f, IV:%.2f, IW:%.2f,IU_filt:%.2f, IV_filt:%.2f, IW_filt:%.2f\r\n", 
            //         debug_iu, debug_iv,debug_iw, debug_iu_filt, debug_iv_filt, debug_iw_filt);
            
            // printf("Vd:%.2f, id:%.2f\r\n", debug_Vd, debug_id);

            // printf("iq:%.2f, id:%.2f\r\n", debug_iq, debug_id);

            printf("iq:%.2f, id:%.2f,Vq:%.2f, Vd:%.2f\r\n", 
                    debug_iq, debug_id,debug_Vq, debug_Vd);

            
            // printf("CCR4: %d, ARR: %d\n", TIM1->CCR4, TIM1->ARR);

            // printf("angle_raw: %d\r\n",angle_raw);

            // printf("elec_angle:%.2f, mech_angle:%.2f\r\n", 
            //         elec_angle, mech_angle);
            // printf("iq:%.2f, id:%.2f, Vq:%.2f, Vd:%.2f, Flag:%d\r\n", 
            //         debug_iq, debug_id, debug_Vq, debug_Vd, run_foc_flag);

            // printf("Target:%.2f, Act:%.2f, Iq_Ref:%.2f, Iq_Act:%.2f\r\n", 
            //         target_speed, actual_speed_filt, target_iq, debug_iq);

        }


        //  TIM1->SR &= ~TIM_SR_UIF;
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
    TIM1->RCR = 1; // 确保在波谷触发 Update 事件

    // 6. 中心对齐模式 1
    TIM1->CR1 &= ~TIM_CR1_CMS;
    TIM1->CR1 |= (1 << TIM_CR1_CMS_Pos);

    // 7. PWM 模式 1 配置
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    // --- 配置 CH4 作为 ADC 触发源 ---
    TIM1->CCMR2 &= ~TIM_CCMR2_OC4M;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; // CH4 同样设为 PWM 模式 1
    TIM1->CCR4 = 20;          // 固定谷值中心
    TIM1->CCER |= TIM_CCER_CC4E;    // 使能 OC4（内部产生 REF 信号）

    // 8. 使能输出
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
                   TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC4E);

    // 9. 死区与主输出
    TIM1->BDTR &= ~TIM_BDTR_DTG;
    TIM1->BDTR |= 100; 
    TIM1->BDTR |= TIM_BDTR_MOE; 

    // --- 【关键修改：映射 TRGO2 到 OC4REF】 ---
    TIM1->CR2 &= ~TIM_CR2_MMS2;
    // MMS2 = 0110: OC4REF 的上升/下降沿产生 TRGO2 信号
    TIM1->CR2 |= (4U << 20);    // 0110 = falling edge

    // 11. 启动
    TIM1->EGR |= TIM_EGR_UG;   // 更新寄存器
    TIM1->CR1 |= TIM_CR1_CEN;
    
    // 12. 中断配置
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0); 
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

void SVPWM_Output_Standard(float Valpha, float Vbeta)
{
    // 1. 平方限幅 (防止过调制)
    float v_sq = Valpha * Valpha + Vbeta * Vbeta;
    if (v_sq > 1.0f) {
        float inv_mod = 1.0f / sqrtf(v_sq);
        Valpha *= inv_mod;
        Vbeta *= inv_mod;
    }

    // 2. 反克拉克变换
    float U_u = Valpha;
    float U_v = -0.5f * Valpha - 0.8660254f * Vbeta;
    float U_w = -0.5f * Valpha + 0.8660254f * Vbeta;

    // 3. 寻找极值 (用于注入零序分量/中点平移)
    float max_v = U_u, min_v = U_u;
    if (U_v > max_v) max_v = U_v; if (U_v < min_v) min_v = U_v;
    if (U_w > max_v) max_v = U_w; if (U_w < min_v) min_v = U_w;

    // 4. 注入零序分量 (标准 SVPWM)
    float V_offset = (max_v + min_v) * 0.5f;

    // 映射到 0.0 ~ 1.0 占空比范围
    float Ta = (U_u - V_offset) + 0.5f;
    float Tb = (U_v - V_offset) + 0.5f;
    float Tc = (U_w - V_offset) + 0.5f;

    // --- [关键修改] 5. 计算并限制 CCR 值以预留采样窗口 ---
    
    // 将占空比转换为计数值
    uint16_t ccr1 = (uint16_t)(Ta * PWM_ARR);
    uint16_t ccr2 = (uint16_t)(Tb * PWM_ARR);
    uint16_t ccr3 = (uint16_t)(Tc * PWM_ARR);

    // 计算最大允许的 CCR 值
    // 保证下桥臂导通时间（即 CCR 之后到 ARR 结束的时间）不小于 MIN_OFF_TICKS / 2
    // 在中心对齐模式下，采样窗宽度 = 2 * (ARR - CCR)
    // --- 采样窗保护 ---
    // 防止占空比过高（接近100%）导致下桥臂导通时间太短，ADC 采样失败
    uint16_t max_ccr = PWM_ARR - 200; 
    if (ccr1 > max_ccr) ccr1 = max_ccr;
    if (ccr2 > max_ccr) ccr2 = max_ccr;
    if (ccr3 > max_ccr) ccr3 = max_ccr;

    // 6. 写入寄存器
    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;

    // ---  更新 CCR4 采样点  ---
    // 强制在计数器接近 ARR (波谷) 的时刻触发 ADC
    // 这样能确保在三个下桥臂都导通的最稳定时刻采样
    TIM1->CCR4 = 80;
    // TIM1->CCR4 = (uint16_t)(PWM_ARR - 20);
}

float PID_Calc(PID_Controller* pid, float target, float current) 
{
    float error = target - current;
    // float error = current - target;
    float dt = 0.000066f; 

    // 1. 正常的积分累加
    pid->integral += error; 

    // 2. 修正积分限幅逻辑 (重要！)
    // 目标：让积分项(I部分)能够输出的最大电压等于输出限幅
    // 倒推公式：integral_limit = output_limit / (ki * dt)
    float i_term_limit = pid->output_limit / (pid->ki * dt + 0.000001f); // 加微小值防止除0
    
    if (pid->integral > i_term_limit) pid->integral = i_term_limit;
    if (pid->integral < -i_term_limit) pid->integral = -i_term_limit;

    // 3. 计算 P 和 I 项
    float p_term = pid->kp * error;
    float i_term = pid->ki * dt * pid->integral;

    float out = p_term + i_term;

    // 4. 总输出限幅
    if (out > pid->output_limit) out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;
    
    return out;
}

float PID_Calc_Speed(PID_Controller* pid, float target, float current) 
{
    float error = target - current;
    
    // 1. 积分累加
    pid->integral += error; 

    // 2. 积分限幅 (核心修改)
    // 这里的限幅应该基于“你允许积分项贡献多少电流”
    // 假设速度环输出 target_iq，最大为 2.0A，我们可以允许积分项占满整个输出
    // float i_limit = 2.0f / (pid->ki * 0.001f); // 反推积分累加器的限幅
    // if (pid->integral > i_limit) pid->integral = i_limit;
    // if (pid->integral < -i_limit) pid->integral = -i_limit;

    if (pid->integral > 15.0f) pid->integral = 15.0f;
    if (pid->integral < -15.0f) pid->integral = -15.0f;

    // 3. 计算输出 (PI控制)
    float dt = 0.001f;
    float p_out = pid->kp * error;
    float i_out = pid->ki * dt * pid->integral;
    
    float out = p_out + i_out;

    // 4. 总输出限幅 (限制 target_iq)
    if (out > pid->output_limit) out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;
    
    return out;
}

void FOC_SVPWM_Update(float Vd, float Vq, float angle)
{
    // === 1. 逆 Park 变换 (将 Vd, Vq 旋转到静止坐标系 Alpha, Beta) ===
    float s = sinf(angle);
    float c = cosf(angle);
    float Valpha = Vd * c - Vq * s;
    float Vbeta  = Vd * s + Vq * c;

    // === 2. 平方限幅 (防止过调制，保持你原有的逻辑) ===
    float v_sq = Valpha * Valpha + Vbeta * Vbeta;
    if (v_sq > 1.0f) {
        float inv_mod = 1.0f / sqrtf(v_sq);
        Valpha *= inv_mod;
        Vbeta *= inv_mod;
    }

    // === 3. 反克拉克变换 (生成三相电压矢量) ===
    float U_u = Valpha;
    float U_v = -0.5f * Valpha - 0.8660254f * Vbeta; // -0.5, -sqrt(3)/2
    float U_w = -0.5f * Valpha + 0.8660254f * Vbeta; // -0.5, +sqrt(3)/2

    // === 4. 寻找极值并注入零序分量 (中点平移，生成马鞍波) ===
    float max_v = U_u, min_v = U_u;
    if (U_v > max_v) max_v = U_v; if (U_v < min_v) min_v = U_v;
    if (U_w > max_v) max_v = U_w; if (U_w < min_v) min_v = U_w;

    float V_offset = (max_v + min_v) * 0.5f;

    // === 5. 映射到 0.0 ~ 1.0 占空比范围 ===
    float Ta = (U_u - V_offset) + 0.5f;
    float Tb = (U_v - V_offset) + 0.5f;
    float Tc = (U_w - V_offset) + 0.5f;

    // === 6. 转换为计数值并强制执行采样窗保护 (保持你的核心修改) ===
    uint16_t ccr1 = (uint16_t)(Ta * PWM_ARR);
    uint16_t ccr2 = (uint16_t)(Tb * PWM_ARR);
    uint16_t ccr3 = (uint16_t)(Tc * PWM_ARR);

    uint16_t max_ccr = PWM_ARR - 200; 
    if (ccr1 > max_ccr) ccr1 = max_ccr;
    if (ccr2 > max_ccr) ccr2 = max_ccr;
    if (ccr3 > max_ccr) ccr3 = max_ccr;

    // === 7. 写入寄存器 ===
    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;

    // === 8. 更新采样触发点 (CCR4) ===
    TIM1->CCR4 = 80; 
}

