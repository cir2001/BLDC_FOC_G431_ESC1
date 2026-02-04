#include "mt6826s.h"
#include "delay.h"
#include <math.h>
#include "timer.h"
#include <stdio.h>
#include "align.h"
#include "control.h"
//-----------------------------------------------------
// --- 宏定义与常量 ---
#define POLE_PAIRS 7
#ifndef M_PI
#define M_PI 3.1415926535f
#endif
//-----------------------------------------------------
// --- 变量声明 ---
float test_v = 1.5f;           // 测试电压 (根据母线电压调节，12V下建议0.8-1.5V)
int test_cycles = 10;          // 电角度转动的圈数
float theta_e_step = 0.005f;    // 步进速度

//-----------------------------------------------------
/**
 * @brief 电角度对齐寻零
 * @return zero_offset: 机械零位偏移量 (0-16383)
 */
uint16_t FOC_Align_Sensor(void) 
{
    printf("Starting Smooth Two-Stage Alignment...\r\n");

    float max_v = 0.25f;       // 最终对齐电压占空比 (25%)
    float stage1_v = 0.15f;    // 阶段1预定位电压 (15%)
    int ramp_steps = 500;      // 斜坡步数

    // --- 阶段 1: 预定位到 Phase V (0% -> 15%) ---
    // 目的：先把转子拉到一个已知位置，防止初始死区
    for(int i = 0; i <= ramp_steps; i++) {
        float ramp_v = ((float)i / ramp_steps) * stage1_v;
        TIM1->CCR1 = (uint16_t)(ramp_v * PWM_ARR); 
        TIM1->CCR2 = 0; 
        TIM1->CCR3 = 0;
        delay_ms(2);
    }
    delay_ms(500); // 短暂稳定

    // --- 阶段 2: 平滑切换 (Phase V -> Phase U) ---
    // 目的：让磁场从 V 相线性转移到 U 相，转子会平滑滑行 120 度电角度
    for(int i = 0; i <= ramp_steps; i++) {
        float alpha = (float)i / ramp_steps;
        // V相电压从 15% 降至 0%
        TIM1->CCR1 = (uint16_t)((1.0f - alpha) * stage1_v * PWM_ARR);
        TIM1->CCR2 = 0;
        // U相电压从 0% 升至 25%
        TIM1->CCR3= (uint16_t)(alpha * max_v * PWM_ARR);
        delay_ms(2);
    }

    // --- 阶段 3: 最终静止与采样 ---
    printf("Waiting for stabilization...\r\n");
    delay_ms(1500); // 彻底消除机械余震

    // 稳定性检查
    uint16_t check1 = TIM4->CNT;
    delay_ms(200);
    uint16_t check2 = TIM4->CNT;
    if(abs((int)check1 - (int)check2) > 20) {
        printf("Warning: Motor unstable! Adjusting wait time...\r\n");
        delay_ms(1000);
    }

    // 高精度采样平均 (256次)
    uint32_t sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += TIM4->CNT;
        delay_ms(1);
    }
    uint16_t zero_offset = (uint16_t)(sum / 256);

    // --- 阶段 4: 结果应用 ---
    // manual_adjust 补偿：如果你的 Clarke 变换以 U 轴为 0 度，
    // 而电流环运行时发现输出异常，可在此微调。 485
    int16_t manual_adjust =-10;//60; 
    uint16_t final_offset = (uint16_t)((zero_offset + manual_adjust) & 0x3FFF);

    // 结束时不要立刻断电，保持一个微弱占空比防止转子滑走，直到进入 FOC 循环
    TIM1->CCR3 = (uint16_t)(0.1f * PWM_ARR); 

    printf("Alignment Done! Raw:%d, agj:%d, Final:%d\r\n", zero_offset,manual_adjust,final_offset);
    delay_ms(50);
    
    return final_offset;
}


void Detect_Motor_Pole_Pairs(void){
    // 1. 屏蔽 TIM1 中断
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);

    printf("--- Pole Pair Detection start ---\r\n");
    delay_ms(10); 
    // 2. 预对齐：快速但充分
    // 设定 1 秒对齐时间，足够让转子稳定在 0 度
    for(int i = 0; i < 100; i++) {
        SVPWM_Output_Standard(test_v, 0); 
        delay_ms(5);
    }
    // 3. 记录初始位置
    uint16_t last_raw = (uint16_t)(TIM4->CNT) & ENCODER_MASK;
    int32_t total_accumulated = 0;

    // 4. 开始强拖旋转
    float current_theta_e = 0.0f;
    float target_theta_limit = (float)test_cycles * 2.0f * M_PI;
    printf("Limit: %.2f\r\n", target_theta_limit); // 看看这里是不是 62.83
    delay_ms(10);
    printf("Rotating...\r\n");
    delay_ms(10); 

    while(current_theta_e < target_theta_limit) 
    {
        current_theta_e += theta_e_step;
        
        // float s, c;
        // CORDIC_SinCos(current_theta_e, &s, &c);
        float s = sinf(current_theta_e);
        float c = cosf(current_theta_e);
        
        SVPWM_Output_Standard(test_v * c, test_v * s);

        uint16_t current_raw = (uint16_t)(TIM4->CNT) & ENCODER_MASK;
        int16_t diff = (int16_t)(current_raw - last_raw);
        if (diff > 8192)  diff -= 16384;
        else if (diff < -8192) diff += 16384;
        total_accumulated += diff;
        last_raw = current_raw;

        printf("cur: %.2f\r\n", current_theta_e); // 看看这里是不是 62.83
        delay_ms(1);
    }

    // 5. 停止输出
    SVPWM_Output_Standard(0, 0);

    // 6. 计算
    float mech_rotations = (float)total_accumulated / 16384.0f; 
    float measured_pn = (float)test_cycles / mech_rotations;
    if(measured_pn < 0) measured_pn = -measured_pn;

    printf("End. Mech Rot: %.4f, Pole Pairs: %.2f\r\n", mech_rotations, measured_pn);

    // 7. 恢复中断
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}


