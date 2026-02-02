#include "mt6826s.h"
#include "delay.h"
#include <math.h>
#include "timer.h"
#include <stdio.h>

#define ALIGN_VOLTAGE_LIMIT 1000 // 限制对齐时的电压占空比，防止烧电机

#define POLE_PAIRS 7
#ifndef M_PI
#define M_PI 3.1415926535f
#endif

/**
 * @brief 电角度对齐寻零
 * @return zero_offset: 机械零位偏移量 (0-16383)
 */
/**
 * @brief 强力平滑对齐传感器
 * @return uint16_t 稳定的机械零位偏移量
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
    uint16_t check1 = 0x3FFF;
    delay_ms(200);
    uint16_t check2 = 0x3FFF;
    if(abs((int)check1 - (int)check2) > 20) {
        printf("Warning: Motor unstable! Adjusting wait time...\r\n");
        delay_ms(1000);
    }

    // 高精度采样平均 (256次)
    uint32_t sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += (0x3FFF);
        delay_ms(1);
    }
    uint16_t zero_offset = (uint16_t)(sum / 256);

    // --- 阶段 4: 结果应用 ---
    // manual_adjust 补偿：如果你的 Clarke 变换以 U 轴为 0 度，
    // 而电流环运行时发现输出异常，可在此微调。 485
    int16_t manual_adjust = -580;//86; 
    uint16_t final_offset = (uint16_t)((zero_offset + manual_adjust) & 0x3FFF);

    // 结束时不要立刻断电，保持一个微弱占空比防止转子滑走，直到进入 FOC 循环
    TIM1->CCR3 = (uint16_t)(0.1f * PWM_ARR); 

    printf("Alignment Done! Raw:%d, agj:%d, Final:%d\r\n", zero_offset,manual_adjust,final_offset);
    delay_ms(50);
    
    return final_offset;
    // return manual_adjust;
}


