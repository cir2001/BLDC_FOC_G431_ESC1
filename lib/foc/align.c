#include "as5047p.h"
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
 * @brief 设置三相 PWM 占空比
 * 引脚映射：U->PA8, V->PA9, W->PA10 
 */
void Set_PWM_Duty(uint16_t u, uint16_t v, uint16_t w) {
    TIM1->CCR1 = u;
    TIM1->CCR2 = v;
    TIM1->CCR3 = w;
    TIM1->BDTR |= TIM_BDTR_MOE;
}

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

    // 确保定时器主输出已使能 (Main Output Enable)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // --- 阶段 1: 预定位到 Phase V (0% -> 15%) ---
    // 目的：先把转子拉到一个已知位置，防止初始死区
    for(int i = 0; i <= ramp_steps; i++) {
        float ramp_v = ((float)i / ramp_steps) * stage1_v;
        TIM1->CCR1 = 0; 
        TIM1->CCR2 = (uint16_t)(ramp_v * PWM_ARR); 
        TIM1->CCR3 = 0;
        delay_ms(2);
    }
    delay_ms(500); // 短暂稳定

    // --- 阶段 2: 平滑切换 (Phase V -> Phase U) ---
    // 目的：让磁场从 V 相线性转移到 U 相，转子会平滑滑行 120 度电角度
    for(int i = 0; i <= ramp_steps; i++) {
        float alpha = (float)i / ramp_steps;
        // V相电压从 15% 降至 0%
        TIM1->CCR2 = (uint16_t)((1.0f - alpha) * stage1_v * PWM_ARR);
        // U相电压从 0% 升至 25%
        TIM1->CCR1 = (uint16_t)(alpha * max_v * PWM_ARR);
        TIM1->CCR3 = 0;
        delay_ms(2);
    }

    // --- 阶段 3: 最终静止与采样 ---
    printf("Waiting for stabilization...\r\n");
    delay_ms(1500); // 彻底消除机械余震

    // 稳定性检查
    uint16_t check1 = FOC_ReadAngle_Optimized() & 0x3FFF;
    delay_ms(200);
    uint16_t check2 = FOC_ReadAngle_Optimized() & 0x3FFF;
    if(abs((int)check1 - (int)check2) > 20) {
        printf("Warning: Motor unstable! Adjusting wait time...\r\n");
        delay_ms(1000);
    }

    // 高精度采样平均 (256次)
    uint32_t sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += (FOC_ReadAngle_Optimized() & 0x3FFF);
        delay_ms(1);
    }
    uint16_t zero_offset = (uint16_t)(sum / 256);

    // --- 阶段 4: 结果应用 ---
    // manual_adjust 补偿：如果你的 Clarke 变换以 U 轴为 0 度，
    // 而电流环运行时发现输出异常，可在此微调。
    int16_t manual_adjust = 120; 
    uint16_t final_offset = (uint16_t)((zero_offset + manual_adjust) & 0x3FFF);

    // 结束时不要立刻断电，保持一个微弱占空比防止转子滑走，直到进入 FOC 循环
    TIM1->CCR1 = (uint16_t)(0.05f * PWM_ARR); 

    printf("Alignment Done! Raw:%d, Final:%d\r\n", zero_offset, final_offset);
    
    return final_offset;
}

float Update_Electrical_Angle(uint16_t current_mech_angle, uint16_t zero_offset)
{
    // 1. 计算机械角度偏差
    int32_t delta_angle = (int32_t)current_mech_angle - (int32_t)zero_offset;
    
    // 2. 修正机械回环 (确保 delta_angle 在 -8192 到 8191 之间)
    if (delta_angle > 8191) delta_angle -= 16384;
    if (delta_angle < -8192) delta_angle += 16384;
    
    // 3. 转换为机械弧度
    float mech_angle_rad = (float)delta_angle * (2.0f * M_PI / 16384.0f);
    
    // 4. 计算电角度 (12N14P 电机极对数为 7)
    float elec_angle = mech_angle_rad * 7.0f; // 这里的 7.0f 就是 POLE_PAIRS

    // 5. 弧度归一化到 [0, 2*PI]
    while(elec_angle >= 2.0f * M_PI) elec_angle -= 2.0f * M_PI;
    while(elec_angle < 0.0f)         elec_angle += 2.0f * M_PI;

    // 尝试在此处添加方向取反逻辑
    //elec_angle = 2.0f * 3.14159265f - elec_angle;
    
    return elec_angle; // 根据实际电机安装方向调整电角度零点
}

