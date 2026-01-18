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
    printf("Starting Two-Stage Robust Alignment...\r\n");

    // --- 阶段 1: 预定位到 30 度 (U+V 通电) ---
    // 这样可以确保转子被拉离原来的位置，消除死区
    float ramp_v = 0.0f;
    for(ramp_v = 0.0f; ramp_v <= 0.15f; ramp_v += 0.001f) {
        TIM1->CCR1 = 0; 
        TIM1->CCR2 = (uint16_t)(ramp_v * PWM_ARR); // 增加一点电压到 25%
        TIM1->CCR3 = 0;
        TIM1->BDTR |= TIM_BDTR_MOE;
        delay_ms(2);
    }
    delay_ms(1500); // 稳定一下

    // --- 阶段 2: 最终定位到 0 度 (仅 U 通电) ---
    // 从 30 度位置平滑拉回到 U 相
    TIM1->CCR2 = 0;
    TIM1->CCR1 = (uint16_t)(0.25f * PWM_ARR);
    delay_ms(2000); // 确保彻底静止

    // --- 2. 等待机械余震消除 ---
    // 转子滑到 U 相后会产生微小摆动，强制等待 1.5 秒确保彻底静止
    printf("Waiting for stabilization...\r\n");
    delay_ms(1500); 

    // --- 3. 稳定性二次检查 (可选) ---
    // 读取两次角度，确保电机真的没在转了
    uint16_t check1 = FOC_ReadAngle_Optimized() & 0x3FFF;
    delay_ms(100);
    uint16_t check2 = FOC_ReadAngle_Optimized() & 0x3FFF;
    if(abs((int)check1 - (int)check2) > 20) {
        printf("Warning: Motor still moving during alignment!\r\n");
    }

    // --- 4. 高精度采样平均 ---
    // 连续读取 256 次取平均，消除传感器的高频随机噪声
    uint32_t sum = 0;
    for(int i = 0; i < 256; i++) {
        sum += (FOC_ReadAngle_Optimized() & 0x3FFF);
        delay_ms(1);
    }
    
    uint16_t zero_offset = (uint16_t)(sum / 256);

    // --- 5. 释放电压或切换到 FOC 模式 ---
    // 这里如果直接设为 0，电机会松开。建议保留微弱电压或直接进入闭环。
    TIM1->CCR1 = 0.05f * PWM_ARR; 
    
    // ---------------------------------------------------------
    // 微调逻辑  定义电角度 90 度对应的机械增量： 16384 / 7 / 4 = 585
    // ---------------------------------------------------------
    // 如果你的电机在 Id 闭环时 Vq 很大，就修改这个偏置
    // 选项: 0, ELECTRIC_90_DEG_OFFSET, -ELECTRIC_90_DEG_OFFSET
    // 开环 190  -20
    int16_t manual_adjust = -20; 
    
    uint16_t final_offset = (uint16_t)((zero_offset + manual_adjust) & 0x3FFF);

    printf("Raw:%d, Adjust:%d, Final:%d\r\n", zero_offset, manual_adjust, final_offset);
    
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

