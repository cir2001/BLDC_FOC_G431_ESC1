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
uint16_t FOC_Align_Sensor(void) 
{
    printf("Force Aligning...\r\n");

    // 第一步：用较大电压强制锁死，确保克服摩擦力
    TIM1->CCR1 = (uint16_t)(0.15f * PWM_ARR); 
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->BDTR |= TIM_BDTR_MOE;
    delay_ms(1000); 

    // 第二步：稍微降低电压，减少发热，等待摆动停止
    TIM1->CCR1 = (uint16_t)(0.15f * PWM_ARR); 
    delay_ms(1000);

    // 第三步：多次采样取平均值，过滤噪声
    uint32_t avg = 0;
    for(int i=0; i<32; i++) { // 增加采样次数到 32 次
        avg += (FOC_ReadAngle_Optimized() & 0x3FFF);
        delay_ms(2);
    }
    
    uint16_t zero_offset = (uint16_t)(avg / 32);

    // 释放电机，准备进入闭环
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    printf("Verified Offset: %d\r\n", zero_offset);
    return zero_offset;
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

