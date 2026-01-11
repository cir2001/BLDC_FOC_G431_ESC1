#include "as5047p.h"
#include "delay.h"
#include <math.h>
#include "timer.h"
// 假设 TIM1->ARR = 5666 (15kHz)
#define PWM_PERIOD 5666
#define ALIGN_VOLTAGE_LIMIT 1000 // 限制对齐时的电压占空比，防止烧电机

#define POLE_PAIRS 7
#define M_PI 3.1415926535f

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
    printf("Starting Alignment...\r\n");

    float test_angle = 0.0f; // 软件零度
    float v_align = 0.25f;   // 足够把转子拉动的对齐电压
    
    // 强制设定 Vd 为对齐电压，Vq 为 0
    float Valpha = v_align * cosf(test_angle); 
    float Vbeta  = v_align * sinf(test_angle);

    SVPWM_Output(Valpha, Vbeta); // 使用你的 SVPWM 函数进行对齐
    TIM1->BDTR |= TIM_BDTR_MOE;  // 确保输出开启

    // 2. 等待电机转子稳定 (对于机器人关节，1-2秒足够)
    delay_ms(2000);

    // 3. 读取此时编码器的机械角度作为零位偏移
    uint32_t avg_offset = 0;
    for(int i=0; i<10; i++) {
        avg_offset += (AS5047P_GetAngle() & 0x3FFF);
        delay_ms(10);
    }

    uint16_t zero_offset = (avg_offset / 10) & 0x3FFF;

    // 4. 关闭输出，防止电机长时间通电过热
    Set_PWM_Duty(0, 0, 0);

    printf("Alignment Done! Zero Offset: %d\r\n", zero_offset);
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
    // 使用 fmodf 效率更高
    elec_angle = fmodf(elec_angle, 2.0f * M_PI);
    if (elec_angle < 0) elec_angle += 2.0f * M_PI;

    // 尝试在此处添加方向取反逻辑
    elec_angle = 2.0f * 3.14159265f - elec_angle;
    
    return elec_angle; // 根据实际电机安装方向调整电角度零点
}

