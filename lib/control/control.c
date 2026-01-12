#include "control.h"

// 速度计算与滤波变量
float current_vel = 0.0f;
float filtered_vel = 0.0f;
float last_mech_angle = 0.0f;
const float LPF_ALPHA = 0.2f; // 滤波系数，越小越平滑但延迟越高 (建议 0.05-0.2)



// 初始化 PID
PID_Controller vel_pid = {
    .Kp = 0.015f,          // 初始值给小一点，慢慢往上调
    .Ki = 0.0005f, 
    .Kd = 0.0f,
    .output_limit = 0.8f,  // 最大 Vq 电压限幅
    .integral_limit = 0.2f // 积分限幅，防止静止时积分过大
};

// 在中断中调用
float calculate_speed_rad_s(float mech_angle) {
    // 1. 计算角度差 (弧度)
    float delta_angle = mech_angle - last_mech_angle;
    
    // 2. 处理 0 ~ 2PI 的回环跳转
    if (delta_angle > 3.14159265f) delta_angle -= 6.2831853f;
    if (delta_angle < -3.14159265f) delta_angle += 6.2831853f;
    
    // 3. 计算原始速度: rad/s (15000 是你的中断频率)
    float raw_vel = delta_angle * 15000.0f;
    
    // 4. 一阶低通滤波 (非常重要！)
    filtered_vel = LPF_ALPHA * raw_vel + (1.0f - LPF_ALPHA) * filtered_vel;
    
    last_mech_angle = mech_angle;
    return filtered_vel;
}

float PID_Compute(PID_Controller *pid, float measure) {
    float error = pid->target - measure;
    
    // 1. 比例项
    float P_out = pid->Kp * error;
    
    // 2. 积分项 (累加并限幅)
    pid->integral += pid->Ki * error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    // 3. 微分项 (这里暂不使用，速度环 D 项容易引起噪声)
    float D_out = pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    
    // 4. 总输出并限幅
    float output = P_out + pid->integral + D_out;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}
