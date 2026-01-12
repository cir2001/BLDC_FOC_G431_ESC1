#ifndef __CONTROL_H
#define __CONTROL_H

#include <stm32g4xx.h>
//---------------------------------------------------------
//
//---------------------------------------------------------
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;
    float integral;
    float last_error;
    float output_limit;
    float integral_limit;
} PID_Controller;

// 函数原型
// 速度计算与滤波变量
float calculate_speed_rad_s(float mech_angle);
float PID_Compute(PID_Controller *pid, float measure);

#endif 



