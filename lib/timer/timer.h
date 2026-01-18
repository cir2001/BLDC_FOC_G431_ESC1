#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
//
//
//********************************************************************************

////////////////////////////////////////////////////////////////////////////////// 	
#define ENCODER_TIM2_PERIOD  0xFFFFFFFF		//1322
#define ENCODER_TIM3_PERIOD  0xFFFF	//1322
#define ENCODER_TIM4_PERIOD  0xFFFF    //1043
#define ENCODER_TIM5_PERIOD  0xFFFF    //1043

#define COUNTER_RESET 0			//0

// 宏定义：基于 ARR = 5666
#define PWM_ARR             5666.0f
#define PWM_ARR_INT         5666

// 1. 先定义结构体
typedef struct {
    float kp;
    float ki;
    float integral;
    float output_limit;
} PID_Controller;

void TIM1_PWM_Init(u16 arr);
void SVPWM_Output_Standard(float Valpha, float Vbeta);
float PID_Calc(PID_Controller* pid, float target, float current);
float PID_Calc_Speed(PID_Controller* pid, float target, float current);
void FOC_SVPWM_Update(float Vd, float Vq, float angle);


#endif
