#ifndef ADC_FOC_H
#define ADC_FOC_H

#include "stm32g4xx.h"

// 电流采样相关全局变量
extern float offset_u, offset_v, offset_w;
extern float i_u, i_v, i_w;

// 电流转换系数: 3.3V / 4096 / (16倍增益 * 0.003欧姆电阻)
#define I_COEFF 0.016815f 

// 函数声明
void ADC_Init_Registers(void);
void Calibrate_Current_Offset(void);
void Get_Current_Phases(void);

#endif


