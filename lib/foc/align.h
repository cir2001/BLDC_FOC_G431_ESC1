#ifndef __ALIGN_H
#define __ALIGN_H

#include <stm32g4xx.h>

// 函数原型
uint16_t FOC_Align_Sensor(void);
float Update_Electrical_Angle(uint16_t current_mech_angle, uint16_t zero_offset);

#endif 



