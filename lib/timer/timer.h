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


void TIM1_PWM_Init(u16 arr);


#endif
