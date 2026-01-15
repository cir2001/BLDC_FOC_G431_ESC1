//*****************************************************************
//  Project Name: BLDC_FOC_G431  B-G431B-ESC1开发板FOC电机控制项目
//	File Name: main.c
//  芯片：STM32G431CUb6
//  version: V0.0
//	Author: Zhou Jian
//	At:	Xi'an China
//*****************************************************************
//	功能：
//		1. BLDC 电机 FOC 控制框架搭建
//		2. TIM1 产生中心对齐 PWM 波形，驱动三相桥式驱动器
//		3. ADC 采样三相电流 与总线电压  
//*****************************************************************	
//=================================================================		
//		date		comment
//	2026-01-01   	初始化 uart2上位机发送， LED翻转测试
//  
//*****************************************************************
//----------------LED 定义 ---------------------------------------
//--- 开发板自带LED ---
//	LED0	<------->		PB   
//---------------- USART2接口  --------------------------------------- 
//	UART2   115200，1start，8bit，1stop，no parity  虚拟串口向上位机发送
//		USART2_TX			PB3
//		USART2_RX			PB4
//---- TIM1 PWM 输出定义 ----------------------------------------
//		TIM1_CH1		<------->		PA8		(U相PWM)
//		TIM1_CH1N		<------->		PC13	(U相PWM-)           
//		TIM1_CH2		<------->		PA9	    (V相PWM)
//		TIM1_CH2N		<------->		PA12	(V相PWM-)
//		TIM1_CH3		<------->		PA10	(W相PWM)
//		TIM1_CH3N		<------->		PB15	(W相PWM-)
//---- ADC 输入定义 ------------------------------------------------
//		ADC1_IN5		<------->		PA1		(U相    电流采样)Curr_fdbk1_OPAmp+
//		ADC1_IN6		<------->		PA7		(V相    电流采样)Curr_fdbk2_OPAmp+   
//		ADC1_IN7		<------->		PB0		(W相    电流采样)Curr_fdbk3_OPAmp+
//		ADC1_IN10		<------->		PA0		(总线电压采样)
//---- AS5047P 编码器接口定义 模拟SPI ----------------------------------------
//		SPI1_NSS		<------->		PA15
//		SPI1_SCK		<------->		PB8
//		SPI1_MISO		<------->		PB7
//		SPI1_MOSI		<------->		PB6
//=========================================================
#include <stm32g4xx.h>
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "led.h"
#include "as5047p.h"
#include <stdio.h>
#include "align.h"
#include "button.h"
#include "opamp.h"
// #include "control.h"
#include "adc_foc.h"
#include <stdlib.h>
//-----------------------------------------

//-----------------------------------------
uint16_t my_zero_offset = 0;
//-----------------------------------------
PID_Controller pid_id;
PID_Controller pid_iq;
PID_Controller pid_speed;

float target_iq = 0.0f;

extern int16_t raw_diff;
//-----------------------------------------
volatile uint32_t led_tick = 0;

// 用于主循环打印的调试变量
volatile float debug_theta = 0;
volatile float debug_id = 0;
volatile float debug_iq = 0;

extern float elec_angle;
extern float offset_u, offset_v, offset_w;

volatile uint16_t interrupt_flag_count = 0;
//-----------------------------------------
int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    // 系统时钟初始化 (170MHz)
    SystemClock_Config(); 
    // 延时函数初始化
    delay_init(170); 
    uart2_init(921600); // USART2 初始化，波特率 115200
    AS5047P_Init();
    delay_ms(100);

    AS5047P_Transfer_Fast(0xFFFF);
    printf("Error Flag Cleared. Starting Angle Measurement...\r\n");
        
    LED_Init(); // 初始化LED
    Button_Init(); // 初始化按键

    // 2. 驱动层初始化
    TIM1_PWM_Init(5666);     // PWM 频率设置
    printf("TIM1_PWM_Init Done...\r\n");
    delay_ms(100);

    // 执行电角度校准，获取零位偏移
    // 这个函数会强行给电机通电并对齐
    my_zero_offset = FOC_Align_Sensor();
    printf("zero_offset: %d\r\n", my_zero_offset);

    ADC_Init_Registers();    // 2. 再开启 ADC (这个函数你需要补上)
    printf("ADC Initialized.\r\n");
    delay_ms(100);

    OPAMP_Init_Registers();  // 1. 先开启运放
    printf("OPAMP Initialized.\r\n");
    delay_ms(100);

    // 3. 电流零位校准 (零电流偏置)
    // 此时 PWM 还没输出，电机电流为 0，正是采集 1.65V 偏置（约2048）的最佳时机 
    Calibrate_Current_Offset();
    delay_ms(100);

    //强制软件触发注入组一次（读取当前零电流 raw 值）
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    // 等待 JEOC 标志（注入转换完成）
    while (!(ADC1->ISR & ADC_ISR_JEOC));
    while (!(ADC2->ISR & ADC_ISR_JEOC));

    // 读取数据
    uint32_t ref_U = ADC1->JDR1;
    uint32_t ref_V = ADC2->JDR1;
    uint32_t ref_W = ADC2->JDR2;

    //设定目标值
    target_iq = -1.0f; 

    //设定 ID 环 (目标电流 0)
    pid_id.kp = 0.3f;   
    pid_id.ki = 0.05f; 
    pid_id.output_limit = 1.0f; // 这里的 limit 建议对应 SVPWM 的 v_max (0.8)

    //设定 IQ 环 (驱动电机)
    pid_iq.kp = 0.3f;   
    pid_iq.ki = 0.05f; 
    pid_iq.output_limit = 1.0f; 

    //开启中断
    TIM1->DIER |= TIM_DIER_UIE;

    //最后开启主输出 (MOE)
    // 提醒：开启这一行后，电机就会受 PID 控制了
    TIM1->BDTR |= TIM_BDTR_MOE; 

    printf("[System] FOC Loop Running. Target Iq: %.2f\r\n", target_iq);

// --- 主循环 ---
    while (1) 
    {
    //-----------------------------------------------------
        if (interrupt_flag_count >= 150) 
        {
            interrupt_flag_count = 0;
        }
    //-----------------------------------------------------
        led_tick++;
        if (led_tick >= 1000000) // 每500ms翻转一次LED0
        {
            led_tick = 0;
            // LED0_TOGGLE();      // 翻转LED0
        }
     }
}


