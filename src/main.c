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
#include <math.h>
//------------------------------------------
typedef enum { false = 0, true = 1 } bool;
//-----------------------------------------
void FOC_PreAlign(float vd_align, uint32_t duration_ms);
void Reset_PID_Controllers(void);
//-----------------------------------------
uint16_t my_zero_offset = 0;
//-----------------------------------------
PID_Controller pid_id;
PID_Controller pid_iq;
PID_Controller pid_speed;

extern float target_speed;
extern float actual_speed_filt;
extern float target_iq;
extern float target_id;  

extern volatile float Vd;
extern volatile float Vq;

extern int16_t raw_diff;
//-----------------------------------------
// --- 新增：全局控制标志位 ---
volatile uint8_t run_foc_flag = 0; // 0: 强制输出模式, 1: PID 闭环模式

volatile uint32_t led_tick = 0;

// 用于主循环打印的调试变量
volatile float debug_theta = 0;
volatile float debug_id = 0;
volatile float debug_iq = 0;
volatile float debug_Vq = 0;
volatile float debug_Vd = 0;

volatile float debug_iu = 0;
volatile float debug_iv = 0;
volatile float debug_iw = 0;
volatile float debug_mech_angle  = 0;

extern float elec_angle;
extern float offset_u, offset_v, offset_w;
//================================================================================
int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    // 系统时钟初始化 (170MHz)
    SystemClock_Config(); 
    // 延时函数初始化
    delay_init(170); 
    uart2_init(115200); // USART2 初始化，波特率 115200
    AS5047P_Init();
    delay_ms(100);

    AS5047P_Transfer_Fast(0xFFFF);
    printf("Error Flag Cleared. Starting Angle Measurement...\r\n");
        
    LED_Init(); // 初始化LED
    Button_Init(); // 初始化按键

    // 2. 驱动层初始化
    TIM1_PWM_Init(5666);     // PWM 频率设置
    delay_ms(100);
    printf("TIM1_PWM_Init Done...\r\n");

    // 执行电角度校准，获取零位偏移
    // 这个函数会强行给电机通电并对齐
    my_zero_offset = FOC_Align_Sensor();
    printf("zero_offset: %d\r\n", my_zero_offset);
    delay_ms(500);

    OPAMP_Init_Registers();  
    printf("OPAMP Initialized.\r\n");
    delay_ms(50);

    ADC_Init_Registers();   
    printf("ADC Initialized.\r\n");
    delay_ms(50);

    // 电流零位校准 (零电流偏置)
    // 此时 PWM 还没输出，电机电流为 0，正是采集 1.65V 偏置（约2048）的最佳时机 
    printf("Starting current offset calibration...\r\n");
    Calibrate_Current_Offset();
    printf("Current offset calibration complete.\r\n");

    // 可选：标定后立即验证一次零电流值（调试用）
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
    while (!(ADC1->ISR & ADC_ISR_JEOC) && !(ADC2->ISR & ADC_ISR_JEOC));
    ADC1->ISR |= ADC_ISR_JEOC | ADC_ISR_JEOS;
    ADC2->ISR |= ADC_ISR_JEOC | ADC_ISR_JEOS;

    printf("Verify zero current raw: U=%lu, V=%lu, W=%lu\r\n",
           ADC1->JDR1, ADC2->JDR1, ADC2->JDR2);

    // 重要：需要开启才能自启动
    TIM1->BDTR |= TIM_BDTR_MOE;

    // --- 速度环 (外环) ---
    target_speed = 10.0f;      
    pid_speed.kp = 0.1f;      // 速度环 Kp 通常较小
    pid_speed.ki = 0.01f; 
    pid_speed.output_limit = 2.0f; // 限制最大电流

    // --- 电流环 (内环) ---
    pid_id.kp = 0.005f;   pid_id.ki = 10.0f; 
    pid_id.output_limit = 1.5; // 对应 SVPWM 最大电压 (1.0)
    
    pid_iq.kp = 0.005f;   pid_iq.ki = 10.0f; 
    pid_iq.output_limit = 1.5;

    // --- 第四步：执行【开机预对齐】逻辑 ---
    FOC_PreAlign(0.4f, 1000);  // 锁死
    Vq = 0.0f;

    // // 完全松开（确保电压输出 0）
    Vd = 0.05f;
    Vq = 0.0f;
    FOC_SVPWM_Update(Vd, Vq, 0.0f);
    delay_ms(200);  // 800ms 彻底松开

    // --- 第五步：简单自动启动（扰动 + 冲击） ---
    // 先开启闭环
    run_foc_flag = 1;

    Vd = 0.0f; // 保持 PreAlign 函数中给的值
    Vq = 0.3f;

    // 反向小脉冲 + 正向冲击（模拟手拨）
    target_iq = -1.2f;  // 反向 1.2A
    delay_ms(100);
    target_iq = 0.0f;
    delay_ms(50);
    target_iq = 1.5f;   // 正向冲击 1.5A
    delay_ms(300);
    target_iq = 0.5f;   // 降回正常
    // target_iq = 0.5f;
    // target_id = 0.0f;

    // 3. 启动控制
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;    // 确保计数器在运行

    printf("[System] FOC Loop Running. Target Iq: %.2f\r\n", target_iq);
// -------------------------- 主循环 --------------------------
    while (1) 
    {

    //-----------------------------------------------------
        led_tick++;
        if (led_tick >= 100000) // 每500ms翻转一次LED0
        {
            led_tick = 0;
            // LED0_TOGGLE();      // 翻转LED0
        }
     }
}

/**
 * @brief 强行将转子拉到电角度 0 度位置
 * @param vd_align 对齐电压（建议 0.15f ~ 0.25f）
 */
void FOC_PreAlign(float vd_align, uint32_t duration_ms) {
    printf("[System] Starting Pre-Align Ramp...\r\n");
    
    run_foc_flag = 0; // 强制进入预对齐模式（屏蔽中断里的 PID）

    // 1. 爬升阶段：缓慢增加 Vd
    for (int i = 0; i <= 100; i++) {
        // 同步更新全局变量，这样中断里的打印和输出才能统一
        Vd = (vd_align * i) / 100.0f; 
        Vq = 0.0f; 
        
        // 强制角度为 0
        FOC_SVPWM_Update(Vd, Vq, 0.0f); 
        delay_ms(2);
    }

    // 2. 锁定阶段：保持高 Vd 锁定转子
    printf("[System] Locking Rotor at Vd = %.2f...\r\n", Vd);

    delay_ms(duration_ms);

    // 3. 关键：清理 PID 积分记忆
    Reset_PID_Controllers();
    
    printf("[System] Pre-Align Done.\r\n");
}

void Reset_PID_Controllers(void) {
    // 仅重置结构体中存在的 integral 成员
    pid_id.integral = 0.0f;
    pid_iq.integral = 0.0f;
    
    pid_speed.integral = 0.0f;
}




