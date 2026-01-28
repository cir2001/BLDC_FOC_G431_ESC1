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
#include "control.h"
#include "adc_foc.h"
#include <stdlib.h>
#include <math.h>
//------------------------------------------

//-----------------------------------------
void FOC_PreAlign(float vd_align, uint32_t duration_ms);
void Reset_PID_Controllers(void);
void TIM1_Force_Update_Test(u16 arr);
void Check_DMA_Complete(void);
//-----------------------------------------
int16_t my_zero_offset = 0;
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

extern VofaData_t DataLog[2][SAMPLE_NUM][FRAME_SIZE];

extern volatile uint8_t  write_bank;    // 当前正在写入哪个缓冲 (0 或 1)
extern volatile uint32_t write_ptr;     // 写入指针
extern volatile uint8_t  bank_ready; // 哪一个缓冲准备好了发送 (0, 1 或 0xFF表示无)
extern volatile uint8_t  is_dma_busy;   // DMA 发送状态

//-----------------------------------------
// --- 新增：全局控制标志位 ---
volatile uint8_t run_foc_flag = 0; // 0: 强制输出模式, 1: PID 闭环模式

volatile uint32_t led_tick = 0;

// 用于主循环打印的调试变量
volatile float debug_id = 0;
volatile float debug_iq = 0;
volatile float debug_Vq = 0;
volatile float debug_Vd = 0;

volatile float debug_iu = 0;
volatile float debug_iv = 0;
volatile float debug_iw = 0;

extern float elec_angle;
//================================================================================
int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    // 系统时钟初始化 (170MHz)

    SystemClock_Config(); 
    // 延时函数初始化
    delay_init(170); 

    uart2_init(921600); // USART2 初始化，波特率 115200

    __enable_irq(); // 【必须】开启全局中断

    // 3. CORDIC 硬件加速器初始化 (必须在算法调用前)
    CORDIC_Init();
    delay_ms(100);
    AS5047P_Init();
    delay_ms(100);

    AS5047P_Transfer_Fast(0xFFFF);
    printf("Error Flag Cleared. Starting Angle Measurement...\r\n");
        
    LED_Init(); // 初始化LED
    Button_Init(); // 初始化按键

    // 2. 驱动层初始化
    TIM1_PWM_Init(5666);     // PWM 频率设置
    printf("TIM1_PWM_Init Done...\r\n");
    delay_ms(50);
    OPAMP_Init_Registers(); 
    printf("OPAMP Initialized.\r\n");
    delay_ms(50); 

    ADC_Init_Registers();   
    printf("ADC Initialized.\r\n");
    delay_ms(50);

    // 电流零位校准 (零电流偏置)
    Calibrate_Current_Offset();
    printf("Current offset calibration complete.\r\n");
    delay_ms(50);

    // 执行电角度校准，获取零位偏移
    my_zero_offset = FOC_Align_Sensor();
    printf("zero_offset: %d\r\n", my_zero_offset);
    delay_ms(100);

    // 标定后立即验证一次零电流值（调试用）
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
    while (!(ADC1->ISR & ADC_ISR_JEOC) && !(ADC2->ISR & ADC_ISR_JEOC));
    ADC1->ISR |= ADC_ISR_JEOC | ADC_ISR_JEOS;
    ADC2->ISR |= ADC_ISR_JEOC | ADC_ISR_JEOS;

    // --- 速度环 (外环) ---
    target_speed = -35.0f;      
    pid_speed.kp = 15.0f;      // 速度环 Kp 通常较小
    pid_speed.ki = 0.5f; 
    pid_speed.output_limit = 8000.0f; // 限制最大电流

    // --- 电流环 (内环) ---
    pid_id.kp = 80.0f;   pid_id.ki = 400.0f; 
    pid_id.output_limit = 8000.0f; // 对应 SVPWM 最大电压 
    
    pid_iq.kp = 50.0f;   pid_iq.ki = 200.0f; 
    pid_iq.output_limit = 8000.0f; // 对应 SVPWM 最大电压

    // // --- 启动 ---
    run_foc_flag = 1;

    target_id = 0.0f;
    target_iq = 1.0f;

    // 3. 启动控制
    TIM1->DIER |= TIM_DIER_UIE; // 开启中断，正式进入 FOC 闭环
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;    // 确保计数器在运行

// -------------------------- 主循环 --------------------------
    while (1) 
    {
        // --- 1. 启动发送逻辑 ---
        // 检查是否有 bank 准备好，且 DMA 此时没活干
        if (bank_ready != 0xFF && is_dma_busy == 0) {
            is_dma_busy = 1; 
            uint8_t current_send_bank = bank_ready;
            bank_ready = 0xFF; // 释放标志，允许 Timer 准备下一个

            DMA1_Channel1->CCR &= ~DMA_CCR_EN;
            DMA1->IFCR = DMA_IFCR_CGIF1; // 清除之前的残留标志
            
            DMA1_Channel1->CMAR = (uint32_t)DataLog[current_send_bank];
            // 关键：字节数 = 采样数 * 通道数 * 4
            DMA1_Channel1->CNDTR = SAMPLE_NUM * FRAME_SIZE * 4; 
            
            DMA1_Channel1->CCR |= DMA_CCR_EN;
            // 确保串口的 DMAT 位是开启的
            USART2->CR3 |= USART_CR3_DMAT; 
        }

        // --- 2. 检查完成逻辑 ---
        // 在主循环查 DMA 状态，彻底替代中断
        // 只要 DMA 的 TCIF1 标志位置 1，说明搬运完成
        if (is_dma_busy && (DMA1->ISR & DMA_ISR_TCIF1)) {
            DMA1->IFCR = DMA_IFCR_CTCIF1; // 清除完成标志
            // 这里千万不要去动 USART2->CR3，保持 DMAT 开启即可
            is_dma_busy = 0; // 释放锁，允许下一波发送
        }
    //-----------------------------------------------------
        led_tick++;
        if (led_tick >= 100000) // 每500ms翻转一次LED0
        {
            led_tick = 0;
            
            // LED0_TOGGLE();      // 翻转LED0
        }
     }
}



