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
//	LED0	<------->		PC6   
//---------------- USART2接口  --------------------------------------- 
//	UART2   115200，1start，8bit，1stop，no parity  虚拟串口向上位机发送
//		USART2_TX			PB3
//		USART2_RX			PB4
//---- TIM1 PWM 输出定义 ----------------------------------------
//		TIM1_CH1		<------->		PA8		(U相PWM)
//		TIM1_CH1N		<------->		PB13	(U相PWM-)           
//		TIM1_CH2		<------->		PA11	(V相PWM)
//		TIM1_CH2N		<------->		PB14	(V相PWM-)
//		TIM1_CH3		<------->		PA12	(W相PWM)
//		TIM1_CH3N		<------->		PB15	(W相PWM-)
//---- ADC 输入定义 ------------------------------------------------
//		ADC1_IN5		<------->		PA5		(U相    电流采样)
//		ADC1_IN6		<------->		PA6		(V相    电流采样)   
//		ADC1_IN7		<------->		PA7		(W相    电流采样)
//		ADC1_IN10		<------->		PB0		(总线电压采样)
//=========================================================
#include <stm32g4xx.h>
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
// --- FOC 参数与变量定义 ---
typedef struct {
    float target_voltage_q;
    float target_voltage_d;
    float current_angle;
    // ... 其他 PID 参数
} FOC_Controller;

FOC_Controller foc;


int main(void) {
    // 1. 系统时钟初始化 (170MHz)
    SystemClock_Config(); 

    // 2. 延时函数初始化
    delay_init(170); 

    //uart1_init(115200); // 3. USART1 初始化，波特率 115200
    uart2_init(115200); // 3. USART1 初始化，波特率 115200
    //simple_uart_init(115200);
    //printf("FOC Project Start! CPU Clock: 170MHz\r\n");

    // 170,000,000 / (0+1) / (5666 * 2) ≈ 15,000 Hz (15kHz)
    TIM1_PWM_Init(5666, 0);

    // 3. 配置 PB6 为输出 (B-G431B-ESC1 板载绿灯)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // 开启 GPIOB 时钟
    GPIOC->MODER &= ~(3U << (6 * 2));    // 清除 PB6 的模式位
    GPIOC->MODER |= (1U << (6 * 2));     // 设置为通用输出 (01)

    // 4. 暴力翻转测试
    while (1) {
        GPIOC->ODR ^= (1U << 6);  // 翻转 PB6 电平
        //delay_ms(500);            // 延时 500ms
        printf("System Running...\r\n");

       
        delay_ms(100);
    }
}





