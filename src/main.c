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
//-----------------------------------------
uint16_t my_zero_offset = 0;
//-----------------------------------------
// --- FOC 参数与变量定义 ---
typedef struct {
    float target_voltage_q;
    float target_voltage_d;
    float current_angle;
    // ... 其他 PID 参数
} FOC_Controller;

FOC_Controller foc;
//-----------------------------------------
volatile uint32_t led_tick = 0;

extern uint16_t current_raw;
extern float elec_angle;
//-----------------------------------------
int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    // 系统时钟初始化 (170MHz)
    SystemClock_Config(); 
    // 延时函数初始化
    delay_init(170); 
    uart2_init(115200); // USART2 初始化，波特率 115200
    //printf("FOC Project Start! CPU Clock: 170MHz\r\n");
    AS5047P_Init();
    delay_ms(100);
    // 连续读取两次错误寄存器以清除标志
    AS5047P_ReadRegister(0x0001); 
    delay_ms(1);
    AS5047P_ReadRegister(0x0001); 
    printf("Error Flag Cleared. Starting Angle Measurement...\r\n");
        
    LED_Init(); // 初始化LED
    Button_Init(); // 初始化按键

    TIM1_PWM_Init(5666);

    // 等待用户准备好 (可选：按下 PC10 按钮再开始)
    // 根据手册 Table 4, PC10 是用户按钮 
    printf("Press User Button to start Alignment...\r\n");
    while((GPIOC->IDR & (1U << 10))); // 等待按钮按下 (低电平触发)

    // 执行电角度校准，获取零位偏移
    // 这个函数会强行给电机通电并对齐
    my_zero_offset = FOC_Align_Sensor();
    printf("zero_offset: %d\r\n", my_zero_offset);

    // 3. 校准完后再开启中断
    TIM1->SR = ~TIM_SR_UIF;     // 先清除可能的悬挂标志位
    TIM1->DIER |= TIM_DIER_UIE; // 正式开启中断

// --- 主循环 ---
    while (1) 
    {
        //TIM1->BDTR |= TIM_BDTR_MOE; // 开启输出
        led_tick++;
        if (led_tick >= 10000000) // 每500ms翻转一次LED0
        {
            led_tick = 0;
            //LED0_TOGGLE();      // 翻转LED0
        }

        // u16 error_reg = AS5047P_ReadRegister(0x3FFC); // 读取错误寄存器测试
        // printf("Error Register: 0x%04X\r\n", error_reg);    
        
        // delay_ms(100);
        // uint16_t raw = AS5047P_GetAngle();
        // uint16_t angle_14bit = current_raw & 0x3FFF; // 强制提取 14 位角度
        // float deg = (float)(angle_14bit * 360.0f / 16384.0f);
        // printf("Angle: %5d | Deg: %d.%02d\r\n",angle_14bit,(int)deg,(int)((deg - (int)deg) * 100));

        // printf("Angle: %5d | elec_angle: %d.%02d\r\n",current_raw,(int)elec_angle,(int)((elec_angle - (int)elec_angle) * 100));

        // if (GPIOB->IDR & (1U << 7)) printf("MISO is HIGH\r\n");
        // else printf("MISO is LOW\r\n");
        // delay_ms(500);
        
        // uint16_t current_raw = AS5047P_GetAngle();
        // float elec_angle = Update_Electrical_Angle(current_raw, my_zero_offset);

        // printf("elec_angle: %d.%03d rad\r\n",(int)elec_angle,(int)((elec_angle - (int)elec_angle) * 1000));
        //printf("elec_angle: %d.%03d rad\r\n",(int)angle,(int)((angle - (int)angle) * 1000));
        // 只有角度发生较大变化时才打印，或者降低打印频率
       

        // uint16_t error_reg = SPI_Transfer(0x4000 | 0x3FFC); 
        // printf("Error Register: 0x%04X\r\n", error_reg);    

        //printf("System Running...\r\n");

        //delay_ms(100);
    }
}





