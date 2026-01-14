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
//-----------------------------------------

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
extern float offset_u, offset_v, offset_w;
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

    // 2. 驱动层初始化
    TIM1_PWM_Init(5666);     // PWM 频率设置
    printf("TIM1_PWM_Init Done...\r\n");
    delay_ms(100);

    ADC_Init_Registers();    // 2. 再开启 ADC (这个函数你需要补上)
    printf("ADC Initialized.\r\n");
    delay_ms(100);

    OPAMP_Init_Registers();  // 1. 先开启运放
    printf("OPAMP Initialized.\r\n");
    delay_ms(100);

    printf("\r\n[OPAMP] Forced Config After Init:\r\n");
    printf("OP1: 0x%08lX   OP2: 0x%08lX   OP3: 0x%08lX\r\n",
       OPAMP1->CSR, OPAMP2->CSR, OPAMP3->CSR);


    // 3. 电流零位校准 (零电流偏置)
    // 此时 PWM 还没输出，电机电流为 0，正是采集 1.65V 偏置（约2048）的最佳时机 
    Calibrate_Current_Offset();
    printf("Current Offset Calibrated.\r\n");
    delay_ms(100);

    printf("[OPAMP] After Current Calibration:\r\n");
    printf("OP1: 0x%08lX   OP2: 0x%08lX   OP3: 0x%08lX\r\n",
           OPAMP1->CSR, OPAMP2->CSR, OPAMP3->CSR);

    printf("[Test] Reading pure zero-current reference before hardware trigger...\r\n");

    delay_ms(20);  // 等待模拟稳定
   // 强制软件触发注入组一次（读取当前零电流 raw 值）
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    // 等待 JEOC 标志（注入转换完成）
    while (!(ADC1->ISR & ADC_ISR_JEOC));
    while (!(ADC2->ISR & ADC_ISR_JEOC));

    // 读取数据
    uint32_t ref_U = ADC1->JDR1;
    uint32_t ref_V = ADC2->JDR1;
    uint32_t ref_W = ADC2->JDR2;

    // 打印参考值（可用于对比或手动验证）
    printf("Pure Zero-Current Ref (before trigger): U=%lu V=%lu W=%lu\r\n", ref_U, ref_V, ref_W);

    // 可选：与校准 offsets 对比，检查是否一致
    printf("Offsets from calib: U=%.2f V=%.2f W=%.2f\r\n", offset_u, offset_v, offset_w);

    // 继续后面的代码...

    // 启动 ADC 硬件触发（TIM1_TRGO2 触发注入组）
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;

    // 开启下桥臂短路模式
    TIM1->CCR1 = 0; 
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE); 
    TIM1->BDTR |= TIM_BDTR_MOE; 

    printf("[System] Testing Mode: Low-side FETs Shorted.\r\n");
    printf("[System] Rotate the motor manually to see current swings!\r\n");

uint32_t count = 0;
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
        if (ADC2->ISR & ADC_ISR_JEOC) 
        {
            float iu = (float)ADC1->JDR1 - offset_u;
            float iv = (float)ADC2->JDR1 - offset_v;
            float iw = (float)ADC2->JDR2 - offset_w;

            float i_sum = iu + iv + iw;

            // 4. 打印 (请仔细检查这里的变量名顺序！)
            if (count++ >= 10000) 
            {
                // 这里的参数必须是 iu, iv, iw，而不是 offset_u...
                printf("U:%.1f V:%.1f W:%.1f | Sum:%.1f\r\n", iu, iv, iw, i_sum);
                count = 0;
            }

            // 5. 清除标志位
            ADC1->ISR |= ADC_ISR_JEOC;
            ADC2->ISR |= ADC_ISR_JEOC;
        }
     }
}


