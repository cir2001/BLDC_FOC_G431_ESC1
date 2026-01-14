#ifndef __AS5047P_H
#define __AS5047P_H

#include <stm32g4xx.h>

// 寄存器地址定义
#define AS5047P_REG_ERRFL   0x0001
#define AS5047P_REG_PROG    0x0003
#define AS5047P_REG_DIAAGC  0x3FFC
#define AS5047P_REG_MAG     0x3FFD
#define AS5047P_REG_ANGLE   0x3FFE
#define AS5047P_REG_ANGLECOM 0x3FFF

// --- 寄存器操作宏 ---
#define AS_NSS_H    (GPIOA->BSRR = GPIO_BSRR_BS15)
#define AS_NSS_L    (GPIOA->BSRR = GPIO_BSRR_BR15)
#define AS_SCK_H    (GPIOB->BSRR = GPIO_BSRR_BS8)
#define AS_SCK_L    (GPIOB->BSRR = GPIO_BSRR_BR8)
#define AS_MOSI_H   (GPIOB->BSRR = GPIO_BSRR_BS6)
#define AS_MOSI_L   (GPIOB->BSRR = GPIO_BSRR_BR6)
#define AS_MISO_IN  (GPIOB->IDR & GPIO_IDR_ID7)

// 函数原型
void AS5047P_Init(void);
uint16_t AS5047P_GetAngle(void);
uint16_t SPI_Transfer(uint16_t data);
uint16_t AS5047P_ReadRegister(uint16_t addr);
uint16_t FOC_ReadAngle_Optimized(void);

uint16_t AS5047P_Transfer_Fast(uint16_t data);

#endif /* __AS5047P_H */



