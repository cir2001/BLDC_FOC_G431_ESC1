#ifndef __AS5047P_SPI_H
#define __AS5047P_SPI_H

#include <stm32g4xx.h>
#include "sys.h"

// 寄存器地址定义
#define AS5047P_REG_ERRFL   0x0001
#define AS5047P_REG_PROG    0x0003
#define AS5047P_REG_DIAAGC  0x3FFC
#define AS5047P_REG_MAG     0x3FFD
#define AS5047P_REG_ANGLE   0x3FFE
#define AS5047P_REG_ANGLECOM 0x3FFF

#define AS5047P_REG_SETTINGS1   0x0018
#define AS5047P_REG_SETTINGS2   0x0019

// 常用配置：ABI 分辨率 (SETTINGS1)
// 0x0000 = 4096步 (1024 PPR)
// 0x03E7 = 1000步 (250 PPR)
#define AS5047P_ABI_RES_1024PPR 0x0000
#define AS5047P_ABI_RES_250PPR  0x03E7

/**
 * @brief 引脚操作宏定义 (寄存器方式)
 */
#define AS_NSS_H    (GPIOC->BSRR = GPIO_BSRR_BS10)
#define AS_NSS_L    (GPIOC->BSRR = GPIO_BSRR_BR10)
#define AS_SCK_H    (GPIOA->BSRR = GPIO_BSRR_BS11)
#define AS_SCK_L    (GPIOA->BSRR = GPIO_BSRR_BR11)

#define AS_MOSI_H   (GPIOB->BSRR = GPIO_BSRR_BS3)
#define AS_MOSI_L   (GPIOB->BSRR = GPIO_BSRR_BR3)

// #define AS_MOSI_H   (GPIOA->BSRR = GPIO_BSRR_BS10)
// #define AS_MOSI_L   (GPIOA->BSRR = GPIO_BSRR_BR10)

#define AS_MISO_IN  (GPIOA->IDR & GPIO_IDR_ID15)

// 函数原型
void AS5047P_SPI_Init(void);
void AS5047P_WriteReg(uint16_t addr, uint16_t data);
uint16_t AS5047P_ReadReg(uint16_t addr);

#endif /* __AS5047P_SPI_H */



