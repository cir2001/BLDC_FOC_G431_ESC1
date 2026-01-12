# BLDC_FOC_G431_ESC1
## 2026-01-01 V0.0 初始化
- ESC1开发板 LED UART2上位机发送
## 2026-01-11 V0.1 闭环转动 初步测试
- ESC1开发板 模拟SPI采集AS5407P磁编码器位置，电机转动，初步
## 2026-01-12 V0.1 电压FOC 基础
- TIM1中断里，实现开环测试，电压FOC测试，闭环不带SVPWM_Output_Standard测试，标准  SVPWM 开环测试；SVPWM_Output_Standard (Min-Max 法)测试