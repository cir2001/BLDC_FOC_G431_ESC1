# BLDC_FOC_G431_ESC1
## 2026-01-01 V0.0 初始化
- ESC1开发板 LED UART2上位机发送
## 2026-01-11 V0.1 闭环转动 初步测试
- ESC1开发板 模拟SPI采集AS5407P磁编码器位置，电机转动，初步
## 2026-01-12 V0.1 电压FOC 基础
- TIM1中断里，实现开环测试，电压FOC测试，闭环不带SVPWM_Output_Standard测试，标准  SVPWM 开环测试；SVPWM_Output_Standard (Min-Max 法)测试
- 实现速度环控制PID，给定vq，实现速度的控制   
## 2026-01-14 V0.1 OPAMP测试 通过
- OPAMP初始化代码，测试通过
## 2026-01-14 V0.1 电流FOC PID 首次转动
- 电流FOC PID 首次转动