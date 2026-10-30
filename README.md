# 周期信号测量分析装置（STM32F407）

2026 年全国大学生电子设计竞赛 G 题作品。以 STM32F407VET6 为核心，实现"示波器 + 频谱分析仪 + 信号发生器"三合一。

## 功能

- **波形显示**：实时采集并显示 480 点波形，支持横向缩放与多周期显示
- **参数测量**：频率、峰峰值、真有效值、直流分量、占空比
- **频谱分析**：4096 点 FFT，输出频谱曲线
- **谐波分析**：自动搜索前 3 个频率分量，给出各自的频率、峰值、有效值与相位
- **信号发生**：DAC + DMA 输出，支持正弦、方波、三角、锯齿、反锯齿，频率 1 Hz – 500 kHz，可调幅值、占空比、偏置
- **自动测量**：自动切换采样率完成一帧采集与计算
- **人机交互**：串口 HMI 触摸屏，通过 USART 指令控制

## 硬件方案

| 项目 | 说明 |
| --- | --- |
| 主控 | STM32F407VET6，LQFP100，HCLK 144 MHz |
| 信号采集 | ADC1 / ADC2 / ADC3 三重交替采样，12 bit，2 MSPS，TIM8 TRGO 触发，DMA 搬运 4096 点 |
| 信号输出 | DAC1 + TIM2 TRGO 触发 + DMA，最高 1440 点/周期 |
| 频率测量 | TIM5 输入捕获，计数时钟 2 MHz，同时给出占空比 |
| 显示 | 串口 HMI 屏（USART） |
| 开发环境 | STM32CubeMX + Keil MDK 5（MDK-ARM V5.32） |

## 目录结构

```
Core/          CubeMX 生成的外设初始化代码与 main.c
Hardware/      外设驱动层：ADC/DAC 的 DMA 采样与输出、HMI 串口屏、串口协议解析
myCode/        业务逻辑层：主状态机、FFT 与谐波分析、测频、参数配置
MDK-ARM/       Keil 工程（OSC.uvprojx）与启动文件
OSC.ioc        STM32CubeMX 工程文件
Drivers/       STM32 HAL 与 CMSIS（ST 官方代码）
Middlewares/   CMSIS-DSP 预编译库（ST 官方代码）
指令.docx       HMI 串口屏指令表
```

## 编译与烧录

1. 安装 Keil MDK 5，并装好 `Keil::STM32F4xx_DFP` 器件支持包
2. 用 Keil 打开 `MDK-ARM/OSC.uvprojx`
3. `F7` 编译，`F8` 下载（ST-Link / DAP-Link 均可）
4. 需要修改引脚或外设时，用 STM32CubeMX 打开 `OSC.ioc` 重新生成代码

> 仓库中保留了 `Drivers/` 与 `Middlewares/`，克隆后可直接编译。若只想保留自己写的代码，可在 `.gitignore` 中启用对应两行，再用 CubeMX 重新生成。

## 主要指标

| 项目 | 指标 |
| --- | --- |
| 采样率 | 2 MSPS（单帧 4096 点） |
| 可测频率上限 | 1 MHz |
| FFT 点数 | 4096 |
| 输出波形 | 正弦 / 方波 / 三角 / 锯齿 / 反锯齿 |
| 输出频率范围 | 1 Hz – 500 kHz |

## 说明

本项目为竞赛学习存档，`Drivers/`、`Middlewares/` 版权归 STMicroelectronics 所有，遵循其原始许可。

作者：朱莅欣 · 电子信息学院 · 2026
