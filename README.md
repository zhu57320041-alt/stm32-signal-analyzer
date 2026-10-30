# STM32F407 周期信号测量分析装置

2026 年全国大学生电子设计竞赛 G 题作品。以 STM32F407VET6 为核心，实现"示波器 + 频谱分析仪 + 信号发生器"三合一。

---

## ⚠️ 来源声明

**本项目基于开源项目 [Tangcy666/stm32-osc](https://github.com/Tangcy666/stm32-osc)（作者：Tang）二次开发。**

原始项目提供了完整的工程骨架，本项目的以下部分直接沿用或在其基础上修改：

- CubeMX 工程配置（`OSC.ioc`）与文件夹结构
- `Hardware/` 层的模块划分：`ADC_DMA`、`DAC_DMA`、`HMI`、`message_UART`
- DAC 波形输出（正弦、方波、三角、锯齿）的实现
- TIM5 输入捕获测频的实现
- 串口屏通信协议与指令表

本项目在此基础上做的工作，在各版本的说明中逐条列出。原项目未声明开源许可证，版权归原作者所有；本仓库仅作学习与竞赛存档用途。如原作者有任何异议，请联系删除。

---

## 版本说明

本仓库包含两个分支，对应两条不同的技术路线：

| 分支 | 路线 | 适用场景 |
| --- | --- | --- |
| `main` | 单 ADC，固定 2 MSPS，4096 点 | 结构清晰、分析功能完整，作为主推版本 |
| `triple-adc` | 保留原版六档采样率，7.2 MHz 时 ADC1/2/3 三重交替，1024 点 | 采样率高、低频覆盖好，保留原版能力 |

---

## 两个版本的差异

| 项目 | `main` | `triple-adc` |
| --- | --- | --- |
| 采样方式 | 单 ADC1 + DMA | ADC1/2/3 三重交替 + MultiMode DMA |
| 采样率 | 固定 2 MSPS | 6 档：100 Hz / 1 kHz / 10 kHz / 100 kHz / 1 MHz / 7.2 MHz |
| 每帧点数 | 4096（`uint16` 缓冲） | 1024（多模式时 512 × `uint32`，2 样本打包） |
| 一帧时长 | 2.048 ms | 7.2 MHz 档约 142 µs |
| FFT 点数 | 4096 | 1024 |
| 主程序结构 | 状态机，12 个显示模式 | 原版 `while(1)` 结构 |
| 谐波分析 | 前 3 个分量的频率、峰值、有效值、相位 | 无，仅主频识别 |
| 波形重建 | 480 点 | 无 |
| 串口屏 | USART3（PC10 / PC11，921600） | USART1（PB6 / PB7，921600） |
| DAC 输出 | 与原版一致 | 与原版一致 |
| TIM5 测频 | 与原版一致 | 与原版一致 |

**取舍**：`main` 固定 2 MSPS，频点换算简单、标定容易，但 4096 点只覆盖 2 ms，低频信号周期数少；`triple-adc` 采样率上限高、档位多，但每档需单独标定，且三路拼接需要处理偏移。

---

## 相对原版的改动

### `main` 分支

| 文件 | 差异行数 | 改动内容 |
| --- | ---: | --- |
| `Hardware/ADC_DMA.c` | 437 | 采样链路重写：六档切换 → 固定 2 MSPS；三重交替 → 单 ADC；`uint32` 缓冲 → `uint16`；DMA 缓冲移出 CCM RAM |
| `myCode/App_StateMachine.c` | 新增 1488 行 | 主循环逻辑收进状态机，拆成 Q1/Q2/Q3 × 波形/测量/频谱/分量 共 12 个模式 |
| `myCode/Signal_Analysis.c` | 新增 548 行 | 4096 点 FFT、前 3 个分量的频率/峰值/有效值/相位、480 点重建波形 |
| `Core/Src/main.c` | 100 | `while(1)` 中逐段判断 → `App_Init()` / `App_Run()` |
| `Core/Src/usart.c` | 79 | 新增 USART3（PC10/PC11），串口屏改接 USART3 |
| `myCode/FFT.c` | 49 | 算法迁至 `Signal_Analysis.c`，本文件退化为封装 |
| `Hardware/HMI.c` | 41 | `0x01` 改为固定 2 MSa/s；新增 `0x20` 模式切换、`0x21` 周期数切换 |

### `triple-adc` 分支

| 文件 | 差异行数 | 改动内容 |
| --- | ---: | --- |
| `myCode/FFT.c` / `FFT.h` | 299 / 17 | FFT 流程调整 |
| `Core/Src/main.c` | 120 | 主循环调整 |
| `Hardware/ADC_DMA.c` | 29 | 7.2 MHz 档改用连续转换 + 软件触发重新初始化 ADC1/2/3，再配置三重交替 |
| `Core/Src/usart.c` | 16 | 串口屏引脚 PA9/PA10 → PB6/PB7 |
| `myCode/Config.c` / `Config.h` | 9 / 12 | 配置项调整 |
| `Hardware/HMI.c` | 7 | 指令微调 |
| `Core/Src/adc.c` | 6 | ADC 初始化细节 |

---

## 硬件方案

| 项目 | 说明 |
| --- | --- |
| 主控 | STM32F407VET6，LQFP100，HCLK 144 MHz |
| 信号采集 | 12 bit，输入经 `ADC123_IN1`（PA1）进入转换器 |
| 信号输出 | DAC1 + TIM2 TRGO + DMA，最高 1440 点/周期 |
| 频率测量 | TIM5 输入捕获，计数时钟 2 MHz，同时给出占空比 |
| 显示 | 串口 HMI 屏（USART，921600） |
| 开发环境 | STM32CubeMX + Keil MDK 5（MDK-ARM V5.32） |

---

## 目录结构

```
Core/          CubeMX 生成的外设初始化代码与 main.c
Hardware/      外设驱动层：ADC/DAC 的 DMA 采样与输出、HMI 串口屏、串口协议解析
myCode/        业务逻辑层（main 分支额外包含状态机与信号分析）
MDK-ARM/       Keil 工程（OSC.uvprojx）与启动文件
OSC.ioc        STM32CubeMX 工程文件
Drivers/       STM32 HAL 与 CMSIS（ST 官方代码）
Middlewares/   CMSIS-DSP 预编译库（ST 官方代码）
指令.docx      HMI 串口屏指令表
tools/matlab/  MATLAB 调试脚本（FFT 仿真、串口波形分析）
```

---

## 编译与烧录

1. 安装 Keil MDK 5，并装好 `Keil::STM32F4xx_DFP` 器件支持包
2. 用 Keil 打开 `MDK-ARM/OSC.uvprojx`
3. `F7` 编译，`F8` 下载（ST-Link / DAP-Link 均可）
4. 需要修改引脚或外设时，用 STM32CubeMX 打开 `OSC.ioc` 重新生成

切换分支：

```bash
git clone https://github.com/zhu57320041-alt/stm32-signal-analyzer.git
git checkout triple-adc     # 切换到三重交替版本
```

---

## 调试工具

`tools/matlab/` 下是开发阶段用来验证采样与 FFT 的 MATLAB 脚本（需要 R2019b 及以上，因为用到了 `serialport()`）：

| 文件 | 用途 |
| --- | --- |
| `osc_fft_example.m` | 不接硬件的仿真：生成 10 kHz / 250 mVpp 信号，验证 FFT 结果与频率轴换算 |
| `serial_fft_scope.m` | 串口读取单帧，收完后绘制波形与频谱 |
| `serial_fft_scope_dynamic.m` | 串口实时版本，循环刷新波形与频谱 |

使用说明：

- 脚本中的串口写死为 `COM16`，按实际端口修改
- 单帧版波特率 115200，动态版 921600
- 数据帧格式：`AA BB + 点数(2B) + 采样率(2B) + N × uint16(小端)`，与当前固件的 HMI 协议不同，用于独立验证采样数据

---

## 主要指标

| 项目 | `main` | `triple-adc` |
| --- | --- | --- |
| 采样率 | 2 MSPS | 100 Hz – 7.2 MHz |
| FFT 点数 | 4096 | 1024 |
| 可测频率上限 | 1 MHz | 3.6 MHz（7.2 MSPS 档） |
| 输出波形 | 正弦 / 方波 / 三角 / 锯齿 / 反锯齿 | 同上 |
| 输出频率范围 | 1 Hz – 500 kHz | 同上 |

---

## 致谢与许可

- 原始工程：[Tangcy666/stm32-osc](https://github.com/Tangcy666/stm32-osc)，作者 Tang
- `Drivers/`、`Middlewares/` 版权归 STMicroelectronics 所有，遵循其原始许可
- 原项目未声明开源许可证，本仓库对其衍生部分不主张任何权利；如原作者要求，将立即删除相关代码

