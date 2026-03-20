# Reducer Flexspline State Detection System

## 1. 项目简介

Reducer Flexspline 状态检测系统是一个用于监测柔节轮（Flexspline）状态的实时多通道数据采集与监控系统。

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        PC Host (Python GUI)                       │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  ReducerMonitor  │  │   CAN Receiver   │  │  SLCAN Prot  │ │
│  │   (PyQt6 GUI)     │  │   (Background)   │  │  (Protocol)  │ │
│  └──────────────────┘  └──────────────────┘  └──────────────┘ │
└───────────────────────────────┬─────────────────────────────────┘
                                │ USB/CAN
                                │ SLCAN Protocol (500Kbps)
┌───────────────────────────────┴─────────────────────────────────┐
│                    STM32G431CB (Embedded)                        │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  FDCAN Controller │  │   ADS1256 ADC    │  │  Flexspline  │ │
│  │  (CAN RX/TX)      │  │  (6-ch, 24-bit)  │  │  Calculator  │ │
│  └──────────────────┘  └──────────────────┘  └──────────────┘ │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  Moving Average  │  │  Outlier Filter  │  │   Statistics │ │
│  │     Filter       │  │  (3σ rejection)  │  │   (Welford)  │ │
│  └──────────────────┘  └──────────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 主要特性

- **6通道ADC数据采集** - 使用ADS1256芯片，24位分辨率
- **实时数据处理** - 滑动平均滤波 + 异常值剔除（3σ准则）
- **物理量转换** - 电压 → 应变 → 应力 → 位移
- **CAN总线通信** - 标准CAN 2.0A (11-bit ID)，500Kbps
- **CRC校验** - CRC-8 XOR数据完整性校验
- **PyQt6图形界面** - 实时波形显示、数据面板、CSV日志

### 1.3 技术规格

| 参数          | 值                     |
| ----------- | --------------------- |
| MCU         | STM32G431CB           |
| ADC         | ADS1256 (24-bit, 6通道) |
| CAN速率       | 500 Kbps              |
| CAN ID (TX) | 0x101 (设备→PC)         |
| CAN ID (RX) | 0x100 (PC→设备)         |
| 采样率         | ~60 Hz (每通道)          |
| 滤波器         | 16点滑动平均               |
| 异常值阈值       | 3σ (标准差)              |

---

## 2. 目录结构

```
Reducer/
├── Application/                    # 应用层代码
│   ├── algorithm/
│   │   ├── can_data.h              # CAN数据帧定义
│   │   ├── filter.h/c              # 滑动平均滤波器
│   │   └── flexspline_math.h/c     # 柔节轮物理计算
│   └── user.c                       # 主循环和命令处理
├── BSP/                            # 板级支持包
│   ├── ads1256/                     # ADS1256驱动
│   ├── can.h/c                     # CAN总线驱动
│   └── fdcan.h/c                   # FDCAN硬件抽象
├── Core/                           # STM32CubeMX生成代码
│   └── Src/fdcan.c                 # FDCAN配置（含滤波器）
├── host_pc/
│   └── python/
│       ├── reducer_monitor.py      # PC GUI主程序
│       ├── slcan_protocol.py       # SLCAN协议实现
│       └── test_gui.py            # GUI单元测试脚本
├── Drivers/                        # STM32 HAL驱动
└── README.md                       # 本文档
```

---

## 3. 硬件连接

### 3.1 CAN接口

| STM32引脚          | 功能    | CAN适配器 |
| ---------------- | ----- | ------ |
| PA11 (FDCAN1_RX) | CAN接收 | CAN_L  |
| PA12 (FDCAN1_TX) | CAN发送 | CAN_H  |

推荐使用LCAN FD100或兼容SLCAN协议的CAN适配器。

### 3.2 ADS1256接口

| STM32引脚 | 功能          | ADS1256     |
| ------- | ----------- | ----------- |
| SPI相关   | SPI通信       | SI, SO, CLK |
| GPIO    | DRDY (数据就绪) | DRDY        |
| GPIO    | RESET       | RESET       |
| GPIO    | CS          | CS          |

---

## 4. 通信协议

### 4.1 CAN数据帧 (设备→PC) - 合并帧格式

**CAN ID: 0x101**

每周期6帧 (每通道1帧)，减少CAN总线负载。

| 字节  | 内容         | 类型         | 说明                    |
| --- | ---------- | ---------- | --------------------- |
| 0   | frame_type | uint8      | 0x05 (合并帧)            |
| 1   | channel    | uint8      | 通道号 (0-5)             |
| 2-3 | voltage    | int16 (BE) | 电压值 (0.1 mV)          |
| 4-5 | strain     | int16 (BE) | 应变值 (µε)              |
| 6   | stress     | int8       | 应力值 (0.1 MPa, signed) |
| 7   | crc8       | uint8      | XOR校验和                |

### 4.2 CAN命令帧 (PC→设备)

**CAN ID: 0x100**

| 字节  | 内容       | 类型          | 说明   |
| --- | -------- | ----------- | ---- |
| 0   | cmd_type | uint8       | 命令类型 |
| 1   | param    | uint8       | 参数   |
| 2-5 | value    | uint32 (LE) | 命令值  |

**命令类型定义:**

| cmd_type | 命令              | 说明                     |
| -------- | --------------- | ---------------------- |
| 0x01     | SET_SAMPLE_RATE | 设置采样率 (硬件相关,预留)        |
| 0x02     | SET_FILTER_SIZE | 设置滤波器窗口大小 (2-64点)      |
| 0x03     | ZERO_DATUM      | 零点校准 - 保存到Flash并重置滤波器  |
| 0x04     | START_CALIB     | ADS1256自校准             |
| 0x05     | SAVE_ZERO       | 保存零点偏移到Flash (仅保存,不重置) |
| 0x06     | LOAD_ZERO       | 从Flash加载零点偏移           |
| 0x07     | CLEAR_ZERO      | 清除Flash中的零点偏移          |

### 4.4 数据处理流程

```
ADC Raw (24-bit)
      │
      ▼
┌─────────────┐
│  16-tap     │  滑动平均滤波
│  Moving Avg │
└─────────────┘
      │
      ▼
┌─────────────┐
│  Outlier    │  3σ异常值剔除 (优化: 避免sqrtf)
│  Rejection  │
└─────────────┘
      │
      ├─► 统计计算 (Welford算法)
      │
      ▼
┌─────────────────────┐
│  Flexspline Calc    │  物理量转换
│  V→ε→σ→δ            │
└─────────────────────┘
      │
      ▼
┌─────────────┐
│ CAN TX Frame│  发送至PC (优化: 合并帧 18→6帧)
│ (0x101)     │
└─────────────┘
```

---

## 5. 编译与烧录

### 5.1 环境要求

- **构建工具**: CMake ≥ 3.25, GCC ARM Embedded ≥ 10
- **IDE**: STM32CubeMX (可选，用于代码生成)
- **Python**: ≥ 3.8 (用于PC GUI)

### 5.2 编译固件

```bash
# 使用CMake构建
mkdir build && cd build
cmake ..
make -j4

# 或使用STM32CubeIDE打开Reducer.ioc
```

### 5.3 烧录

使用ST-Link或J-Link连接STM32G431CB，烧录`build/Reducer.elf`或通过IDE直接下载。

---

## 6. PC端软件安装

### 6.1 环境准备

```bash
cd host_pc/python

# 创建虚拟环境 (推荐)
python -m venv .venv

# 激活虚拟环境
# Windows:
.venv\Scripts\activate
# Linux/Mac:
source .venv/bin/activate

# 安装依赖
pip install pyserial pyqtgraph pyqt6 numpy
```

### 6.2 运行GUI

```bash
python reducer_monitor.py
```

### 6.3 GUI功能说明

| 功能                     | 说明                 |
| ---------------------- | ------------------ |
| **COM端口选择**            | 选择连接的CAN适配器串口      |
| **波特率设置**              | 选择CAN波特率 (默认500K)  |
| **Connect/Disconnect** | 连接/断开CAN适配器        |
| **Start Logging**      | 开启/停止CSV数据记录       |
| **Waveforms**          | 6通道实时电压波形显示        |
| **Data Panel**         | 电压/应变/应力/位移数值面板    |
| **Zero Sensor**        | 零点校准 - 保存到Flash并重置 |
| **Calibrate**          | 发送ADS1256自校准命令     |
| **Save Zero**          | 保存零点偏移到Flash       |
| **Load Zero**          | 从Flash加载零点偏移       |
| **Clear Zero**         | 清除Flash中的零点偏移      |
| **Filter Size**        | 设置滤波器窗口大小 (2-64)   |

### 6.4 CAN协议说明

统一使用合并帧格式 (frame_type=0x05)：

- 每周期6帧 (每通道1帧)
- 包含电压、应变、应力数据
- CRC-8校验

### 6.5 运行测试

```bash
cd host_pc/python
.venv\Scripts\python.exe test_gui.py
```

测试覆盖：CRC校验、帧解析、CSV日志、波形缓冲等。

---

## 7. 使用流程

### 7.1 硬件连接

1. 将CAN适配器通过USB连接至PC
2. 将CAN_H/CAN_L连接至STM32对应引脚
3. 给ADS1256和传感器上电
4. 确保120Ω终端电阻已连接（CAN总线两端）

### 7.2 软件操作

1. 运行 `python reducer_monitor.py`
2. 选择正确的COM端口和波特率(500K)
3. 点击 **Connect** 连接CAN适配器
4. 确认STM32已上电并运行固件
5. 观察波形显示区是否有数据更新
6. 如需清零，点击 **Zero Sensor** 按钮
7. 记录数据时点击 **Start Logging**

### 7.3 故障排查

| 现象    | 可能原因    | 解决方法            |
| ----- | ------- | --------------- |
| 无数据   | CAN连接错误 | 检查CAN_H/CAN_L接线 |
| 无数据   | 波特率不匹配  | 确认适配器和设备都是500K  |
| 无数据   | 终端电阻缺失  | 在总线两端加120Ω电阻    |
| CRC错误 | 数据干扰    | 检查CAN线缆质量       |
| 数值异常  | 传感器未校准  | 检查传感器接线         |

---

## 8. CRC校验算法

合并帧 (frame_type=0x05) 使用CRC-8 XOR校验：

```c
// C语言实现
uint8_t can_calc_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}
```

```python
# Python实现
def crc8_xor(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
    return crc
```

---

## 9. 配置文件

物理参数在 `Application/algorithm/flexspline_math.c` 中定义：

```c
void flexspline_params_set_default(flexspline_params_t *p)
{
    p->ref_voltage = 3.0f;           // ADC参考电压 (V)
    p->pga = 64;                     // PGA增益
    p->excitation_v = 3.3f;          // 惠斯通电桥激励电压 (V)
    p->gauge_k = 2.0f;              // 应变片灵敏度系数
    p->elastic_modulus = 210000.0f; // 弹性模量 (MPa, 钢≈210000)
    p->flexspline_area = 100.0f;    // 横截面积 (mm²)
    p->moment_of_inertia = 1000.0f;  // 截面二次矩 (mm⁴)
}
```

根据实际传感器规格修改这些参数。

---

## 10. 版本历史

| 版本  | 日期         | 说明                                                         |
| --- | ---------- | ---------------------------------------------------------- |
| 1.5 | 2026-03-20 | 零点校准数据保存到Flash、上电自动加载、添加SAVE/LOAD/CLEAR命令                  |
| 1.4 | 2026-03-20 | 实现预留命令(SET_FILTER_SIZE, START_CALIB)、更新GUI滤波器控制、去除CRC-16描述 |
| 1.3 | 2026-03-20 | 统一合并帧格式(去除Legacy帧)、嵌入式+PC端联调测试通过                           |
| 1.2 | 2026-03-20 | PC GUI支持合并帧协议(自动识别新旧格式)、添加测试脚本                             |
| 1.1 | 2026-03-20 | 优化: CAN合并帧(18→6帧)、O(1)通道查表、异常值检测优化(避免sqrtf)                |
| 1.0 | 2026-03-19 | 初始版本，支持6通道CAN通信和PC GUI                                     |

---

## 11. 许可证

本项目仅供学习和研究使用。
