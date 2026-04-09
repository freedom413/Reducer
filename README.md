# Reducer Flexspline State Detection System

## 1. 项目简介

Reducer Flexspline 状态检测系统是一个用于监测柔节轮（Flexspline）状态的实时多通道数据采集与监控系统。

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        PC Host (Python GUI)                       │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  ReducerMonitor  │  │   CAN Receiver   │  │  python-can  │ │
│  │   (PyQt6 GUI)     │  │   (Background)   │  │ socketcan/gs │ │
│  └──────────────────┘  └──────────────────┘  └──────────────┘ │
└───────────────────────────────┬─────────────────────────────────┘
                                │ USB/CAN
                                │ candleLight / gs_usb / socketcan
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
| CAN ID (TX) | 0x101 / 0x102           |
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
│       ├── can_protocol.py         # python-can传输层 + 协议辅助函数
│       ├── slcan_protocol.py       # 兼容导出层
│       └── test_gui.py             # GUI单元测试脚本
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

推荐使用刷写了 candleLight 固件的 CANable/兼容 gs_usb 适配器。

### 3.2 ADS1256接口

| STM32引脚 | 功能          | ADS1256     |
| ------- | ----------- | ----------- |
| SPI相关   | SPI通信       | SI, SO, CLK |
| GPIO    | DRDY (数据就绪) | DRDY        |
| GPIO    | RESET       | RESET       |
| GPIO    | CS          | CS          |

---

## 4. 通信协议

### 4.1 CAN遥测帧 (设备→PC)

**CAN ID: 0x101**

每周期6帧 (每通道1帧)，减少CAN总线负载。

| 字节  | 内容         | 类型         | 说明                    |
| --- | ---------- | ---------- | --------------------- |
| 0   | frame_type | uint8      | 0x51 (Telemetry)      |
| 1   | channel    | uint8      | 通道号 (0-5)             |
| 2-3 | voltage    | int16 (BE) | 电压值 (0.1 mV)          |
| 4-5 | strain     | int16 (BE) | 应变值 (µε)              |
| 6   | stress     | int8       | 应力预览值 (0.1 MPa, signed, 超量程钳位) |
| 7   | crc8       | uint8      | XOR校验和                |

上位机实际显示的应力/位移由 `strain` 按材料参数推导，避免 1 字节预览字段在大载荷下回绕。

### 4.2 CAN命令帧 (PC→设备)

**CAN ID: 0x100**

| 字节  | 内容       | 类型          | 说明   |
| --- | -------- | ----------- | ---- |
| 0   | frame_type | uint8     | 0xA0 (Command) |
| 1   | sequence   | uint8     | 主机命令序号 |
| 2   | cmd_type   | uint8     | 命令类型 |
| 3   | param      | uint8     | 参数 |
| 4-5 | value      | uint16 (LE) | 命令值 |
| 6   | reserved   | uint8     | 预留 |
| 7   | crc8       | uint8     | XOR校验和 |

### 4.3 CAN状态应答帧 (设备→PC)

**CAN ID: 0x102**

| 字节  | 内容       | 类型          | 说明   |
| --- | -------- | ----------- | ---- |
| 0   | frame_type | uint8     | 0xA1 (Status) |
| 1   | sequence   | uint8     | 回显命令序号 |
| 2   | cmd_type   | uint8     | 回显命令类型 |
| 3   | status     | uint8     | 0x00成功，其余为错误码 |
| 4-5 | value      | uint16 (LE) | 实际应用值 |
| 6   | detail     | uint8     | 额外错误细节 |
| 7   | crc8       | uint8     | XOR校验和 |

**命令类型定义:**

| cmd_type | 命令              | 说明                     |
| -------- | --------------- | ---------------------- |
| 0x01     | SET_SAMPLE_RATE | 设置ADS1256数据率，支持 `5/10/15/25/30/50/60/100/500/1000/2000/3750/7500/15000/30000 SPS` |
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
- **推荐**: 直接复用 VS Code STM32 插件下载到 `~/.local/share/stm32cube/bundles` 的 `cmake` 和 `gnu-tools-for-stm32`

### 5.2 编译固件

```bash
# 推荐: 自动探测 STM32Cube bundles 并完成 configure + build
./scripts/build_firmware.sh

# 指定 bundle 根目录或输出目录
STM32_BUNDLES_DIR="$HOME/.local/share/stm32cube/bundles" \
BUILD_DIR="$PWD/build/stm32cube-debug" \
./scripts/build_firmware.sh
```

脚本会自动：

- 选择 `~/.local/share/stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc`
- 选择 `~/.local/share/stm32cube/bundles/cmake/*/bin/cmake`
- 使用 [gcc-arm-none-eabi.cmake](/home/ff/test/Reducer/cmake/gcc-arm-none-eabi.cmake) 作为 toolchain file
- 输出到默认目录 `build/stm32cube-debug`

如果你想手动覆盖路径，可以设置：

```bash
ARM_GCC_BIN_DIR=/path/to/gnu-tools/bin \
STM32_CMAKE_BIN_DIR=/path/to/cmake/bin \
BUILD_TYPE=Release \
./scripts/build_firmware.sh
```

或者使用 STM32CubeIDE / VS Code STM32 插件直接打开 `Reducer.ioc`。

### 5.3 已验证环境

- `gnu-tools-for-stm32 14.3.1+st.2`
- `cmake 4.2.3+st.1`
- 本项目已在该组合下成功生成 `Reducer.elf`

### 5.4 烧录

使用ST-Link或J-Link连接STM32G431CB，烧录 `build/stm32cube-debug/Reducer.elf`，或通过IDE直接下载。

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
pip install -r requirements.txt
```

### 6.2 运行GUI

```bash
python reducer_monitor.py
```

如果你在 Linux 上用 candleLight/socketcan，先按 CANable 教程把接口拉起，例如：

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 6.3 GUI功能说明

| 功能                     | 说明                 |
| ---------------------- | ------------------ |
| **Interface**             | 选择 `socketcan` 或 `gs_usb` |
| **Channel**               | 选择 `can0` 或 `gs_usb` 设备索引 |
| **波特率设置**              | 选择CAN波特率 (默认500K)  |
| **Connect/Disconnect** | 连接/断开CAN适配器        |
| **Start Logging**      | 开启/停止CSV数据记录       |
| **Waveforms**          | 6通道实时电压波形显示        |
| **Data Panel**         | 电压/应变/应力/位移数值面板    |
| **Sample Rate**        | 设置 ADS1256 数据率，默认 `100 SPS` |
| **Zero Sensor**        | 零点校准 - 保存到Flash并重置 |
| **Calibrate**          | 发送ADS1256自校准命令     |
| **Save Zero**          | 保存零点偏移到Flash       |
| **Load Zero**          | 从Flash加载零点偏移       |
| **Clear Zero**         | 清除Flash中的零点偏移      |
| **Filter Size**        | 设置滤波器窗口大小 (2-64)   |

### 6.4 CAN协议说明

协议统一使用 8 字节经典 CAN 帧：

- `0x101` 遥测帧：每周期 6 帧
- `0x100` 命令帧：带 `sequence` 和 `crc8`
- `0x102` 状态帧：设备对命令做 ACK/NACK

### 6.5 运行测试

```bash
cd host_pc/python
python test_gui.py
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
2. 选择正确的接口类型、通道和波特率(500K)
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
| 更新太慢 | ADC数据率过低 | 将 GUI 中 `Sample Rate` 调高到 `100 SPS` 或以上 |
| 无数据   | 终端电阻缺失  | 在总线两端加120Ω电阻    |
| CRC错误 | 数据干扰    | 检查CAN线缆质量       |
| 数值异常  | 传感器未校准  | 检查传感器接线         |

---

## 8. CRC校验算法

遥测/命令/状态帧都使用 CRC-8 XOR 校验：

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
| 1.6 | 2026-04-09 | 修复Flash校准数据写入越界；ADS1256改为EXTI置位+主循环批处理；默认采样率提升到100 SPS；GUI应力改为由应变推导 |
| 1.5 | 2026-03-20 | 零点校准数据保存到Flash、上电自动加载、添加SAVE/LOAD/CLEAR命令                  |
| 1.4 | 2026-03-20 | 实现预留命令(SET_FILTER_SIZE, START_CALIB)、更新GUI滤波器控制、去除CRC-16描述 |
| 1.3 | 2026-03-20 | 统一合并帧格式(去除Legacy帧)、嵌入式+PC端联调测试通过                           |
| 1.2 | 2026-03-20 | PC GUI支持合并帧协议(自动识别新旧格式)、添加测试脚本                             |
| 1.1 | 2026-03-20 | 优化: CAN合并帧(18→6帧)、O(1)通道查表、异常值检测优化(避免sqrtf)                |
| 1.0 | 2026-03-19 | 初始版本，支持6通道CAN通信和PC GUI                                     |

---

## 11. 许可证

本项目仅供学习和研究使用。
