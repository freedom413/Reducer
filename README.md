# Reducer

STM32G431 + 双 ADS1256 的 6 通道柔轮状态采集系统。

MCU 端负责采集应变片数据、滤波、零点扣除、异常值处理、柔轮物理量换算，并通过经典 CAN 发送遥测帧、接收控制命令。PC 上位机使用 `python-can` 的 `slcan` 后端连接 USB-CAN 串口适配器，完成实时显示、命令下发、ACK 处理和 CSV 记录。

## 当前状态

- MCU 固件可构建通过。
- ADS1256 双 ADC、6 逻辑通道采集流程已接入应用层。
- CAN 协议统一为固定 8 字节经典 CAN 帧。
- PC 上位机协议解析、命令发送、ACK 超时处理、实时曲线、数据表和 CSV 记录已实现。
- PC GUI 单元测试通过。
- 当前开放采样率：`5/10/15/25/30/50/60/100/500/1000 SPS`。

## 目录结构

```text
Application/
  user.c                         主循环、CAN 命令处理、采集处理
  algorithm/
    can_data.*                   MCU 端 CAN 协议帧定义与 CRC
    filter.*                     滑动平均、零点偏移
    flash_storage.*              Flash 校准数据保存
    flexspline_math.*            原始值到电压/应变/应力换算
BSP/
  ads1256/                       ADS1256 驱动与双 ADC 轮询采集
  can.*                          FDCAN 收发封装
  delay.*                        微秒延时
  flash_storage_port.c           STM32 Flash 存储端口层
Core/                            STM32CubeMX 生成代码
docs/
  user_application_flow_zh.md    应用层运行流程中文说明
  ads1256_c_driver_comparison_zh.md
host_pc/python/
  can_protocol.py                python-can/SLCAN 传输与协议解析
  reducer_monitor.py             PyQt6 上位机 GUI
  test_gui.py                    GUI 与协议测试
```

## CAN 协议

所有业务帧均为经典 CAN 2.0A、标准 11-bit ID、8 字节数据、CRC-8 XOR。

| 方向 | CAN ID | 类型 | 说明 |
|---|---:|---:|---|
| PC -> STM32 | `0x100` | `0xA0` | 命令帧 |
| STM32 -> PC | `0x101` | `0x51` | 单通道遥测帧 |
| STM32 -> PC | `0x102` | `0xA1` | 命令 ACK/状态帧 |

### 遥测帧 `0x101`

| Byte | 字段 | 格式 |
|---:|---|---|
| 0 | frame type | `0x51` |
| 1 | channel | `0..5` |
| 2-3 | voltage | `int16 BE`，单位 0.01 mV |
| 4-5 | strain | `int16 BE`，单位 microstrain |
| 6 | stress preview | `int8`，单位 0.1 MPa |
| 7 | crc | XOR bytes 0..6 |

`stress preview` 字段用于紧凑预览，受 `int8` 范围限制。PC 上位机使用 strain
重新计算高精度 stress，用于表格显示和 CSV 记录。

### 命令帧 `0x100`

| Byte | 字段 | 格式 |
|---:|---|---|
| 0 | frame type | `0xA0` |
| 1 | sequence | `uint8` |
| 2 | command | `uint8` |
| 3 | param | `uint8` |
| 4-5 | value | `uint16 LE` |
| 6 | reserved | `0` |
| 7 | crc | XOR bytes 0..6 |

### 状态帧 `0x102`

| Byte | 字段 | 格式 |
|---:|---|---|
| 0 | frame type | `0xA1` |
| 1 | sequence | 与命令帧一致 |
| 2 | command | 与命令帧一致 |
| 3 | status | 状态码 |
| 4-5 | value | `uint16 LE` |
| 6 | detail | 细节码 |
| 7 | crc | XOR bytes 0..6 |

### 支持命令

| 命令 | 值 | 说明 |
|---|---:|---|
| `SET_SAMPLE_RATE` | `0x01` | 设置 ADS1256 SPS |
| `SET_FILTER_SIZE` | `0x02` | 设置滑动平均窗口，范围 `2..64` |
| `ZERO_DATUM` | `0x03` | 用当前值归零并保存到 Flash |
| `START_CALIB` | `0x04` | ADS1256 自校准 |
| `SAVE_ZERO` | `0x05` | 保存当前零点 |
| `LOAD_ZERO` | `0x06` | 读取 Flash 零点 |
| `CLEAR_ZERO` | `0x07` | 清除内存和 Flash 零点 |
| `SET_CHANNEL_MASK` | `0x08` | 设置 ADS1256 扫描通道掩码，未订阅通道不采集 |

### 状态码

| 状态 | 值 | 说明 |
|---|---:|---|
| `OK` | `0x00` | 命令执行成功 |
| `BAD_CRC` | `0xE1` | CRC 错误 |
| `BAD_TYPE` | `0xE2` | 帧类型错误 |
| `BAD_CMD` | `0xE3` | 不支持的命令 |
| `BAD_VALUE` | `0xE4` | 参数非法或 ADC 操作失败 |
| `STORAGE_ERROR` | `0xE5` | Flash 保存、读取或清除失败 |

## 构建 MCU 固件

推荐使用 STM32CubeCLT/CMake：

```powershell
cmake -S . -B build\Debug -DCMAKE_TOOLCHAIN_FILE="cmake/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build build\Debug
```

输出文件：

```text
build/Debug/Reducer.elf
```

## PC 上位机

创建虚拟环境并安装依赖：

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r host_pc\python\requirements.txt
```

运行测试：

```powershell
.venv\Scripts\python host_pc\python\test_gui.py
```

启动 GUI：

```powershell
.venv\Scripts\python host_pc\python\reducer_monitor.py
```

### GUI 功能

- 自动枚举 SLCAN 串口和 SocketCAN 接口。
- CAN 波特率固定为 `500K`，与 MCU 固件保持一致。
- 支持选择 SLCAN 串口波特率，默认 `115200`。
- SLCAN 带宽不足时会拒绝过高采样率，避免串口侧静默丢帧。
- 默认显示 4 张图表，可新增到 8 张；每张图可叠加选择电压、应变和应力曲线。图表绑定的通道会自动同步到 MCU，未展示通道不采集。
- 表格显示每通道电压、应变、应力和样本数。
- 统计每通道电压最小值、最大值和平均值。
- 支持 CSV 记录。
- 支持下发采样率、滤波窗口、零点、校准、保存/读取/清除零点命令。
- 支持命令 sequence、ACK 匹配和 ACK 超时提示。
- 丢弃 CRC 或格式错误的协议帧。

### GUI 连接参数

| 参数 | 推荐值 |
|---|---|
| Interface | `slcan` |
| Channel | Windows: `COMx`; Linux: `/dev/ttyUSBx` |
| Adapter Baud | 适配器串口波特率；`100 SPS` 可使用 `115200`，`500 SPS` 至少使用 `460800`，`1000 SPS` 至少使用 `921600` |
| CAN Baudrate | 固定 `500K` |

## 硬件联调顺序

1. 烧录固件，确认 CANH/CANL、终端电阻和共地正常。
2. 打开上位机，选择 `slcan` 和正确串口。CAN 波特率固定为 `500K`。
3. 连接后观察是否持续收到 `0x101` 遥测帧，6 通道是否轮流更新。
4. 点击 `Calibrate`，确认收到 `0x102` ACK。
5. 修改 `Filter Size` 或 `Sample Rate`，确认上位机状态栏显示 ACK，MCU 端采样行为正常。
6. 执行 `Zero Sensor`，确认零点保存成功；断电重启后检查零点是否从 Flash 恢复。
7. 从 `100 SPS` 开始，逐步测试 `500/1000 SPS` 下的丢帧、延迟和噪声；提高采样率前同步提高 Adapter Baud。

## 参考文档

- [应用层运行流程](docs/user_application_flow_zh.md)
