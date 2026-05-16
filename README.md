# Reducer

STM32G431 + 双 ADS1256 的 6 通道柔轮状态采集系统。STM32 端采集并处理应变片数据，通过经典 CAN 发送；PC 上位机使用 `python-can` 的 `slcan` 后端，通过 USB-CAN 串口适配器接收数据、显示波形、发送控制命令和记录 CSV。

## 当前状态

- 固件构建通过，校准数据页已从 Flash 链接空间中预留。
- PC GUI 单元测试通过。
- 通讯协议已收敛为固定 8 字节经典 CAN 帧。
- 当前采样率开放范围：`5/10/15/25/30/50/60/100/500/1000 SPS`。
- 下一步需要接真实硬件验证命令 ACK、遥测帧、采样率和丢帧情况。

## 目录

```text
Application/
  user.c                         主循环、命令处理、采集处理
  algorithm/
    can_data.*                   CAN 协议帧定义与 CRC
    filter.*                     移动平均、零点偏置
    flash_storage.*              Flash 校准数据保存
    flexspline_math.*            原始值到电压/应变/应力换算
BSP/
  ads1256/                       ADS1256 驱动与双 ADC 轮询采集
  can.*                          FDCAN 收发封装
  delay.*                        延时
Core/                            STM32CubeMX 生成代码
host_pc/python/
  can_protocol.py                python-can/SLCAN 传输与协议解析
  reducer_monitor.py             PyQt6 上位机
  test_gui.py                    GUI 与协议测试
```

## CAN 协议

所有业务帧均为经典 CAN 2.0A、标准 11-bit ID、8 字节数据、CRC-8 XOR。

| 方向 | CAN ID | 类型 | 说明 |
|---|---:|---:|---|
| PC -> STM32 | `0x100` | `0xA0` | 命令帧 |
| STM32 -> PC | `0x101` | `0x51` | 单通道遥测帧 |
| STM32 -> PC | `0x102` | `0xA1` | 命令 ACK/状态帧 |

遥测帧 `0x101`：

| Byte | 字段 | 格式 |
|---:|---|---|
| 0 | frame type | `0x51` |
| 1 | channel | `0..5` |
| 2-3 | voltage | `int16 BE`, 0.1 mV |
| 4-5 | strain | `int16 BE`, microstrain |
| 6 | stress preview | `int8`, 0.1 MPa |
| 7 | crc | XOR bytes 0..6 |

命令帧 `0x100`：

| Byte | 字段 | 格式 |
|---:|---|---|
| 0 | frame type | `0xA0` |
| 1 | sequence | `uint8` |
| 2 | command | `uint8` |
| 3 | param | `uint8` |
| 4-5 | value | `uint16 LE` |
| 6 | reserved | `0` |
| 7 | crc | XOR bytes 0..6 |

支持命令：

| 命令 | 值 | 说明 |
|---|---:|---|
| `SET_SAMPLE_RATE` | `0x01` | 设置 ADS1256 SPS |
| `SET_FILTER_SIZE` | `0x02` | 设置移动平均窗口，范围 `2..64` |
| `ZERO_DATUM` | `0x03` | 当前值归零并保存到 Flash |
| `START_CALIB` | `0x04` | ADS1256 自校准 |
| `SAVE_ZERO` | `0x05` | 保存零点 |
| `LOAD_ZERO` | `0x06` | 读取零点 |
| `CLEAR_ZERO` | `0x07` | 清除零点 |

## 构建固件

推荐直接使用本机 STM32CubeCLT/CMake：

```powershell
cmake -S . -B build\codex-debug -DCMAKE_TOOLCHAIN_FILE="cmake/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build build\codex-debug
```

输出文件：

```text
build/codex-debug/Reducer.elf
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

GUI 连接参数：

| 参数 | 推荐值 |
|---|---|
| Interface | `slcan` |
| Channel | Windows: `COMx`; Linux: `/dev/ttyUSBx` |
| Adapter Baud | 适配器串口速率，默认 `115200` |
| CAN Baudrate | `500K` |

## 硬件联调顺序

1. 烧录固件，确认 CANH/CANL、终端电阻、共地正常。
2. 打开上位机，选择 `slcan` 和正确串口，CAN 波特率选 `500K`。
3. 点击 `Calibrate` 或修改 `Filter Size`，确认收到 `0x102` ACK。
4. 观察 `0x101` 六通道遥测是否连续更新。
5. 从 `100 SPS` 开始，逐步测试 `500/1000 SPS` 下的丢帧、延迟和噪声。

## 清理原则

当前仓库只保留主链路需要的代码：STM32 HAL/CMSIS、ADS1256、CAN、算法、`lwrb`、PC SLCAN GUI。未使用的旧算法、JSON/printf/FlashDB 子模块和临时缓存已移除。
