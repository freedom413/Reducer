# Reducer

STM32G431 + 双 ADS1256 的 8 通道柔轮状态采集系统。固件负责采集、滤波、零点存储和物理量换算；PC 上位机负责 CAN FD 连接、实时图表、健康监控、命令交互和 CSV 记录。

本分支统一使用 CAN FD+BRS：

- 仲裁段：`500 kbit/s`
- 数据段：`2 Mbit/s`
- 推荐适配器：CANable 2.0 官方 SLCAN FD 固件
- CANable 2.0 使用 USB CDC 虚拟串口，主机设置的串口波特率不会限制 USB 吞吐。
- 配置说明：[docs/canfd_2m_setup.md](docs/canfd_2m_setup.md)

## 当前能力

- 双 ADS1256，每片 4 个差分输入，共 8 个逻辑通道。
- 支持 ADS1256 全部采样档位：`2.5/5/10/15/25/30/50/60/100/500/1000/2000/3750/7500/15000/30000 SPS`。
- SPI1 为 `2.65625 Mbit/s`，低于 ADS1256 在 `7.68 MHz` 晶振下的 SCLK 上限。
- MCU 对每个采样值执行滤波和物理量换算，仅对外发遥测自动抽取，避免高速档位压满链路。
- MCU 每秒发送健康帧，上报采样率、抽取倍数、CAN TX 丢弃、ADC 缓冲区溢出、自动恢复次数和运行状态。
- 上位机使用官方 `python-can` SLCAN FD 扩展，支持 CANable 2.0 的 `S6`、`Y2` 和 FD+BRS `b` 帧。
- GUI 支持最多 8 张动态图表、指标叠加、通道订阅、设备健康、ACK、CSV 记录和离线 CSV 回放。

## 目录结构

```text
Application/
  user.c                         主循环、命令处理、采样处理和健康帧
  algorithm/
    can_data.*                   MCU CAN 协议帧与 CRC
    filter.*                     滑动平均和零点偏移
    flash_storage.*              Flash 校准数据
    flexspline_math.*            电压、应变和应力换算
BSP/
  ads1256/                       ADS1256 驱动与双 ADC 轮询
  can.*                          FDCAN 收发封装
Core/                            STM32CubeMX 生成代码
docs/
  canfd_2m_setup.md              CAN FD 部署和联调说明
  user_application_flow.md       应用层流程英文说明
  user_application_flow_zh.md    应用层流程中文说明
host_pc/python/
  can_protocol.py                python-can CAN FD 传输与协议解析
  reducer_monitor.py             PyQt6 上位机
  test_gui.py                    GUI 与协议测试
```

## CAN 协议

所有业务帧均为标准 11-bit ID 的 CAN FD+BRS 帧，并使用 XOR 校验。

| 方向 | CAN ID | 类型 | 说明 |
|---|---:|---:|---|
| PC -> STM32 | `0x100` | `0xA0` | 8 字节命令帧 |
| STM32 -> PC | `0x101` | `0x51` | 8 字节单通道遥测 |
| STM32 -> PC | `0x102` | `0xA1` | 8 字节命令 ACK |
| STM32 -> PC | `0x103` | `0x52` | 16 字节健康帧 |

详细字段和运行逻辑见 [docs/user_application_flow_zh.md](docs/user_application_flow_zh.md)。

## 构建固件

```powershell
cmake -S . -B build\Debug -DCMAKE_TOOLCHAIN_FILE="cmake/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build build\Debug
```

生成文件：

```text
build/Debug/Reducer.elf
```

## 启动上位机

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r host_pc\python\requirements.txt
.venv\Scripts\python host_pc\python\test_gui.py
.venv\Scripts\python host_pc\python\reducer_monitor.py
```

GUI 中选择 `CANable 2.0 SLCAN FD` 和对应 `COMx`。CAN 波特率固定为 `500K / 2M FD+BRS`。

## 联调顺序

1. 烧录 `build/Debug/Reducer.elf`，检查 CANH、CANL、共地和两端 `120 ohm` 终端电阻。
2. 启动 GUI，选择 CANable 2.0 SLCAN FD 和正确串口后连接。
3. 确认 `0x101` 遥测持续到达，`0x103` 健康信息每秒刷新。
4. 点击校准并确认收到 `0x102` ACK。
5. 从 `100 SPS` 逐步提高到 `30000 SPS`，观察抽取倍数、TX 丢弃和 ADC 溢出。
6. 执行归零并重启 MCU，确认 Flash 中的零点能够恢复。
