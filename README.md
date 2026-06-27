# Reducer

STM32G431 + 双 ADS1256 的 8 通道柔轮状态采集系统。固件负责采集、滤波、零点存储和物理量换算；PC 上位机负责 CAN FD 连接、实时图表、健康监控、命令交互和 CSV 记录。

本分支统一使用 CAN FD+BRS：

- 仲裁段：`500 kbit/s`
- 数据段：`2 Mbit/s`
- 速率摘要：`500K / 2M`
- 推荐适配器：CANable 2.0 官方 SLCAN FD 固件
- CANable 2.0 使用 USB CDC 虚拟串口，主机设置的串口波特率不会限制 USB 吞吐。
- 配置说明：[docs/canfd_2m_setup.md](docs/canfd_2m_setup.md)

## 当前能力

- 双 ADS1256，每片 4 个差分输入，共 8 个逻辑通道。
- 支持 ADS1256 全部采样档位：`2.5/5/10/15/25/30/50/60/100/500/1000/2000/3750/7500/15000/30000 SPS`。
- SPI1 为 `1.328125 Mbit/s`，低于 ADS1256 在 `7.68 MHz` 晶振下的 SCLK 上限。
- Raw 帧最多打包 14 条记录，Physical 帧最多打包 6 条记录；当前不做遥测抽取，全部采样记录进入发送队列。
- MCU 每秒发送健康帧，上报采样率、实际样本/帧速率、CAN TX 丢弃、ADC 缓冲区溢出、自动恢复次数和运行状态。
- Flash 配置使用 64 字节记录和 CRC32，保存零点、通道、PGA、采样率、滤波窗口和遥测模式。
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

业务帧均为标准 11-bit ID 的 CAN FD+BRS 帧，v3 帧依赖 CAN 帧 CRC；只有 classic 诊断帧和兼容旧协议的帧保留应用层 XOR 校验。

| 方向 | CAN ID | 类型 | 说明 |
|---|---:|---:|---|
| STM32 -> PC | `0x0F0` | `0xA1` | 12 字节命令 ACK/status |
| STM32 -> PC | `0x0F0` | `0x56` | 64 字节配置快照 |
| PC -> STM32 | `0x0F1` | `0xA0` | 12 字节命令帧 |
| STM32 -> PC | `0x0F2` | `0x52` | 24 字节健康帧 |
| STM32 -> PC | `0x0FF` | `0x57` | 8 字节 classic CAN 诊断心跳 |
| STM32 -> PC | `0x110` | `0x54/0x55` | 64 字节批量遥测 |

详细字段和运行逻辑见 [docs/user_application_flow_zh.md](docs/user_application_flow_zh.md)。

## 构建固件

```powershell
.\scripts\build_firmware.ps1 -Configuration Debug
.\scripts\build_firmware.ps1 -Configuration Release
```

脚本固定使用 STM32Cube 扩展安装在 `%LOCALAPPDATA%\stm32cube\bundles` 下的 CMake、Ninja 和 GCC，避免 PATH 中其他 Ninja 参与 ABI 探测。

生成文件：

```text
build/Debug/Reducer.elf
```

## 启动上位机

```powershell
host_pc\python\.venv\Scripts\python.exe -m pip install -r host_pc\python\requirements.txt
host_pc\python\.venv\Scripts\python.exe -m unittest host_pc.python.test_gui
host_pc\python\.venv\Scripts\python.exe host_pc\python\reducer_monitor.py
```

GUI 中选择 `CANable 2.0 SLCAN FD` 和对应 `COMx`。CAN 波特率固定为 `500K / 2M FD+BRS`。
Cangaroo 与 Python GUI/`can_link_probe.py` 不能同时占用同一个 COM 口；运行 Python 工具前必须关闭 Cangaroo 的测量连接。

## 联调顺序

1. 烧录 `build/Debug/Reducer.elf`，检查 CANH、CANL、共地和两端 `120 ohm` 终端电阻。
2. 先运行 `host_pc/python/can_link_probe.py --channel COMx`，确认 classic 诊断、FD health、ACK 和 config 都能收到。
3. 启动 GUI，选择 CANable 2.0 SLCAN FD 和正确串口后连接。
4. 确认 `0x110` 遥测持续到达，`0x0F2` 健康信息每秒刷新。
5. 点击校准并确认收到 `0x0F0` ACK。
6. 从 `100 SPS` 逐步提高到 `30000 SPS`，观察实际遥测速率、TX 丢弃和 ADC 溢出。
7. 执行归零并重启 MCU，确认 Flash 中的零点能够恢复。
