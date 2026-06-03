# 用户应用流程

本文说明 `Application/user.c` 当前实现的运行逻辑。

## 主循环

`setup()` 初始化延时、FDCAN、双 ADS1256、Flash 零点存储、滑动平均滤波器和柔轮物理量换算参数。

每次 `loop()`：

1. 最多处理 4 条待处理 CAN FD+BRS 命令。
2. 到达周期时发送每秒一次的健康帧。
3. 轮询两片 ADS1256。
4. 从 ADS1256 环形缓冲区取出已完成的转换记录。
5. 每条记录到达后立即完成滤波、扣零、统计、物理量换算和可选遥测发送。

应用层不会等待全部通道收齐再处理。这样可以及时轮询 ADC，也可以减少 FDCAN TX FIFO 瞬时突发。

## 通道与采集

每片 ADS1256 扫描 4 个差分输入：

| ADC | 逻辑通道 | 差分输入 |
|---|---|---|
| ADS1256 A | `0..3` | `AIN0-AIN1`、`AIN2-AIN3`、`AIN4-AIN5`、`AIN6-AIN7` |
| ADS1256 B | `4..7` | `AIN0-AIN1`、`AIN2-AIN3`、`AIN4-AIN5`、`AIN6-AIN7` |

上位机通过 `CAN_CMD_SET_CHANNEL_MASK` 控制运行时扫描掩码。未订阅的通道不采集，掩码为 `0` 时停止 ADC 轮询。

当 `DRDY` 表示转换完成时，ADS1256 层先切换 MUX，再发送 `SYNC` 和 `WAKEUP`，然后读取已经锁存的上一通道结果，并将 `{logical_channel, raw_value}` 写入环形缓冲区。SPI1 当前为 `2.65625 Mbit/s`，低于本板 ADS1256 的串行时钟上限。

## 采样率与吞吐

支持 ADS1256 全部采样档位：

```text
2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
3750, 7500, 15000, 30000 SPS
```

多路输入轮询时，真实吞吐按照 ADS1256 数据手册 Table 14 计算。例如设置为 `30000 SPS` 时，每片活动 ADC 约输出 `4374` 次转换每秒。

每个采样值都会完成滤波和物理量换算。固件将最多 10 条记录打包为一条 64 字节遥测帧，并在三槽 FDCAN 硬件 TX FIFO 前缓存最多 128 条待发记录。双 ADC 最大轮询速率下可以发送全部采样记录；仅当未来数据源超过 `10000 records/s` 时才启用安全抽取。

## 滤波、统计与零点

`filter_apply()` 执行滑动平均并扣除零点偏移，滤波窗口可设置为 `2..64`。

应用层不会剔除突变值。真实负载阶跃和异常值在这一层无法可靠区分，因此突变必须进入遥测。Welford 运行统计仍保留在 MCU 内部，供诊断使用。

8 个通道的零点偏移存储在 STM32 最后一页 Flash 中。记录为 40 字节，带版本、CRC-16 和 magic 校验。正常采样不会写 Flash，只有显式零点命令会写入。

## CAN FD 协议

所有业务帧均使用标准 11-bit ID、CAN FD+BRS、`1M / 2M` 和 XOR 校验。

| 方向 | ID | 类型 | 长度 | 用途 |
|---|---:|---:|---:|---|
| PC -> MCU | `0x100` | `0xA0` | 8 | 命令 |
| MCU -> PC | `0x101` | `0x53` | 64 | 最多 10 条遥测记录 |
| MCU -> PC | `0x102` | `0xA1` | 8 | 命令 ACK |
| MCU -> PC | `0x103` | `0x52` | 16 | 每秒健康信息 |

命令帧：

```text
byte 0: 类型 0xA0
byte 1: sequence
byte 2: command
byte 3: param
byte 4-5: value uint16 LE
byte 6: 保留
byte 7: bytes 0..6 的 XOR CRC
```

`SET_SAMPLE_RATE` 使用 `param=0` 表示整数 SPS，使用 `param=1` 表示十分之一 SPS，因此 `2.5 SPS` 编码为 `value=25`。

遥测帧：

```text
byte 0: 类型 0x53
byte 1: 遥测记录数量 1..10
byte 2-61: 十个 6 字节记录槽，未使用槽填零
byte 62: 保留
byte 63: bytes 0..62 的 XOR CRC

记录 byte 0: 逻辑通道 0..7
记录 byte 1-2: 电压 int16 BE，单位 0.01 mV
记录 byte 3-4: 应变 int16 BE，单位 microstrain
记录 byte 5: 应力预览 int8，单位 0.1 MPa，溢出时饱和
```

健康帧：

```text
byte 0: 类型 0x52
byte 1: 协议版本 0x01
byte 2-5: 采样率 x10，uint32 LE
byte 6-7: 遥测抽取倍数，uint16 LE
byte 8-9: CAN TX 丢弃数，uint16 LE
byte 10-11: ADC 环形缓冲区溢出数，uint16 LE
byte 12-13: ADC 自动恢复次数，uint16 LE
byte 14: 高 4 位为活动 ADC 数量，bit 0 表示采集运行中
byte 15: bytes 0..14 的 XOR CRC
```

## 命令

| 命令 | 值 | 说明 |
|---|---:|---|
| `SET_SAMPLE_RATE` | `0x01` | 设置 ADS1256 采样率 |
| `SET_FILTER_SIZE` | `0x02` | 设置滑动平均窗口 |
| `ZERO_DATUM` | `0x03` | 采集当前零点并保存 |
| `START_CALIB` | `0x04` | 执行 ADS1256 自校准 |
| `SAVE_ZERO` | `0x05` | 保存当前零点 |
| `LOAD_ZERO` | `0x06` | 从 Flash 读取零点 |
| `CLEAR_ZERO` | `0x07` | 清除内存和 Flash 零点 |
| `SET_CHANNEL_MASK` | `0x08` | 设置扫描通道 |
