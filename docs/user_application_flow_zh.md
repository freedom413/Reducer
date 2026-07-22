# 用户应用流程

本文说明 `Application/user.c` 当前实现的运行逻辑。

## 主循环

`setup()` 初始化延时、FDCAN、双 ADS1256、Flash 零点存储、滑动平均滤波器和柔轮物理量换算参数。

每次 `loop()`：

1. 最多处理 4 条待处理 CAN FD+BRS 命令。
2. 处理延迟 Flash 配置保存和待发送配置快照。
3. 到达周期时发送 250 ms classic CAN 诊断心跳和每秒一次的 FD 健康帧。
4. 轮询两片 ADS1256。
5. 从 ADS1256 环形缓冲区取出已完成的转换记录。
6. 每条记录到达后立即完成滤波、扣零、统计、物理量换算和可选遥测发送。

应用层不会等待全部通道收齐再处理。这样可以及时轮询 ADC，也可以减少 FDCAN TX FIFO 瞬时突发。

未连接上位机时，ADC 仍持续采样和更新滤波状态，但周期性 CAN 遥测、健康帧、诊断帧和配置快照由主机会话门控。任意合法协议命令都会激活会话；GUI 每秒发送一次静默心跳。连续 3 秒未收到合法命令后，固件结束会话并清空待发送 ACK 和遥测队列。之后收到 `GET_CONFIG` 会立即恢复发送。GUI 正常断开时会先发送显式停止会话参数，再关闭 CAN 适配器。

## MCU 状态 LED

`MCU_LED` 为低电平有效：输出低电平时点亮，输出高电平时熄灭，因此 GPIO 初始化后默认保持熄灭。MCU 正常但未建立主机会话时，每秒短亮 75 ms 作为心跳；主机会话处于活动状态时，每 250 ms 短亮 75 ms；CAN 初始化失败或 ADS1256 未运行时，以 50% 占空比闪烁表示故障。LED 不再随每一次 ADC 转换翻转，避免高 SPS 采集路径承担额外 GPIO 写入；`Error_Handler` 也使用明确的低电平故障闪烁。

## 通道与采集

每片 ADS1256 扫描 4 个差分输入：

| ADC | 逻辑通道 | 差分输入 |
|---|---|---|
| ADS1256 A | `0..3` | `AIN0-AIN1`、`AIN2-AIN3`、`AIN4-AIN5`、`AIN6-AIN7` |
| ADS1256 B | `4..7` | `AIN0-AIN1`、`AIN2-AIN3`、`AIN4-AIN5`、`AIN6-AIN7` |

上位机通过 `CAN_CMD_SET_CHANNEL_MASK` 控制运行时扫描掩码。未订阅的通道不采集，掩码为 `0` 时停止 ADC 轮询。

当 `DRDY` 表示转换完成时，ADS1256 层先切换 MUX，再发送 `SYNC` 和 `WAKEUP`，然后读取已经锁存的上一通道结果，并将 `{logical_channel, raw_value}` 写入环形缓冲区。SPI1 当前为 `1.328125 Mbit/s`，低于本板 ADS1256 的串行时钟上限。

## 采样率与吞吐

支持 ADS1256 全部采样档位：

```text
2.5, 5, 10, 15, 25, 30, 50, 60, 100, 500, 1000, 2000,
3750, 7500, 15000, 30000 SPS
```

多路输入轮询时，真实吞吐按照 ADS1256 数据手册 Table 14 计算。例如设置为 `30000 SPS` 时，每片活动 ADC 约输出 `4374` 次转换每秒。

每个采样值都会完成滤波和物理量换算。Raw 帧最多打包 14 条记录，Physical 帧最多打包 6 条记录，并在三槽 FDCAN 硬件 TX FIFO 前缓存最多 128 条待发记录。固件当前不进行遥测抽取：所有采样记录都进入发送队列，无法入队或发送时通过 CAN TX 丢弃计数上报。

## 滤波、统计与零点

`filter_apply()` 执行滑动平均并扣除零点偏移，滤波窗口可设置为 `2..64`。

应用层不会剔除突变值。真实负载阶跃和异常值在这一层无法可靠区分，因此突变必须进入遥测。运行时电压统计由上位机 GUI 维护，MCU 当前未维护 Welford 统计量。

8 个通道的零点偏移和运行配置存储在 STM32 最后两页 Flash 中。每条为 64 字节记录，使用 CRC32、版本和 magic 校验。记录在活动页内追加；活动页写满后先擦除备用页，再写入并校验新记录，因此换页期间写入失败或掉电时仍保留上一页的有效配置。

## CAN FD 协议

所有业务帧均使用标准 11-bit ID、CAN FD+BRS、`500K / 2M` 和固定长度载荷。v3 FD 帧依赖 CAN 帧 CRC；只有 classic 诊断帧和兼容旧协议帧使用应用层 XOR。classic 诊断心跳只用于联调观测，不作为业务协议回退。

| 方向 | ID | 类型 | 长度 | 用途 |
|---|---:|---:|---:|---|
| MCU -> PC | `0x0F0` | `0xA1` | 12 FD+BRS | 命令 ACK/status |
| MCU -> PC | `0x0F0` | `0x56` | 64 FD+BRS | 配置快照 |
| PC -> MCU | `0x0F1` | `0xA0` | 12 FD+BRS | 命令 |
| MCU -> PC | `0x0F2` | `0x52` | 24 FD+BRS | 每秒健康信息 |
| MCU -> PC | `0x0FF` | `0x57` | 8 classic CAN | 链路诊断心跳 |
| MCU -> PC | `0x110` | `0x54` | 64 FD+BRS | Raw 批量遥测 |
| MCU -> PC | `0x110` | `0x55` | 64 FD+BRS | 物理量批量遥测 |

命令帧：

```text
byte 0: 类型 0xA0
byte 1: 协议版本 0x03
byte 2: sequence
byte 3: command
byte 4: param
byte 5: 保留
byte 6-9: value uint32 LE
byte 10-11: 保留
```

`SET_SAMPLE_RATE` 使用 `param=0` 表示整数 SPS，使用 `param=1` 表示十分之一 SPS，因此 `2.5 SPS` 编码为 `value=25`。

Raw 遥测帧：

```text
byte 0: 类型 0x54
byte 1: 协议版本 0x03
byte 2: 遥测模式 raw
byte 3: sequence
byte 4: 记录数量 1..14
byte 5-6: drop delta uint16 LE
byte 7: 保留
byte 8-63: 十四个 4 字节 raw 记录
```

物理量遥测帧：

```text
byte 0: 类型 0x55
byte 1: 协议版本 0x03
byte 2: 遥测模式 physical
byte 3: sequence
byte 4: 记录数量 1..6
byte 5-6: drop delta uint16 LE
byte 7: 保留
byte 8-61: 六个 9 字节物理量记录
byte 62-63: 保留
```

健康帧：

```text
byte 0: 类型 0x52
byte 1: 协议版本 0x03
byte 2-5: 采样率 x10，uint32 LE
byte 6-7: CAN TX 丢弃数，uint16 LE
byte 8-9: ADC 环形缓冲区溢出数，uint16 LE
byte 10-11: ADC 自动恢复次数，uint16 LE
byte 12-13: 遥测样本/秒，uint16 LE
byte 14-15: 遥测帧/秒，uint16 LE
byte 16: 活动 ADC 数量
byte 17: 遥测模式
byte 18: flags，bit 0 表示 ADC 硬件运行中，bit 1 表示配置待保存，bit 2 表示零点有效，bit 3 表示通道掩码非零
byte 19-23: 保留
```

诊断帧：

```text
byte 0: 类型 0x57
byte 1: flags，CAN ready/主循环/最近 RX FD/最近 RX BRS/bus-off/passive
byte 2: 最近 0x0F1 DLC 字节数
byte 3: 最近命令拒绝原因
byte 4: FDCAN TX error counter
byte 5: FDCAN RX error counter
byte 6: 诊断序号
byte 7: bytes 0..6 的 XOR CRC
```

## 命令

| 命令 | 值 | 说明 |
|---|---:|---|
| `SET_SAMPLE_RATE` | `0x01` | 设置 ADS1256 采样率 |
| `SET_FILTER_SIZE` | `0x02` | 设置滑动平均窗口 |
| `ZERO_DATUM` | `0x03` | 采集当前零点并保存 |
| `START_CALIB` | `0x04` | 执行 ADS1256 自校准 |
| `SAVE_ZERO` | `0x05` | 旧版保留命令，当前不处理且 GUI 不暴露 |
| `LOAD_ZERO` | `0x06` | 旧版保留命令，当前不处理且 GUI 不暴露 |
| `CLEAR_ZERO` | `0x07` | 清除内存和 Flash 零点 |
| `SET_CHANNEL_MASK` | `0x08` | 设置扫描通道 |
| `SET_TELEMETRY_MODE` | `0x09` | 选择 Raw 或 Physical 遥测 |
| `GET_CONFIG` | `0x0A` | 请求配置快照 |
| `SET_VREF_UV` | `0x0B` | 设置 ADC 参考电压，单位微伏 |
| `SET_PGA` | `0x0C` | 设置 ADS1256 PGA 并重新校准 |
| `RESTORE_DEFAULTS` | `0x0D` | 应用默认值、校准、重启并保存 |
| `SET_ZERO_OFFSET` | `0x0E` | 设置单通道零点偏移 |
| `HOST_KEEPALIVE` | `0x0F` | 刷新上位机遥测会话 |

`SAVE_ZERO/LOAD_ZERO` 是旧版保留命令。当前零点和配置持久化由有效命令直接完成，GUI 不提供这两个命令入口。
