# 用户应用流程说明

本文说明当前 `Application/user.c` 中的应用层运行流程。这个文件把 ADS1256 采集、滤波、零点存储、柔轮物理量计算、CAN 命令处理和 CAN 遥测发送串在一起，是当前固件的主业务入口。

## 总体流程

应用层有两个主要入口：

- `setup()`：系统启动时执行一次。
- `loop()`：主循环中反复执行。

每次 `loop()` 大致执行以下流程：

1. 处理最多 4 条待处理 CAN 命令。
2. 轮询 ADS1256，收集已经完成转换的 ADC 数据。
3. 从 ADS1256 环形缓冲区读取可用采样。
4. 按逻辑通道保存最新原始值。
5. 等所有启用通道都收齐一轮数据。
6. 对每个通道做滑动平均滤波和零点扣除。
7. 用运行均值和方差做异常值剔除。
8. 将滤波后的原始值换算为电压、应变、应力。
9. 每个通道发送一帧 CAN 遥测数据。

## 启动初始化

`setup()` 的初始化顺序如下：

1. `delay_init()`

   初始化延时模块。ADS1256 底层驱动发送命令、读写数据时需要微秒级延时。

2. `can_init()`

   初始化 CAN。初始化结果保存在 `can_ready` 中。只有 `can_ready == true` 时，状态帧和遥测帧才会发送。

3. `adc_ads1256_start()`

   初始化 ADS1256 采集模块。它会完成 ADC 初始化、参数配置、环形缓冲区初始化、选择每片 ADC 的第一个通道，并启动第一轮转换。

4. `flash_storage_register_user_ops()`

   注册板级 Flash 操作函数。具体实现位于 `BSP/flash_storage_port.c`，包括 Flash 解锁、上锁、擦页、双字写入和读取。

   这一步必须在 `filter_init()` 之前执行，因为滤波模块初始化时会尝试从 Flash 加载零点偏移。

5. `filter_init()`

   初始化每个通道的滑动平均滤波器，并调用 `filter_load_zero_from_flash()` 尝试从 Flash 恢复零点偏移。

6. `flexspline_params_set_default(&flexspline_params)`

   初始化柔轮计算默认参数：

   - 参考电压：`2.5 V`
   - PGA：`64`
   - 电桥激励电压：`5.0 V`
   - 应变片灵敏系数：`2.0`
   - 弹性模量：`210000 MPa`

## 通道模型

`ADC_CHANNEL_COUNT` 来自 `ADS1256_LOGICAL_CHANNEL_COUNT`。

当前 ADS1256 采集配置为：

- 每片 ADS1256 采集 `ADS1256_CHANNELS_PER_DEVICE` 个通道，目前为 `3`。
- ADS1256 A 映射到逻辑通道 `0..2`。
- ADS1256 B 启用时映射到逻辑通道 `3..5`。

应用层使用下面的数组保存每个逻辑通道最近一次原始值：

```c
static int32_t adc_raw_value[ADC_CHANNEL_COUNT];
```

使用下面的掩码记录当前批次哪些通道已经收到新数据：

```c
static uint8_t adc_all_ch_mask;
```

当 `adc_all_ch_mask == ADC_ALL_CH_MASK` 时，说明所有启用通道都已经收齐一轮数据，可以进入统一处理。

## ADS1256 采样流程

`loop()` 中会调用：

```c
adc_ads1256_poll();
```

ADS1256 采集层会检查每片启用的 ADC。如果某片 ADC 的 `DRDY` 表示转换完成，则执行高效 MUX 轮询流程：

1. 记住当前已经完成转换的通道。
2. 写 ADS1256 `MUX` 寄存器，切到下一个通道。
3. 发送 `SYNC` 和 `WAKEUP`，让下一通道开始转换。
4. 调用 `ads1256_read_data_nowait()` 读取上一通道已经锁存的结果。
5. 将 `{logical_channel, raw_value}` 写入环形缓冲区。

这个流程符合 ADS1256 数据手册推荐的通道轮询方式：先让下一通道开始采集，再读取上一通道结果，从而减少通道切换带来的空等时间。

随后 `loop()` 通过下面的接口读取环形缓冲区中的数据：

```c
adc_ads1256_get_data(adc_ads1256_data, ADC_CHANNEL_COUNT);
```

每条返回记录都会更新：

- `adc_raw_value[ch]`
- `adc_all_ch_mask` 中对应通道的 bit

## 滤波和零点偏移

当所有通道收齐一轮数据后，每个通道都会执行：

```c
filtered = filter_apply(channel, adc_raw_value[channel]);
```

`filter_apply()` 做两件事：

1. 对当前通道做滑动平均。
2. 减去当前通道的 `zero_offset`。

滑动平均窗口默认值定义在 `filter.h` 中，也可以通过 CAN 命令 `CAN_CMD_SET_FILTER_SIZE` 在运行时修改。

零点偏移保存在滤波模块内部，可以通过以下方式管理：

- `filter_init()` 时从 Flash 加载。
- `CAN_CMD_ZERO_DATUM`：用当前各通道滤波前均值作为零点，并保存到 Flash。
- `CAN_CMD_SAVE_ZERO`：保存当前零点到 Flash。
- `CAN_CMD_LOAD_ZERO`：从 Flash 重新加载零点。
- `CAN_CMD_CLEAR_ZERO`：清空内存零点，并擦除 Flash 中的零点记录。
- `CAN_CMD_SET_CHANNEL_MASK`：设置运行时 ADS1256 扫描通道掩码。

注意：`filter_reset_all()` 只清空滑动平均窗口，不会清除零点偏移。

## 异常值剔除

滤波和零点扣除之后，应用层会判断当前值是否为异常值。

每个通道前 `OUTLIER_MIN_SAMPLES` 个样本默认接受，不做异常判断。之后使用 Welford 在线算法维护运行均值和方差。

常规判断规则为：

```text
(value - mean)^2 > 9 * variance
```

这相当于 3σ 判断，但避免了 `sqrtf()` 计算。

如果方差接近 0，则使用固定阈值：

```text
abs(value - mean) > 100
```

正常值会更新：

- `running_mean[channel]`
- `running_m2[channel]`
- `sample_count[channel]`
- `adc_filtered_value[channel]`

异常值不会更新统计量，应用层会沿用 `adc_filtered_value[channel]` 中保存的上一次有效值。

以下操作会重置异常值统计：

- 重新设置零点
- 修改采样率
- ADS1256 自校准
- 从 Flash 加载零点
- 清除零点

## 物理量换算

异常值处理后的滤波值会传给：

```c
flexspline_calculate(filtered, &flexspline_params, &result);
```

换算路径为：

```text
ADC 原始码 -> 电压 mV -> 微应变 -> 应力 MPa
```

默认参数下，主要公式为：

```text
voltage_mV = raw * (2 * ref_voltage / pga) * 1000 / 8388608
strain     = voltage_mV * 1000 / (excitation_v * gauge_k)
stress     = strain * elastic_modulus / 1000000
```

发送 CAN 前，会把浮点结果压缩为整数单位：

- `voltage_001mv`：`int16_t`，单位 0.01 mV
- `strain_ue`：`int16_t`，单位 微应变
- `stress_01mpa`：`int8_t`，单位 0.1 MPa

打包前会使用 clamp 函数做饱和限制，避免整数溢出回绕。

## CAN 命令处理流程

每次 `loop()` 都会调用：

```c
process_can_commands();
```

该函数每次最多处理 `CAN_COMMANDS_PER_LOOP` 条命令，目前为 `4`。这样可以避免 CAN 命令太多时长时间占用主循环。

接收命令帧要求：

- CAN ID：`CAN_ID_RX_COMMAND`，即 `0x100`
- DLC：`8`
- byte 0 `frame_type`：`CAN_FRAME_TYPE_COMMAND`，即 `0xA0`
- byte 7 CRC：bytes `0..6` 的异或校验

如果 CAN ID 或 DLC 不匹配，帧会被忽略。如果 frame type 或 CRC 错误，会返回状态帧。

有效命令会交给 `process_can_command()` 执行，并在执行后发送状态响应。

### 命令帧格式

CAN ID：`0x100`

```text
byte 0: frame_type = 0xA0
byte 1: sequence
byte 2: cmd_type
byte 3: param
byte 4: value LSB
byte 5: value MSB
byte 6: reserved
byte 7: crc8 XOR(bytes 0..6)
```

其中 `value` 为 little-endian。

### 状态帧格式

CAN ID：`0x102`

```text
byte 0: frame_type = 0xA1
byte 1: sequence，复制自命令帧
byte 2: cmd_type，复制自命令帧
byte 3: status
byte 4: value LSB
byte 5: value MSB
byte 6: detail
byte 7: crc8 XOR(bytes 0..6)
```

其中 `value` 为 little-endian。

### 支持的命令

`CAN_CMD_SET_SAMPLE_RATE` (`0x01`)

根据 `value` 设置 ADS1256 采样率。支持的采样率由 ADS1256 采集模块定义。成功后，状态帧中的 `value` 返回实际应用的采样率。滤波器和异常值统计会被重置。

`CAN_CMD_SET_FILTER_SIZE` (`0x02`)

设置滑动平均窗口。合法范围为 `2..64`。成功后，状态帧中的 `value` 返回应用后的窗口大小。

`CAN_CMD_ZERO_DATUM` (`0x03`)

把当前每个通道的滑动平均原始值作为零点偏移，保存到 Flash，然后重置滤波器和异常值统计。

`CAN_CMD_START_CALIB` (`0x04`)

执行 ADS1256 自校准，重启采集，并重置滤波器和异常值统计。

`CAN_CMD_SAVE_ZERO` (`0x05`)

把当前滤波模块中的零点偏移保存到 Flash。

`CAN_CMD_LOAD_ZERO` (`0x06`)

从 Flash 加载零点偏移。成功后重置滤波器和异常值统计。

`CAN_CMD_CLEAR_ZERO` (`0x07`)

将内存中的零点偏移清零，擦除 Flash 中的校准页，并重置滤波器和异常值统计。

`CAN_CMD_SET_CHANNEL_MASK` (`0x08`)

使用 `value` 设置运行时 ADS1256 扫描掩码。MCU 只扫描上位机图表正在展示的通道。
掩码为 `0` 时停止 ADC 通道轮询。启用状态发生变化的通道会重置滤波器和统计量。

## CAN 遥测发送流程

当所有通道完成一轮处理后，`loop()` 会对每个通道调用：

```c
send_can_telemetry(channel, voltage_001mv, strain_ue, stress_01mpa);
```

如果 `can_ready == false`，遥测帧不会发送。

### 遥测帧格式

CAN ID：`0x101`

```text
byte 0: frame_type = 0x51
byte 1: channel
byte 2: voltage MSB
byte 3: voltage LSB
byte 4: strain MSB
byte 5: strain LSB
byte 6: stress
byte 7: crc8 XOR(bytes 0..6)
```

其中：

- 电压为 big-endian 的 `int16_t`，单位 0.01 mV。
- 应变为 big-endian 的 `int16_t`，单位微应变。
- 应力为 `int8_t`，单位 0.1 MPa。

## Flash 零点存储

Flash 存储模块在 `FLASH_STORAGE_ADDR` 处保存一条 32 字节校准记录。

记录内容包括：

- 数据版本号
- 6 个 `int32_t` 零点偏移
- 零点偏移区域的 CRC-16
- magic 校验值
- 填充字节

加载零点前会校验：

1. Flash 操作函数是否已经注册。
2. magic 是否匹配。
3. version 是否匹配。
4. CRC-16 是否匹配。

如果校验失败，`flash_storage_load_zero()` 会把传入的 offset 数组清零并返回错误。`filter_init()` 不会因为这个错误阻止系统启动，因此没有有效校准数据时系统会以零偏移继续运行。

## 错误状态

CAN 命令状态值如下：

- `CAN_STATUS_OK` (`0x00`)：命令执行成功。
- `CAN_STATUS_BAD_CRC` (`0xE1`)：命令帧 CRC 错误。
- `CAN_STATUS_BAD_TYPE` (`0xE2`)：frame type 错误。
- `CAN_STATUS_BAD_CMD` (`0xE3`)：不支持的命令。
- `CAN_STATUS_BAD_VALUE` (`0xE4`)：参数非法或 ADC 操作失败。
- `CAN_STATUS_STORAGE_ERROR` (`0xE5`)：Flash 保存、加载或清除失败。

部分命令还会填写 `detail` 字段，用来标记更具体的错误来源。`detail` 的含义由对应命令内部定义。

## 运行注意事项

- 当前 `loop()` 已启用 CAN 命令处理。
- 当前 `loop()` 已启用 CAN 遥测发送。
- 应用层会等所有逻辑通道收齐后再统一计算和发送遥测。
- ADS1256 不同通道的数据可能不是同一瞬间到达，但遥测会在批次完成后按通道顺序发送。
- 异常值剔除发生在滑动平均和零点扣除之后，不直接作用于原始 ADC 码。
- `filter_reset_all()` 不会清除零点，只会清空滑动平均缓存。
- 正常采样不会写 Flash。只有零点保存、零点设置、零点清除等命令会擦写 Flash。
