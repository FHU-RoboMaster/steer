# imu_rs485

DM-IMU-L1 六轴 IMU 的 RS485 通道驱动模块（适配固件 v2.1.0.0+，24 字节请求应答帧 + CRC-8/SAE-J1850）。底层基于 `bsp_uart` 的 RX 循环 DMA（空闲线切分）与 TX 变长帧双缓冲 DMA，支持默认被动自动轮询，并自动兼容主动上报分组帧。

## 协议帧布局（固定 24 字节）

| 字节偏移  | 字段              | 说明                                                                                          |
| --------- | ----------------- | --------------------------------------------------------------------------------------------- |
| `[0]`     | 帧头              | 固定 `0xA5`（`IMU485_HEADER`）                                                                |
| `[1]`     | 数据类型          | `0x0C` 命令帧（`IMU485_DATA_TYPE_CMD`）/ `0x0D` 数据帧（`IMU485_DATA_TYPE_DATA`）             |
| `[2]`     | 从机地址          | 默认 `1`（`IMU485_DEFAULT_SLAVE_ID`）                                                         |
| `[3]`     | RID               | 数据帧 `0~3`；命令帧 `0~9` 或扩展寄存器 `0x20~0x3B`                                           |
| `[4]`     | 操作类型          | `0x00` 读（`IMU485_OP_READ`）/ `0x01` 写（`IMU485_OP_WRITE`）                                 |
| `[5..20]` | 数据区            | 16 字节（`DATA0~DATA3`），小端序；数据帧应答为 3 或 4 个 IEEE-754 `float`                     |
| `[21]`    | 应答码 / 分组标志 | 被动应答：`0` 成功，`1~6` 错误码；主动上报：`0x80` 组内帧，`0x81` 组末帧                      |
| `[22]`    | CRC8              | **CRC-8/SAE-J1850**（多项式 `0x1D`、初值 `0xFF`、结果异或 `0xFF`），覆盖 `[1..21]` 共 21 字节 |
| `[23]`    | 帧尾              | 固定 `0x5A`（`IMU485_TAIL`）                                                                  |

### 姿态数据 RID（`data_type = 0x0D`）

| RID | 枚举               | 数据内容              | 单位 / 顺序                 | 对应 `dm_imu_t_t` 字段   |
| --- | ------------------ | --------------------- | --------------------------- | ------------------------ |
| `0` | `IMU485_RID_ACCEL` | 三轴加速度（3 float） | `m/s²`（X/Y/Z）             | `accel[0..2]`            |
| `1` | `IMU485_RID_GYRO`  | 三轴角速度（3 float） | `rad/s`（X/Y/Z）            | `gyro[0..2]`             |
| `2` | `IMU485_RID_EULER` | 欧拉角（3 float）     | `deg`（Roll / Pitch / Yaw） | `roll` / `pitch` / `yaw` |
| `3` | `IMU485_RID_QUAT`  | 四元数（4 float）     | 无量纲（W/X/Y/Z）           | `quaternion[0..3]`       |

## 工作机制

### 收发与线程模型

- RX DMA（256 B 循环缓冲）与 TX DMA（双 64 B 缓冲）均通过 `BSP_UART_DMA_BUFFER` 放在 `.axi_ram` 且 32 字节对齐，满足 STM32H7 DMA 访问与 D-Cache 一致性要求。
- `IMU485_RxCallback()` 运行在 HAL 中断上下文，仅将字节写入 512 B 单生产者单消费者环形缓冲并通过 `SIGNAL_IMU485_RAW_READY` 通知任务；帧同步、CRC8 校验、事务超时重试及数据发布全部在任务上下文执行。
- 更新 `self->data` 与兼容全局 `imu_485` 时使用 FreeRTOS 临界区保护，避免跨任务读到撕裂数据。

### 被动轮询 vs 主动上报对比

| 对比项           | 被动轮询模式（默认）                                       | 主动上报模式（单机直连）                                             |
| ---------------- | ---------------------------------------------------------- | -------------------------------------------------------------------- |
| 通信发起方       | 主机按 `RID 0→1→2→3` 逐帧请求，模块应答                    | 模块按设定周期（`1~1000 ms`）成组连续上报                            |
| 偏移 `[21]` 字段 | 应答码：`0x00` 成功，`1~6` 错误码                          | 分组标志：`0x80` 组内帧，`0x81` 组末帧                               |
| 数据更新速率     | 受 `auto_cycle_gap_ms` 控制（默认 `2 ms` 约 300~400 帧/s） | 最高 **1 kHz 组率**（1 ms 一组 4 帧 = 96 字节）                      |
| 总线拓扑         | 支持多从机挂载或 485/CAN 双链路并存                        | **仅限单台 IMU 直连一台 MCU**                                        |
| 模块前置配置     | 出厂默认即支持，开箱即用                                   | 需 RS485 模式、波特率 ≥ 2 Mbps、正常模式、温控放行且至少开启一项输出 |
| 期间发普通命令   | 支持（暂停轮询后可随时读写命令/寄存器）                    | 不支持（仅在 `0x81` 组尾接管窗口内处理 `RID 6 写 0` 关闭帧）         |

### 被动轮询模式（默认）

1. `imu_485_start()` 启动后默认使能 `auto_cycle = 1`，`imu_485_update()` 驱动状态机按 `RID 0（加速度）→ RID 1（角速度）→ RID 2（欧拉角）→ RID 3（四元数）` 顺序发送 24 字节读请求（`data_type = 0x0D, op = 0x00`）。
2. 每发出一帧请求后置 `tx_inflight = 1`，任务在 `xTaskNotifyWait()` 中阻塞等待应答；收到合法应答（`ack == 0x00` 且 CRC8 通过）立即写入缓存并结束本次事务，间隔 `cfg.auto_cycle_gap_ms`（默认 `2 ms`）后发下一项。
3. 若应答字节 CRC8 错误，驱动直接丢弃并计入 `stats.rx_crc_err`，不提前结束事务；当超过 `cfg.timeout_ms`（默认 `80 ms`）未收到合法应答时自动重发（最多 `cfg.retry` 次，默认 `2` 次），耗尽后计入 `stats.tx_timeout` 并继续轮询下一项。
4. 同一时刻总线仅允许一笔在途事务；若业务需主动发单帧请求或配置命令，可先调用 `imu_485_enable_auto_cycle(dev, false)` 暂停自动轮询。

### 主动上报模式（单机直连，最高 1 kHz）

1. **分组帧格式**：开启主动上报后，模块按 `RID 0→1→2→3` 顺序将 `0x29` 输出位图中启用的数据项拼成一组连续发出；帧头/帧尾/从机地址/CRC8 与被动模式完全一致，仅偏移 `[21]` 固定为 `0x80`（组内帧）或 `0x81`（本组最后一帧；只启用一项时该帧直接标 `0x81`）。
2. **标准开启流程**：
   - 先确认模块波特率 ≥ 2 Mbps、通信类型为 RS485（`1`）；
   - 进入设置模式（`imu_485_enter_setting`）配置上报周期（`imu_485_cmd_set_interval_ms`，如 `1 ms`）、输出位图（`IMU485_REG_OUTPUT_BITMAP`，如 `0x0F` 四项全开）与温控（`IMU485_REG_TEMP_CTRL`），再退出设置模式（`imu_485_exit_setting`）；
   - 发送开启命令 `imu_485_cmd_set_active_report(dev, true)`，通过 `imu_485_get_cmd_result()` 确认 `ack == IMU485_ACK_SUCCESS` 后，再调用 `imu_485_set_active_rx(dev, true)` 切换驱动进入主动接收状态（同时自动关闭 `auto_cycle`）。
3. **安全关闭流程（接管窗口）**：
   - 主动上报期间模块持续占用半双工总线，直接调用 `imu_485_cmd_set_active_report(dev, false)` 会返回 `BUSY`；
   - 需调用 `imu_485_active_request_stop(dev)` 武装关闭：驱动预生成 `RID 6 写 0` 关闭帧，在收到下一组 `0x81` 组尾帧时立即通过 TX DMA 发出（协议要求组尾后 `20~100 us` 内开始发送，设备端接管窗口截止在释放 DE 后 `350 us`）；
   - 若本次错过窗口或 `cfg.active_stop_timeout_ms`（默认 `100 ms`）未收到关闭 ACK，驱动会自动在下一组 `0x81` 组尾重试，直到收到 `ack == 0` 后自动退出主动模式（`active_rx = 0`）并将该 ACK 存入 `cmd_result`；
   - 可通过 `imu_485_get_active_stats()` 查看 `running / frames / groups / last_group_frames / stop_sent / stop_ack / stop_timeout` 判断实际收包速率与关闭状态。

### 主动帧自动兼容与回退机制

- 即使驱动以默认被动轮询启动，一旦在总线上收到合法 CRC8 的 `0x80`/`0x81` 主动数据帧，驱动也会立即解析写入数据，并自动切换 `active_rx = 1`（标记 `active_auto_detected = 1`）暂停轮询发送，避免半双工总线碰撞。
- 对于自动侦测切入的主动模式，若超过 `cfg.stale_ms` 未再收到主动帧（例如模块掉电重启后回到被动模式），驱动会自动清除 `active_rx` 并恢复被动自动轮询。
- 若由用户代码显式调用 `imu_485_set_active_rx(dev, true)` 进入主动模式，则不会自动回退，需由用户调用 `imu_485_active_request_stop(dev)` 或 `imu_485_set_active_rx(dev, false)` 退出。

## 典型用法

### 1. 任务初始化与轮询（参考 `task/imu_485_task`）

```c
#include "imu_rs485.h"
#include "usart.h"

static IMU485_t imu_inst;

void imu_485_task(void *argument) {
  (void)argument;

  err_t status = imu_485_init(&imu_inst, &huart2); /* MC02 RS485-2, 2 Mbps, 硬件 DE */
  imu_inst.thread_alert = xTaskGetCurrentTaskHandle();
  if (status == OK) {
    status = imu_485_start(&imu_inst);
  }
  if (status != OK) {
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    imu_485_update(&imu_inst, IMU485_OFFLINE_TIMEOUT_MS);
  }
}
```

### 2. 读取与观测姿态数据、链路状态

任务启动后，`imu_485_device` 指向静态驱动实例，全局 `imu_485` 与 `imu_485_device->data` 同步更新。调试器（Live Expressions / Watch）或业务代码可直接查看以下字段：

**姿态数据**
- `imu_485_device->data.roll`（或 `imu_485.roll`）：横滚角（`deg`）
- `imu_485_device->data.pitch`（或 `imu_485.pitch`）：俯仰角（`deg`）
- `imu_485_device->data.yaw`（或 `imu_485.yaw`）：偏航角（`deg`）
- `imu_485_device->data.gyro[0..2]`（或 `imu_485.gyro[0..2]`）：三轴角速度 X/Y/Z（`rad/s`）
- `imu_485_device->data.accel[0..2]`（或 `imu_485.accel[0..2]`）：三轴加速度 X/Y/Z（`m/s²`）
- `imu_485_device->data.quaternion[0..3]`（或 `imu_485.quaternion[0..3]`）：四元数 W/X/Y/Z

**链路与状态诊断**
- `imu_485_device->online_`：是否在线（`true` 表示近期收到有效帧）
- `imu_485_device->meta.valid`：数据有效位图（`0x0F` 表示加速度/角速度/欧拉角/四元数四类数据均已收到；对应各自时间戳 `meta.t_accel_ms` / `t_gyro_ms` / `t_euler_ms` / `t_quat_ms`）
- `imu_485_device->stats`：`rx_ok`、`rx_crc_err`、`rx_bad`、`tx_timeout`、`tx_retry`、`tx_err`、`rx_dropped`、`rx_overflow` 等通信统计
- `imu_485_device->active_rx` / `active_frames` / `active_groups`：主动上报运行状态与分组帧统计（或通过 `imu_485_get_active_stats()` 获取快照）

```c
/* 方式一：读取全局兼容结构 */
float yaw_deg = imu_485.yaw;
float gz_rads = imu_485.gyro[2];

/* 方式二：经设备对象检查在线状态与数据有效位 */
if (imu_485_device != NULL && imu_485_device->online_ &&
    (imu_485_device->meta.valid & IMU485_VALID_EULER)) {
  float roll  = imu_485_device->data.roll;
  float pitch = imu_485_device->data.pitch;
  float yaw   = imu_485_device->data.yaw;
}
```

### 3. 发送命令与读写扩展寄存器（可选）

同一时刻仅允许一笔在途事务；发配置命令前可先暂停自动轮询，每条命令通过 `imu_485_get_cmd_result()` 等待应答成功后再发下一条：

```c
imu_485_enable_auto_cycle(imu_485_device, false);

/* 例如：航向角归零（有副作用命令默认不自动重发） */
if (imu_485_cmd_yaw_zero(imu_485_device) == OK) {
  imu485_frame_t rsp;
  /* 在后续循环中轮询应答 */
  if (imu_485_get_cmd_result(imu_485_device, &rsp) &&
      rsp.ack == IMU485_ACK_SUCCESS) {
    /* 命令执行成功 */
  }
}

/* 写 0x21~0x3B 扩展配置前需先进入设置模式：
 * imu_485_enter_setting() -> imu_485_write_reg32() -> imu_485_cmd_save() -> imu_485_exit_setting() */
```

### 4. 主动上报模式启停流程（可选，单机直连）

```c
/* 1) 在设置模式下按顺序逐条配置并等待各步 ACK 成功：
 *    imu_485_enter_setting(dev)
 * -> imu_485_cmd_set_interval_ms(dev, 1)                       // 1 ms 周期 (1 kHz)
 * -> imu_485_write_reg32(dev, IMU485_REG_OUTPUT_BITMAP, 0x0F)  // 四类数据全开
 * -> imu_485_write_reg32(dev, IMU485_REG_TEMP_CTRL, 0)         // 放行温控
 * -> imu_485_exit_setting(dev)
 */

/* 2) 开启主动上报：等待 RID 6 写 1 应答成功后，再切换驱动进入主动接收 */
imu_485_cmd_set_active_report(imu_485_device, true);
/* 收到 ack == IMU485_ACK_SUCCESS 后： */
imu_485_set_active_rx(imu_485_device, true);

/* 3) 读取主动统计 */
imu485_active_stats_t act;
imu_485_get_active_stats(imu_485_device, &act);

/* 4) 关闭主动上报：武装关闭，驱动将在下一组 0x81 组尾窗口内发出关闭帧 */
imu_485_active_request_stop(imu_485_device);
/* 收到关闭 ACK 后 act.running 自动清 0，可重新开启自动轮询 */
```

## 对外接口一览

### 生命周期与轮询

| 接口                                                                                                    | 说明                                                                                 |
| ------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `void imu_485_cfg_default(imu485_cfg_t *cfg)`                                                           | 填充默认配置（从机 `1`、超时 `80 ms`、重试 `2`、轮询间隔 `2 ms`、掉线阈值 `250 ms`） |
| `err_t imu_485_init(IMU485_t *self, UART_HandleTypeDef *uart_handle)`                                   | 使用默认配置初始化 RX/TX DMA 通道                                                    |
| `err_t imu_485_init_with_cfg(IMU485_t *self, UART_HandleTypeDef *uart_handle, const imu485_cfg_t *cfg)` | 使用自定义参数初始化                                                                 |
| `err_t imu_485_start(IMU485_t *self)`                                                                   | 启动 RX/TX DMA 并开启默认自动轮询                                                    |
| `void imu_485_update(IMU485_t *self, uint32_t timeout_ms)`                                              | 任务主循环入口：消费接收数据、推进轮询/重试、刷新 `online_`、恢复 RX 异常            |
| `void imu_485_enable_auto_cycle(IMU485_t *self, bool enable)`                                           | 启停 RID `0~3` 自动轮询                                                              |
| `bool imu_485_is_stale(const IMU485_t *self, uint32_t now_ms, uint32_t max_age_ms)`                     | 判断缓存姿态数据是否过期                                                             |
| `void imu_485_data_unpack(const uint8_t *pData)`                                                        | 兼容接口：解包单帧 24 字节数据帧并写入全局 `imu_485`                                 |

### 纯编解码与校验

| 接口                                                                                            | 说明                                         |
| ----------------------------------------------------------------------------------------------- | -------------------------------------------- |
| `uint8_t imu_485_crc8_j1850(const uint8_t *data, uint16_t len)`                                 | 计算 CRC-8/SAE-J1850                         |
| `err_t imu_485_build_frame(...)` / `imu_485_build_req(...)`                                     | 构造 24 字节命令帧 / 数据读请求帧            |
| `err_t imu_485_parse_frame(...)` / `imu_485_parse_rsp(...)` / `imu_485_parse_active_frame(...)` | 校验并解析通用帧、被动数据应答帧、主动上报帧 |

### 请求、命令与主动上报

| 接口                                                                                                  | 说明                                                      |
| ----------------------------------------------------------------------------------------------------- | --------------------------------------------------------- |
| `err_t imu_485_request(IMU485_t *self, imu485_rid_t rid)`                                             | 单次请求指定姿态项（`ACCEL` / `GYRO` / `EULER` / `QUAT`） |
| `err_t imu_485_send_cmd(...)` / `bool imu_485_get_cmd_result(...)`                                    | 发送命令帧与非阻塞读取命令应答                            |
| `imu_485_cmd_reboot` / `save` / `yaw_zero` / `gyro_cal` / `accel_cal` / `factory_reset`               | 常用控制命令（默认不重试）                                |
| `imu_485_cmd_set_baud_index` / `set_interval_ms` / `set_com_type` / `read_reg`                        | 链路与上报参数读写                                        |
| `imu_485_enter_setting` / `exit_setting` / `read_reg32` / `write_reg32` / `result_u32` / `result_f32` | 扩展寄存器 `0x20~0x3B` 读写与结果解包                     |
| `imu_485_cmd_set_active_report` / `set_active_rx` / `active_request_stop` / `get_active_stats`        | 485 主动上报启停与统计（单机直连）                        |

## 依赖与硬件配置

- **串口外设**：MC02 RS485-2 使用 `USART2`（`PD4=DE`、`PD5=TX`、`PD6=RX`），需配置为 **2 Mbps / 8N1 / 硬件 RS485 DE（高有效）**，并开启 `USART2_RX`（Circular DMA）与 `USART2_TX`（Normal DMA）。
- **软件依赖**：`bsp/bsp_uart`、`component/comp_def`（`SIGNAL_IMU485_RAW_READY`）、`component/comp_utils`、FreeRTOS。

## 调试与排错

调试器中可重点观察 `imu_485_device->stats` 与 `imu_485_device->meta`：

- `stats.rx_ok` 持续递增且 `online_ == true`：链路正常；`meta.valid == 0x0F` 表示四类数据均已就绪。
- `stats.tx_timeout` 持续增长、`rx_ok == 0`：检查 A/B 线序、模块供电、波特率（默认 2 Mbps）、模块通信类型是否为 RS485（`1`）以及从机地址是否为 `1`。
- `stats.rx_crc_err` 或 `stats.rx_dropped` 增长：检查共地、RS485 总线干扰、引脚速度及波特率误差。
- `stats.rx_overflow` 非零：说明任务长时间未调度导致 512 B 环形缓冲溢出，需检查任务优先级与阻塞情况。
