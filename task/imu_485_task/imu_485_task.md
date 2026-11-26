# imu_485_task

DM-IMU-L1（RS485 24 字节 + CRC-8/SAE-J1850 协议）任务。初始化 MC02 RS485-2（`USART2` / `huart2`，2 Mbps，硬件 DE 方向控制）上的 DMA 收发与自动轮询逻辑，并周期性刷新解码结果与在线状态。

## 任务流程

```c
void imu_485_task(void* argument) {
  static IMU485_t instance;
  err_t status = imu_485_init(&instance, &huart2);
  imu_485_device = &instance; /* 全局指针，供业务读取 IMU 数据 */
  instance.thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) status = imu_485_start(imu_485_device);
  ASSERT(status == OK);
  if (status != OK) { vTaskDelete(NULL); return; }

  for (;;) {
    imu_485_update(imu_485_device, IMU485_OFFLINE_TIMEOUT_MS);
  }
}
```

## 对外接口

| 接口 | 说明 |
| --- | --- |
| `void imu_485_task(void* argument)` | 任务入口，`freertos.c` 中 `imu_rs485` 线程调用 |
| `IMU485_t* imu_485_device` | 全局对象指针，经 `->data`、`->meta`、`->online_`、`->stats` 读取状态与姿态数据 |
| `dm_imu_t_t imu_485` | 兼容全局缓存（加速度/角速度/欧拉角/四元数） |

## 依赖模块

- `modules/dm_imu`（`imu_rs485`）：DM-IMU-L1 RS485 24 字节协议编解码、CRC-8/SAE-J1850 校验、RID 0~3 自动轮询、主动上报帧（`0x80`/`0x81`）兼容解析，底层走 `bsp_uart` RX 循环 DMA + TX 变长帧双缓冲 DMA
- `Core/Src/usart.c`：`huart2`（USART2，PD4=DE / PD5=TX / PD6=RX，2 Mbps，8N1，硬件 RS485 DE）
- `component/comp_def`：`SIGNAL_IMU485_RAW_READY` 信号位

## 注意事项

- 默认开启被动自动轮询（依次读取加速度、角速度、欧拉角、四元数），同时自动兼容模块处于主动上报（`0x80`/`0x81`）模式时的帧接收。
- 任务阻塞在 `imu_485_update()` 内等待任务通知或轮询间隔；超时未收到新数据则置 `online_ = false`。
- 初始化失败会 `ASSERT` 并自删，全局指针仍有效但数据冻结。
