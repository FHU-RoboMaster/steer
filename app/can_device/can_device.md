# can_device

> FDCAN 设备统一初始化模块

`can_device` 是全车三路 FDCAN 总线设备的**集中初始化与注册入口**。它负责 HAL FDCAN 句柄与 BSP 对象的绑定、全通过滤器/全局过滤器配置、各电机总线的注册，以及三路 FDCAN 总线的统一启动。

## 文件说明

| 文件 | 说明 |
| --- | --- |
| `can_device.h` | 对外 API：`can_device_Init()`、`FDCAN_RegisterAllDevices()`、`FDCAN_StartAll()` |
| `can_device.c` | 模块实现：持有 FDCAN1 / FDCAN2 / FDCAN3 的 `STM32FDCAN_t` 实例，完成初始化时序 |

## 总线与设备分配

| 总线 | Message RAM 区间 | 过滤器配置 | 挂载设备 |
| --- | --- | --- | --- |
| FDCAN1 | `[0, 736)` | 全通 → FIFO1 | 发射机构电机（`shoot_control`）/ 预留云台电机 |
| FDCAN2 | `[800, 1536)` | 全通 → FIFO0 | 4 台底盘 M3508（`chassis_control`，控制组 `0x200`） |
| FDCAN3 | `[1600, 2336)` | 全通 → FIFO0 | 扩展总线（预留 DM 电机/外设） |

> 三路过滤器均通过 `STM32FDCAN_ConfigAcceptAllFilter()` 配置全零掩码 + 全局过滤器直通；具体设备的帧过滤由各设备模块的 RX 订阅回调自行完成。

## 初始化时序

`can_device_Init()` 须在 **RTOS 调度器启动前**调用一次（如 `main.c` 中 `MX_FDCAN1_Init()`/`MX_FDCAN2_Init()`/`MX_FDCAN3_Init()` 之后、`osKernelStart()` 之前）。内部按以下顺序执行，任一步骤失败立即进入 `Error_Handler()`：

```
main.c (CubeMX)
  └─ MX_FDCAN1_Init() / MX_FDCAN2_Init() / MX_FDCAN3_Init()

can_device_Init()                                              # 调度器启动前调用
  ├─ STM32FDCAN_Init(&fdcan1_instance, &hfdcan1)               # 1. 绑定对象并校验 Message RAM
  ├─ STM32FDCAN_Init(&fdcan2_instance, &hfdcan2)
  ├─ STM32FDCAN_Init(&fdcan3_instance, &hfdcan3)
  ├─ STM32FDCAN_ConfigAcceptAllFilter(...)                     # 2. 配置三路过滤器与全局过滤器
  ├─ FDCAN_RegisterAllDevices()                                # 3. 注册所有设备及其 RX 回调
  │    ├─ chassis_control_init()                               #    底盘 4 台 M3508（FDCAN2）
  │    └─ shoot_control_init()                                 #    发射机构 3 台电机（FDCAN1）
  └─ FDCAN_StartAll()                                          # 4. 启动 FDCAN1 / FDCAN2 / FDCAN3
```

顺序约束：

1. **必须先 `STM32FDCAN_Init()` 再注册设备**：`chassis_control_init()` / `shoot_control_init()` 通过 BSP 对象表反查 FDCAN 实例（`STM32FDCAN_GetInstance()`）。
2. **必须最后 `FDCAN_StartAll()`**：启动后禁止再调用 `STM32FDCAN_SubscribeRx()` 新增 RX 订阅者，避免与中断竞争。

## 依赖关系

| 依赖模块 | 用途 |
| --- | --- |
| `bsp/bsp_fdcan` | `STM32FDCAN_t` 对象绑定、Message RAM 校验、过滤器、启动、RX 订阅 |
| `calculate/dr16_calculate/chassis_control_dr16` | 底盘 4 台 M3508 电机注册（经 `modules/motor/dj_motor` 走 FDCAN2） |
| `calculate/dr16_calculate/shoot_control_dr16` | 发射机构 3 台电机注册（经 `modules/motor/dj_motor` 走 FDCAN1） |
| `Core/Src/fdcan.c` | `hfdcan1` / `hfdcan2` / `hfdcan3` HAL 句柄与时钟/RAM 初始化 |
