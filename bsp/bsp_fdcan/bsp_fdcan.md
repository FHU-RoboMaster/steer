# bsp_fdcan

FDCAN BSP，以静态对象控制块 `STM32FDCAN_t` 封装 STM32H7 HAL FDCAN。只负责 Classical CAN / CAN FD 帧收发、Message RAM 校验、Bus-Off 自动恢复与运行期位时序重配，不承载电机等业务协议；RX 采用「订阅-广播」模型，TX 不排队。

## 常用功能

### 初始化与启动

对象由调用方静态持有，按「绑定 → 过滤器 → 订阅 → 启动」的顺序初始化：

```c
#include "bsp_fdcan.h"

static STM32FDCAN_t fdcan1_instance;

err_t fdcan_init(void)
{
    err_t err = STM32FDCAN_Init(&fdcan1_instance, &hfdcan1);       /* 1. 绑定句柄并校验 Message RAM */
    err = STM32FDCAN_ConfigAcceptAllFilter(&fdcan1_instance,
                                           FDCAN_RX_FIFO0);        /* 2. 配置过滤器与全局过滤器 */
    err = STM32FDCAN_SubscribeRx(&fdcan1_instance, my_rx_cb, NULL);/* 3. 注册订阅 */
    return STM32FDCAN_Start(&fdcan1_instance);                     /* 4. 启动并开启中断 */
}
```

### 接收：订阅回调

FIFO0 / FIFO1 新消息中断内循环排空当前 FIFO，按注册顺序向订阅者广播栈上帧快照：

```c
static void my_rx_cb(STM32FDCAN_t* self, const BSP_FDCAN_Frame_t* frame,
                     void* context)
{
    if ((frame->id_type_ == FDCAN_STANDARD_ID) &&
        (frame->frame_type_ == FDCAN_DATA_FRAME) &&
        (frame->fd_format_ == FDCAN_CLASSIC_CAN) &&
        (frame->id_ == 0x201U)) {
        /* frame->data_[0..frame->size_) 为有效数据，剩余尾部字节已清零 */
    }
}
```

### 发送一帧

```c
/* Classical CAN 标准数据帧（0..8 字节） */
uint8_t data[8] = {0};
err_t err = STM32FDCAN_SendStd(&fdcan1_instance, 0x200U, data, sizeof(data));

/* CAN FD 数据帧（0..64 字节，开启 BRS 数据段加速） */
uint8_t fd_data[16] = {0};
err = STM32FDCAN_SendFD(&fdcan1_instance, 0x100U, FDCAN_STANDARD_ID, true,
                        fd_data, sizeof(fd_data));
```

### 运行期位时序与模式重配

字段填 `0` 表示保持原寄存器值；支持在 `Start` 前或运行中调用（先做参数范围校验，再停机改 `NBTP`/`DBTP`/`CCCR` 并按原状态恢复）：

```c
/* 例如 104 MHz FDCAN 核时钟下配置仲裁段 1 Mbps + 数据段 4 Mbps */
BSP_FDCAN_Config_t cfg = {
    .nominal_timing_ = {.brp_ = 8U, .phase_seg1_ = 9U, .phase_seg2_ = 3U, .sjw_ = 3U},
    .data_timing_ = {.brp_ = 2U, .phase_seg1_ = 9U, .phase_seg2_ = 3U, .sjw_ = 3U},
    .mode_ = {.fd_enable_ = true, .brs_enable_ = true},
    .apply_mode_ = true,
};
err_t err = STM32FDCAN_SetConfig(&fdcan1_instance, &cfg);
```

### 错误状态与核时钟查询

```c
BSP_FDCAN_ErrorState_t err_state = {0};
if (STM32FDCAN_GetErrorState(&fdcan1_instance, &err_state) == OK) {
    /* 查看 tx_error_counter_ / rx_error_counter_ / bus_off_ 等 */
}
uint32_t clk_hz = STM32FDCAN_GetClockFreq(&fdcan1_instance);
```

## 注意事项

- STM32H7 三路 FDCAN 共享 2560 words（10 KB）Message RAM：`STM32FDCAN_Init()` 会自动校验越界与多实例重叠。
- 发送不排队：硬件 TX FIFO 满立即返回 `BUSY`，由上层下一周期用最新数据重发。
- 订阅必须在 `STM32FDCAN_Start()` 之前；启动后再订阅返回 `STATE_ERR`，不支持运行期退订。
- 全局过滤器 `STM32FDCAN_ConfigGlobalFilter()` 必须在 `Start` 前（`HAL_FDCAN_STATE_READY`）调用。
- 回调运行在中断上下文，必须短且非阻塞；`frame` 为栈上快照，延后处理须先拷贝。
- 触发 `Bus-Off` 中断时，ISR 自动清除 `CCCR.INIT` 启动硬件自动恢复并累计 `bus_off_count_`。
