#pragma once

#include "comp_cmd.h"
#include "comp_utils.h"
#include "main.h"
#include "usart.h"

#ifndef __cplusplus
#include <stdalign.h>
#endif

// ==================== DMA 缓冲对齐与内存段宏（兼容 F4 / H7 Cache） ====================

//! D-Cache cache line 字节数；有 D-Cache（如 STM32H723）为 32，无 Cache（如 F4）为 1。
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
#ifndef BSP_UART_DCACHE_LINE_SIZE
#ifdef __SCB_DCACHE_LINE_SIZE
#define BSP_UART_DCACHE_LINE_SIZE (__SCB_DCACHE_LINE_SIZE)
#else
#define BSP_UART_DCACHE_LINE_SIZE (32U)
#endif
#endif
#else
#ifndef BSP_UART_DCACHE_LINE_SIZE
#define BSP_UART_DCACHE_LINE_SIZE (1U)
#endif
#endif

//! 将缓冲区长度向上对齐到 D-Cache line 整数倍（无 Cache 平台保持原长度）。
#define BSP_UART_DMA_ALIGN_UP(size)                             \
  ((((size) + (BSP_UART_DCACHE_LINE_SIZE - 1U)) /               \
    BSP_UART_DCACHE_LINE_SIZE) *                                \
   BSP_UART_DCACHE_LINE_SIZE)

//! 普通 DMA（DMA1/DMA2）缓冲区所在内存段：
//! STM32H7 默认放到链接脚本 .axi_ram（RAM_D1，避免落入 DMA 不可访问的 DTCMRAM）；
//! STM32F4 等单 SRAM 芯片为空，落入默认 .bss。
#ifndef BSP_UART_DMA_SECTION
#if defined(D1_AXISRAM_BASE)
#define BSP_UART_DMA_SECTION __attribute__((section(".axi_ram")))
#else
#define BSP_UART_DMA_SECTION
#endif
#endif

//! H7 BDMA（如 LPUART1）缓冲区所在内存段：放到 .ram_d3（RAM_D3）。
#ifndef BSP_UART_BDMA_SECTION
#if defined(D3_SRAM_BASE)
#define BSP_UART_BDMA_SECTION __attribute__((section(".ram_d3")))
#else
#define BSP_UART_BDMA_SECTION
#endif
#endif

//! 声明兼容 F4 / H7（含 D-Cache）的 UART DMA 缓冲区（可前加 static）。
//! 示例：static BSP_UART_DMA_BUFFER(uart_rx_buf, 128U);
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
#define BSP_UART_DMA_BUFFER(name, size) \
  alignas(32) uint8_t name[BSP_UART_DMA_ALIGN_UP(size)] BSP_UART_DMA_SECTION

//! 声明兼容 H7 BDMA（如 LPUART1）的 UART DMA 缓冲区（可前加 static）。
#define BSP_UART_BDMA_BUFFER(name, size) \
  alignas(32) uint8_t name[BSP_UART_DMA_ALIGN_UP(size)] BSP_UART_BDMA_SECTION
#else
#define BSP_UART_DMA_BUFFER(name, size) \
  uint8_t name[BSP_UART_DMA_ALIGN_UP(size)] BSP_UART_DMA_SECTION

//! 声明兼容 H7 BDMA（如 LPUART1）的 UART DMA 缓冲区（可前加 static）。
#define BSP_UART_BDMA_BUFFER(name, size) \
  uint8_t name[BSP_UART_DMA_ALIGN_UP(size)] BSP_UART_BDMA_SECTION
#endif

//! H7 兼容旧模块回退缓冲开关：当调用方未将 RX / FrameTx 缓冲放到 .axi_ram
//! 或未按 32 字节对齐时，自动切换到 BSP 内部 .axi_ram 缓冲（F4 默认关闭，零开销）。
#ifndef BSP_UART_DMA_FALLBACK_ENABLE
#if defined(D1_AXISRAM_BASE)
#define BSP_UART_DMA_FALLBACK_ENABLE (1U)
#else
#define BSP_UART_DMA_FALLBACK_ENABLE (0U)
#endif
#endif

#ifndef BSP_UART_DMA_FALLBACK_RX_SIZE
#define BSP_UART_DMA_FALLBACK_RX_SIZE (256U)
#endif

#ifndef BSP_UART_DMA_FALLBACK_TX_SIZE
#define BSP_UART_DMA_FALLBACK_TX_SIZE (256U)
#endif

// ==================== UART 类型与缓冲描述 ====================

//! UART BSP 逻辑设备编号。
//! 枚举项按当前芯片实际启用的 USART/UART/LPUART 宏展开，保持和 CubeMX 外设定义一致。
typedef enum {
#ifdef USART1
  BSP_USART1,
#endif
#ifdef USART2
  BSP_USART2,
#endif
#ifdef USART3
  BSP_USART3,
#endif
#ifdef USART4
  BSP_USART4,
#endif
#ifdef USART5
  BSP_USART5,
#endif
#ifdef USART6
  BSP_USART6,
#endif
#ifdef USART7
  BSP_USART7,
#endif
#ifdef USART8
  BSP_USART8,
#endif
#ifdef USART9
  BSP_USART9,
#endif
#ifdef USART10
  BSP_USART10,
#endif
#ifdef UART1
  BSP_UART1,
#endif
#ifdef UART2
  BSP_UART2,
#endif
#ifdef UART3
  BSP_UART3,
#endif
#ifdef UART4
  BSP_UART4,
#endif
#ifdef UART5
  BSP_UART5,
#endif
#ifdef UART6
  BSP_UART6,
#endif
#ifdef UART7
  BSP_UART7,
#endif
#ifdef UART8
  BSP_UART8,
#endif
#ifdef UART9
  BSP_UART9,
#endif
#ifdef UART10
  BSP_UART10,
#endif
#ifdef LPUART1
  BSP_LPUART1,
#endif
#ifdef LPUART2
  BSP_LPUART2,
#endif
#ifdef LPUART3
  BSP_LPUART3,
#endif

  BSP_UART_NUMBER,
  BSP_UART_ID_ERROR
} BSP_UART_t;

//! 用户接收回调。
//! data 指向本次新收到的数据片段，size 为片段长度；回调由 HAL RX
//! 事件分发路径触发。
typedef void (*STM32UART_RxCallback_t)(uint8_t* data, size_t size);

//! 用户发送完成回调。
//! 变长帧 TX 的 UART TC 完成后触发，此时最后一个停止位已发出。
typedef void (*STM32UART_TxCompleteCallback_t)(void);

//! 硬件 DBM 换页回调：data 指向刚发送完的空闲页，size 为固定页长。
//! 在 DMA ISR
//! 中填充下一轮数据，必须在另一页发送完成前返回；不更新则循环发送旧内容。
typedef void (*STM32UART_TxRefillCallback_t)(uint8_t* data, size_t size);

//! 通用原始数据缓冲描述。
//! BSP 只保存地址和长度，不拥有缓冲区生命周期，调用方必须保证缓冲区长期有效。
typedef struct {
  void* addr_;   //! 缓冲区起始地址
  size_t size_;  //! 缓冲区容量，单位字节
} BSP_UART_RawData_t;

//! 单缓冲 DMA RX 控制块。
//! RX 使用硬件循环 DMA + last_rx_pos_ 软件读指针，HAL
//! 空闲线事件到来时切分新数据。
typedef struct {
  BSP_UART_t id_;                       //! BSP 逻辑串口编号
  volatile size_t last_rx_pos_;         //! 软件读指针，上次处理到的 DMA 写入位置
  BSP_UART_RawData_t dma_buff_rx_;      //! RX 循环 DMA 原始缓冲
  UART_HandleTypeDef* uart_handle_;     //! CubeMX 生成的 HAL UART 句柄
  STM32UART_RxCallback_t rx_callback_;  //! 新数据片段回调
  volatile err_t last_error_;           //! 最近一次 BSP 操作结果
  volatile uint32_t rx_error_count_;    //! RX 错误累计次数（可用于上层丢弃半包）
  uint32_t rx_err_window_tick_;         //! 错误突发计数窗口起始时间戳
  uint8_t rx_err_burst_count_;          //! 当前时间窗口内的连续错误次数
  volatile bool rx_enabled_;            //! SetRxDMA 是否已启用接收
  volatile bool rx_restart_pending_;    //! 是否待通过定时器/任务重启 RX DMA
  volatile bool rx_restarting_;         //! 任务上下文正在执行 RestartRxDMA
} STM32UART_t;

//! 变长帧 DMA TX 控制块。
//! active_buf_ 表示当前 DMA 正在发送的缓冲，pending_size_
//! 表示另一块缓冲中待发送字节数。 两块 TX 缓冲须等长、非重叠且容量为 1..65535
//! 字节，单次写入不能超过容量。
typedef struct {
  BSP_UART_t id_;                               //! BSP 逻辑串口编号
  size_t last_tx_pos_;                          //! 预留发送位置记录
  BSP_UART_RawData_t dma_buff_0_;               //! TX 软件缓冲 0
  BSP_UART_RawData_t dma_buff_1_;               //! TX 软件缓冲 1
  UART_HandleTypeDef* uart_handle_;             //! CubeMX 生成的 HAL UART 句柄
  STM32UART_TxCompleteCallback_t tx_callback_;  //! UART TC 发送完成回调
  err_t last_error_;                            //! 最近一次 BSP 操作结果
  volatile uint8_t active_buf_;                 //! 当前 DMA 使用的缓冲编号
  volatile size_t pending_size_;                //! 等待提交的字节数
  volatile bool tx_busy_;                       //! HAL DMA 是否正在发送
  bool dma_ready_;                              //! SetTxDMA 是否成功
} STM32UARTFrameTx_t;

//! 硬件双缓冲 DMA TX 控制块。
//! M0/M1 以相同 NDTR 自动循环，缓冲区必须位于 DMA 可访问的 SRAM（F4 不能位于
//! CCM，H7 不能位于 DTCM/ITCM，推荐使用 BSP_UART_DMA_BUFFER() 放到 .axi_ram
//! 并按 32 字节对齐）。
typedef struct {
  BSP_UART_t id_;                             //! BSP 逻辑串口编号
  BSP_UART_RawData_t dma_buff_0_;             //! 硬件 M0 缓冲
  BSP_UART_RawData_t dma_buff_1_;             //! 硬件 M1 缓冲
  UART_HandleTypeDef* uart_handle_;           //! HAL UART 句柄
  STM32UART_TxRefillCallback_t tx_callback_;  //! 空闲页填充回调
  volatile err_t last_error_;                 //! 最近一次操作结果
  volatile uint8_t active_buf_;  //! 完成 ISR 采样的 CT，硬件会自动切换
  volatile bool tx_busy_;        //! 连续 DMA 是否已启动
  bool dma_ready_;               //! SetTxDMA 是否成功
} STM32UARTDoubleBufTx_t;

#ifdef __cplusplus
extern "C" {
#endif

// ==================== UART 公开接口 ====================

//! 将 HAL 外设 Instance 映射到 BSP UART 逻辑 ID。
//! HAL 回调里只有 USART_TypeDef* 或 UART_HandleTypeDef*，BSP
//! 通过该函数反查对象表。
BSP_UART_t BSP_UART_get_id(USART_TypeDef* addr);

// ==================== 单缓冲 DMA RX ====================
//! 初始化单缓冲 DMA RX 控制块。
//! 只绑定句柄、DMA 缓冲和回调；真正启动 DMA 接收由 STM32UART_SetRxDMA() 完成。
err_t STM32UART_Init(STM32UART_t* self, UART_HandleTypeDef* uart_handle,
                     BSP_UART_RawData_t dma_buff_rx,
                     STM32UART_RxCallback_t callback);

//! 启动单缓冲 DMA 接收。
//! 将 RX DMA 配置为 DMA_CIRCULAR，并调用 HAL_UARTEx_ReceiveToIdle_DMA()
//! 开始接收。
err_t STM32UART_SetRxDMA(STM32UART_t* self);

//! 在 UART 错误后停止并重新启动单缓冲 RX DMA。
//! 仅在任务上下文调用；用于恢复 HAL 因帧错/溢出而停止的接收状态。
err_t STM32UART_RestartRxDMA(STM32UART_t* self);

//! 更新单缓冲 RX 用户回调。
//! 可在初始化后替换回调，不会重启 DMA。
void STM32UART_SetRxCallback(STM32UART_t* self,
                             STM32UART_RxCallback_t callback);

//! 读取单缓冲 RX 最近一次错误码。
err_t STM32UART_GetLastError(const STM32UART_t* self);

//! 读取单缓冲 RX 累计错误次数（可用于上层在错误后丢弃不完整半包）。
uint32_t STM32UART_GetRxErrorCount(const STM32UART_t* self);

//! 处理一段已经切分好的 RX 数据。
//! 内部会做空指针和长度保护，然后调用用户回调。
void STM32UART_HandleRxData(STM32UART_t* self, uint8_t* data, size_t size);

// ==================== 变长帧 DMA TX ====================
//! 初始化变长帧 DMA TX 控制块。
//! 只绑定句柄、两块发送缓冲和完成回调；真正配置 TX DMA 由
//! STM32UARTFrameTx_SetTxDMA() 完成。
err_t STM32UARTFrameTx_Init(STM32UARTFrameTx_t* self,
                            UART_HandleTypeDef* uart_handle,
                            BSP_UART_RawData_t dma_buff_0,
                            BSP_UART_RawData_t dma_buff_1,
                            STM32UART_TxCompleteCallback_t callback);

//! 配置 TX DMA。
//! 将 TX DMA 配置为 DMA_NORMAL，并复位双缓冲发送状态。
err_t STM32UARTFrameTx_SetTxDMA(STM32UARTFrameTx_t* self);

//! 写入一帧待发送数据。
//! DMA 空闲时立即 Flush；忙时排队一帧；pending 已满返回 BUSY，不覆盖旧帧。
err_t STM32UARTFrameTx_Write(STM32UARTFrameTx_t* self, const uint8_t* data,
                             size_t size);

//! 将 pending 数据提交给 HAL_UART_Transmit_DMA()。
//! 若当前 DMA 正忙会返回 BUSY，调用方不应在中断里阻塞等待。
err_t STM32UARTFrameTx_Flush(STM32UARTFrameTx_t* self);

//! 更新变长帧 TX 完成回调。
void STM32UARTFrameTx_SetTxCompleteCallback(
    STM32UARTFrameTx_t* self, STM32UART_TxCompleteCallback_t callback);

//! 读取变长帧 TX 最近一次错误码。
err_t STM32UARTFrameTx_GetLastError(const STM32UARTFrameTx_t* self);

//! 处理变长帧 UART TC 完成事件。
//! HAL_UART_TxCpltCallback() 调用该函数，切换 active buffer 并尝试续发 pending
//! 数据。
void STM32UARTFrameTx_HandleTxComplete(STM32UARTFrameTx_t* self);

// ==================== 硬件双缓冲 DMA TX ====================
//! 绑定两块等长、非重叠的 DMA 缓冲（1..65535 字节）和空闲页填充回调。
//! 与变长帧接口互斥，同一 UART 只能注册一个 TX 对象。
err_t STM32UARTDoubleBufTx_Init(STM32UARTDoubleBufTx_t* self,
                                UART_HandleTypeDef* uart_handle,
                                BSP_UART_RawData_t dma_buff_0,
                                BSP_UART_RawData_t dma_buff_1,
                                STM32UART_TxRefillCallback_t callback);

//! 配置 DMA_CIRCULAR、DBM、M0AR/M1AR 和固定 NDTR，暂不产生 UART DMA 请求。
err_t STM32UARTDoubleBufTx_SetTxDMA(STM32UARTDoubleBufTx_t* self);

//! 空闲时将一个完整定长页复制到 M0/M1 并启动连续发送；运行中返回 BUSY。
//! 运行时由空闲页回调提供新内容，任务不得直接修改 DMA 页。
err_t STM32UARTDoubleBufTx_Write(STM32UARTDoubleBufTx_t* self,
                                 const uint8_t* data, size_t size);

//! 启动连续 DBM；调用前两页必须均已填满有效数据。不会自动停机或按 pending
//! 续发。
err_t STM32UARTDoubleBufTx_Flush(STM32UARTDoubleBufTx_t* self);

//! 任务上下文且未进入临界区时停止 TX DMA，不影响 RX；可能截断当前页，已进入
//! UART 的字节仍会发出。
err_t STM32UARTDoubleBufTx_Stop(STM32UARTDoubleBufTx_t* self);

//! 更新空闲页填充回调；回调耗时和中断延迟必须小于一页发送时间。
void STM32UARTDoubleBufTx_SetTxCompleteCallback(
    STM32UARTDoubleBufTx_t* self, STM32UART_TxRefillCallback_t callback);
//! 读取硬件 DBM 最近一次错误码。
err_t STM32UARTDoubleBufTx_GetLastError(const STM32UARTDoubleBufTx_t* self);

//! 仅由 DMA M0/M1 完成回调分发；读取 CT，通知上层填充已完成页。
void STM32UARTDoubleBufTx_HandleTxComplete(STM32UARTDoubleBufTx_t* self);

#ifdef __cplusplus
}
#endif
