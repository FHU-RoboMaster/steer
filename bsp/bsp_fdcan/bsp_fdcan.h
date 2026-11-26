#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "comp_cmd.h"
#include "comp_utils.h"
#include "fdcan.h"

// ==================== FDCAN 常量与类型描述 ====================

/**
 * @brief Classical CAN 单帧数据区最大字节数
 */
#define BSP_FDCAN_CLASSIC_DATA_SIZE (8U)

/**
 * @brief CAN FD 单帧数据区最大字节数
 */
#define BSP_FDCAN_DATA_SIZE (64U)

/**
 * @brief 每个 FDCAN 控制块可注册的 RX 订阅者容量
 *
 * 启动前通过 STM32FDCAN_SubscribeRx 注册；满员后返回 FULL。
 * 不支持运行期退订，也不使用堆内存。
 */
#ifndef STM32FDCAN_RX_SUBSCRIBER_CAPACITY
#define STM32FDCAN_RX_SUBSCRIBER_CAPACITY (4U)
#endif

/**
 * @brief STM32H7 共享 Message RAM 总字数上限（2560 words = 10 KB）
 */
#ifndef BSP_FDCAN_MESSAGE_RAM_WORDS_MAX
#define BSP_FDCAN_MESSAGE_RAM_WORDS_MAX (2560U)
#endif

/**
 * @brief FDCAN BSP 逻辑设备编号
 *
 * 随芯片头文件启用的 FDCAN 宏展开生成枚举项，用于索引对象表
 * stm32_fdcan_map[]。BSP_FDCAN_NUMBER 为合法 ID 数量；
 * BSP_FDCAN_ID_ERROR 表示无效外设地址。
 */
typedef enum {
#ifdef FDCAN1
  BSP_FDCAN1,
#endif
#ifdef FDCAN2
  BSP_FDCAN2,
#endif
#ifdef FDCAN3
  BSP_FDCAN3,
#endif
  BSP_FDCAN_NUMBER,   //!< 合法 FDCAN 数量（作数组上界）
  BSP_FDCAN_ID_ERROR  //!< 无效 ID / 未识别外设
} BSP_FDCAN_t;

/**
 * @brief FDCAN 接收帧快照（RX 路径构造，也可供测试/上层注入）
 *
 * 在 HAL_FDCAN_GetRxMessage 成功后由 ISR 填充；
 * data_ 仅包含 size_ 有效字节，剩余字节已清零。
 * 回调内指针仅本次有效，延后处理须先拷贝到自有缓冲区。
 */
typedef struct {
  uint32_t id_;          //!< 帧 ID：标准帧为 11-bit，扩展帧为 29-bit
  uint32_t id_type_;     //!< 标识符类型：FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  uint32_t frame_type_;  //!< 帧类型：FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME
  uint32_t fd_format_;   //!< 帧格式：FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  uint32_t brs_;         //!< 比特率切换：FDCAN_BRS_OFF / FDCAN_BRS_ON
  uint32_t fifo_;        //!< 来源 FIFO：FDCAN_RX_FIFO0 / FDCAN_RX_FIFO1
  uint8_t size_;         //!< 有效数据长度：Classic 0..8，FD 0..64
  uint8_t data_[BSP_FDCAN_DATA_SIZE];  //!< 数据区；无效尾部字节已清零
} BSP_FDCAN_Frame_t;

/**
 * @brief FDCAN 通用发送帧描述
 *
 * 同时支持 Classical CAN（0..8B，标准/扩展，数据/远程）与
 * CAN FD（0..64B，标准/扩展，数据帧，可选 BRS）。
 * CAN FD 发送非阶梯字节数时，BSP 内部自动向上对齐至合法 FD DLC 并对补齐字节填零。
 */
typedef struct {
  uint32_t id_;          //!< 帧 ID：标准帧 ≤ 0x7FF，扩展帧 ≤ 0x1FFFFFFF
  uint32_t id_type_;     //!< FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  uint32_t frame_type_;  //!< FDCAN_DATA_FRAME / FDCAN_REMOTE_FRAME（FD 仅支持 DATA）
  uint32_t fd_format_;   //!< FDCAN_CLASSIC_CAN / FDCAN_FD_CAN
  uint32_t brs_;         //!< FDCAN_BRS_OFF / FDCAN_BRS_ON（Classic 强制 OFF）
  const uint8_t* data_;  //!< 待发数据指针；size_ == 0 或远程帧可为 NULL
  uint8_t size_;         //!< 有效字节数：Classic 0..8，FD 0..64
} BSP_FDCAN_TxFrame_t;

/**
 * @brief FDCAN 位时序配置（字段为 0 表示保持当前寄存器值不变）
 *
 * 与硬件寄存器语义对应：
 * - TSEG1 = prop_seg_ + phase_seg1_（也可直接填 phase_seg1_、prop_seg_ 置 0）
 * - 单 bit 总 TQ = 1 + TSEG1 + phase_seg2_
 * - 波特率 = FDCAN_CLK / (brp_ * (1 + TSEG1 + phase_seg2_))
 * - 采样点 = (1 + TSEG1) / (1 + TSEG1 + phase_seg2_)
 */
typedef struct {
  uint32_t brp_;         //!< 预分频：仲裁段 1..512，数据段 1..32（0 保持原值）
  uint32_t prop_seg_;    //!< 传播段 TQ（与 phase_seg1_ 之和为 TSEG1；0 保持原值）
  uint32_t phase_seg1_;  //!< 相位段 1 TQ（0 保持原值）
  uint32_t phase_seg2_;  //!< 相位段 2 TQ（0 保持原值）
  uint32_t sjw_;         //!< 同步跳宽 TQ（须 ≤ 有效 phase_seg2；0 保持原值）
} BSP_FDCAN_BitTiming_t;

/**
 * @brief FDCAN 运行模式配置
 */
typedef struct {
  bool loopback_;     //!< 内部回环模式（CCCR.TEST + TEST.LBCK + CCCR.MON）
  bool listen_only_;  //!< 只听/总线监控模式（CCCR.MON）
  bool one_shot_;     //!< 单次发送模式，禁用自动重传（CCCR.DAR）
  bool fd_enable_;    //!< 使能 CAN FD 帧格式（CCCR.FDOE）
  bool brs_enable_;   //!< 使能数据段比特率切换（CCCR.BRSE；需 fd_enable_=true）
} BSP_FDCAN_Mode_t;

/**
 * @brief FDCAN 运行期重配参数（供 STM32FDCAN_SetConfig 使用）
 */
typedef struct {
  BSP_FDCAN_BitTiming_t nominal_timing_;  //!< 仲裁相位时序（NBTP）
  BSP_FDCAN_BitTiming_t data_timing_;     //!< 数据相位时序（DBTP）
  BSP_FDCAN_Mode_t mode_;                 //!< 工作模式（apply_mode_=true 时生效）
  bool apply_mode_;                       //!< true 应用 mode_，false 保持当前模式
} BSP_FDCAN_Config_t;

/**
 * @brief FDCAN 控制器错误状态快照
 */
typedef struct {
  uint8_t tx_error_counter_;      //!< 发送错误计数 TEC
  uint8_t rx_error_counter_;      //!< 接收错误计数 REC
  bool bus_off_;                  //!< true：处于 Bus-Off 状态
  bool error_passive_;            //!< true：处于 Error Passive 状态
  bool error_warning_;            //!< true：处于 Error Warning 状态
  uint8_t last_error_code_;       //!< 仲裁段最近错误码 LEC（0..7）
  uint8_t data_last_error_code_;  //!< 数据段最近错误码 DLEC（0..7）
} BSP_FDCAN_ErrorState_t;

/** 前向声明：RX 回调参数中需要完整对象类型。 */
typedef struct STM32FDCAN STM32FDCAN_t;

/**
 * @brief RX 订阅回调类型
 *
 * 通常在 HAL/ISR 路径中调用（Rx FIFO0 / FIFO1 新消息中断）。
 * @param self    触发回调的 FDCAN 控制块
 * @param frame   指向栈上帧快照，仅本次回调有效；延后处理必须先拷贝
 * @param context 订阅时传入的用户上下文，可为 NULL
 * @note 回调必须短且非阻塞；禁止在回调内做协议状态机、日志或长时间处理。
 */
typedef void (*STM32FDCAN_RxCallback_t)(STM32FDCAN_t* self,
                                        const BSP_FDCAN_Frame_t* frame,
                                        void* context);

/**
 * @brief 单个 RX 订阅槽
 *
 * 保存回调函数与用户上下文；相同 callback + context 重复注册保持幂等。
 */
typedef struct {
  STM32FDCAN_RxCallback_t callback_;  //!< 数据到达时的用户回调（不可为 NULL）
  void* context_;                     //!< 用户上下文，生命周期须覆盖运行期
} STM32FDCAN_RxSubscriber_t;

/**
 * @brief FDCAN 控制块
 *
 * 绑定一个 HAL FDCAN 句柄，管理固定容量 RX 订阅表、启动状态与错误计数。
 * 发送不排队：硬件 TX FIFO 满立即返回 BUSY，由上层在下一周期用最新数据重试。
 * 协议打包、帧 ID 路由归业务模块，本结构不承载电机或传感器协议。
 */
struct STM32FDCAN {
  BSP_FDCAN_t id_;                     //!< 逻辑 FDCAN 编号，用于挂入对象表
  FDCAN_HandleTypeDef* fdcan_handle_;  //!< 对应 HAL FDCAN 句柄
  STM32FDCAN_RxSubscriber_t
      subscribers_[STM32FDCAN_RX_SUBSCRIBER_CAPACITY];  //!< 固定容量订阅表
  uint8_t subscriber_count_;  //!< 已注册订阅者数量 [0, CAPACITY]
  bool started_;              //!< true：已成功 Start，此后禁止再 SubscribeRx
  err_t last_error_;          //!< 最近一次操作错误码
  volatile uint32_t bus_off_count_;    //!< ISR 触发 Bus-Off 恢复累计次数
  volatile uint32_t error_irq_count_;  //!< ISR 错误状态回调累计次数
};

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 公开接口 ====================

/**
 * @brief 由硬件外设基址映射到逻辑 FDCAN ID
 * @param addr FDCAN 寄存器基址（如 FDCAN1 / FDCAN2 / FDCAN3）
 * @return 对应 BSP_FDCAN_t；未识别或 NULL 返回 BSP_FDCAN_ID_ERROR
 */
BSP_FDCAN_t BSP_FDCAN_get_id(const FDCAN_GlobalTypeDef* addr);

/**
 * @brief 通过逻辑 FDCAN ID 获取已注册的控制块
 * @param id 逻辑 FDCAN ID（如 BSP_FDCAN1 / BSP_FDCAN2 / BSP_FDCAN3）
 * @return 已初始化的 STM32FDCAN_t 指针；未注册或 ID 非法返回 NULL
 */
STM32FDCAN_t* STM32FDCAN_GetInstance(BSP_FDCAN_t id);

/**
 * @brief 绑定 FDCAN 控制块到指定 HAL 句柄并校验共享 Message RAM
 *
 * 仅完成对象初始化、Message RAM 越界/重叠校验与对象表注册，不启动外设、
 * 不配置过滤器。真正启动由 STM32FDCAN_Start() 完成。
 * @param self         FDCAN 控制块（调用方分配，生命周期须覆盖使用期）
 * @param fdcan_handle HAL FDCAN 句柄，Instance 须为已启用的 FDCAN 外设
 * @return OK 成功；PTR_NULL / NOT_FOUND / BUSY / INIT_ERR / OUT_OF_RANGE 等
 */
err_t STM32FDCAN_Init(STM32FDCAN_t* self, FDCAN_HandleTypeDef* fdcan_handle);

/**
 * @brief 在启动前注册 RX 订阅者
 *
 * 回调按注册顺序在每帧到达时依次调用。
 * 相同 callback + context 重复注册幂等返回 OK，不占用新槽位。
 * @param self     FDCAN 控制块
 * @param callback 数据到达回调，不可为 NULL
 * @param context  用户上下文，可为 NULL；生命周期须覆盖运行期
 * @return OK 成功；PTR_NULL / STATE_ERR（已 Start）/ FULL（槽位已满）
 */
err_t STM32FDCAN_SubscribeRx(STM32FDCAN_t* self,
                             STM32FDCAN_RxCallback_t callback, void* context);

/**
 * @brief 启动 FDCAN 外设并开启 RX FIFO 与错误状态中断通知
 *
 * 若尚未启动（HAL_FDCAN_STATE_READY），调用 HAL_FDCAN_Start()；
 * 随后按已分配的 FIFO 激活新消息中断以及 Bus-Off/ErrorPassive/ErrorWarning 通知。
 * 成功后 started_ 置 true，此后 SubscribeRx 将返回 STATE_ERR。
 * @param self FDCAN 控制块
 * @return OK 成功；PTR_NULL / INIT_ERR 等
 */
err_t STM32FDCAN_Start(STM32FDCAN_t* self);

/**
 * @brief 配置单条 HAL FDCAN 过滤器
 *
 * 校验索引未超出句柄的 StdFiltersNbr/ExtFiltersNbr 后透传 HAL_FDCAN_ConfigFilter。
 * @param self   FDCAN 控制块
 * @param filter HAL 过滤器配置，不可为 NULL
 * @return OK 成功；PTR_NULL / OUT_OF_RANGE / FAILED
 */
err_t STM32FDCAN_ConfigFilter(STM32FDCAN_t* self,
                              const FDCAN_FilterTypeDef* filter);

/**
 * @brief 配置 FDCAN 全局过滤器（未匹配帧与远程帧策略）
 *
 * 必须在 STM32FDCAN_Start() 之前（HAL_FDCAN_STATE_READY）调用。
 * @param self              FDCAN 控制块
 * @param non_matching_std  未匹配标准帧策略（FDCAN_ACCEPT_IN_RX_FIFO0/1 / FDCAN_REJECT）
 * @param non_matching_ext  未匹配扩展帧策略（FDCAN_ACCEPT_IN_RX_FIFO0/1 / FDCAN_REJECT）
 * @param reject_remote_std 远程标准帧策略（FDCAN_FILTER_REMOTE / FDCAN_REJECT_REMOTE）
 * @param reject_remote_ext 远程扩展帧策略（FDCAN_FILTER_REMOTE / FDCAN_REJECT_REMOTE）
 * @return OK 成功；PTR_NULL / STATE_ERR / FAILED
 */
err_t STM32FDCAN_ConfigGlobalFilter(STM32FDCAN_t* self,
                                    uint32_t non_matching_std,
                                    uint32_t non_matching_ext,
                                    uint32_t reject_remote_std,
                                    uint32_t reject_remote_ext);

/**
 * @brief 便捷配置全通接收过滤器并路由到指定 RX FIFO
 *
 * 在 Start 前调用：若句柄分配了标准/扩展过滤器槽则配置全零掩码全通，
 * 并通过全局过滤器将未匹配的标准/扩展帧均路由到 rx_fifo。
 * @param self    FDCAN 控制块
 * @param rx_fifo 目标 FIFO：FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1
 * @return OK 成功；PTR_NULL / ARG_ERR / STATE_ERR / FAILED
 */
err_t STM32FDCAN_ConfigAcceptAllFilter(STM32FDCAN_t* self, uint32_t rx_fifo);

/**
 * @brief 运行期重配 FDCAN 仲裁/数据位时序及工作模式
 *
 * 先完整校验参数合法性，再执行停机（若已运行）→ 进 INIT/CCE → 写 CCCR/TEST/NBTP/DBTP
 * → 同步句柄 Init 字段 → 恢复运行与中断通知（若原本处于运行态）。
 * 任一参数非法均在修改硬件前返回 ARG_ERR，不会导致外设停机。
 * @param self FDCAN 控制块
 * @param cfg  重配参数，不可为 NULL；位时序字段填 0 表示保持原值
 * @return OK 成功；PTR_NULL / ARG_ERR / STATE_ERR / FAILED
 */
err_t STM32FDCAN_SetConfig(STM32FDCAN_t* self, const BSP_FDCAN_Config_t* cfg);

/**
 * @brief 通过控制块发送一帧通用 FDCAN/Classic 帧（无排队）
 *
 * 硬件 TX FIFO 满立即返回 BUSY，不排队、不覆盖、不自动重试。
 * @param self  FDCAN 控制块
 * @param frame 待发帧描述，不可为 NULL
 * @return OK / BUSY / PTR_NULL / ARG_ERR / SIZE_ERR / OUT_OF_RANGE / FAILED
 */
err_t STM32FDCAN_SendFrame(STM32FDCAN_t* self,
                           const BSP_FDCAN_TxFrame_t* frame);

/**
 * @brief 便捷发送一帧 Classical CAN 标准数据帧（11-bit ID，0..8 字节）
 * @param self   FDCAN 控制块
 * @param std_id 标准帧 ID，须 ≤ 0x7FF
 * @param data   待发数据指针（size > 0 时不可为 NULL）
 * @param size   字节数，0..BSP_FDCAN_CLASSIC_DATA_SIZE
 * @return 见 STM32FDCAN_SendFrame
 */
err_t STM32FDCAN_SendStd(STM32FDCAN_t* self, uint32_t std_id,
                         const uint8_t* data, size_t size);

/**
 * @brief 便捷发送一帧 CAN FD 数据帧（0..64 字节）
 * @param self    FDCAN 控制块
 * @param id      帧 ID（标准帧 ≤ 0x7FF，扩展帧 ≤ 0x1FFFFFFF）
 * @param id_type FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
 * @param brs     true 开启数据段加速（FDCAN_BRS_ON），false 关闭
 * @param data    待发数据指针（size > 0 时不可为 NULL）
 * @param size    字节数，0..BSP_FDCAN_DATA_SIZE
 * @return 见 STM32FDCAN_SendFrame
 */
err_t STM32FDCAN_SendFD(STM32FDCAN_t* self, uint32_t id, uint32_t id_type,
                        bool brs, const uint8_t* data, size_t size);

/**
 * @brief 通过已注册的 HAL 句柄适配发送 Classical CAN 标准数据帧
 *
 * 先按句柄反查已 Init 的控制块，再转发 STM32FDCAN_SendStd()。
 * @param fdcan_handle HAL FDCAN 句柄
 * @param std_id       标准帧 ID
 * @param data         待发数据
 * @param size         字节数（0..8）
 * @return 见 STM32FDCAN_SendStd；未注册返回 NOT_FOUND
 */
err_t STM32FDCAN_SendStdByHandle(FDCAN_HandleTypeDef* fdcan_handle,
                                 uint32_t std_id, const uint8_t* data,
                                 size_t size);

/**
 * @brief 向全部订阅者广播一帧 RX 快照
 *
 * 通常由内部 ISR 路径调用；也可供测试/上层注入使用。
 * @param self  FDCAN 控制块
 * @param frame 帧快照，不可为 NULL
 */
void STM32FDCAN_HandleRxFrame(STM32FDCAN_t* self,
                              const BSP_FDCAN_Frame_t* frame);

/**
 * @brief 查询 FDCAN 外设核时钟频率（Hz）
 * @param self FDCAN 控制块
 * @return 核时钟频率（Hz）；self 无效时返回 0
 */
uint32_t STM32FDCAN_GetClockFreq(const STM32FDCAN_t* self);

/**
 * @brief 查询当前硬件错误计数器与协议状态快照
 * @param self  FDCAN 控制块
 * @param state 输出快照指针，不可为 NULL
 * @return OK 成功；PTR_NULL / FAILED
 */
err_t STM32FDCAN_GetErrorState(const STM32FDCAN_t* self,
                               BSP_FDCAN_ErrorState_t* state);

/**
 * @brief 读取控制块最近一次操作错误码
 * @param self FDCAN 控制块
 * @return last_error_；self 为 NULL 时返回 PTR_NULL
 */
err_t STM32FDCAN_GetLastError(const STM32FDCAN_t* self);

#ifdef __cplusplus
}
#endif
