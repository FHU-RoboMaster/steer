#include "bsp_fdcan.h"

#include <stdatomic.h>
#include <string.h>

#include "bsp_log.h"

// ==================== 内部常量与对象表 ====================

/** 等待 CCCR.INIT 置位的最大轮询次数，避免异常硬件状态导致死等。 */
#define BSP_FDCAN_INIT_TIMEOUT_LOOP (100000U)

/** 单次 RX FIFO 中断最大连续取帧上限（与 H7 FIFO 最大深度 64 对齐）。 */
#define BSP_FDCAN_RX_DRAIN_MAX (64U)

/**
 * 按逻辑 ID 索引的 FDCAN 控制块指针表；Init 时注册，HAL 回调时反查。
 * 下标范围为 [0, BSP_FDCAN_NUMBER)。
 */
static STM32FDCAN_t* stm32_fdcan_map[BSP_FDCAN_NUMBER] = {0};

#if defined(FDCAN_DATA_BYTES_8)
/**
 * STM32H7 共享 Message RAM 占用表：在 Init 阶段校验越界与多实例区域重叠。
 */
typedef struct {
  bool inited_;
  uint32_t offset_;
  uint32_t size_;
} BSP_FDCAN_RamSlot_t;

static BSP_FDCAN_RamSlot_t bsp_fdcan_ram_map[BSP_FDCAN_NUMBER] = {0};
#endif

/* 每条 FDCAN 总线的发送故障只报告一次；成功发送后清除故障标记。 */
#if BSP_LOG_ENABLED
static _Atomic uint8_t bsp_fdcan_tx_fault_reported;
#endif

// ==================== 内部辅助函数 ====================

/**
 * @brief 硬件外设基址 → 逻辑 FDCAN ID
 */
BSP_FDCAN_t BSP_FDCAN_get_id(const FDCAN_GlobalTypeDef* addr) {
  if (addr == NULL) {
    return BSP_FDCAN_ID_ERROR;
  }

#ifdef FDCAN1
  if (addr == FDCAN1) {
    return BSP_FDCAN1;
  }
#endif
#ifdef FDCAN2
  if (addr == FDCAN2) {
    return BSP_FDCAN2;
  }
#endif
#ifdef FDCAN3
  if (addr == FDCAN3) {
    return BSP_FDCAN3;
  }
#endif

  return BSP_FDCAN_ID_ERROR;
}

/**
 * @brief 判断逻辑 FDCAN ID 是否合法
 */
static bool BSP_FDCAN_is_valid_id(BSP_FDCAN_t id) {
  return (id != BSP_FDCAN_ID_ERROR) && (id < BSP_FDCAN_NUMBER);
}

/**
 * @brief 通过逻辑 FDCAN ID 获取已注册的控制块
 */
STM32FDCAN_t* STM32FDCAN_GetInstance(BSP_FDCAN_t id) {
  if (!BSP_FDCAN_is_valid_id(id)) {
    return NULL;
  }
  return stm32_fdcan_map[id];
}

/**
 * @brief 由 HAL 句柄反查已注册的 FDCAN 控制块
 *
 * 除 ID 合法外，还要求 map 中对象的 fdcan_handle_ 与入参一致，
 * 避免句柄复用或错误绑定导致误分发。
 */
static STM32FDCAN_t* BSP_FDCAN_get_object(
    const FDCAN_HandleTypeDef* fdcan_handle) {
  if ((fdcan_handle == NULL) || (fdcan_handle->Instance == NULL)) {
    return NULL;
  }

  const BSP_FDCAN_t id = BSP_FDCAN_get_id(fdcan_handle->Instance);
  if (!BSP_FDCAN_is_valid_id(id)) {
    return NULL;
  }

  STM32FDCAN_t* self = stm32_fdcan_map[id];
  if ((self == NULL) || (self->fdcan_handle_ != fdcan_handle)) {
    return NULL;
  }
  return self;
}

/**
 * @brief 有效字节数 → HAL FDCAN DLC 宏（向上对齐到合法 CAN FD DLC 阶梯）
 */
static bool BSP_FDCAN_bytes_to_dlc(uint8_t bytes, uint32_t* dlc_out,
                                   uint8_t* wire_bytes_out) {
  if ((dlc_out == NULL) || (wire_bytes_out == NULL) ||
      (bytes > BSP_FDCAN_DATA_SIZE)) {
    return false;
  }

  static const uint32_t k_small_dlc[9] = {
      FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
      FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
      FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};

  if (bytes <= 8U) {
    *dlc_out = k_small_dlc[bytes];
    *wire_bytes_out = bytes;
    return true;
  }
  if (bytes <= 12U) {
    *dlc_out = FDCAN_DLC_BYTES_12;
    *wire_bytes_out = 12U;
    return true;
  }
  if (bytes <= 16U) {
    *dlc_out = FDCAN_DLC_BYTES_16;
    *wire_bytes_out = 16U;
    return true;
  }
  if (bytes <= 20U) {
    *dlc_out = FDCAN_DLC_BYTES_20;
    *wire_bytes_out = 20U;
    return true;
  }
  if (bytes <= 24U) {
    *dlc_out = FDCAN_DLC_BYTES_24;
    *wire_bytes_out = 24U;
    return true;
  }
  if (bytes <= 32U) {
    *dlc_out = FDCAN_DLC_BYTES_32;
    *wire_bytes_out = 32U;
    return true;
  }
  if (bytes <= 48U) {
    *dlc_out = FDCAN_DLC_BYTES_48;
    *wire_bytes_out = 48U;
    return true;
  }
  *dlc_out = FDCAN_DLC_BYTES_64;
  *wire_bytes_out = 64U;
  return true;
}

/**
 * @brief HAL FDCAN DLC 宏 → 有效字节数
 */
static bool BSP_FDCAN_dlc_to_bytes(uint32_t dlc, uint8_t* bytes_out) {
  if (bytes_out == NULL) {
    return false;
  }

  switch (dlc) {
    case FDCAN_DLC_BYTES_0:
      *bytes_out = 0U;
      return true;
    case FDCAN_DLC_BYTES_1:
      *bytes_out = 1U;
      return true;
    case FDCAN_DLC_BYTES_2:
      *bytes_out = 2U;
      return true;
    case FDCAN_DLC_BYTES_3:
      *bytes_out = 3U;
      return true;
    case FDCAN_DLC_BYTES_4:
      *bytes_out = 4U;
      return true;
    case FDCAN_DLC_BYTES_5:
      *bytes_out = 5U;
      return true;
    case FDCAN_DLC_BYTES_6:
      *bytes_out = 6U;
      return true;
    case FDCAN_DLC_BYTES_7:
      *bytes_out = 7U;
      return true;
    case FDCAN_DLC_BYTES_8:
      *bytes_out = 8U;
      return true;
    case FDCAN_DLC_BYTES_12:
      *bytes_out = 12U;
      return true;
    case FDCAN_DLC_BYTES_16:
      *bytes_out = 16U;
      return true;
    case FDCAN_DLC_BYTES_20:
      *bytes_out = 20U;
      return true;
    case FDCAN_DLC_BYTES_24:
      *bytes_out = 24U;
      return true;
    case FDCAN_DLC_BYTES_32:
      *bytes_out = 32U;
      return true;
    case FDCAN_DLC_BYTES_48:
      *bytes_out = 48U;
      return true;
    case FDCAN_DLC_BYTES_64:
      *bytes_out = 64U;
      return true;
    default:
      return false;
  }
}

#if defined(FDCAN_DATA_BYTES_8)
/**
 * @brief STM32H7 Message RAM 单个元素占用的 32-bit word 数（2 word 头 + 数据）
 */
static uint32_t BSP_FDCAN_elmt_words(uint32_t elmt_size) {
  switch (elmt_size) {
    case FDCAN_DATA_BYTES_8:
      return 4U;
    case FDCAN_DATA_BYTES_12:
      return 5U;
    case FDCAN_DATA_BYTES_16:
      return 6U;
    case FDCAN_DATA_BYTES_20:
      return 7U;
    case FDCAN_DATA_BYTES_24:
      return 8U;
    case FDCAN_DATA_BYTES_32:
      return 10U;
    case FDCAN_DATA_BYTES_48:
      return 14U;
    case FDCAN_DATA_BYTES_64:
      return 18U;
    default:
      return 0U;
  }
}

/**
 * @brief STM32H7 FDCAN_DATA_BYTES_* 对应的最大载荷字节数
 */
static uint8_t BSP_FDCAN_elmt_max_bytes(uint32_t elmt_size) {
  switch (elmt_size) {
    case FDCAN_DATA_BYTES_8:
      return 8U;
    case FDCAN_DATA_BYTES_12:
      return 12U;
    case FDCAN_DATA_BYTES_16:
      return 16U;
    case FDCAN_DATA_BYTES_20:
      return 20U;
    case FDCAN_DATA_BYTES_24:
      return 24U;
    case FDCAN_DATA_BYTES_32:
      return 32U;
    case FDCAN_DATA_BYTES_48:
      return 48U;
    case FDCAN_DATA_BYTES_64:
      return 64U;
    default:
      return 0U;
  }
}

/**
 * @brief 校验 STM32H7 共享 Message RAM 区域是否越界或与其他实例重叠
 */
static err_t BSP_FDCAN_check_message_ram(
    BSP_FDCAN_t id, const FDCAN_HandleTypeDef* fdcan_handle) {
  const FDCAN_InitTypeDef* c = &fdcan_handle->Init;

  /* 若收发 FIFO 全为 0，说明 CubeMX 尚未为本实例分配任何收发缓存。 */
  if ((c->TxFifoQueueElmtsNbr == 0U) && (c->RxFifo0ElmtsNbr == 0U) &&
      (c->RxFifo1ElmtsNbr == 0U)) {
    LOG_ERROR("FDCAN", "FDCAN%u Message RAM 未分配收发 FIFO 元素",
              (unsigned)id + 1U);
    return INIT_ERR;
  }

  const uint32_t rx0_w =
      (c->RxFifo0ElmtsNbr > 0U) ? BSP_FDCAN_elmt_words(c->RxFifo0ElmtSize) : 0U;
  const uint32_t rx1_w =
      (c->RxFifo1ElmtsNbr > 0U) ? BSP_FDCAN_elmt_words(c->RxFifo1ElmtSize) : 0U;
  const uint32_t rxb_w =
      (c->RxBuffersNbr > 0U) ? BSP_FDCAN_elmt_words(c->RxBufferSize) : 0U;
  const uint32_t tx_elmt_cnt = c->TxBuffersNbr + c->TxFifoQueueElmtsNbr;
  const uint32_t tx_w =
      (tx_elmt_cnt > 0U) ? BSP_FDCAN_elmt_words(c->TxElmtSize) : 0U;

  if (((c->RxFifo0ElmtsNbr > 0U) && (rx0_w == 0U)) ||
      ((c->RxFifo1ElmtsNbr > 0U) && (rx1_w == 0U)) ||
      ((c->RxBuffersNbr > 0U) && (rxb_w == 0U)) ||
      ((tx_elmt_cnt > 0U) && (tx_w == 0U))) {
    LOG_ERROR("FDCAN", "FDCAN%u Message RAM 元素尺寸配置非法",
              (unsigned)id + 1U);
    return ARG_ERR;
  }

  const uint32_t words = c->StdFiltersNbr * 1U + c->ExtFiltersNbr * 2U +
                         c->RxFifo0ElmtsNbr * rx0_w +
                         c->RxFifo1ElmtsNbr * rx1_w + c->RxBuffersNbr * rxb_w +
                         c->TxEventsNbr * 2U + tx_elmt_cnt * tx_w;

  const uint32_t start = c->MessageRAMOffset;
  if ((start > BSP_FDCAN_MESSAGE_RAM_WORDS_MAX) ||
      (words > (BSP_FDCAN_MESSAGE_RAM_WORDS_MAX - start))) {
    LOG_ERROR("FDCAN",
              "FDCAN%u Message RAM 越界 offset=%u words=%u 上限=%u",
              (unsigned)id + 1U, (unsigned)start, (unsigned)words,
              (unsigned)BSP_FDCAN_MESSAGE_RAM_WORDS_MAX);
    return OUT_OF_RANGE;
  }
  const uint32_t end = start + words;

  for (uint32_t idx = 0U; idx < (uint32_t)BSP_FDCAN_NUMBER; ++idx) {
    if ((idx == (uint32_t)id) || !bsp_fdcan_ram_map[idx].inited_) {
      continue;
    }
    const uint32_t a0 = bsp_fdcan_ram_map[idx].offset_;
    const uint32_t a1 = a0 + bsp_fdcan_ram_map[idx].size_;
    if ((a0 < end) && (start < a1)) {
      LOG_ERROR("FDCAN",
                "FDCAN%u[%u,%u) 与 FDCAN%u[%u,%u) Message RAM 重叠",
                (unsigned)id + 1U, (unsigned)start, (unsigned)end,
                (unsigned)idx + 1U, (unsigned)a0, (unsigned)a1);
      return INIT_ERR;
    }
  }

  bsp_fdcan_ram_map[id].offset_ = start;
  bsp_fdcan_ram_map[id].size_ = words;
  bsp_fdcan_ram_map[id].inited_ = true;
  return OK;
}
#endif

/**
 * @brief 计算当前句柄需要开启的中断掩码
 */
static uint32_t BSP_FDCAN_build_notification_mask(
    const FDCAN_HandleTypeDef* fdcan_handle) {
  uint32_t mask =
      FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING;

  if (fdcan_handle->Init.RxFifo0ElmtsNbr > 0U) {
    mask |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
  }
  if (fdcan_handle->Init.RxFifo1ElmtsNbr > 0U) {
    mask |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
  }
  return mask;
}

/**
 * @brief 启动 HAL FDCAN 并激活 RX FIFO 与错误状态中断
 */
static err_t BSP_FDCAN_start_handle(FDCAN_HandleTypeDef* fdcan_handle) {
  if ((fdcan_handle == NULL) || (fdcan_handle->Instance == NULL)) {
    return PTR_NULL;
  }

  if (fdcan_handle->State == HAL_FDCAN_STATE_READY) {
    if (HAL_FDCAN_Start(fdcan_handle) != HAL_OK) {
      return INIT_ERR;
    }
  } else if (fdcan_handle->State != HAL_FDCAN_STATE_BUSY) {
    return INIT_ERR;
  }

  const uint32_t active_its = BSP_FDCAN_build_notification_mask(fdcan_handle);
  return (HAL_FDCAN_ActivateNotification(fdcan_handle, active_its, 0U) ==
          HAL_OK)
             ? OK
             : INIT_ERR;
}

/**
 * @brief 发送一帧通用 FDCAN/Classic 帧（无排队）
 */
static err_t BSP_FDCAN_send_frame_hw(FDCAN_HandleTypeDef* fdcan_handle,
                                     const BSP_FDCAN_TxFrame_t* frame) {
  if ((fdcan_handle == NULL) || (fdcan_handle->Instance == NULL) ||
      (frame == NULL)) {
    return PTR_NULL;
  }
  if (fdcan_handle->State != HAL_FDCAN_STATE_BUSY) {
    return STATE_ERR;
  }

  /* 校验 ID 类型与范围 */
  if (frame->id_type_ == FDCAN_STANDARD_ID) {
    if (frame->id_ > 0x7FFU) {
      return OUT_OF_RANGE;
    }
  } else if (frame->id_type_ == FDCAN_EXTENDED_ID) {
    if (frame->id_ > 0x1FFFFFFFU) {
      return OUT_OF_RANGE;
    }
  } else {
    return ARG_ERR;
  }

  /* 校验帧类型、格式与 BRS */
  if ((frame->frame_type_ != FDCAN_DATA_FRAME) &&
      (frame->frame_type_ != FDCAN_REMOTE_FRAME)) {
    return ARG_ERR;
  }
  if ((frame->brs_ != FDCAN_BRS_OFF) && (frame->brs_ != FDCAN_BRS_ON)) {
    return ARG_ERR;
  }

  if (frame->fd_format_ == FDCAN_CLASSIC_CAN) {
    if (frame->size_ > BSP_FDCAN_CLASSIC_DATA_SIZE) {
      return OUT_OF_RANGE;
    }
    if (frame->brs_ != FDCAN_BRS_OFF) {
      return ARG_ERR;
    }
  } else if (frame->fd_format_ == FDCAN_FD_CAN) {
    if (frame->frame_type_ != FDCAN_DATA_FRAME) {
      return ARG_ERR;
    }
    if (frame->size_ > BSP_FDCAN_DATA_SIZE) {
      return OUT_OF_RANGE;
    }
#ifdef FDCAN_CCCR_FDOE
    if ((fdcan_handle->Instance->CCCR & FDCAN_CCCR_FDOE) == 0U) {
      return STATE_ERR;
    }
#endif
#ifdef FDCAN_CCCR_BRSE
    if ((frame->brs_ == FDCAN_BRS_ON) &&
        ((fdcan_handle->Instance->CCCR & FDCAN_CCCR_BRSE) == 0U)) {
      return STATE_ERR;
    }
#endif
  } else {
    return ARG_ERR;
  }

  if ((frame->frame_type_ == FDCAN_DATA_FRAME) && (frame->size_ > 0U) &&
      (frame->data_ == NULL)) {
    return PTR_NULL;
  }

  uint32_t dlc = 0U;
  uint8_t wire_bytes = 0U;
  if (!BSP_FDCAN_bytes_to_dlc(frame->size_, &dlc, &wire_bytes)) {
    return OUT_OF_RANGE;
  }

#if defined(FDCAN_DATA_BYTES_8)
  if (fdcan_handle->Init.TxFifoQueueElmtsNbr == 0U) {
    return INIT_ERR;
  }
  const uint8_t max_elmt_bytes =
      BSP_FDCAN_elmt_max_bytes(fdcan_handle->Init.TxElmtSize);
  if ((max_elmt_bytes == 0U) || (wire_bytes > max_elmt_bytes)) {
    return SIZE_ERR;
  }
#endif

  if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan_handle) == 0U) {
    return BUSY;
  }

  FDCAN_TxHeaderTypeDef tx_header = {0};
  tx_header.Identifier = frame->id_;
  tx_header.IdType = frame->id_type_;
  tx_header.TxFrameType = frame->frame_type_;
  tx_header.DataLength = dlc;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = frame->brs_;
  tx_header.FDFormat = frame->fd_format_;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  /* 当需要补零对齐（非阶梯 FD 长度）或无数据指针时使用零初始化局部缓冲，
   * 避免 HAL 按 wire_bytes 拷贝时越界读取调用方缓冲区。 */
  uint8_t padded_buf[BSP_FDCAN_DATA_SIZE];
  const uint8_t* tx_ptr = frame->data_;
  if ((frame->frame_type_ == FDCAN_REMOTE_FRAME) || (frame->size_ == 0U) ||
      (wire_bytes != frame->size_)) {
    memset(padded_buf, 0, wire_bytes);
    if ((frame->frame_type_ == FDCAN_DATA_FRAME) && (frame->size_ > 0U)) {
      memcpy(padded_buf, frame->data_, frame->size_);
    }
    tx_ptr = padded_buf;
  }

  return (HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle, &tx_header, tx_ptr) ==
          HAL_OK)
             ? OK
             : FAILED;
}

// ==================== 对象初始化与订阅 ====================

/**
 * @brief 初始化 FDCAN 控制块、校验 Message RAM 并注册到对象表
 */
err_t STM32FDCAN_Init(STM32FDCAN_t* self, FDCAN_HandleTypeDef* fdcan_handle) {
  if (self == NULL) {
    LOG_ERROR("FDCAN", "初始化失败：对象为空");
    return PTR_NULL;
  }

  self->last_error_ = PENDING;
  if ((fdcan_handle == NULL) || (fdcan_handle->Instance == NULL)) {
    self->last_error_ = PTR_NULL;
    LOG_ERROR("FDCAN", "初始化失败：HAL句柄无效");
    return self->last_error_;
  }

  const BSP_FDCAN_t id = BSP_FDCAN_get_id(fdcan_handle->Instance);
  if (!BSP_FDCAN_is_valid_id(id)) {
    self->last_error_ = NOT_FOUND;
    LOG_ERROR("FDCAN", "初始化失败：不支持的FDCAN外设");
    return self->last_error_;
  }
  if ((stm32_fdcan_map[id] != NULL) && (stm32_fdcan_map[id] != self)) {
    self->last_error_ = BUSY;
    LOG_ERROR("FDCAN", "FDCAN%u 初始化失败：已被其他对象注册",
              (unsigned)id + 1U);
    return self->last_error_;
  }

#if defined(FDCAN_DATA_BYTES_8)
  const err_t ram_err = BSP_FDCAN_check_message_ram(id, fdcan_handle);
  if (ram_err != OK) {
    self->last_error_ = ram_err;
    return self->last_error_;
  }
#endif

  self->id_ = id;
  self->fdcan_handle_ = fdcan_handle;
  memset(self->subscribers_, 0, sizeof(self->subscribers_));
  self->subscriber_count_ = 0U;
  self->started_ = false;
  self->bus_off_count_ = 0U;
  self->error_irq_count_ = 0U;
  stm32_fdcan_map[id] = self;
  self->last_error_ = OK;
  LOG_DEBUG("FDCAN", "FDCAN%u 初始化完成", (unsigned)id + 1U);
  return self->last_error_;
}

/**
 * @brief 注册 RX 订阅者（仅 Start 前允许）
 */
err_t STM32FDCAN_SubscribeRx(STM32FDCAN_t* self,
                             STM32FDCAN_RxCallback_t callback, void* context) {
  if ((self == NULL) || (callback == NULL)) {
    LOG_ERROR("FDCAN", "接收订阅失败：对象或回调为空");
    return PTR_NULL;
  }

  for (uint8_t index = 0U; index < self->subscriber_count_; ++index) {
    const STM32FDCAN_RxSubscriber_t* subscriber = &self->subscribers_[index];
    if ((subscriber->callback_ == callback) &&
        (subscriber->context_ == context)) {
      self->last_error_ = OK;
      LOG_DEBUG("FDCAN", "接收订阅已存在 订阅数=%u",
                (unsigned)self->subscriber_count_);
      return self->last_error_;
    }
  }

  if (self->started_) {
    self->last_error_ = STATE_ERR;
    LOG_WARN("FDCAN", "拒绝接收订阅：FDCAN%u 已启动", (unsigned)self->id_ + 1U);
    return self->last_error_;
  }
  if (self->subscriber_count_ >= STM32FDCAN_RX_SUBSCRIBER_CAPACITY) {
    self->last_error_ = FULL;
    LOG_ERROR("FDCAN", "接收订阅失败：FDCAN%u 订阅表已满",
              (unsigned)self->id_ + 1U);
    return self->last_error_;
  }

  STM32FDCAN_RxSubscriber_t* subscriber =
      &self->subscribers_[self->subscriber_count_++];
  subscriber->callback_ = callback;
  subscriber->context_ = context;
  self->last_error_ = OK;
  LOG_DEBUG("FDCAN", "FDCAN%u 接收订阅添加完成 订阅数=%u",
            (unsigned)self->id_ + 1U, (unsigned)self->subscriber_count_);
  return self->last_error_;
}

// ==================== 启动、过滤器与运行期重配 ====================

/**
 * @brief 启动 FDCAN 并开启中断通知
 */
err_t STM32FDCAN_Start(STM32FDCAN_t* self) {
  if (self == NULL) {
    LOG_ERROR("FDCAN", "启动失败：对象为空");
    return PTR_NULL;
  }

  self->last_error_ = BSP_FDCAN_start_handle(self->fdcan_handle_);
  if (self->last_error_ == OK) {
    self->started_ = true;
    LOG_DEBUG("FDCAN", "FDCAN%u 启动完成", (unsigned)self->id_ + 1U);
  } else {
    LOG_ERROR("FDCAN", "FDCAN%u 启动失败 错误码=%d", (unsigned)self->id_ + 1U,
              (int)self->last_error_);
  }
  return self->last_error_;
}

/**
 * @brief 配置单条 HAL FDCAN 过滤器
 */
err_t STM32FDCAN_ConfigFilter(STM32FDCAN_t* self,
                              const FDCAN_FilterTypeDef* filter) {
  if ((self == NULL) || (filter == NULL)) {
    LOG_ERROR("FDCAN", "过滤器配置失败：参数为空");
    return PTR_NULL;
  }
  if (self->fdcan_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    LOG_ERROR("FDCAN", "过滤器配置失败：HAL句柄为空");
    return self->last_error_;
  }

  if (filter->IdType == FDCAN_STANDARD_ID) {
    if (filter->FilterIndex >= self->fdcan_handle_->Init.StdFiltersNbr) {
      self->last_error_ = OUT_OF_RANGE;
      LOG_ERROR("FDCAN", "FDCAN%u 标准过滤器索引越界 idx=%u nbr=%u",
                (unsigned)self->id_ + 1U, (unsigned)filter->FilterIndex,
                (unsigned)self->fdcan_handle_->Init.StdFiltersNbr);
      return self->last_error_;
    }
  } else if (filter->IdType == FDCAN_EXTENDED_ID) {
    if (filter->FilterIndex >= self->fdcan_handle_->Init.ExtFiltersNbr) {
      self->last_error_ = OUT_OF_RANGE;
      LOG_ERROR("FDCAN", "FDCAN%u 扩展过滤器索引越界 idx=%u nbr=%u",
                (unsigned)self->id_ + 1U, (unsigned)filter->FilterIndex,
                (unsigned)self->fdcan_handle_->Init.ExtFiltersNbr);
      return self->last_error_;
    }
  } else {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }

  self->last_error_ =
      (HAL_FDCAN_ConfigFilter(self->fdcan_handle_, filter) == HAL_OK) ? OK
                                                                      : FAILED;
  if (self->last_error_ == OK) {
    LOG_DEBUG("FDCAN", "FDCAN%u 过滤器配置完成", (unsigned)self->id_ + 1U);
  } else {
    LOG_ERROR("FDCAN", "FDCAN%u 过滤器配置失败", (unsigned)self->id_ + 1U);
  }
  return self->last_error_;
}

/**
 * @brief 配置 FDCAN 全局过滤器（须在 Start 前调用）
 */
err_t STM32FDCAN_ConfigGlobalFilter(STM32FDCAN_t* self,
                                    uint32_t non_matching_std,
                                    uint32_t non_matching_ext,
                                    uint32_t reject_remote_std,
                                    uint32_t reject_remote_ext) {
  if (self == NULL) {
    LOG_ERROR("FDCAN", "全局过滤器配置失败：对象为空");
    return PTR_NULL;
  }
  if (self->fdcan_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }
  if (self->fdcan_handle_->State != HAL_FDCAN_STATE_READY) {
    self->last_error_ = STATE_ERR;
    LOG_WARN("FDCAN", "FDCAN%u 全局过滤器须在 Start 前配置",
             (unsigned)self->id_ + 1U);
    return self->last_error_;
  }

  self->last_error_ =
      (HAL_FDCAN_ConfigGlobalFilter(self->fdcan_handle_, non_matching_std,
                                    non_matching_ext, reject_remote_std,
                                    reject_remote_ext) == HAL_OK)
          ? OK
          : FAILED;
  return self->last_error_;
}

/**
 * @brief 便捷配置全通过滤器并路由到指定 RX FIFO
 */
err_t STM32FDCAN_ConfigAcceptAllFilter(STM32FDCAN_t* self, uint32_t rx_fifo) {
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->fdcan_handle_ == NULL) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  uint32_t filter_cfg = 0U;
  uint32_t non_matching = 0U;
  if (rx_fifo == FDCAN_RX_FIFO0) {
    if (self->fdcan_handle_->Init.RxFifo0ElmtsNbr == 0U) {
      self->last_error_ = INIT_ERR;
      return self->last_error_;
    }
    filter_cfg = FDCAN_FILTER_TO_RXFIFO0;
    non_matching = FDCAN_ACCEPT_IN_RX_FIFO0;
  } else if (rx_fifo == FDCAN_RX_FIFO1) {
    if (self->fdcan_handle_->Init.RxFifo1ElmtsNbr == 0U) {
      self->last_error_ = INIT_ERR;
      return self->last_error_;
    }
    filter_cfg = FDCAN_FILTER_TO_RXFIFO1;
    non_matching = FDCAN_ACCEPT_IN_RX_FIFO1;
  } else {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }

  if (self->fdcan_handle_->Init.StdFiltersNbr > 0U) {
    FDCAN_FilterTypeDef std_filter = {0};
    std_filter.IdType = FDCAN_STANDARD_ID;
    std_filter.FilterIndex = 0U;
    std_filter.FilterType = FDCAN_FILTER_MASK;
    std_filter.FilterConfig = filter_cfg;
    std_filter.FilterID1 = 0x000U;
    std_filter.FilterID2 = 0x000U;
    const err_t err = STM32FDCAN_ConfigFilter(self, &std_filter);
    if (err != OK) {
      return err;
    }
  }

  if (self->fdcan_handle_->Init.ExtFiltersNbr > 0U) {
    FDCAN_FilterTypeDef ext_filter = {0};
    ext_filter.IdType = FDCAN_EXTENDED_ID;
    ext_filter.FilterIndex = 0U;
    ext_filter.FilterType = FDCAN_FILTER_MASK;
    ext_filter.FilterConfig = filter_cfg;
    ext_filter.FilterID1 = 0x00000000U;
    ext_filter.FilterID2 = 0x00000000U;
    const err_t err = STM32FDCAN_ConfigFilter(self, &ext_filter);
    if (err != OK) {
      return err;
    }
  }

  return STM32FDCAN_ConfigGlobalFilter(self, non_matching, non_matching,
                                       FDCAN_FILTER_REMOTE,
                                       FDCAN_FILTER_REMOTE);
}

/**
 * @brief 运行期重配 FDCAN 位时序与工作模式（借鉴 LibXR SetConfig，先校验后停机）
 */
err_t STM32FDCAN_SetConfig(STM32FDCAN_t* self, const BSP_FDCAN_Config_t* cfg) {
  if ((self == NULL) || (cfg == NULL)) {
    return PTR_NULL;
  }
  FDCAN_HandleTypeDef* hfdcan = self->fdcan_handle_;
  if ((hfdcan == NULL) || (hfdcan->Instance == NULL)) {
    self->last_error_ = PTR_NULL;
    return self->last_error_;
  }

  FDCAN_GlobalTypeDef* can = hfdcan->Instance;

  /* 1. 先校验全部参数，失败时不触碰正在运行的外设。 */
  if (cfg->apply_mode_ && cfg->mode_.brs_enable_ && !cfg->mode_.fd_enable_) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }

#ifdef FDCAN_NBTP_NBRP_Msk
  const uint32_t nbrp_fmax = FDCAN_NBTP_NBRP_Msk >> FDCAN_NBTP_NBRP_Pos;
  const uint32_t nt1_fmax = FDCAN_NBTP_NTSEG1_Msk >> FDCAN_NBTP_NTSEG1_Pos;
  const uint32_t nt2_fmax = FDCAN_NBTP_NTSEG2_Msk >> FDCAN_NBTP_NTSEG2_Pos;
  const uint32_t nsjw_fmax = FDCAN_NBTP_NSJW_Msk >> FDCAN_NBTP_NSJW_Pos;

  const BSP_FDCAN_BitTiming_t* bt = &cfg->nominal_timing_;
  const uint32_t ntseg1 = bt->prop_seg_ + bt->phase_seg1_;
  const uint32_t cur_ntseg2 =
      ((can->NBTP & FDCAN_NBTP_NTSEG2_Msk) >> FDCAN_NBTP_NTSEG2_Pos) + 1U;
  const uint32_t eff_ntseg2 = (bt->phase_seg2_ != 0U) ? bt->phase_seg2_ : cur_ntseg2;

  if ((bt->brp_ != 0U) && (bt->brp_ > (nbrp_fmax + 1U))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
  if (((bt->prop_seg_ != 0U) || (bt->phase_seg1_ != 0U)) &&
      ((ntseg1 < 1U) || (ntseg1 > (nt1_fmax + 1U)))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
  if ((bt->phase_seg2_ != 0U) && (bt->phase_seg2_ > (nt2_fmax + 1U))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
  if ((bt->sjw_ != 0U) &&
      ((bt->sjw_ > (nsjw_fmax + 1U)) || (bt->sjw_ > eff_ntseg2))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
#endif

#ifdef FDCAN_DBTP_DBRP_Msk
  const uint32_t dbrp_fmax = FDCAN_DBTP_DBRP_Msk >> FDCAN_DBTP_DBRP_Pos;
  const uint32_t dt1_fmax = FDCAN_DBTP_DTSEG1_Msk >> FDCAN_DBTP_DTSEG1_Pos;
  const uint32_t dt2_fmax = FDCAN_DBTP_DTSEG2_Msk >> FDCAN_DBTP_DTSEG2_Pos;
  const uint32_t dsjw_fmax = FDCAN_DBTP_DSJW_Msk >> FDCAN_DBTP_DSJW_Pos;

  const BSP_FDCAN_BitTiming_t* dbt = &cfg->data_timing_;
  const uint32_t dtseg1 = dbt->prop_seg_ + dbt->phase_seg1_;
  const uint32_t cur_dtseg2 =
      ((can->DBTP & FDCAN_DBTP_DTSEG2_Msk) >> FDCAN_DBTP_DTSEG2_Pos) + 1U;
  const uint32_t eff_dtseg2 =
      (dbt->phase_seg2_ != 0U) ? dbt->phase_seg2_ : cur_dtseg2;

  if ((dbt->brp_ != 0U) && (dbt->brp_ > (dbrp_fmax + 1U))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
  if (((dbt->prop_seg_ != 0U) || (dbt->phase_seg1_ != 0U)) &&
      ((dtseg1 < 1U) || (dtseg1 > (dt1_fmax + 1U)))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
  if ((dbt->phase_seg2_ != 0U) && (dbt->phase_seg2_ > (dt2_fmax + 1U))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
  if ((dbt->sjw_ != 0U) &&
      ((dbt->sjw_ > (dsjw_fmax + 1U)) || (dbt->sjw_ > eff_dtseg2))) {
    self->last_error_ = ARG_ERR;
    return self->last_error_;
  }
#endif

  /* 2. 若外设正在运行，先关中断并停止，进入 INIT/CCE 配置态。 */
  const bool was_running = (hfdcan->State == HAL_FDCAN_STATE_BUSY);
  if (was_running) {
    const uint32_t active_its = BSP_FDCAN_build_notification_mask(hfdcan);
    (void)HAL_FDCAN_DeactivateNotification(hfdcan, active_its);
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) {
      self->last_error_ = FAILED;
      return self->last_error_;
    }
  } else if (hfdcan->State != HAL_FDCAN_STATE_READY) {
    self->last_error_ = STATE_ERR;
    return self->last_error_;
  }

  SET_BIT(can->CCCR, FDCAN_CCCR_INIT);
  uint32_t wait_count = 0U;
  while ((can->CCCR & FDCAN_CCCR_INIT) == 0U) {
    if (++wait_count > BSP_FDCAN_INIT_TIMEOUT_LOOP) {
      self->last_error_ = TIMEOUT;
      return self->last_error_;
    }
  }
  SET_BIT(can->CCCR, FDCAN_CCCR_CCE);

  /* 3. 可选更新工作模式寄存器，并同步 hfdcan->Init。 */
  if (cfg->apply_mode_) {
#ifdef FDCAN_CCCR_DAR
    if (cfg->mode_.one_shot_) {
      SET_BIT(can->CCCR, FDCAN_CCCR_DAR);
      hfdcan->Init.AutoRetransmission = DISABLE;
    } else {
      CLEAR_BIT(can->CCCR, FDCAN_CCCR_DAR);
      hfdcan->Init.AutoRetransmission = ENABLE;
    }
#endif

#if defined(FDCAN_CCCR_TEST) && defined(FDCAN_TEST_LBCK)
    if (cfg->mode_.loopback_) {
      SET_BIT(can->CCCR, FDCAN_CCCR_TEST);
      SET_BIT(can->TEST, FDCAN_TEST_LBCK);
    } else {
      CLEAR_BIT(can->TEST, FDCAN_TEST_LBCK);
      CLEAR_BIT(can->CCCR, FDCAN_CCCR_TEST);
    }
#endif

#ifdef FDCAN_CCCR_MON
    if (cfg->mode_.loopback_ || cfg->mode_.listen_only_) {
      SET_BIT(can->CCCR, FDCAN_CCCR_MON);
    } else {
      CLEAR_BIT(can->CCCR, FDCAN_CCCR_MON);
    }
#endif

    if (cfg->mode_.loopback_) {
      hfdcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    } else if (cfg->mode_.listen_only_) {
      hfdcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
    } else {
      hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    }

#ifdef FDCAN_CCCR_FDOE
    if (cfg->mode_.fd_enable_) {
      SET_BIT(can->CCCR, FDCAN_CCCR_FDOE);
    } else {
      CLEAR_BIT(can->CCCR, FDCAN_CCCR_FDOE);
    }
#endif
#ifdef FDCAN_CCCR_BRSE
    if (cfg->mode_.fd_enable_ && cfg->mode_.brs_enable_) {
      SET_BIT(can->CCCR, FDCAN_CCCR_BRSE);
    } else {
      CLEAR_BIT(can->CCCR, FDCAN_CCCR_BRSE);
    }
#endif

    if (!cfg->mode_.fd_enable_) {
      hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    } else if (cfg->mode_.brs_enable_) {
      hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    } else {
      hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
    }
  }

  /* 4. 更新仲裁相位 NBTP（字段为 0 保持原值）。 */
#ifdef FDCAN_NBTP_NBRP_Msk
  uint32_t nbtp = can->NBTP;
  bool nbtp_changed = false;
  if (bt->brp_ != 0U) {
    nbtp = (nbtp & ~FDCAN_NBTP_NBRP_Msk) |
           (((bt->brp_ - 1U) & nbrp_fmax) << FDCAN_NBTP_NBRP_Pos);
    hfdcan->Init.NominalPrescaler = bt->brp_;
    nbtp_changed = true;
  }
  if ((bt->prop_seg_ != 0U) || (bt->phase_seg1_ != 0U)) {
    nbtp = (nbtp & ~FDCAN_NBTP_NTSEG1_Msk) |
           (((ntseg1 - 1U) & nt1_fmax) << FDCAN_NBTP_NTSEG1_Pos);
    hfdcan->Init.NominalTimeSeg1 = ntseg1;
    nbtp_changed = true;
  }
  if (bt->phase_seg2_ != 0U) {
    nbtp = (nbtp & ~FDCAN_NBTP_NTSEG2_Msk) |
           (((bt->phase_seg2_ - 1U) & nt2_fmax) << FDCAN_NBTP_NTSEG2_Pos);
    hfdcan->Init.NominalTimeSeg2 = bt->phase_seg2_;
    nbtp_changed = true;
  }
  if (bt->sjw_ != 0U) {
    nbtp = (nbtp & ~FDCAN_NBTP_NSJW_Msk) |
           (((bt->sjw_ - 1U) & nsjw_fmax) << FDCAN_NBTP_NSJW_Pos);
    hfdcan->Init.NominalSyncJumpWidth = bt->sjw_;
    nbtp_changed = true;
  }
  if (nbtp_changed) {
    can->NBTP = nbtp;
  }
#endif

  /* 5. 更新数据相位 DBTP（字段为 0 保持原值）。 */
#ifdef FDCAN_DBTP_DBRP_Msk
  uint32_t dbtp = can->DBTP;
  bool dbtp_changed = false;
  if (dbt->brp_ != 0U) {
    dbtp = (dbtp & ~FDCAN_DBTP_DBRP_Msk) |
           (((dbt->brp_ - 1U) & dbrp_fmax) << FDCAN_DBTP_DBRP_Pos);
    hfdcan->Init.DataPrescaler = dbt->brp_;
    dbtp_changed = true;
  }
  if ((dbt->prop_seg_ != 0U) || (dbt->phase_seg1_ != 0U)) {
    dbtp = (dbtp & ~FDCAN_DBTP_DTSEG1_Msk) |
           (((dtseg1 - 1U) & dt1_fmax) << FDCAN_DBTP_DTSEG1_Pos);
    hfdcan->Init.DataTimeSeg1 = dtseg1;
    dbtp_changed = true;
  }
  if (dbt->phase_seg2_ != 0U) {
    dbtp = (dbtp & ~FDCAN_DBTP_DTSEG2_Msk) |
           (((dbt->phase_seg2_ - 1U) & dt2_fmax) << FDCAN_DBTP_DTSEG2_Pos);
    hfdcan->Init.DataTimeSeg2 = dbt->phase_seg2_;
    dbtp_changed = true;
  }
  if (dbt->sjw_ != 0U) {
    dbtp = (dbtp & ~FDCAN_DBTP_DSJW_Msk) |
           (((dbt->sjw_ - 1U) & dsjw_fmax) << FDCAN_DBTP_DSJW_Pos);
    hfdcan->Init.DataSyncJumpWidth = dbt->sjw_;
    dbtp_changed = true;
  }
  if (dbtp_changed) {
    can->DBTP = dbtp;
  }
#endif

  /* 6. 若重配前已运行，重新启动并恢复中断通知；否则保持 READY 状态。 */
  if (was_running) {
    self->last_error_ = BSP_FDCAN_start_handle(hfdcan);
  } else {
    self->last_error_ = OK;
  }

  if (self->last_error_ == OK) {
    LOG_DEBUG("FDCAN", "FDCAN%u 运行期参数配置完成", (unsigned)self->id_ + 1U);
  } else {
    LOG_ERROR("FDCAN", "FDCAN%u 重配后重启失败 错误码=%d",
              (unsigned)self->id_ + 1U, (int)self->last_error_);
  }
  return self->last_error_;
}

// ==================== 发送接口 ====================

/**
 * @brief 通用帧发送（记录 last_error_ 与单次故障去重告警日志）
 */
err_t STM32FDCAN_SendFrame(STM32FDCAN_t* self,
                           const BSP_FDCAN_TxFrame_t* frame) {
  if (self == NULL) {
    LOG_ERROR("FDCAN", "发送失败：对象为空");
    return PTR_NULL;
  }
  if (frame == NULL) {
    self->last_error_ = PTR_NULL;
    LOG_ERROR("FDCAN", "FDCAN%u 发送失败：帧描述为空",
              (unsigned)self->id_ + 1U);
    return self->last_error_;
  }

  const err_t result = BSP_FDCAN_send_frame_hw(self->fdcan_handle_, frame);
  self->last_error_ = result;

#if BSP_LOG_ENABLED
  const bool valid_id = BSP_FDCAN_is_valid_id(self->id_);
  const uint8_t fault_bit =
      valid_id ? (uint8_t)(1U << (unsigned)self->id_) : 0U;
  const unsigned fdcan_number = (unsigned)self->id_ + 1U;
  if (result == OK) {
    if (valid_id &&
        ((atomic_load_explicit(&bsp_fdcan_tx_fault_reported,
                               memory_order_relaxed) &
          fault_bit) != 0U)) {
      const err_t reported =
          LOG_INFO("FDCAN", "FDCAN%u 发送恢复正常", fdcan_number);
      if (reported == OK) {
        atomic_fetch_and_explicit(&bsp_fdcan_tx_fault_reported,
                                  (uint8_t)~fault_bit, memory_order_relaxed);
      }
    }
    LOG_DEBUG("FDCAN", "FDCAN%u 发送已提交 帧ID=0x%X 长度=%u字节 FD=%u",
              fdcan_number, (unsigned)frame->id_, (unsigned)frame->size_,
              (unsigned)(frame->fd_format_ == FDCAN_FD_CAN ? 1U : 0U));
  } else if (!valid_id ||
             ((atomic_load_explicit(&bsp_fdcan_tx_fault_reported,
                                    memory_order_relaxed) &
               fault_bit) == 0U)) {
    const FDCAN_GlobalTypeDef* regs =
        (self->fdcan_handle_ != NULL) ? self->fdcan_handle_->Instance : NULL;
    const uint32_t psr = (regs != NULL) ? regs->PSR : 0U;
    const uint32_t ecr = (regs != NULL) ? regs->ECR : 0U;
    const uint32_t txfqs = (regs != NULL) ? regs->TXFQS : 0U;
    const err_t reported = LOG_WARN(
        "FDCAN",
        "FDCAN%u 发送失败 帧ID=0x%X 错误码=%d PSR=0x%08X ECR=0x%08X "
        "TXFQS=0x%08X LEC=%u TEC=%u REC=%u BO=%u TFFL=%u",
        fdcan_number, (unsigned)frame->id_, (int)result, (unsigned)psr,
        (unsigned)ecr, (unsigned)txfqs,
        (unsigned)((psr & FDCAN_PSR_LEC_Msk) >> FDCAN_PSR_LEC_Pos),
        (unsigned)((ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos),
        (unsigned)((ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos),
        (unsigned)((psr & FDCAN_PSR_BO_Msk) >> FDCAN_PSR_BO_Pos),
        (unsigned)((txfqs & FDCAN_TXFQS_TFFL_Msk) >> FDCAN_TXFQS_TFFL_Pos));
    if ((reported == OK) && valid_id) {
      atomic_fetch_or_explicit(&bsp_fdcan_tx_fault_reported, fault_bit,
                               memory_order_relaxed);
    }
  }
#endif

  return result;
}

/**
 * @brief 便捷发送 Classical CAN 标准数据帧（0..8B）
 */
err_t STM32FDCAN_SendStd(STM32FDCAN_t* self, uint32_t std_id,
                         const uint8_t* data, size_t size) {
  if (size > BSP_FDCAN_CLASSIC_DATA_SIZE) {
    if (self != NULL) {
      self->last_error_ = OUT_OF_RANGE;
    }
    return OUT_OF_RANGE;
  }

  const BSP_FDCAN_TxFrame_t frame = {
      .id_ = std_id,
      .id_type_ = FDCAN_STANDARD_ID,
      .frame_type_ = FDCAN_DATA_FRAME,
      .fd_format_ = FDCAN_CLASSIC_CAN,
      .brs_ = FDCAN_BRS_OFF,
      .data_ = data,
      .size_ = (uint8_t)size,
  };
  return STM32FDCAN_SendFrame(self, &frame);
}

/**
 * @brief 便捷发送 CAN FD 数据帧（0..64B）
 */
err_t STM32FDCAN_SendFD(STM32FDCAN_t* self, uint32_t id, uint32_t id_type,
                        bool brs, const uint8_t* data, size_t size) {
  if (size > BSP_FDCAN_DATA_SIZE) {
    if (self != NULL) {
      self->last_error_ = OUT_OF_RANGE;
    }
    return OUT_OF_RANGE;
  }

  const BSP_FDCAN_TxFrame_t frame = {
      .id_ = id,
      .id_type_ = id_type,
      .frame_type_ = FDCAN_DATA_FRAME,
      .fd_format_ = FDCAN_FD_CAN,
      .brs_ = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
      .data_ = data,
      .size_ = (uint8_t)size,
  };
  return STM32FDCAN_SendFrame(self, &frame);
}

/**
 * @brief HAL 句柄适配发送 Classical CAN 标准数据帧
 */
err_t STM32FDCAN_SendStdByHandle(FDCAN_HandleTypeDef* fdcan_handle,
                                 uint32_t std_id, const uint8_t* data,
                                 size_t size) {
  if (fdcan_handle == NULL) {
    LOG_ERROR("FDCAN", "按句柄发送失败：HAL句柄为空");
    return PTR_NULL;
  }
  STM32FDCAN_t* self = BSP_FDCAN_get_object(fdcan_handle);
  if (self == NULL) {
    LOG_ERROR("FDCAN", "按句柄发送失败：HAL句柄未注册");
    return NOT_FOUND;
  }
  return STM32FDCAN_SendStd(self, std_id, data, size);
}

// ==================== RX 广播与状态查询 ====================

/**
 * @brief 向全部订阅者按注册顺序广播 RX 帧快照
 */
void STM32FDCAN_HandleRxFrame(STM32FDCAN_t* self,
                              const BSP_FDCAN_Frame_t* frame) {
  if (self == NULL) {
    return;
  }
  if (frame == NULL) {
    self->last_error_ = PTR_NULL;
    return;
  }
  if (frame->size_ > BSP_FDCAN_DATA_SIZE) {
    self->last_error_ = OUT_OF_RANGE;
    return;
  }

  self->last_error_ = OK;
  for (uint8_t index = 0U; index < self->subscriber_count_; ++index) {
    STM32FDCAN_RxSubscriber_t* subscriber = &self->subscribers_[index];
    subscriber->callback_(self, frame, subscriber->context_);
  }
}

/**
 * @brief 查询 FDCAN 外设核时钟频率（Hz）
 */
uint32_t STM32FDCAN_GetClockFreq(const STM32FDCAN_t* self) {
  if ((self == NULL) || (self->fdcan_handle_ == NULL)) {
    return 0U;
  }
#if defined(RCC_PERIPHCLK_FDCAN)
  return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
#elif defined(RCC_PERIPHCLK_FDCAN1)
  return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN1);
#else
  return 0U;
#endif
}

/**
 * @brief 读取当前硬件错误计数与协议状态快照
 */
err_t STM32FDCAN_GetErrorState(const STM32FDCAN_t* self,
                               BSP_FDCAN_ErrorState_t* state) {
  if ((self == NULL) || (state == NULL) || (self->fdcan_handle_ == NULL) ||
      (self->fdcan_handle_->Instance == NULL)) {
    return PTR_NULL;
  }

  FDCAN_ErrorCountersTypeDef counters = {0};
  if (HAL_FDCAN_GetErrorCounters(self->fdcan_handle_, &counters) != HAL_OK) {
    return FAILED;
  }

  FDCAN_ProtocolStatusTypeDef proto = {0};
  if (HAL_FDCAN_GetProtocolStatus(self->fdcan_handle_, &proto) != HAL_OK) {
    return FAILED;
  }

  state->tx_error_counter_ = (uint8_t)counters.TxErrorCnt;
  state->rx_error_counter_ = (uint8_t)counters.RxErrorCnt;
  state->bus_off_ = (proto.BusOff != 0U);
  state->error_passive_ = (proto.ErrorPassive != 0U);
  state->error_warning_ = (proto.Warning != 0U);
  state->last_error_code_ = (uint8_t)(proto.LastErrorCode & 0x07U);
  state->data_last_error_code_ = (uint8_t)(proto.DataLastErrorCode & 0x07U);
  return OK;
}

/**
 * @brief 获取最近一次操作错误码
 */
err_t STM32FDCAN_GetLastError(const STM32FDCAN_t* self) {
  return (self != NULL) ? self->last_error_ : PTR_NULL;
}

// ==================== HAL 回调分发 ====================

/**
 * @brief RX ISR 核心：循环排空指定 FIFO、构快照并广播给全部订阅者
 */
static void BSP_FDCAN_RX_ISR_Handler(FDCAN_HandleTypeDef* fdcan_handle,
                                     uint32_t fifo) {
  STM32FDCAN_t* self = BSP_FDCAN_get_object(fdcan_handle);
  if (self == NULL) {
    return;
  }

  uint32_t drain_limit = (fifo == FDCAN_RX_FIFO0)
                             ? fdcan_handle->Init.RxFifo0ElmtsNbr
                             : fdcan_handle->Init.RxFifo1ElmtsNbr;
  if ((drain_limit == 0U) || (drain_limit > BSP_FDCAN_RX_DRAIN_MAX)) {
    drain_limit = BSP_FDCAN_RX_DRAIN_MAX;
  }

  for (uint32_t count = 0U;
       (count < drain_limit) &&
       (HAL_FDCAN_GetRxFifoFillLevel(fdcan_handle, fifo) > 0U);
       ++count) {
    FDCAN_RxHeaderTypeDef rx_header = {0};
    BSP_FDCAN_Frame_t frame = {0};

    if (HAL_FDCAN_GetRxMessage(fdcan_handle, fifo, &rx_header, frame.data_) !=
        HAL_OK) {
      self->last_error_ = FAILED;
      return;
    }

    uint8_t bytes = 0U;
    if (!BSP_FDCAN_dlc_to_bytes(rx_header.DataLength, &bytes)) {
      self->last_error_ = OUT_OF_RANGE;
      continue;
    }
    if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
        (bytes > BSP_FDCAN_CLASSIC_DATA_SIZE)) {
      bytes = BSP_FDCAN_CLASSIC_DATA_SIZE;
    }

    frame.id_ = rx_header.Identifier;
    frame.id_type_ = rx_header.IdType;
    frame.frame_type_ = rx_header.RxFrameType;
    frame.fd_format_ = rx_header.FDFormat;
    frame.brs_ = rx_header.BitRateSwitch;
    frame.fifo_ = fifo;
    frame.size_ = bytes;

    if (frame.size_ < BSP_FDCAN_DATA_SIZE) {
      /* 仅保留有效载荷字节，尾部清零，避免上层读到脏数据。 */
      memset(&frame.data_[frame.size_], 0, BSP_FDCAN_DATA_SIZE - frame.size_);
    }

    STM32FDCAN_HandleRxFrame(self, &frame);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
    BSP_FDCAN_RX_ISR_Handler(hfdcan, FDCAN_RX_FIFO0);
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
    BSP_FDCAN_RX_ISR_Handler(hfdcan, FDCAN_RX_FIFO1);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan,
                                   uint32_t ErrorStatusITs) {
  STM32FDCAN_t* self = BSP_FDCAN_get_object(hfdcan);
  if (self == NULL) {
    return;
  }

  self->error_irq_count_ += 1U;

  FDCAN_ProtocolStatusTypeDef proto = {0};
  if (HAL_FDCAN_GetProtocolStatus(hfdcan, &proto) != HAL_OK) {
    return;
  }

#if defined(FDCAN_IT_BUS_OFF) && defined(FDCAN_CCCR_INIT)
  if (((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) && (proto.BusOff != 0U)) {
    /* 清除 CCCR.INIT 触发硬件 Bus-Off 自动恢复序列。 */
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    self->bus_off_count_ += 1U;
  }
#else
  RM_UNUSED(ErrorStatusITs);
#endif
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {
  if (hfdcan == NULL) {
    return;
  }
  hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  STM32FDCAN_t* self = BSP_FDCAN_get_object(hfdcan);
  if (self != NULL) {
    self->error_irq_count_ += 1U;
  }
}
