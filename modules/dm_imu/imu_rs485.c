/*
 * imu_rs485.c - DM-IMU-L1 RS485 通道驱动（24 字节请求应答帧 + CRC-8/SAE-J1850）
 *
 * 兼容协议：DM-IMU-L1 固件 v2.1.0.0+ RS485 协议（参考 dmimu_rs485.c）：
 *   - 帧格式：[0]=0xA5, [1]=数据类型(0x0C命令/0x0D数据), [2]=从机地址,
 *             [3]=RID, [4]=操作(0读/1写), [5..20]=DATA0~DATA3,
 *             [21]=应答码(或主动上报 0x80/0x81), [22]=CRC-8/SAE-J1850(覆盖[1..21]),
 *             [23]=0x5A；
 *   - 底层收发接入 bsp_uart（RX 循环 DMA 空闲线切分 + TX 变长帧双缓冲 DMA）；
 *   - 支持默认被动自动轮询（RID 0~3）及单机直连主动上报（0x80/0x81）兼容接收。
 */
#include "imu_rs485.h"

#include <stdalign.h>
#include <string.h>

#include "comp_utils.h"

#define IMU485_CRC8_POLY (0x1Du)
#define IMU485_CRC8_INIT (0xFFu)
#define IMU485_CRC8_XOROUT (0xFFu)
#define IMU485_SANITY_LIMIT (1.0e4f)
#define IMU485_RX_RING_MASK (IMU485_RX_RING_SIZE - 1u)

_Static_assert((IMU485_RX_RING_SIZE & (IMU485_RX_RING_SIZE - 1u)) == 0u,
               "IMU485_RX_RING_SIZE must be a power of two");

static BSP_UART_DMA_BUFFER(dma_rx_buffer_, IMU485_DMA_BUFFER_SIZE);
static BSP_UART_DMA_BUFFER(dma_tx_buffer_0_, IMU485_TX_DMA_BUFFER_SIZE);
static BSP_UART_DMA_BUFFER(dma_tx_buffer_1_, IMU485_TX_DMA_BUFFER_SIZE);

/* ISR -> 任务 单生产者单消费者环形缓冲 */
static uint8_t rx_ring_buf_[IMU485_RX_RING_SIZE];
static volatile uint16_t rx_ring_head_ = 0u;
static volatile uint16_t rx_ring_tail_ = 0u;
static volatile uint32_t rx_ring_overflow_ = 0u;
static volatile bool rx_error_reset_ = false;
static uint32_t imu_last_rx_error_count_ = 0u;
static IMU485_t *instance_ = NULL;

dm_imu_t_t imu_485;

/* ------------------------------------------------------------------------- */
/* 内部基础工具                                                               */
/* ------------------------------------------------------------------------- */
static uint32_t imu485_now_ms(void)
{
  return HAL_GetTick();
}

static float imu485_read_f32_le(const uint8_t *p)
{
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  float f = 0.0f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

static void imu485_put_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static bool imu485_f32_sane(float v, float limit)
{
  if (v != v) {
    return false;
  }
  if ((v > limit) || (v < -limit)) {
    return false;
  }
  return true;
}

static uint8_t imu485_frame_crc(const uint8_t frame[IMU485_FRAME_LEN])
{
  return imu_485_crc8_j1850(&frame[IMU485_CRC_START], (uint16_t)IMU485_CRC_LEN);
}

static void imu485_put_crc_and_tail(uint8_t frame[IMU485_FRAME_LEN])
{
  frame[IMU485_CRC_OFFSET] = imu485_frame_crc(frame);
  frame[IMU485_FRAME_LEN - 1u] = IMU485_TAIL;
}

/* ------------------------------------------------------------------------- */
/* 纯编解码与 CRC-8/SAE-J1850                                                 */
/* ------------------------------------------------------------------------- */
uint8_t imu_485_crc8_j1850(const uint8_t *data, uint16_t len)
{
  uint8_t crc = IMU485_CRC8_INIT;

  if (data == NULL) {
    return (uint8_t)(IMU485_CRC8_INIT ^ IMU485_CRC8_XOROUT);
  }

  for (uint16_t i = 0u; i < len; ++i) {
    crc = (uint8_t)(crc ^ data[i]);
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
      if ((crc & 0x80u) != 0u) {
        crc = (uint8_t)((uint8_t)(crc << 1) ^ IMU485_CRC8_POLY);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }

  return (uint8_t)(crc ^ IMU485_CRC8_XOROUT);
}

err_t imu_485_build_frame(uint8_t out[IMU485_FRAME_LEN], uint8_t slave_id,
                          uint8_t data_type, uint8_t rid, uint8_t op,
                          const uint8_t data[IMU485_DATA_LEN])
{
  if (out == NULL) {
    return PTR_NULL;
  }
  if ((data_type != IMU485_DATA_TYPE_CMD) &&
      (data_type != IMU485_DATA_TYPE_DATA)) {
    return ARG_ERR;
  }
  if ((op != IMU485_OP_READ) && (op != IMU485_OP_WRITE)) {
    return ARG_ERR;
  }

  memset(out, 0, IMU485_FRAME_LEN);
  out[0] = IMU485_HEADER;
  out[1] = data_type;
  out[2] = slave_id;
  out[3] = rid;
  out[4] = op;
  if (data != NULL) {
    memcpy(&out[IMU485_DATA_OFFSET], data, IMU485_DATA_LEN);
  }
  out[IMU485_ACK_OFFSET] = IMU485_ACK_SUCCESS;
  imu485_put_crc_and_tail(out);
  return OK;
}

err_t imu_485_build_req(uint8_t out[IMU485_FRAME_LEN], uint8_t slave_id,
                        imu485_rid_t rid)
{
  if (rid >= IMU485_RID_COUNT) {
    return ARG_ERR;
  }
  return imu_485_build_frame(out, slave_id, IMU485_DATA_TYPE_DATA, (uint8_t)rid,
                             IMU485_OP_READ, NULL);
}

err_t imu_485_parse_frame(const uint8_t in[IMU485_FRAME_LEN], uint8_t slave_id,
                          imu485_frame_t *out)
{
  if ((in == NULL) || (out == NULL)) {
    return PTR_NULL;
  }
  if ((in[0] != IMU485_HEADER) || (in[IMU485_FRAME_LEN - 1u] != IMU485_TAIL)) {
    return FAILED;
  }
  if (in[2] != slave_id) {
    return FAILED;
  }
  if (in[IMU485_CRC_OFFSET] != imu485_frame_crc(in)) {
    return CHECK_ERR;
  }

  out->data_type = in[1];
  out->slave_id = in[2];
  out->rid = in[3];
  out->op = in[4];
  out->ack = in[IMU485_ACK_OFFSET];
  memcpy(out->data, &in[IMU485_DATA_OFFSET], IMU485_DATA_LEN);
  return OK;
}

err_t imu_485_parse_rsp(const uint8_t in[IMU485_FRAME_LEN], uint8_t slave_id,
                        imu485_rid_t rid, float out[4], uint8_t *out_n)
{
  imu485_frame_t fr;

  if ((in == NULL) || (out == NULL) || (out_n == NULL)) {
    return PTR_NULL;
  }
  if (rid >= IMU485_RID_COUNT) {
    return ARG_ERR;
  }

  err_t rc = imu_485_parse_frame(in, slave_id, &fr);
  if (rc != OK) {
    return rc;
  }
  if ((fr.data_type != IMU485_DATA_TYPE_DATA) || (fr.op != IMU485_OP_READ) ||
      (fr.rid != (uint8_t)rid)) {
    return FAILED;
  }
  if (fr.ack != IMU485_ACK_SUCCESS) {
    return STATE_ERR;
  }

  const uint8_t n = (rid == IMU485_RID_QUAT) ? 4u : 3u;
  for (uint8_t i = 0u; i < n; ++i) {
    const float v = imu485_read_f32_le(&fr.data[(uint16_t)i * 4u]);
    if (!imu485_f32_sane(v, IMU485_SANITY_LIMIT)) {
      return FAILED;
    }
    out[i] = v;
  }
  *out_n = n;
  return OK;
}

err_t imu_485_parse_active_frame(const uint8_t in[IMU485_FRAME_LEN],
                                 uint8_t slave_id, imu485_frame_t *out,
                                 uint8_t *out_last)
{
  imu485_frame_t fr;

  if ((in == NULL) || (out == NULL) || (out_last == NULL)) {
    return PTR_NULL;
  }

  err_t rc = imu_485_parse_frame(in, slave_id, &fr);
  if (rc != OK) {
    return rc;
  }
  if ((fr.data_type != IMU485_DATA_TYPE_DATA) || (fr.op != IMU485_OP_READ) ||
      (fr.rid >= (uint8_t)IMU485_RID_COUNT) ||
      ((fr.ack != IMU485_ACK_GROUP_FRAME) &&
       (fr.ack != IMU485_ACK_GROUP_LAST))) {
    return FAILED;
  }

  *out = fr;
  *out_last = (fr.ack == IMU485_ACK_GROUP_LAST) ? 1u : 0u;
  return OK;
}

/* ------------------------------------------------------------------------- */
/* 数据写入与兼容解包                                                         */
/* ------------------------------------------------------------------------- */
static bool imu485_extract_data(const imu485_frame_t *fr, float vals[4],
                                uint8_t *out_n)
{
  if ((fr == NULL) || (vals == NULL) || (out_n == NULL) ||
      (fr->rid >= (uint8_t)IMU485_RID_COUNT)) {
    return false;
  }

  const uint8_t n = (fr->rid == (uint8_t)IMU485_RID_QUAT) ? 4u : 3u;
  for (uint8_t i = 0u; i < n; ++i) {
    const float v = imu485_read_f32_le(&fr->data[(uint16_t)i * 4u]);
    if (!imu485_f32_sane(v, IMU485_SANITY_LIMIT)) {
      return false;
    }
    vals[i] = v;
  }
  *out_n = n;
  return true;
}

static void imu485_apply_data(dm_imu_t_t *dst, imu485_rid_t rid,
                              const float vals[4])
{
  switch (rid) {
    case IMU485_RID_ACCEL:
      dst->accel[0] = vals[0];
      dst->accel[1] = vals[1];
      dst->accel[2] = vals[2];
      break;
    case IMU485_RID_GYRO:
      dst->gyro[0] = vals[0];
      dst->gyro[1] = vals[1];
      dst->gyro[2] = vals[2];
      break;
    case IMU485_RID_EULER:
      dst->roll = vals[0];
      dst->pitch = vals[1];
      dst->yaw = vals[2];
      break;
    case IMU485_RID_QUAT:
      dst->quaternion[0] = vals[0];
      dst->quaternion[1] = vals[1];
      dst->quaternion[2] = vals[2];
      dst->quaternion[3] = vals[3];
      break;
    default:
      break;
  }
}

static bool imu485_store_data(IMU485_t *self, const imu485_frame_t *fr,
                              uint32_t now_ms)
{
  float vals[4];
  uint8_t n = 0u;

  if (!imu485_extract_data(fr, vals, &n)) {
    return false;
  }
  RM_UNUSED(n);

  taskENTER_CRITICAL();
  imu485_apply_data(&self->data, (imu485_rid_t)fr->rid, vals);
  imu_485 = self->data;
  switch ((imu485_rid_t)fr->rid) {
    case IMU485_RID_ACCEL:
      self->meta.t_accel_ms = now_ms;
      break;
    case IMU485_RID_GYRO:
      self->meta.t_gyro_ms = now_ms;
      break;
    case IMU485_RID_EULER:
      self->meta.t_euler_ms = now_ms;
      break;
    case IMU485_RID_QUAT:
      self->meta.t_quat_ms = now_ms;
      break;
    default:
      break;
  }
  self->meta.valid =
      (uint16_t)(self->meta.valid | (uint16_t)(1u << (uint8_t)fr->rid));
  taskEXIT_CRITICAL();
  return true;
}

void imu_485_data_unpack(const uint8_t *pData)
{
  imu485_frame_t fr;
  float vals[4];
  uint8_t n = 0u;

  if (pData == NULL) {
    return;
  }
  const uint8_t slave_id =
      (instance_ != NULL) ? instance_->cfg.slave_id : IMU485_DEFAULT_SLAVE_ID;
  if (imu_485_parse_frame(pData, slave_id, &fr) != OK) {
    return;
  }
  if ((fr.data_type != IMU485_DATA_TYPE_DATA) || (fr.op != IMU485_OP_READ)) {
    return;
  }
  if ((fr.ack != IMU485_ACK_SUCCESS) && (fr.ack != IMU485_ACK_GROUP_FRAME) &&
      (fr.ack != IMU485_ACK_GROUP_LAST)) {
    return;
  }
  if (!imu485_extract_data(&fr, vals, &n)) {
    return;
  }
  RM_UNUSED(n);

  imu485_apply_data(&imu_485, (imu485_rid_t)fr.rid, vals);
}

/* ------------------------------------------------------------------------- */
/* 底层发送与事务状态机                                                       */
/* ------------------------------------------------------------------------- */
static err_t imu485_write_raw(IMU485_t *self, const uint8_t *buf, size_t len)
{
  if ((self == NULL) || (buf == NULL) || (len == 0u)) {
    return PTR_NULL;
  }
  err_t rc = STM32UARTFrameTx_Write(&self->uart_tx_, buf, len);
  if (rc != OK) {
    self->stats.tx_err++;
    self->last_error_ = rc;
  }
  return rc;
}

static void imu485_finish_tx(IMU485_t *self, err_t err, uint32_t now_ms)
{
  self->tx_inflight = 0u;
  self->last_done_ms = now_ms;
  self->last_error_ = err;
}

static bool imu485_match_tx(const IMU485_t *self, const imu485_frame_t *fr)
{
  if (self->tx_inflight == 0u) {
    return false;
  }
  return (fr->data_type == self->tx_data_type) && (fr->rid == self->tx_rid) &&
         (fr->op == self->tx_op);
}

static err_t imu485_start_tx(IMU485_t *self, uint8_t kind, uint8_t data_type,
                             uint8_t rid, uint8_t op,
                             const uint8_t data[IMU485_DATA_LEN], uint8_t retry,
                             uint32_t now_ms)
{
  err_t rc = imu_485_build_frame(self->tx_frame, self->cfg.slave_id, data_type,
                                 rid, op, data);
  if (rc != OK) {
    return rc;
  }

  rc = imu485_write_raw(self, self->tx_frame, IMU485_FRAME_LEN);
  if (rc != OK) {
    return rc;
  }

  self->tx_inflight = 1u;
  self->tx_kind = kind;
  self->tx_data_type = data_type;
  self->tx_rid = rid;
  self->tx_op = op;
  self->tx_start_ms = now_ms;
  self->tx_retry_left = retry;
  return OK;
}

static void imu485_shift_rx(IMU485_t *self, uint16_t nbytes)
{
  if (nbytes >= self->rx_len) {
    self->rx_len = 0u;
    return;
  }
  self->rx_len = (uint16_t)(self->rx_len - nbytes);
  memmove(self->rx_frame, &self->rx_frame[nbytes], self->rx_len);
}

static void imu485_handle_active_data(IMU485_t *self, const imu485_frame_t *fr,
                                      uint32_t now_ms)
{
  if (!imu485_store_data(self, fr, now_ms)) {
    self->stats.rx_bad++;
    return;
  }

  self->stats.rx_ok++;
  self->active_frames++;
  self->active_group_frames++;

  if (fr->ack == IMU485_ACK_GROUP_LAST) {
    self->active_groups++;
    self->active_last_group_frames = (uint32_t)self->active_group_frames;
    self->active_last_group_ms = now_ms;
    self->active_group_frames = 0u;

    /* 已武装关闭：组尾到达后立即发关闭帧 */
    if ((self->active_stop_state != 0u) &&
        (imu485_write_raw(self, self->active_stop_frame, IMU485_FRAME_LEN) ==
         OK)) {
      self->active_stop_sent++;
      self->active_stop_sent_ms = now_ms;
      self->active_stop_state = 2u;
    }
  }
}

static void imu485_handle_frame(IMU485_t *self,
                                const uint8_t raw[IMU485_FRAME_LEN],
                                uint32_t now_ms)
{
  imu485_frame_t fr;
  const err_t rc = imu_485_parse_frame(raw, self->cfg.slave_id, &fr);
  if (rc == CHECK_ERR) {
    /* CRC8 错误直接丢弃，按无有效应答交给超时重试 */
    self->stats.rx_crc_err++;
    return;
  }
  if (rc != OK) {
    self->stats.rx_bad++;
    return;
  }

  memcpy(self->raw_frame, raw, IMU485_FRAME_LEN);

  /* 主动上报帧处理（显式开启或模块当前处于主动上报时自动兼容） */
  if ((fr.data_type == IMU485_DATA_TYPE_DATA) && (fr.op == IMU485_OP_READ) &&
      ((fr.ack == IMU485_ACK_GROUP_FRAME) ||
       (fr.ack == IMU485_ACK_GROUP_LAST))) {
    if (self->active_rx == 0u) {
      /* 检测到模块正在主动上报：暂停被动轮询发送，避免半双工总线冲突 */
      self->active_rx = 1u;
      self->active_auto_detected = 1u;
      self->tx_inflight = 0u;
      self->active_group_frames = 0u;
      self->active_last_group_ms = now_ms;
    }
    imu485_handle_active_data(self, &fr, now_ms);
    return;
  }

  if (self->active_rx != 0u) {
    /* 主动模式期间仅处理关闭主动上报命令（RID 6 写）的应答 */
    if ((fr.data_type == IMU485_DATA_TYPE_CMD) &&
        (fr.rid == (uint8_t)IMU485_CMD_ACTIVE_MODE) &&
        (fr.op == IMU485_OP_WRITE)) {
      self->cmd_result = fr;
      self->cmd_result_pending = 1u;
      if (fr.ack == IMU485_ACK_SUCCESS) {
        self->active_stop_ack++;
        self->active_rx = 0u;
        self->active_auto_detected = 0u;
        self->active_stop_state = 0u;
        self->last_error_ = OK;
      } else {
        self->last_error_ = STATE_ERR;
        if (self->active_stop_state == 2u) {
          self->active_stop_state = 1u;
          self->active_stop_sent_ms = now_ms;
        }
      }
    }
    return;
  }

  const bool matched = imu485_match_tx(self, &fr);

  if (fr.data_type == IMU485_DATA_TYPE_DATA) {
    if (fr.op != IMU485_OP_READ) {
      self->stats.rx_bad++;
      return;
    }
    if (fr.ack != IMU485_ACK_SUCCESS) {
      self->stats.rx_bad++;
      if (matched) {
        imu485_finish_tx(self, STATE_ERR, now_ms);
      }
      return;
    }
    if (!imu485_store_data(self, &fr, now_ms)) {
      self->stats.rx_bad++;
      if (matched) {
        imu485_finish_tx(self, FAILED, now_ms);
      }
      return;
    }
    self->stats.rx_ok++;
    if (matched) {
      imu485_finish_tx(self, OK, now_ms);
    }
    return;
  }

  if (fr.data_type == IMU485_DATA_TYPE_CMD) {
    self->stats.rx_ok++;
    self->cmd_result = fr;
    self->cmd_result_pending = 1u;
    if (matched) {
      imu485_finish_tx(self, (fr.ack == IMU485_ACK_SUCCESS) ? OK : STATE_ERR,
                       now_ms);
    }
    return;
  }

  self->stats.rx_bad++;
}

static void imu485_drain_rx_frame(IMU485_t *self, uint32_t now_ms)
{
  for (;;) {
    if (self->rx_len == 0u) {
      return;
    }
    if (self->rx_frame[0] != IMU485_HEADER) {
      uint16_t drop = 1u;
      while ((drop < self->rx_len) && (self->rx_frame[drop] != IMU485_HEADER)) {
        drop++;
      }
      self->stats.rx_dropped += drop;
      imu485_shift_rx(self, drop);
      continue;
    }
    if (self->rx_len < IMU485_FRAME_LEN) {
      return;
    }
    if (((self->rx_frame[1] == IMU485_DATA_TYPE_DATA) ||
         (self->rx_frame[1] == IMU485_DATA_TYPE_CMD)) &&
        (self->rx_frame[IMU485_FRAME_LEN - 1u] == IMU485_TAIL)) {
      imu485_handle_frame(self, self->rx_frame, now_ms);
      imu485_shift_rx(self, IMU485_FRAME_LEN);
      continue;
    }

    /* 帧类型或帧尾不匹配：当前 0xA5 可能是数据区内字节，滑动 1 字节重同步 */
    self->stats.rx_dropped++;
    imu485_shift_rx(self, 1u);
  }
}

static void imu485_feed_bytes(IMU485_t *self, const uint8_t *buf, size_t len,
                              uint32_t now_ms)
{
  for (size_t i = 0u; i < len; ++i) {
    if (self->rx_len >= IMU485_FRAME_LEN) {
      self->rx_len = 0u;
    }
    self->rx_frame[self->rx_len++] = buf[i];
    imu485_drain_rx_frame(self, now_ms);
  }
}

static void imu485_drain_ring(IMU485_t *self, uint32_t now_ms)
{
  if (rx_error_reset_) {
    rx_error_reset_ = false;
    self->rx_len = 0u;
  }

  const uint32_t overflow = rx_ring_overflow_;
  if (overflow != self->stats.rx_overflow) {
    self->stats.rx_overflow = overflow;
  }

  uint16_t tail = rx_ring_tail_;
  const uint16_t head = rx_ring_head_;
  __DMB();

  if (tail == head) {
    return;
  }

  while (tail != head) {
    const uint16_t chunk = (head > tail) ? (uint16_t)(head - tail)
                                         : (uint16_t)(IMU485_RX_RING_SIZE - tail);
    imu485_feed_bytes(self, &rx_ring_buf_[tail], chunk, now_ms);
    tail = (uint16_t)((tail + chunk) & IMU485_RX_RING_MASK);
  }

  __DMB();
  rx_ring_tail_ = tail;
}

static void imu485_poll(IMU485_t *self, uint32_t now_ms)
{
  if (self->active_rx != 0u) {
    if ((self->active_stop_state == 2u) &&
        ((uint32_t)(now_ms - self->active_stop_sent_ms) >
         (uint32_t)self->cfg.active_stop_timeout_ms)) {
      self->active_stop_timeout++;
      self->active_stop_state = 1u;
      self->active_stop_sent_ms = now_ms;
    }

    /* 若主动上报是自动侦测切入的，长时间无数据则回退到被动自动轮询 */
    if ((self->active_auto_detected != 0u) &&
        ((uint32_t)(now_ms - self->active_last_group_ms) >
         self->cfg.stale_ms)) {
      self->active_rx = 0u;
      self->active_auto_detected = 0u;
      self->active_stop_state = 0u;
    }
    return;
  }

  if (self->tx_inflight != 0u) {
    const uint32_t elapsed = (uint32_t)(now_ms - self->tx_start_ms);
    if (elapsed < (uint32_t)self->cfg.timeout_ms) {
      return;
    }
    if (self->tx_retry_left > 0u) {
      self->tx_retry_left--;
      self->stats.tx_retry++;
      (void)imu485_write_raw(self, self->tx_frame, IMU485_FRAME_LEN);
      self->tx_start_ms = now_ms;
      return;
    }
    self->stats.tx_timeout++;
    imu485_finish_tx(self, TIMEOUT, now_ms);
  }

  if (self->auto_cycle != 0u) {
    const uint32_t gap = (uint32_t)(now_ms - self->last_done_ms);
    if (gap >= (uint32_t)self->cfg.auto_cycle_gap_ms) {
      const imu485_rid_t rid = (imu485_rid_t)self->auto_rid;
      if (imu_485_request(self, rid) == OK) {
        self->auto_rid =
            (uint8_t)(((uint8_t)rid + 1u) % (uint8_t)IMU485_RID_COUNT);
      } else {
        self->last_done_ms = now_ms;
      }
    }
  }
}

/* ------------------------------------------------------------------------- */
/* UART RX DMA 回调（运行在 HAL 中断上下文，仅做入环与唤醒任务）               */
/* ------------------------------------------------------------------------- */
static void IMU485_RxCallback(uint8_t *data, size_t size)
{
  if ((data == NULL) || (size == 0u) || (instance_ == NULL)) {
    return;
  }

  const uint32_t rx_err_count = STM32UART_GetRxErrorCount(&instance_->uart_);
  if (rx_err_count != imu_last_rx_error_count_) {
    imu_last_rx_error_count_ = rx_err_count;
    rx_error_reset_ = true;
  }

  uint16_t head = rx_ring_head_;
  const uint16_t tail = rx_ring_tail_;
  for (size_t i = 0u; i < size; ++i) {
    const uint16_t next = (uint16_t)((head + 1u) & IMU485_RX_RING_MASK);
    if (next == tail) {
      rx_ring_overflow_++;
      break;
    }
    rx_ring_buf_[head] = data[i];
    head = next;
  }
  __DMB();
  rx_ring_head_ = head;

  if (instance_->thread_alert != NULL) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(instance_->thread_alert, SIGNAL_IMU485_RAW_READY,
                       eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

/* ------------------------------------------------------------------------- */
/* 设备生命周期与主循环更新                                                   */
/* ------------------------------------------------------------------------- */
void imu_485_cfg_default(imu485_cfg_t *cfg)
{
  if (cfg == NULL) {
    return;
  }
  cfg->slave_id = IMU485_DEFAULT_SLAVE_ID;
  cfg->retry = IMU485_DEFAULT_RETRY;
  cfg->timeout_ms = IMU485_DEFAULT_TIMEOUT_MS;
  cfg->auto_cycle_gap_ms = IMU485_DEFAULT_AUTO_CYCLE_GAP_MS;
  cfg->active_stop_timeout_ms = IMU485_DEFAULT_ACTIVE_STOP_MS;
  cfg->stale_ms = IMU485_OFFLINE_TIMEOUT_MS;
}

err_t imu_485_init_with_cfg(IMU485_t *self, UART_HandleTypeDef *uart_handle,
                            const imu485_cfg_t *cfg)
{
  if (self == NULL) {
    return PTR_NULL;
  }

  memset(self, 0, sizeof(*self));
  if (cfg != NULL) {
    self->cfg = *cfg;
  } else {
    imu_485_cfg_default(&self->cfg);
  }

  rx_ring_head_ = 0u;
  rx_ring_tail_ = 0u;
  rx_ring_overflow_ = 0u;
  rx_error_reset_ = false;
  imu_last_rx_error_count_ = 0u;

  err_t rc = STM32UART_Init(
      &self->uart_, uart_handle,
      (BSP_UART_RawData_t){dma_rx_buffer_, sizeof(dma_rx_buffer_)},
      IMU485_RxCallback);
  if (rc == OK) {
    rc = STM32UARTFrameTx_Init(
        &self->uart_tx_, uart_handle,
        (BSP_UART_RawData_t){dma_tx_buffer_0_, sizeof(dma_tx_buffer_0_)},
        (BSP_UART_RawData_t){dma_tx_buffer_1_, sizeof(dma_tx_buffer_1_)}, NULL);
  }

  self->init_error_ = rc;
  self->last_error_ = rc;
  instance_ = self;
  return rc;
}

err_t imu_485_init(IMU485_t *self, UART_HandleTypeDef *uart_handle)
{
  return imu_485_init_with_cfg(self, uart_handle, NULL);
}

err_t imu_485_start(IMU485_t *self)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->init_error_ != OK) {
    return self->init_error_;
  }

  rx_ring_head_ = 0u;
  rx_ring_tail_ = 0u;
  rx_ring_overflow_ = 0u;
  rx_error_reset_ = false;
  self->rx_len = 0u;
  self->tx_inflight = 0u;
  self->active_rx = 0u;
  self->active_auto_detected = 0u;
  self->active_stop_state = 0u;
  self->auto_cycle = 1u;
  self->auto_rid = 0u;
  self->last_done_ms =
      (uint32_t)(imu485_now_ms() - (uint32_t)self->cfg.auto_cycle_gap_ms);
  imu_last_rx_error_count_ = STM32UART_GetRxErrorCount(&self->uart_);

  err_t rc = STM32UART_SetRxDMA(&self->uart_);
  if (rc != OK) {
    self->last_error_ = rc;
    return rc;
  }

  rc = STM32UARTFrameTx_SetTxDMA(&self->uart_tx_);
  if (rc != OK) {
    self->last_error_ = rc;
    return rc;
  }

  self->last_error_ = OK;
  return OK;
}

void imu_485_enable_auto_cycle(IMU485_t *self, bool enable)
{
  if (self == NULL) {
    return;
  }
  self->auto_cycle = enable ? 1u : 0u;
  if (enable) {
    self->auto_rid = 0u;
  }
}

bool imu_485_is_stale(const IMU485_t *self, uint32_t now_ms,
                      uint32_t max_age_ms)
{
  if (self == NULL) {
    return true;
  }

  uint32_t newest = 0u;
  bool any = false;

  if ((self->meta.valid & IMU485_VALID_ACCEL) != 0u) {
    newest = self->meta.t_accel_ms;
    any = true;
  }
  if ((self->meta.valid & IMU485_VALID_GYRO) != 0u) {
    if (!any || ((int32_t)(self->meta.t_gyro_ms - newest) > 0)) {
      newest = self->meta.t_gyro_ms;
    }
    any = true;
  }
  if ((self->meta.valid & IMU485_VALID_EULER) != 0u) {
    if (!any || ((int32_t)(self->meta.t_euler_ms - newest) > 0)) {
      newest = self->meta.t_euler_ms;
    }
    any = true;
  }
  if ((self->meta.valid & IMU485_VALID_QUAT) != 0u) {
    if (!any || ((int32_t)(self->meta.t_quat_ms - newest) > 0)) {
      newest = self->meta.t_quat_ms;
    }
    any = true;
  }

  if (!any) {
    return true;
  }
  const uint32_t limit = (max_age_ms > 0u) ? max_age_ms : self->cfg.stale_ms;
  return ((uint32_t)(now_ms - newest) > limit);
}

static uint32_t imu485_calc_wait_ms(const IMU485_t *self, uint32_t now_ms,
                                    uint32_t offline_ms)
{
  if (rx_ring_head_ != rx_ring_tail_) {
    return 0u;
  }

  uint32_t wait_ms = (offline_ms > 0u) ? MIN(offline_ms, 20u) : 10u;

  if (self->active_rx != 0u) {
    if (self->active_stop_state == 2u) {
      const uint32_t elapsed = (uint32_t)(now_ms - self->active_stop_sent_ms);
      const uint32_t rem = (elapsed < (uint32_t)self->cfg.active_stop_timeout_ms)
                               ? ((uint32_t)self->cfg.active_stop_timeout_ms - elapsed)
                               : 1u;
      wait_ms = MIN(wait_ms, rem);
    }
    return (wait_ms > 0u) ? wait_ms : 1u;
  }

  if (self->tx_inflight != 0u) {
    const uint32_t elapsed = (uint32_t)(now_ms - self->tx_start_ms);
    const uint32_t rem = (elapsed < (uint32_t)self->cfg.timeout_ms)
                             ? ((uint32_t)self->cfg.timeout_ms - elapsed)
                             : 1u;
    wait_ms = MIN(wait_ms, rem);
  } else if (self->auto_cycle != 0u) {
    const uint32_t gap = (uint32_t)(now_ms - self->last_done_ms);
    const uint32_t rem = (gap < (uint32_t)self->cfg.auto_cycle_gap_ms)
                             ? ((uint32_t)self->cfg.auto_cycle_gap_ms - gap)
                             : 1u;
    wait_ms = MIN(wait_ms, rem);
  }

  return (wait_ms > 0u) ? wait_ms : 1u;
}

void imu_485_update(IMU485_t *self, uint32_t timeout_ms)
{
  if (self == NULL) {
    return;
  }

  const uint32_t offline_ms =
      (timeout_ms > 0u) ? timeout_ms : self->cfg.stale_ms;

  uint32_t now_ms = imu485_now_ms();
  imu485_drain_ring(self, now_ms);
  imu485_poll(self, now_ms);
  self->online_ = !imu_485_is_stale(self, now_ms, offline_ms);

  const uint32_t wait_ms = imu485_calc_wait_ms(self, now_ms, offline_ms);
  if (wait_ms > 0u) {
    uint32_t notify_value = 0u;
    (void)xTaskNotifyWait(0u, SIGNAL_IMU485_RAW_READY, &notify_value,
                          pdMS_TO_TICKS(wait_ms));
  }

  now_ms = imu485_now_ms();
  imu485_drain_ring(self, now_ms);
  imu485_poll(self, now_ms);
  self->online_ = !imu_485_is_stale(self, now_ms, offline_ms);

  if (STM32UART_GetLastError(&self->uart_) != OK) {
    taskENTER_CRITICAL();
    rx_ring_head_ = 0u;
    rx_ring_tail_ = 0u;
    rx_error_reset_ = false;
    self->rx_len = 0u;
    imu_last_rx_error_count_ = STM32UART_GetRxErrorCount(&self->uart_);
    taskEXIT_CRITICAL();
    (void)STM32UART_RestartRxDMA(&self->uart_);
  }
}

/* ------------------------------------------------------------------------- */
/* 主动请求、命令帧与扩展寄存器                                               */
/* ------------------------------------------------------------------------- */
err_t imu_485_request(IMU485_t *self, imu485_rid_t rid)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (rid >= IMU485_RID_COUNT) {
    return ARG_ERR;
  }
  if ((self->active_rx != 0u) || (self->tx_inflight != 0u)) {
    self->last_error_ = BUSY;
    return BUSY;
  }
  return imu485_start_tx(self, 0u, IMU485_DATA_TYPE_DATA, (uint8_t)rid,
                         IMU485_OP_READ, NULL, self->cfg.retry,
                         imu485_now_ms());
}

err_t imu_485_send_cmd(IMU485_t *self, uint8_t rid, uint8_t op,
                       const uint8_t data[IMU485_DATA_LEN], bool allow_retry)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if ((rid >= (uint8_t)IMU485_CMD_COUNT) &&
      ((rid < (uint8_t)IMU485_REG_MODE) ||
       (rid > (uint8_t)IMU485_REG_CAN_RECOVERY))) {
    return ARG_ERR;
  }
  if ((op != IMU485_OP_READ) && (op != IMU485_OP_WRITE)) {
    return ARG_ERR;
  }
  if ((self->active_rx != 0u) || (self->tx_inflight != 0u)) {
    self->last_error_ = BUSY;
    return BUSY;
  }

  const uint8_t retry = allow_retry ? self->cfg.retry : 0u;
  return imu485_start_tx(self, 1u, IMU485_DATA_TYPE_CMD, rid, op, data, retry,
                         imu485_now_ms());
}

bool imu_485_get_cmd_result(IMU485_t *self, imu485_frame_t *out)
{
  if ((self == NULL) || (out == NULL) || (self->cmd_result_pending == 0u)) {
    return false;
  }
  *out = self->cmd_result;
  self->cmd_result_pending = 0u;
  return true;
}

static err_t imu485_cmd_write0(IMU485_t *self, uint8_t rid)
{
  return imu_485_send_cmd(self, rid, IMU485_OP_WRITE, NULL, false);
}

err_t imu_485_cmd_reboot(IMU485_t *self)
{
  return imu485_cmd_write0(self, (uint8_t)IMU485_CMD_REBOOT);
}

err_t imu_485_cmd_save(IMU485_t *self)
{
  return imu485_cmd_write0(self, (uint8_t)IMU485_CMD_SAVE);
}

err_t imu_485_cmd_yaw_zero(IMU485_t *self)
{
  return imu485_cmd_write0(self, (uint8_t)IMU485_CMD_YAW_ZERO);
}

err_t imu_485_cmd_gyro_cal(IMU485_t *self)
{
  return imu485_cmd_write0(self, (uint8_t)IMU485_CMD_GYRO_CAL);
}

err_t imu_485_cmd_accel_cal(IMU485_t *self)
{
  return imu485_cmd_write0(self, (uint8_t)IMU485_CMD_ACCEL_CAL);
}

err_t imu_485_cmd_factory_reset(IMU485_t *self)
{
  return imu485_cmd_write0(self, (uint8_t)IMU485_CMD_FACTORY_RESET);
}

err_t imu_485_cmd_set_baud_index(IMU485_t *self, uint8_t index)
{
  uint8_t data[IMU485_DATA_LEN];

  if (index > IMU485_BAUD_INDEX_MAX) {
    return ARG_ERR;
  }
  memset(data, 0, sizeof(data));
  data[0] = index;
  return imu_485_send_cmd(self, (uint8_t)IMU485_CMD_BAUD_INDEX, IMU485_OP_WRITE,
                          data, false);
}

err_t imu_485_cmd_set_interval_ms(IMU485_t *self, uint16_t interval_ms)
{
  uint8_t data[IMU485_DATA_LEN];

  if ((interval_ms < IMU485_INTERVAL_MS_MIN) ||
      (interval_ms > IMU485_INTERVAL_MS_MAX)) {
    return ARG_ERR;
  }
  memset(data, 0, sizeof(data));
  data[0] = (uint8_t)(interval_ms & 0xFFu);
  data[1] = (uint8_t)((interval_ms >> 8) & 0xFFu);
  return imu_485_send_cmd(self, (uint8_t)IMU485_CMD_INTERVAL, IMU485_OP_WRITE,
                          data, false);
}

err_t imu_485_cmd_set_com_type(IMU485_t *self, uint8_t com_type)
{
  uint8_t data[IMU485_DATA_LEN];

  if (com_type > IMU485_COM_TYPE_CAN) {
    return ARG_ERR;
  }
  memset(data, 0, sizeof(data));
  data[0] = com_type;
  return imu_485_send_cmd(self, (uint8_t)IMU485_CMD_COM_TYPE, IMU485_OP_WRITE,
                          data, false);
}

err_t imu_485_cmd_read_reg(IMU485_t *self, uint8_t rid)
{
  if ((rid != (uint8_t)IMU485_CMD_ACTIVE_MODE) &&
      (rid != (uint8_t)IMU485_CMD_BAUD_INDEX) &&
      (rid != (uint8_t)IMU485_CMD_INTERVAL) &&
      (rid != (uint8_t)IMU485_CMD_COM_TYPE)) {
    return ARG_ERR;
  }
  return imu_485_send_cmd(self, rid, IMU485_OP_READ, NULL, true);
}

/* ------------------------------------------------------------------------- */
/* RS485 主动上报控制（单机直连）                                             */
/* ------------------------------------------------------------------------- */
err_t imu_485_cmd_set_active_report(IMU485_t *self, bool enable)
{
  uint8_t data[IMU485_DATA_LEN];

  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->active_rx != 0u) {
    return BUSY;
  }
  memset(data, 0, sizeof(data));
  data[0] = enable ? 1u : 0u;
  return imu_485_send_cmd(self, (uint8_t)IMU485_CMD_ACTIVE_MODE,
                          IMU485_OP_WRITE, data, false);
}

err_t imu_485_cmd_read_active_report(IMU485_t *self)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->active_rx != 0u) {
    return BUSY;
  }
  return imu_485_send_cmd(self, (uint8_t)IMU485_CMD_ACTIVE_MODE, IMU485_OP_READ,
                          NULL, true);
}

err_t imu_485_active_result(const imu485_frame_t *fr, uint8_t *enable)
{
  if ((fr == NULL) || (enable == NULL)) {
    return PTR_NULL;
  }
  if ((fr->data_type != IMU485_DATA_TYPE_CMD) ||
      (fr->rid != (uint8_t)IMU485_CMD_ACTIVE_MODE) ||
      (fr->op != IMU485_OP_READ) || (fr->ack != IMU485_ACK_SUCCESS)) {
    return STATE_ERR;
  }
  *enable = (fr->data[0] != 0u) ? 1u : 0u;
  return OK;
}

err_t imu_485_set_active_rx(IMU485_t *self, bool enable)
{
  if (self == NULL) {
    return PTR_NULL;
  }
  if (enable) {
    if (self->tx_inflight != 0u) {
      return BUSY;
    }
    self->auto_cycle = 0u;
    self->active_rx = 1u;
    self->active_auto_detected = 0u;
    self->active_stop_state = 0u;
    self->active_group_frames = 0u;
    self->active_last_group_ms = imu485_now_ms();
  } else {
    self->active_rx = 0u;
    self->active_auto_detected = 0u;
    self->active_stop_state = 0u;
  }
  return OK;
}

err_t imu_485_active_request_stop(IMU485_t *self)
{
  uint8_t data[IMU485_DATA_LEN];

  if (self == NULL) {
    return PTR_NULL;
  }
  if (self->active_rx == 0u) {
    return STATE_ERR;
  }

  memset(data, 0, sizeof(data));
  err_t rc = imu_485_build_frame(
      self->active_stop_frame, self->cfg.slave_id, IMU485_DATA_TYPE_CMD,
      (uint8_t)IMU485_CMD_ACTIVE_MODE, IMU485_OP_WRITE, data);
  if (rc != OK) {
    return rc;
  }
  self->active_stop_state = 1u;
  self->active_stop_sent_ms = imu485_now_ms();
  return OK;
}

err_t imu_485_get_active_stats(const IMU485_t *self,
                               imu485_active_stats_t *out)
{
  if ((self == NULL) || (out == NULL)) {
    return PTR_NULL;
  }
  out->frames = self->active_frames;
  out->groups = self->active_groups;
  out->last_group_frames = self->active_last_group_frames;
  out->last_group_ms = self->active_last_group_ms;
  out->stop_sent = self->active_stop_sent;
  out->stop_ack = self->active_stop_ack;
  out->stop_timeout = self->active_stop_timeout;
  out->running = self->active_rx;
  out->stop_armed = (self->active_stop_state != 0u) ? 1u : 0u;
  return OK;
}

/* ------------------------------------------------------------------------- */
/* 扩展配置 / 诊断寄存器（0x20 ~ 0x3B）                                       */
/* ------------------------------------------------------------------------- */
err_t imu_485_read_reg32(IMU485_t *self, uint8_t reg)
{
  uint8_t data[IMU485_DATA_LEN];

  if ((reg < IMU485_REG_MODE) || (reg > IMU485_REG_CAN_RECOVERY)) {
    return ARG_ERR;
  }
  memset(data, 0, sizeof(data));
  return imu_485_send_cmd(self, reg, IMU485_OP_READ, data, true);
}

err_t imu_485_write_reg32(IMU485_t *self, uint8_t reg, uint32_t value)
{
  uint8_t data[IMU485_DATA_LEN];

  if ((reg < IMU485_REG_MODE) || (reg > IMU485_REG_CAN_RECOVERY)) {
    return ARG_ERR;
  }
  memset(data, 0, sizeof(data));
  imu485_put_u32_le(data, value);
  return imu_485_send_cmd(self, reg, IMU485_OP_WRITE, data, false);
}

err_t imu_485_result_u32(const imu485_frame_t *fr, uint32_t *value)
{
  if ((fr == NULL) || (value == NULL)) {
    return PTR_NULL;
  }
  *value = (uint32_t)fr->data[0] | ((uint32_t)fr->data[1] << 8) |
           ((uint32_t)fr->data[2] << 16) | ((uint32_t)fr->data[3] << 24);
  return OK;
}

err_t imu_485_result_f32(const imu485_frame_t *fr, float *value)
{
  if ((fr == NULL) || (value == NULL)) {
    return PTR_NULL;
  }
  const float v = imu485_read_f32_le(fr->data);
  if (!imu485_f32_sane(v, IMU485_SANITY_LIMIT)) {
    return FAILED;
  }
  *value = v;
  return OK;
}

err_t imu_485_enter_setting(IMU485_t *self)
{
  return imu_485_write_reg32(self, IMU485_REG_MODE, IMU485_MODE_SETTING);
}

err_t imu_485_exit_setting(IMU485_t *self)
{
  return imu_485_write_reg32(self, IMU485_REG_MODE, IMU485_MODE_NORMAL);
}
