#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "comp_def.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* DM-IMU-L1 RS485 物理帧协议常量（24 字节 + CRC-8/SAE-J1850）                */
/* ------------------------------------------------------------------------- */
#define IMU485_FRAME_LEN (24u)
#define IMU485_FRAME_SIZE IMU485_FRAME_LEN
#define IMU485_HEADER (0xA5u)
#define IMU485_TAIL (0x5Au)
#define IMU485_DATA_OFFSET (5u)
#define IMU485_DATA_LEN (16u)
#define IMU485_ACK_OFFSET (21u)
#define IMU485_CRC_OFFSET (22u)
#define IMU485_CRC_START (1u)
#define IMU485_CRC_LEN (21u)

#define IMU485_DMA_BUFFER_SIZE (256u)
#define IMU485_TX_DMA_BUFFER_SIZE (64u)
#define IMU485_RX_RING_SIZE (512u)

/* 偏移 1：数据类型 */
#define IMU485_DATA_TYPE_CMD (0x0Cu)
#define IMU485_DATA_TYPE_DATA (0x0Du)

/* 偏移 4：操作类型 */
#define IMU485_OP_READ (0x00u)
#define IMU485_OP_WRITE (0x01u)

/* 偏移 21：应答码 / 主动上报分组标志 */
#define IMU485_ACK_SUCCESS (0u)
#define IMU485_ACK_ERROR (1u)
#define IMU485_ACK_DATA_TYPE_ERR (2u)
#define IMU485_ACK_OPERATE_ERR (3u)
#define IMU485_ACK_REG_NOT_EXIST (4u)
#define IMU485_ACK_REG_UNREADABLE (5u)
#define IMU485_ACK_REG_UNWRITABLE (6u)
#define IMU485_ACK_GROUP_FRAME (0x80u)
#define IMU485_ACK_GROUP_LAST (0x81u)

/* 通信类型（命令 RID 9） */
#define IMU485_COM_TYPE_USB (0u)
#define IMU485_COM_TYPE_RS485 (1u)
#define IMU485_COM_TYPE_CAN (2u)

/* 模式（扩展寄存器 0x20） */
#define IMU485_MODE_NORMAL (0u)
#define IMU485_MODE_SETTING (1u)

/* 主动输出数据类型位图（扩展寄存器 0x29） */
#define IMU485_OUT_BIT_QUAT (0x01u)
#define IMU485_OUT_BIT_EULER (0x02u)
#define IMU485_OUT_BIT_GYRO (0x04u)
#define IMU485_OUT_BIT_ACCEL_TEMP (0x08u)

/* 链路参数范围 */
#define IMU485_BAUD_INDEX_MAX (12u)
#define IMU485_INTERVAL_MS_MIN (1u)
#define IMU485_INTERVAL_MS_MAX (1000u)

/* 扩展配置 / 诊断寄存器（0x20 ~ 0x3B） */
#define IMU485_REG_MODE (0x20u)
#define IMU485_REG_SENSOR_RANGE (0x21u)
#define IMU485_REG_ORIENTATION (0x22u)
#define IMU485_REG_ACCEL_LPF (0x23u)
#define IMU485_REG_GYRO_LPF (0x24u)
#define IMU485_REG_ACCEL_NOTCH (0x25u)
#define IMU485_REG_ACCEL_NOTCH_EN (0x26u)
#define IMU485_REG_TEMP_CTRL (0x27u)
#define IMU485_REG_TEMP_TARGET (0x28u)
#define IMU485_REG_OUTPUT_BITMAP (0x29u)
#define IMU485_REG_REPORT_INTERVAL (0x2Au)
#define IMU485_REG_CONFIG_STATUS (0x2Bu)
#define IMU485_REG_SLAVE_ID (0x2Cu)
#define IMU485_REG_MASTER_ID (0x2Du)
#define IMU485_REG_CAN_BAUD_INDEX (0x2Eu)
#define IMU485_REG_RS485_BAUD_INDEX (0x2Fu)
#define IMU485_REG_TEMPERATURE (0x31u)
#define IMU485_REG_CAN_RECOVERY (0x3Bu)

/* 默认运行参数 */
#define IMU485_DEFAULT_SLAVE_ID (1u)
#define IMU485_DEFAULT_TIMEOUT_MS (80u)
#define IMU485_DEFAULT_RETRY (2u)
#define IMU485_DEFAULT_AUTO_CYCLE_GAP_MS (2u)
#define IMU485_DEFAULT_ACTIVE_STOP_MS (100u)
#define IMU485_OFFLINE_TIMEOUT_MS (250u)

/* 数据标识：RS485 数据帧 RID（data_type = 0x0D） */
typedef enum {
  IMU485_RID_ACCEL = 0, /* 加速度 X/Y/Z，单位 m/s^2 */
  IMU485_RID_GYRO = 1,  /* 角速度 X/Y/Z，单位 rad/s */
  IMU485_RID_EULER = 2, /* 欧拉角 roll/pitch/yaw，单位 deg */
  IMU485_RID_QUAT = 3,  /* 四元数 w/x/y/z */
  IMU485_RID_COUNT = 4
} imu485_rid_t;

/* 数据有效位图 */
#define IMU485_VALID_ACCEL (1u << IMU485_RID_ACCEL)
#define IMU485_VALID_GYRO (1u << IMU485_RID_GYRO)
#define IMU485_VALID_EULER (1u << IMU485_RID_EULER)
#define IMU485_VALID_QUAT (1u << IMU485_RID_QUAT)
#define IMU485_VALID_ALL \
  (IMU485_VALID_ACCEL | IMU485_VALID_GYRO | IMU485_VALID_EULER | IMU485_VALID_QUAT)

/* 命令帧 RID（data_type = 0x0C） */
typedef enum {
  IMU485_CMD_REBOOT = 0,
  IMU485_CMD_SAVE = 1,
  IMU485_CMD_YAW_ZERO = 2,
  IMU485_CMD_GYRO_CAL = 3,
  IMU485_CMD_ACCEL_CAL = 4,
  IMU485_CMD_FACTORY_RESET = 5,
  IMU485_CMD_ACTIVE_MODE = 6,
  IMU485_CMD_BAUD_INDEX = 7,
  IMU485_CMD_INTERVAL = 8,
  IMU485_CMD_COM_TYPE = 9,
  IMU485_CMD_COUNT = 10
} imu485_cmd_t;

/* 兼容既有控制代码的姿态数据结构 */
typedef struct {
  float accel[3];
  float gyro[3];
  float roll;
  float pitch;
  float yaw;
  float quaternion[4];
} dm_imu_t_t;

typedef struct {
  uint8_t data_type;
  uint8_t slave_id;
  uint8_t rid;
  uint8_t op;
  uint8_t ack;
  uint8_t data[IMU485_DATA_LEN];
} imu485_frame_t;

typedef struct {
  uint8_t slave_id;
  uint8_t retry;
  uint16_t timeout_ms;
  uint16_t auto_cycle_gap_ms;
  uint16_t active_stop_timeout_ms;
  uint32_t stale_ms;
} imu485_cfg_t;

typedef struct {
  uint32_t t_accel_ms;
  uint32_t t_gyro_ms;
  uint32_t t_euler_ms;
  uint32_t t_quat_ms;
  uint16_t valid;
} imu485_meta_t;

typedef struct {
  uint32_t rx_ok;
  uint32_t rx_bad;
  uint32_t rx_crc_err;
  uint32_t tx_timeout;
  uint32_t tx_retry;
  uint32_t tx_err;
  uint32_t rx_dropped;
  uint32_t rx_overflow;
} imu485_stats_t;

typedef struct {
  uint32_t frames;
  uint32_t groups;
  uint32_t last_group_frames;
  uint32_t last_group_ms;
  uint32_t stop_sent;
  uint32_t stop_ack;
  uint32_t stop_timeout;
  uint8_t running;
  uint8_t stop_armed;
} imu485_active_stats_t;

typedef struct {
  TaskHandle_t thread_alert;
  dm_imu_t_t data;
  imu485_meta_t meta;
  bool online_;
  STM32UART_t uart_;
  STM32UARTFrameTx_t uart_tx_;
  err_t init_error_;
  err_t last_error_;
  imu485_cfg_t cfg;
  imu485_stats_t stats;

  /* RS485 事务状态机（数据读 / 命令帧共用） */
  uint8_t tx_inflight;
  uint8_t tx_kind; /* 0 = 数据读，1 = 命令帧 */
  uint8_t tx_rid;
  uint8_t tx_op;
  uint8_t tx_data_type;
  uint8_t tx_retry_left;
  uint8_t auto_cycle;
  uint8_t auto_rid;
  uint32_t tx_start_ms;
  uint32_t last_done_ms;
  uint8_t tx_frame[IMU485_FRAME_LEN];

  /* RS485 接收重组缓冲 */
  uint8_t rx_frame[IMU485_FRAME_LEN];
  uint16_t rx_len;

  /* 命令应答缓存 */
  uint8_t cmd_result_pending;
  imu485_frame_t cmd_result;

  /* RS485 主动上报状态 */
  uint8_t active_rx;
  uint8_t active_auto_detected;
  uint8_t active_stop_state; /* 0 空闲 / 1 等组尾 / 2 已发关闭帧等 ACK */
  uint8_t active_group_frames;
  uint8_t active_stop_frame[IMU485_FRAME_LEN];
  uint32_t active_last_group_ms;
  uint32_t active_stop_sent_ms;
  uint32_t active_frames;
  uint32_t active_groups;
  uint32_t active_last_group_frames;
  uint32_t active_stop_sent;
  uint32_t active_stop_ack;
  uint32_t active_stop_timeout;

  /* 最近一帧合法 24 字节原帧（便于调试观测） */
  uint8_t raw_frame[IMU485_FRAME_LEN];
} IMU485_t;

/* 兼容既有控制代码的全局姿态缓存 */
extern dm_imu_t_t imu_485;

/* ------------------------------------------------------------------------- */
/* 纯编解码与校验接口                                                         */
/* ------------------------------------------------------------------------- */
uint8_t imu_485_crc8_j1850(const uint8_t *data, uint16_t len);
err_t imu_485_build_frame(uint8_t out[IMU485_FRAME_LEN], uint8_t slave_id,
                          uint8_t data_type, uint8_t rid, uint8_t op,
                          const uint8_t data[IMU485_DATA_LEN]);
err_t imu_485_build_req(uint8_t out[IMU485_FRAME_LEN], uint8_t slave_id,
                        imu485_rid_t rid);
err_t imu_485_parse_frame(const uint8_t in[IMU485_FRAME_LEN], uint8_t slave_id,
                          imu485_frame_t *out);
err_t imu_485_parse_rsp(const uint8_t in[IMU485_FRAME_LEN], uint8_t slave_id,
                        imu485_rid_t rid, float out[4], uint8_t *out_n);
err_t imu_485_parse_active_frame(const uint8_t in[IMU485_FRAME_LEN],
                                 uint8_t slave_id, imu485_frame_t *out,
                                 uint8_t *out_last);

/* ------------------------------------------------------------------------- */
/* 设备生命周期与数据更新                                                     */
/* ------------------------------------------------------------------------- */
void imu_485_cfg_default(imu485_cfg_t *cfg);
err_t imu_485_init(IMU485_t *self, UART_HandleTypeDef *uart_handle);
err_t imu_485_init_with_cfg(IMU485_t *self, UART_HandleTypeDef *uart_handle,
                            const imu485_cfg_t *cfg);
err_t imu_485_start(IMU485_t *self);
void imu_485_update(IMU485_t *self, uint32_t timeout_ms);
void imu_485_enable_auto_cycle(IMU485_t *self, bool enable);
bool imu_485_is_stale(const IMU485_t *self, uint32_t now_ms,
                      uint32_t max_age_ms);

/* 兼容接口：解包单帧 24 字节 RS485 数据帧并写入全局 imu_485 */
void imu_485_data_unpack(const uint8_t *pData);

/* ------------------------------------------------------------------------- */
/* RS485 请求、命令与主动上报接口                                             */
/* ------------------------------------------------------------------------- */
err_t imu_485_request(IMU485_t *self, imu485_rid_t rid);
err_t imu_485_send_cmd(IMU485_t *self, uint8_t rid, uint8_t op,
                       const uint8_t data[IMU485_DATA_LEN], bool allow_retry);
bool imu_485_get_cmd_result(IMU485_t *self, imu485_frame_t *out);

err_t imu_485_cmd_reboot(IMU485_t *self);
err_t imu_485_cmd_save(IMU485_t *self);
err_t imu_485_cmd_yaw_zero(IMU485_t *self);
err_t imu_485_cmd_gyro_cal(IMU485_t *self);
err_t imu_485_cmd_accel_cal(IMU485_t *self);
err_t imu_485_cmd_factory_reset(IMU485_t *self);
err_t imu_485_cmd_set_baud_index(IMU485_t *self, uint8_t index);
err_t imu_485_cmd_set_interval_ms(IMU485_t *self, uint16_t interval_ms);
err_t imu_485_cmd_set_com_type(IMU485_t *self, uint8_t com_type);
err_t imu_485_cmd_read_reg(IMU485_t *self, uint8_t rid);

err_t imu_485_cmd_set_active_report(IMU485_t *self, bool enable);
err_t imu_485_cmd_read_active_report(IMU485_t *self);
err_t imu_485_active_result(const imu485_frame_t *fr, uint8_t *enable);
err_t imu_485_set_active_rx(IMU485_t *self, bool enable);
err_t imu_485_active_request_stop(IMU485_t *self);
err_t imu_485_get_active_stats(const IMU485_t *self,
                               imu485_active_stats_t *out);

err_t imu_485_read_reg32(IMU485_t *self, uint8_t reg);
err_t imu_485_write_reg32(IMU485_t *self, uint8_t reg, uint32_t value);
err_t imu_485_result_u32(const imu485_frame_t *fr, uint32_t *value);
err_t imu_485_result_f32(const imu485_frame_t *fr, float *value);
err_t imu_485_enter_setting(IMU485_t *self);
err_t imu_485_exit_setting(IMU485_t *self);

#ifdef __cplusplus
}
#endif
