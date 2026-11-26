#include "imu_fdcan.h"

#include <string.h>

#include "convert.h"

imu_t imu;

static uint16_t U16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void IMU_UpdateAccel(imu_t *target, const uint8_t *p) {
  for (uint8_t i = 0; i < 3u; ++i) {
    target->accel[i] = uint_to_float(U16LE(&p[2u + i * 2u]), ACCEL_CAN_MIN,
                                     ACCEL_CAN_MAX, 16u);
  }
}

static void IMU_UpdateGyro(imu_t *target, const uint8_t *p) {
  for (uint8_t i = 0; i < 3u; ++i) {
    target->gyro[i] =
        uint_to_float(U16LE(&p[2u + i * 2u]), GYRO_CAN_MIN, GYRO_CAN_MAX, 16u);
  }
}

static void IMU_UpdateEuler(imu_t *target, const uint8_t *p) {
  target->pitch =
      uint_to_float(U16LE(&p[2]), PITCH_CAN_MIN, PITCH_CAN_MAX, 16u);
  target->yaw = uint_to_float(U16LE(&p[4]), YAW_CAN_MIN, YAW_CAN_MAX, 16u);
  target->roll = uint_to_float(U16LE(&p[6]), ROLL_CAN_MIN, ROLL_CAN_MAX, 16u);
}

static void IMU_UpdateQuaternion(imu_t *target, const uint8_t *p) {
  uint16_t v[4];
  v[0] = (uint16_t)(((uint16_t)p[1] << 6) | ((p[2] & 0xFCu) >> 2));
  v[1] = (uint16_t)(((uint16_t)(p[2] & 0x03u) << 12) | ((uint16_t)p[3] << 4) |
                    ((p[4] & 0xF0u) >> 4));
  v[2] = (uint16_t)(((uint16_t)(p[4] & 0x0Fu) << 10) | ((uint16_t)p[5] << 2) |
                    ((p[6] & 0xC0u) >> 6));
  v[3] = (uint16_t)(((uint16_t)(p[6] & 0x3Fu) << 8) | p[7]);
  for (uint8_t i = 0; i < 4u; ++i) {
    target->q[i] = uint_to_float(v[i], QUATERNION_MIN, QUATERNION_MAX, 14u);
  }
}

static void IMU_UpdateDataTo(imu_t *target, const uint8_t *p) {
  if ((target == NULL) || (p == NULL)) {
    return;
  }
  switch (p[0]) {
  case ACCEL_DATA:
    IMU_UpdateAccel(target, p);
    break;
  case GYRO_DATA:
    IMU_UpdateGyro(target, p);
    break;
  case EULER_DATA:
    IMU_UpdateEuler(target, p);
    break;
  case QUAT_DATA:
    IMU_UpdateQuaternion(target, p);
    break;
  default:
    break;
  }
}

static void IMUFDCAN_RxCallback(STM32FDCAN_t *fdcan,
                                const BSP_FDCAN_Frame_t *frame, void *context) {
  IMUFDCAN_t *self = (IMUFDCAN_t *)context;
  if ((self == NULL) || (fdcan == NULL) || (frame == NULL) ||
      (self->fdcan_ != fdcan) || (frame->id_type_ != FDCAN_STANDARD_ID) ||
      (frame->frame_type_ != FDCAN_DATA_FRAME) ||
      (frame->fd_format_ != FDCAN_CLASSIC_CAN) ||
      (frame->size_ != BSP_FDCAN_CLASSIC_DATA_SIZE) ||
      (frame->id_ != self->mst_id)) {
    return;
  }

  if ((frame->data_[0] >= ACCEL_DATA) && (frame->data_[0] <= QUAT_DATA)) {
    const uint8_t slot = (uint8_t)(frame->data_[0] - ACCEL_DATA);
    memcpy(self->rx_slots_[slot], frame->data_, BSP_FDCAN_CLASSIC_DATA_SIZE);
    self->rx_pending_mask_ |= (uint8_t)(1u << slot);
  }
  memcpy(self->rx_last_frame_, frame->data_, BSP_FDCAN_CLASSIC_DATA_SIZE);
  self->rx_frame_ready_ = true;

  if (self->thread_alert != NULL) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(self->thread_alert, SIGNAL_IMUCAN_RAW_READY, eSetBits,
                       &woken);
    portYIELD_FROM_ISR(woken);
  }
}

static err_t IMU_SendCommand(uint8_t reg_id, uint8_t access, uint32_t data) {
  uint8_t payload[BSP_FDCAN_CLASSIC_DATA_SIZE] = {0xCCu, reg_id, access, 0xDDu,
                                                  0u,    0u,     0u,     0u};
  memcpy(&payload[4], &data, sizeof(data));
  if (imu.fdcan_handle == NULL) {
    return PTR_NULL;
  }
  return STM32FDCAN_SendStdByHandle(imu.fdcan_handle, imu.can_id, payload,
                                    sizeof(payload));
}

err_t imu_fdcan_init(IMUFDCAN_t *self, STM32FDCAN_t *fdcan, uint8_t can_id,
                     uint8_t mst_id) {
  if ((self == NULL) || (fdcan == NULL)) {
    return PTR_NULL;
  }
  if (fdcan->fdcan_handle_ == NULL) {
    return STATE_ERR;
  }

  memset(self, 0, sizeof(*self));
  self->fdcan_ = fdcan;
  self->can_id = can_id;
  self->mst_id = mst_id;
  self->data.can_id = can_id;
  self->data.mst_id = mst_id;
  self->data.fdcan_handle = fdcan->fdcan_handle_;
  self->init_error_ =
      STM32FDCAN_SubscribeRx(fdcan, IMUFDCAN_RxCallback, self);
  if (self->init_error_ == OK) {
    imu = self->data;
  }
  return self->init_error_;
}

void imu_fdcan_update(IMUFDCAN_t *self, uint32_t timeout_ms) {
  if (self == NULL) {
    return;
  }

  uint32_t notify_value = 0u;
  (void)xTaskNotifyWait(0u, SIGNAL_IMUCAN_RAW_READY, &notify_value,
                        pdMS_TO_TICKS(timeout_ms));

  /* FDCAN 在调度器启动前已由 can_device 开启；此时可能没有任务通知，
   * 但 ISR 已经缓存了帧，因此只要有完整帧就应当消费。 */
  bool ready = false;
  uint8_t pending_mask = 0u;
  uint8_t local_slots[4][BSP_FDCAN_CLASSIC_DATA_SIZE];
  uint8_t local_last[BSP_FDCAN_CLASSIC_DATA_SIZE];

  taskENTER_CRITICAL();
  ready = self->rx_frame_ready_;
  pending_mask = self->rx_pending_mask_;
  if (ready) {
    memcpy(local_last, self->rx_last_frame_, sizeof(local_last));
    self->rx_frame_ready_ = false;
  }
  if (pending_mask != 0u) {
    memcpy(local_slots, self->rx_slots_, sizeof(local_slots));
    self->rx_pending_mask_ = 0u;
  }
  taskEXIT_CRITICAL();

  if (ready) {
    memcpy(self->raw_data, local_last, sizeof(self->raw_data));
    if (pending_mask != 0u) {
      for (uint8_t slot = 0u; slot < 4u; ++slot) {
        if ((pending_mask & (uint8_t)(1u << slot)) != 0u) {
          IMU_UpdateDataTo(&self->data, local_slots[slot]);
        }
      }
    } else {
      IMU_UpdateDataTo(&self->data, self->raw_data);
    }
    imu = self->data;
    self->online_ = true;
  } else {
    self->online_ = false;
  }
}

void imu_init(uint8_t can_id, uint8_t mst_id,
              FDCAN_HandleTypeDef *fdcan_handle) {
  imu.can_id = can_id;
  imu.mst_id = mst_id;
  imu.fdcan_handle = fdcan_handle;
}

void imu_write_reg(uint8_t reg_id, uint32_t data) {
  (void)IMU_SendCommand(reg_id, CMD_WRITE, data);
}

void imu_read_reg(uint8_t reg_id) {
  (void)IMU_SendCommand(reg_id, CMD_READ, 0u);
}

void imu_reboot(void) { imu_write_reg(REBOOT_IMU, 0u); }
void imu_accel_calibration(void) { imu_write_reg(ACCEL_CALI, 0u); }
void imu_gyro_calibration(void) { imu_write_reg(GYRO_CALI, 0u); }
void imu_change_com_port(imu_com_port_e p) {
  imu_write_reg(CHANGE_COM, (uint32_t)p);
}
void imu_set_active_mode_delay(uint32_t d) { imu_write_reg(SET_DELAY, d); }
void imu_change_to_active(void) { imu_write_reg(CHANGE_ACTIVE, 1u); }
void imu_change_to_request(void) { imu_write_reg(CHANGE_ACTIVE, 0u); }
void imu_set_baud(imu_baudrate_e b) { imu_write_reg(SET_BAUD, (uint32_t)b); }
void imu_set_can_id(uint8_t id) { imu_write_reg(SET_CAN_ID, id); }
void imu_set_mst_id(uint8_t id) { imu_write_reg(SET_MST_ID, id); }
void imu_save_parameters(void) { imu_write_reg(SAVE_PARAM, 0u); }
void imu_restore_settings(void) { imu_write_reg(RESTORE_SETTING, 0u); }
void imu_request_accel(void) { imu_read_reg(ACCEL_DATA); }
void imu_request_gyro(void) { imu_read_reg(GYRO_DATA); }
void imu_request_euler(void) { imu_read_reg(EULER_DATA); }
void imu_request_quat(void) { imu_read_reg(QUAT_DATA); }

void IMU_UpdateData(uint8_t *p) { IMU_UpdateDataTo(&imu, p); }
