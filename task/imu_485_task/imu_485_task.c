#include "imu_485_task.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "comp_utils.h"
#include "imu_rs485.h"
#include "task.h"
#include "usart.h"

IMU485_t* imu_485_device = NULL;

void imu_485_task(void* argument) {
  RM_UNUSED(argument);

  LOG_DEBUG("IMU485任务", "任务开始初始化 RS485 IMU (USART2)");
  static IMU485_t instance;
  err_t status = imu_485_init(&instance, &huart2);
  imu_485_device = &instance;
  instance.thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) {
    status = imu_485_start(imu_485_device);
  }
  ASSERT(status == OK);
  if (status != OK) {
    LOG_ERROR("IMU485任务", "RS485 IMU 启动失败，错误码=%d", (int)status);
    vTaskDelete(NULL);
    return;
  }

  LOG_INFO("IMU485任务", "RS485 IMU 已启动，进入自动轮询与接收循环");

  for (;;) {
    imu_485_update(imu_485_device, IMU485_OFFLINE_TIMEOUT_MS);
  }
}
