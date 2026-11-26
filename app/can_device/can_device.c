#include "can_device.h"

#include "bsp_fdcan.h"
#include "chassis_control.h"
#include "fdcan.h"
#include "main.h"
#include "shoot_control.h"

/* 三路 FDCAN 控制块：
 * - FDCAN1：发射机构 / 云台电机
 * - FDCAN2：4 台底盘 M3508 电机
 * - FDCAN3：扩展设备（如达妙关节电机/外设） */
static STM32FDCAN_t fdcan1_instance;
static STM32FDCAN_t fdcan2_instance;
static STM32FDCAN_t fdcan3_instance;

/**
 * @brief 统一检查初始化步骤的返回码。
 * @param status 任一步骤的返回码；非 OK 即进入 Error_Handler()。
 */
static void can_device_check(err_t status) {
  if (status != OK) {
    Error_Handler();
  }
}

/**
 * @brief 完成三路 FDCAN 对象绑定、全通过滤器配置、设备注册与总线启动。
 *
 * 须在 RTOS 调度器启动前调用一次。先绑定全部 HAL 句柄并完成过滤器配置，
 * 使上层控制模块可通过 BSP 对象表反查对应 FDCAN 实例并注册 RX 订阅；
 * 全部设备注册完成后再统一启动三路总线。
 */
void can_device_Init(void) {
  can_device_check(STM32FDCAN_Init(&fdcan1_instance, &hfdcan1));
  can_device_check(STM32FDCAN_Init(&fdcan2_instance, &hfdcan2));
  can_device_check(STM32FDCAN_Init(&fdcan3_instance, &hfdcan3));

  can_device_check(
      STM32FDCAN_ConfigAcceptAllFilter(&fdcan1_instance, FDCAN_RX_FIFO1));
  can_device_check(
      STM32FDCAN_ConfigAcceptAllFilter(&fdcan2_instance, FDCAN_RX_FIFO0));
  can_device_check(
      STM32FDCAN_ConfigAcceptAllFilter(&fdcan3_instance, FDCAN_RX_FIFO0));

  can_device_check(FDCAN_RegisterAllDevices());
  can_device_check(FDCAN_StartAll());
}

err_t FDCAN_RegisterAllDevices(void) {
  err_t status;

  /* FDCAN2：底盘总线回调 + 4 台 M3508。 */
  status = chassis_control_init();
  if (status != OK) {
    return status;
  }

  /* FDCAN1：发射机构总线回调 + 2 台 M3508 + 1 台 M2006。 */
  status = shoot_control_init();
  if (status != OK) {
    return status;
  }

  return OK;
}

/**
 * @brief 启动全部 FDCAN 总线（FDCAN1 / FDCAN2 / FDCAN3）。
 *
 * 所有设备注册完成后才允许调用；调用后禁止新增 RX 订阅者。
 */
err_t FDCAN_StartAll(void) {
  err_t status = STM32FDCAN_Start(&fdcan1_instance);
  if (status != OK) {
    return status;
  }
  status = STM32FDCAN_Start(&fdcan2_instance);
  if (status != OK) {
    return status;
  }
  return STM32FDCAN_Start(&fdcan3_instance);
}
