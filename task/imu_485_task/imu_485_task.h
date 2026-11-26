#pragma once

#include "imu_rs485.h"

#ifdef __cplusplus
extern "C" {
#endif

extern IMU485_t* imu_485_device;

void imu_485_task(void* argument);

#ifdef __cplusplus
}
#endif
