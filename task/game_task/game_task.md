# game_task

裁判系统任务.初始化 USART1 上的裁判系统/图传接收逻辑,周期性解析数据帧.

## 任务流程

```c
void game_task(void* argument) {
  static Game_t game_instance;
  remote_control_data_init();                        /* 初始化自定义数据结构 */
  err_t status = Game_Init(&game_instance, &huart1, &custom_robot_data);
  game = &game_instance;
  game->thread_alert = xTaskGetCurrentTaskHandle();

  if (status == OK) status = Game_Start(game);
  ASSERT(status == OK);
  if (status != OK) { vTaskDelete(NULL); return; }

  for (;;) {
    Game_Update(game, GAME_TASK_UPDATE_TIMEOUT_MS);  /* 10ms 无新数据保持上次解析结果 */
  }
}
```

## 对外接口

| 接口 | 说明 |
| --- | --- |
| `void game_task(void* argument)` | 任务入口,CubeMX 在 `freertos.c` 中创建同名线程 |
| `Game_t* game` | 全局对象指针,其他模块经此读取裁判系统/图传数据 |

## 依赖模块

- `modules/game`:裁判系统协议解析与 CRC 校验(`crc_ref`),底层走 `bsp_uart`
- `Core/Src/usart.c`:`huart1`

## 注意事项

- 串口使用 `huart1`,与 `vt13_task`（`huart7`）独立分配,互不冲突.
- `Game_Update()` 阻塞等待任务通知;超时无新数据时保持最近一次合法解析结果,不置离线(在线判断由上层按需扩展).
