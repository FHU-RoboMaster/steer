# vt13_task

VT13 图传遥控器任务.初始化 UART7 上的 VT13 接收逻辑,周期性刷新解析结果.

## 任务流程

```c
void vt13_task(void* argument) {
  static vt13_t instance;
  err_t status = vt13_init(&instance, &huart7);
  vt13 = &instance;                          /* 全局指针,供控制模块读取 */
  if (status == OK) status = vt13_start(vt13);
  ASSERT(status == OK);
  if (status != OK) { vTaskDelete(NULL); return; }

  for (;;) vt13_update(vt13, 20u);           /* 20ms 超时刷新 */
}
```

## 对外接口

| 接口 | 说明 |
| --- | --- |
| `void vt13_task(void* argument)` | 任务入口 |
| `vt13_t* vt13` | 全局对象指针,`calculate/vt13_calculate` 的控制模块经 `vt13->cmd` 读取遥控数据 |

## 依赖模块

- `modules/vt13`:VT13 协议解析(`vt13_init` / `vt13_start` / `vt13_update`)
- `Core/Src/usart.c`:`huart7`

## 注意事项

- **当前未参与编译**:`task/CMakeLists.txt` 中 `vt13_task` 条目被注释,固件中由 `freertos.c` 的 `__weak` 空实现代替.
- 串口独立使用 `huart7`,与 `game_task`（`huart1`）不冲突;实机接入 VT13 图传时请在 CubeMX 按协议确认 `UART7` 波特率（常见为 921600）.
- 头文件直接以相对路径 `../../modules/vt13/vt13.h` 包含,目录移动时需同步修改.
