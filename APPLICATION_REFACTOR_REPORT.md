# STM32G431 + ESP32 HMI 应用层重构报告

## 结果

本次改动只整理自维护应用层，没有替换 STM32CubeMX/MCSDK 生成的电机驱动核心，也没有执行编译、烧录或代码生成。

STM32 调用方向：

```text
CAN_STM / USART_STM / MCP_STM
              ↓
         CommMgr_STM
              ↓
         MotorMgr_STM
              ↓
            MCSDK
```

ESP32 调用方向：

```text
LVGL / MQTT
     ↓
CommMgr_ESP
  ↙       ↘
CAN_ESP  USART_ESP
```

## STM32 文件

- `User/motor_types_STM.h`：统一控制模式与电机状态快照。
- `User/motor_mgr_STM.*`：唯一允许直接调用电机控制 MCSDK API 的应用模块。
- `User/comm_mgr_STM.*`：三个通信适配器的统一命令入口与周期调度入口。
- `User/can_STM.*`：复用 CubeMX 的 FDCAN1 句柄，负责应用过滤器、Bus-Off 恢复、CAN 命令和三类遥测帧。
- `User/usart_STM.*`：复用 CubeMX 的 USART3/DMA 句柄，负责循环 RX DMA 启动、CRC、命令和遥测帧。
- `User/mcp_STM.*`：MCP 用户回调和固定 88 字节 Qt 遥测。
- `User/motor_can_protocol.h`、`User/motor_uart_protocol.h`：双方共用协议。

根 `CMakeLists.txt` 只显式列出上述应用层源文件。FDCAN HAL 源文件、模块宏、USART3、DMA、GPIO 和外设时钟均由 CubeMX 生成代码及 `cmake/stm32cubemx/CMakeLists.txt` 管理，应用层不再重复配置。

## 保留的已验证行为

- 速度上限仍为 ±2600 rpm。
- CAN 速度斜坡仍为 150 ms。
- CAN 最近单圈位置仍按 18000 cdeg/s 计算，最短 200 ms。
- USART 速度斜坡仍为 150 ms，最近单圈位置仍使用 1000 ms。
- 位置轨迹替换前仍复位运行期误差历史。
- 进入 RUN、切换位置/速度模式时仍保留原工程的 PID 运行期复位和位置保持行为；没有改动 PID 增益。
- USART3 仍使用循环 DMA，USART2 仍专用于 MCSDK ASPEP/MCP。
- CAN Bus-Off 恢复、链路超时、命令拒绝、CRC 和通信诊断仍保留。
- MCP 扩展协议版本、命令码、回调 ID、遥测 magic 和 88 字节布局保持不变。

## ESP_HMI 文件职责

```text
ESP_HMI/main/
├── main.c
├── bsp/
├── comm/
│   ├── can_ESP.*
│   ├── usart_ESP.*
│   ├── comm_mgr_ESP.*
│   ├── motor_can_protocol.h
│   └── motor_uart_protocol.h
├── network/
└── ui/
    ├── motor_ui.*
    ├── motor_ui_events.*
    └── motor_ui_style.*
```

`main/CMakeLists.txt` 显式列出各职责目录的源文件。UI 和 MQTT 只调用 `CommMgr_ESP`，不会绕过仲裁层直接操作 CAN 或 USART。

## UI 删除项

已完整删除原有 `SPEED CURVE` 与 `CURRENT CURVE`：

- 页面枚举与导航按钮；
- 图表对象、series、刻度、缓存、量程和刷新状态；
- 所有 `lv_chart_*` 调用；
- 速度图表专用滑动调速事件；
- 图表专用 style 接口。

速度、电流文本反馈、控制命令和 MQTT 遥测未删除。

## 注释范围

所有 `User/` 与 `ESP_HMI/main/` 自维护 C/C 头文件函数均补齐或保留中文 Doxygen 注释，包括：

- 静态辅助函数和协议内联函数；
- FreeRTOS 任务、ESP-IDF 回调；
- LVGL UI 创建、刷新、event 和 style 函数；
- BSP、Wi-Fi、MQTT 与通信管理函数。

STM32CubeMX、ST MCSDK、HAL、ESP-IDF 和第三方组件源码没有批量改写。

## 静态验收约束

- 参数文件必须继续与 `E:/MotorControl/MCSDK_FOC_MIX` 的已验证版本保持 SHA-256 一致。
- STM32 与 ESP32 的两个协议头必须分别保持字节一致。
- 平台文件后缀只允许 `_STM`、`_ESP`，不允许小写平台后缀。
- `CAN_STM`、`USART_STM`、`MCP_STM` 不得直接调用电机控制 MCSDK API。
- UI/MQTT 不得直接调用 `CAN_ESP` 或 `USART_ESP`。
- 工程中不得恢复动态图表符号或 `lv_chart_*` 调用。

本报告只记录静态重构结果；最终编译、下载和硬件回归由工程维护者执行。
