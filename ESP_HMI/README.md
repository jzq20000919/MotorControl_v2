# ESP_HMI

这是从只读参考工程 `E:\MotorControl\ESP32_LVGL\ESP32_LVGL` 独立建立的 ESP32 HMI 工程，保留了原有 BSP、LVGL、Wi-Fi 与 MQTT 功能。

电机通信分层如下：

```text
LVGL / MQTT
    -> CommMgr_ESP
        -> CAN_ESP
        -> USART_ESP
```

- `main/comm/comm_mgr_ESP.*`：唯一的物理链路选择点，状态为 `NONE / CAN / UART`；UI 与 MQTT 只能使用其统一接口。
- `main/comm/can_ESP.*`：经典 CAN 500 kbit/s 的收发、协议编解码、收发器自检及 Bus-Off 处理。
- `main/comm/usart_ESP.*`：USART 的收发、异步重连、协议编解码及 CRC16 检查。
- `main/comm/motor_can_protocol.h` 与 `main/comm/motor_uart_protocol.h`：与 STM32 `User/` 中的副本保持字节一致。
- `main/bsp/`、`main/network/`、`main/ui/`：分别承载板级驱动、Wi-Fi/MQTT 与 LVGL 页面/事件/样式。

原 UI 中的 `SPEED CURVE` 和 `CURRENT CURVE` 两个动态图表及其手势、缓存和样式已经删除；速度、电流文本反馈和 MQTT 遥测仍然保留。

运行中切换速度/位置模式时，ESP32 只发送统一模式命令；STM32 `MotorMgr_STM` 负责保持旧工程已经验证的控制模式切换与轨迹复位行为。

`ZERO_POSITION` 仅为协议兼容保留；STM32 会拒绝该命令。需要回到零度时使用普通位置目标 `SET_POSITION=0`。
