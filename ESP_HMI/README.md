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

## MQTT 自动测试

ESP32 新增 `PID TEST` 本地页面。页面显示当前缓存的速度、位置、Iq、Id PID，
固定 7 秒时长、500 RPM/90.00° 测试目标和测试状态，并提供 `SPEED CONTROL`
与 `POSITION CONTROL` 两个启动按钮。点击后 ESP32 自动选择 CAN，并在本地执行
“应用缓存 PID、设置模式、启动、等待 RUN、设置目标、记录、停止、上传”的完整
状态机。MQTT `run_test` 不再启动测试。

STM32 每 2 ms 发送 0x180、0x181、0x182 三帧完整遥测。ESP32 以 0x182
到达时间作为一组完成样本的时间戳。每次测试开始前只从 PSRAM 分配 3600 组
紧凑样本（约 57.6 KiB），不会回退占用内部 RAM；PSRAM 分配失败时不会启动电机。停止后通过
`motor/control/test/data` 以 QoS 1 二进制分块回传，通过
`motor/control/test/status` 发布阶段、点数和实际平均采样周期。2 ms 周期在
500 kbit/s 经典 CAN 下给控制命令、重发和仲裁保留了总线余量。

DNESP32S3B 的 N16R8 兼容模组使用 8 MB Octal PSRAM。工程在 `sdkconfig` 和
`sdkconfig.defaults` 中启用 80 MHz Octal PSRAM、启动初始化、内存测试和
`heap_caps` 分配。网关初始化时会在串口终端打印 PSRAM 总容量和剩余容量。

测试运行期间暂停周期 `motor/control/telemetry` 发布；数据回传按 50 ms/块
节流，防止 MQTT outbox 和 Wi-Fi 发送缓冲区被突发数据塞满。

PID 与测试启动相互独立。Qt 的 `set_pid` 在电机停止时更新 ESP32 运行时缓存；
CAN 已在线时立即临时下发，否则在下一次本地测试启动前自动下发。速度、Iq、Id
仅使用 Kp/Ki，位置使用 Kp/Ki/Kd。页面状态依次显示 `TESTING`、
`TEST COMPLETE - UPLOADING`、`UPLOAD SUCCESS`，错误时显示具体原因。
