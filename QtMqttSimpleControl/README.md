# QtMqttSimpleControl

Qt 6 Widgets MQTT 电机控制与运行曲线记录程序，从
`E:\MotorControl\QtMqttSimpleControl` 移植并扩展到当前工程。

## 功能

- 连接 Mosquitto，并向 `motor/control/command` 发布 QoS 1 控制命令。
- 订阅 `motor/control/telemetry`，接收 ESP32 每 250 ms 发布的电机遥测。
- 遥测确认电机进入运行状态时自动清空并开始记录；确认停止或 MQTT
  断开时停止记录并绘图。
- 所有控制模式都绘制 `Iq / Iq 目标 / Id / Id 目标` 电流曲线。
- 速度模式额外绘制实测速度和参考速度；位置模式额外绘制实测位置和
  目标位置。
- 横轴统一为记录开始后的秒数；纵轴分别使用 mA、RPM 和度，并根据数据
  自动缩放。

记录由电机遥测中的 `running` 状态驱动，因此只有启动命令真正生效后才会
开始；停止命令真正生效后才会结束。如果程序连接时电机已经在运行，也会
从收到的第一帧运行遥测开始记录。

## 构建

```powershell
cmake -S QtMqttSimpleControl -B QtMqttSimpleControl/build `
  -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/mingw_64
cmake --build QtMqttSimpleControl/build
```

程序只依赖 Qt 6 的 `Network` 和 `Widgets` 模块，不依赖 Qt Charts。
