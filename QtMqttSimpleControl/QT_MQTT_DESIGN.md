# Qt 端设计与功能说明

## 1. 设计目标

`QtMqttSimpleControl` 是 MotorControl_v2 的桌面测试工具。它通过 ESP32 MQTT
网关管理 STM32 电机控制参数，并接收 ESP32 本地高速测试产生的数据。

Qt 端不再自行实现 MQTT 报文编解码、TCP 收发、QoS 1 确认或心跳。以上协议职责
统一交给 Qt 官方 **Qt MQTT** 模块的 `QMqttClient`，应用代码只处理连接配置、主题
和业务载荷。

## 2. 软件结构

| 层次 | 文件/组件 | 职责 |
| --- | --- | --- |
| 界面与业务 | `src/mainwindow.*` | Broker 配置、PID 编辑、命令下发、状态展示、测试数据重组和报告保存 |
| MQTT 适配 | `src/mqttclient.*` | 将现有业务接口映射到 `QMqttClient`，统一输出连接、消息和错误信号 |
| 官方协议组件 | Qt `Mqtt` 模块 | MQTT 3.1.1 连接、自动 Keep Alive、订阅、发布、QoS 1 和协议解析 |
| 报告绘制 | `src/plotrenderer.*` | 生成 PNG 分析报告与同名 CSV 原始数据 |

保留 `MqttClient` 适配层的原因是让窗口业务不依赖 Qt MQTT 的具体类型。界面仍使用
`connectToBroker()`、`subscribeQos1()` 和 `publishQos1()`，以后增加认证、TLS 或
重连策略时也只需修改适配层。

## 3. MQTT 配置与生命周期

当前连接配置如下：

- 协议版本：MQTT 3.1.1；
- 传输：Qt MQTT 默认 TCP 传输；
- Clean Session：启用；
- Keep Alive：20 秒，由 `QMqttClient` 自动维护；
- Client ID：`qt-pid-test-<进程号>`；
- 默认 Broker：`192.168.10.7:1883`，可在界面修改；
- 发布与订阅 QoS：1（At Least Once）；
- Retain：关闭。

连接流程：

1. 用户输入 Broker 地址并点击“连接”；
2. `MainWindow` 生成 Client ID，调用 MQTT 适配层；
3. `QMqttClient` 完成 TCP 和 MQTT 握手并发出 `connected`；
4. Qt 端提交四个 QoS 1 订阅；
5. 收到的消息由 `QMqttClient::messageReceived` 转换成现有业务信号；
6. 断开或协议错误通过 `disconnected` / `errorChanged` 更新界面状态。

`QMqttClient` 会处理 PING、PUBACK、SUBACK、分包与剩余长度等协议细节，Qt 端没有
自定义 MQTT 字节流解析器。

## 4. 主题与数据方向

| 主题 | 方向（以 Qt 为中心） | 格式 | 用途 |
| --- | --- | --- | --- |
| `motor/control/command` | 发布 | JSON | 停机、PID 参数等控制命令 |
| `motor/control/telemetry` | 订阅 | JSON | STM32 链路、运行状态和模式 |
| `motor/control/ack` | 订阅 | JSON | 命令处理确认 |
| `motor/control/test/status` | 订阅 | JSON | ESP32 本地测试阶段、测试 ID 和点数 |
| `motor/control/test/data` | 订阅 | 二进制 | 分块上传的高速采样记录 |

命令 JSON 带单调递增的 `id`。PID 命令使用 `set_pid`，并携带控制器编号及
`kp`、`ki`，位置环额外携带 `kd`。Qt 端不会通过 MQTT 连续采集实时实验曲线；
ESP32 在本地高速缓存完整实验数据，停机后再分块回传。

## 5. Qt 端功能

### 5.1 连接与状态

- 配置 Broker IP 和端口；
- 显示 MQTT 在线、离线、连接中或错误状态；
- 显示 STM32 链路、电机运行状态和控制模式；
- 窗口关闭时，如果测试未结束或电机仍运行，会先发布停止命令。

### 5.2 PID 参数管理

- 编辑速度、位置、Iq 和 Id 控制器的增益分子；
- 仅在电机停止且无测试进行时临时下发 PID；
- 通过 `QSettings` 保存本机界面配置；
- 恢复当前固件工程默认值；
- 临时下发不会写 STM32 Flash，也不会修改固件生成代码。

### 5.3 本地高速测试接收

- 识别 ESP32 发来的测试 ID、模式、阶段和期望采样数；
- 校验 `MCTD` 二进制分块的版本、记录大小、索引和总点数；
- 按索引去重并重组速度、位置、Iq、Id 及其参考值；
- 检查回传是否完整，显示接收进度；
- 支持测试期间发送立即停止命令，并保存已经完整回传的数据。

### 5.4 报告输出

- 输出 1800×2000 PNG 分析报告；
- 输出不降采样、不滤波的同名 CSV；
- 计算上升时间、超调、调节时间、MAE、RMSE 和最大误差等指标；
- 位置误差使用最短角度差，正确处理 0°/360° 环绕；
- 默认目录为用户文档下的 `MotorControl_PID_Tests`。

## 6. 构建与部署

先为目标 Qt 6 套件安装官方 **Qt MQTT**，并确保版本和编译器套件一致。例如本工程
使用 Qt 6.11.1 MinGW 64-bit 时，Qt MQTT 也必须以 Qt 6.11.1 和 MinGW 13.1
64-bit 构建。若 Maintenance Tool 未列出 Qt MQTT，可检出 Qt 官方 `qtmqtt`
仓库的同版本标签，并使用该套件的 `qt-configure-module`、CMake 和 Ninja 构建安装。

```powershell
cmake -S QtMqttSimpleControl -B QtMqttSimpleControl/build `
  -G Ninja -DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/mingw_64
cmake --build QtMqttSimpleControl/build
ctest --test-dir QtMqttSimpleControl/build --output-on-failure
```

CMake 通过以下官方目标声明依赖：

```cmake
find_package(Qt6 REQUIRED COMPONENTS Mqtt Widgets)
target_link_libraries(QtMqttSimpleControl PRIVATE Qt6::Mqtt Qt6::Widgets)
```

部署时除 Qt Core/Gui/Widgets 及其平台插件外，还需一起部署 Qt MQTT 和它依赖的
Qt Network 运行库。可使用对应 Qt 套件中的 `windeployqt` 收集运行时文件。

## 7. 后续扩展点

- 用户名/密码：在连接前调用 `QMqttClient::setUsername()` 和 `setPassword()`；
- TLS：使用 `connectToHostEncrypted()` 并提供 `QSslConfiguration`；
- 自动重连：在适配层根据 `stateChanged` 和定时退避策略实现；
- MQTT 5：切换协议版本，并根据需要使用连接、发布和订阅属性。
