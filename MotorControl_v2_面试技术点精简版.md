# MotorControl_v2 面试技术点精简版

> 用法：先背这份，建立完整框架；能脱稿讲清每条链后，再回到
> `MotorControl_v2_面试技术点详解.md` 学原理、代码证据和追问。

## 一、30 秒项目介绍

> 这是一个 STM32G431、ESP32-S3 和 Qt 组成的 PMSM 有感 FOC 控制与 PID
> 实验平台。STM32基于 MCSDK完成16 kHz电流环、编码器反馈、速度/位置控制和
> PWM；ESP32用FreeRTOS、LVGL做现场HMI，并通过CAN/USART连接STM32，通过
> Wi-Fi和MQTT连接Qt；Qt负责临时PID参数、测试数据重组和PNG/CSV报告。为了不让
> 网络抖动影响采样，7秒实验由ESP32本地启动，约2 ms的CAN反馈先写PSRAM，停机
> 后再按QoS1二进制分块上传，失败保留数据并可RESEND。

## 二、整机架构必须会画

```text
Qt 6 Widgets / MqttClient
          ↕ MQTT 3.1.1 over TCP
Mosquitto Broker 192.168.10.7:1883
          ↕ Wi-Fi
ESP32-S3
├─ LVGL
├─ wifi_manager / mqtt_manager
├─ mqtt_motor_gateway / PSRAM
└─ CommMgr_ESP
      ├─ CAN_ESP
      └─ USART_ESP
          ↕
STM32G431
CAN_STM/USART_STM → CommMgr_STM → MotorMgr_STM → MCSDK
          ↓
ADC + Encoder → FOC → SVPWM → TIM1 → 逆变器 → PMSM
```

一句话分工：

- STM32：硬实时控制。
- ESP32：现场HMI、通信网关和高速数据缓存。
- Broker：消息路由。
- Qt：参数、实验接收、分析和归档。

## 三、关键参数

| 项目 | 当前值 |
| --- | --- |
| MCU | STM32G431，Cortex-M4F |
| 电机 | PMSM，有感FOC，7极对 |
| PWM/电流环 | 16 kHz |
| 中频/位置任务 | 1 kHz |
| 编码器 | MT6701 AB，1024 PPR，四倍频约4096 count/rev |
| 速度范围 | ±2600 RPM |
| CAN | Classic CAN，500 kbit/s |
| UART | USART3，115200，DMA循环接收 |
| Wi-Fi | ESP32-S3 STA，2.4 GHz |
| MQTT | 默认192.168.10.7:1883 |
| 测试 | 7 s；速度500 RPM；位置90° |
| CAN采样 | 约2 ms/组，约500 sample/s |
| PSRAM | 3600×16 = 57600 bytes |
| MQTT块 | 40点，最大20+40×20 = 820 bytes |

## 四、FOC主链

```text
TIM1同步触发ADC注入采样
→ ADC1_2_IRQHandler
→ TSK_HighFrequencyTask
→ FOC_HighFrequencyTask
→ ENC_CalcAngle
→ PWMC_GetPhaseCurrents
→ Clarke: abc → αβ
→ Park: αβ → dq
→ Id/Iq PI: 电流误差 → Vd/Vq
→ Circle Limitation
→ 反Park: dq → αβ
→ SVPWM / TIM1 CCR
→ 三相逆变桥
```

- Iq：主要产生转矩。
- Id：主要影响磁链，表贴式PMSM常令Id_ref≈0。
- 电流环最快，速度/位置环更慢，UI/网络最慢。

当前模式：

```text
速度模式：速度PI → Torque/Iq_ref → Id/Iq电流PI
位置模式：位置PID → Torque/Iq_ref → Id/Iq电流PI
```

当前位置模式不是始终“位置→速度→电流”三级串级，源码中位置PID直接产生转矩参考。

## 五、MCSDK与状态机

```text
IDLE
→ OFFSET_CALIB       电流零偏
→ CHARGE_BOOT_CAP    自举电容
→ ALIGNMENT          编码器电角度对齐
→ WAIT_STOP_MOTOR
→ RUN
→ STOP
→ IDLE
```

故障：

```text
FAULT_NOW → FAULT_OVER → ACK → IDLE
```

重要API：

```text
MC_StartMotor1 / MC_StopMotor1
MC_ProgramSpeedRampMotor1_F
MC_ProgramPositionCommandMotor1
MC_GetAverageMecSpeedMotor1_F
MC_GetIqdMotor1_F / MC_GetIqdrefMotor1_F
```

`MotorMgr_STM`统一处理模式、限幅、轨迹、启停、PID和无扰切换；CAN/UART不能各自
直接操作MCSDK。

## 六、CAN与USART

CAN特点：差分、多主机、ID仲裁、CRC、ACK、错误计数和Bus-Off。

当前CAN ID：

```text
0x100 ESP32 → STM32 命令
0x180 STM32 → ESP32 状态
0x181 STM32 → ESP32 速度/位置参考
0x182 STM32 → ESP32 Id/Iq及参考
```

速度控制链：

```text
LVGL/网络
→ CommMgr_ESP_SetSpeedRPM
→ CAN_ESP TX task
→ CAN 0x100
→ CAN_STM_ProcessRx
→ CommMgr_STM
→ MotorMgr_STM_SetSpeed
→ MC_ProgramSpeedRampMotor1_F
```

ESP32 CAN接收：

```text
TWAI ISR只复制帧
→ FreeRTOS Queue
→ RX task解析
→ snapshot
```

ISR最小化，复杂解析放任务。

UART帧：

```text
0xA5 0x5A + version + type + sequence + length + payload + CRC16
```

USART3用DMA循环缓冲；CRC16/Modbus初值0xFFFF、多项式0xA001。

## 七、Wi-Fi必背

### 1. 分层

```text
MQTT → TCP → IP → Wi-Fi/802.11 → 2.4GHz射频
```

Wi-Fi连上不等于MQTT连上。

### 2. 名词

- AP：路由器/热点。
- STA：接入AP的终端，本工程ESP32。
- SSID：网络名称。
- BSSID：具体AP的MAC。
- RSSI：dBm，越接近0通常越强。
- DHCP：分配IP、掩码、网关、DNS。

### 3. 当前流程

```text
wifi_manager_init
→ NVS / esp_netif / event loop / STA driver
→ 异步scan
→ WIFI_EVENT_SCAN_DONE
→ 选择SSID和密码
→ esp_wifi_connect
→ WIFI_EVENT_STA_CONNECTED
→ IP_EVENT_STA_GOT_IP
→ connected=true
```

扫描最多保留12个AP，按RSSI降序并按SSID去重。

### 4. 重连和线程安全

- 意外断线：1/2/4/8/10秒指数退避。
- 主动断开：不重连。
- `WIFI_PS_NONE`：低延迟优先，代价是功耗。
- 事件回调只更新mutex保护的snapshot；LVGL 50 ms定时器按revision刷新。

## 八、MQTT必背

### 1. 角色

- Broker：Mosquitto，负责连接、订阅表和转发。
- Publisher：发布消息。
- Subscriber：订阅消息。
- Qt和ESP32都既能发布也能订阅。

### 2. 常见报文

```text
CONNECT/CONNACK
SUBSCRIBE/SUBACK
PUBLISH/PUBACK
PINGREQ/PINGRESP
DISCONNECT
```

Qt当前使用MQTT 3.1.1；TCP没有消息边界，所以按Remaining Length处理半包/粘包。

### 3. QoS

- QoS0：最多一次，可能丢；用于普通遥测。
- QoS1：至少一次，PUBLISH/PUBACK，可能重复；用于命令和测试数据。
- QoS2：恰好一次，握手最重；当前不用。

最重要边界：

```text
enqueue成功 ≠ Broker收到
Broker PUBACK ≠ Qt收到
Qt收到全部点 ≠ 文件保存成功
```

当前ESP32的UPLOAD SUCCESS是Broker确认，不是Qt落盘确认。

### 4. Keep Alive、Session、Retain、LWT

- ESP32 Keep Alive 30 s，MQTT重连约3 s。
- Qt Keep Alive 20 s，每10 s PINGREQ。
- Qt使用Clean Session，离线期间不保留持久会话。
- 测试数据不retain，避免新订阅者收到旧块。
- 当前无LWT，可用于生产化在线状态。

### 5. Client ID

```text
ESP32 esp32s3-motor-<MAC>
Qt    qt-pid-test-<PID>
```

ID冲突会导致旧连接被Broker踢下线。

## 九、MQTT Topic和业务消息

| Topic | 方向 | 内容 |
| --- | --- | --- |
| `motor/control/command` | Qt→ESP32 | QoS1 JSON命令 |
| `motor/control/telemetry` | ESP32→Qt | QoS0 JSON状态 |
| `motor/control/ack` | ESP32→Qt | QoS1业务ACK |
| `motor/control/test/status` | ESP32→Qt | 测试阶段JSON |
| `motor/control/test/data` | ESP32→Qt | QoS1 MCTD二进制 |

PID命令：

```json
{"id":1,"cmd":"set_pid","controller":1,"kp":48,"ki":4,"kd":8}
```

ACK：

```json
{"id":1,"ok":true,"message":"..."}
```

为什么还要业务ACK：MQTT到达ESP32不代表CAN在线、参数合法、STM32接受。

## 十、本地PID测试与PSRAM

```text
ESP32页面SPEED/POSITION按钮
→ 检查MQTT在线、电机停止
→ PSRAM分配57600字节
→ 自动选CAN、应用缓存PID
→ 设置模式、START、等待RUN
→ 下发500RPM或90°
→ 新sample_sequence写PSRAM
→ 7秒STOP
→ 电机确认停止
→ MCTD分块上传
```

为什么不实时发MQTT：网络抖动不能影响2 ms CAN采样；测试中只写PSRAM，停机后
才上传。

PSRAM 16-byte样本：时间、主实测/参考、Iq/Id、Iq_ref/Id_ref。

分配失败日志区分：PSRAM未初始化、空闲不足、最大连续块不足/碎片和能力异常；
不回退内部RAM。

## 十一、MCTD上传与RESEND

### 1. 数据格式

```text
20-byte header:
MCTD, version, mode, recordSize, flag,
testId, startIndex, count, total

20-byte record:
timeUs,
speed/speedRef,
position/positionRef,
Iq/Id/IqRef/IdRef
```

全部小端；Qt显式读取，不能直接发送C结构体。

### 2. 可靠上传

```text
40点/块，最大820 bytes
→ 每50 ms最多发下一块
→ 同时只允许一个块等待PUBACK
→ PUBACK后publishIndex才推进
→ complete也等PUBACK
→ 成功后释放PSRAM
```

### 3. 失败重发

```text
断线/enqueue失败/PUBACK超时
→ SEND_FAILED
→ PSRAM retained
→ 新测试按钮禁用
→ MQTT恢复后按RESEND
→ 原test ID从index 0全量发送
→ Qt按index去重
```

从0重发，因为ESP32不知道Qt缺哪些块；全量幂等比缺块协商简单。

## 十二、Qt上位机

### 1. MqttClient

`QTcpSocket`手写最小MQTT 3.1.1：CONNECT、SUBSCRIBE、PUBLISH、接收入站
PUBLISH、返回PUBACK、PING和DISCONNECT。

边界：当前未跟踪出站PUBACK、未解析SUBACK、无自动重连/TLS/LWT/持久Session。

### 2. PID参数

- 临时应用：MQTT下发运行时值，不写STM32 Flash。
- 保存：写PC的QSettings，下次Qt启动加载。
- 恢复默认：删除QSettings的pid组，恢复工程默认分子。

### 3. 数据重组

```text
processTestData
→ 检查MCTD/version/长度/index
→ samples_[startIndex+i]
→ receivedSamples位图去重
→ complete时检查received==expected
→ 保存PNG + CSV
```

速度/位置报告都包含Iq跟踪；CSV保留每一个原始点，不降采样、不删尖峰。

## 十三、上传失败问题怎么回答

> 原因不是单一“MQTT不稳定”。底层Wi-Fi/TCP写超时是触发条件；旧逻辑还把
> enqueue当成已送达，并可能让QoS0消息占用8 KiB outbox，所以弱网时持续积压，
> complete刚入队就释放PSRAM，造成假成功和数据丢失。修复后QoS0即发即弃，测试
> 块一次一个，按message ID等待Broker PUBACK，complete也确认；失败不释放PSRAM，
> 页面可RESEND完整数据集。

## 十四、分层排障口诀

```text
先供电/天线
再Wi-Fi扫描与reason
再GOT_IP/DHCP
再TCP 1883
再MQTT CONNACK/Topic/PUBACK
再ESP网关状态机/PSRAM
再CAN link/fault
最后才调PID和电机控制
```

典型对应：

- 扫不到AP：2.4 GHz、天线、driver。
- 有AP连不上：密码/auth/reason。
- 已关联无IP：DHCP。
- 有IP无MQTT：Broker、端口、防火墙、Client ID。
- 写超时：RSSI/干扰、outbox、Broker负载。
- 测试不转：PSRAM、CAN link、fault、RUN和命令拒绝。
- 点数不完整：test ID/index/Qt掉线/complete时机。
- 点数完整但曲线差：回到CAN采样、单位、编码器和控制环，不是MQTT时间。

## 十五、当前方案不能夸大的边界

1. 1883是明文，无TLS、认证和ACL，只适合可信局域网。
2. Broker PUBACK不是Qt落盘ACK。
3. Qt手写Client不是完整MQTT协议栈。
4. QSettings保存PID不是写入STM32 Flash。
5. PSRAM断电/重启会丢失，RESEND不能跨重启。
6. 网络STOP不是功能安全急停。
7. 当前单缓冲，不支持多测试排队。

生产化优先：MQTTS+ACL、Qt落盘ACK、Qt完整QoS状态机/自动重连、数据集CRC、缺块
选择性重传、LWT在线状态、硬件安全停机。

## 十六、最后必须能脱稿说出的11条链

1. STM32启动链。
2. FOC高频链。
3. MCSDK启动状态机。
4. CAN命令到MCSDK。
5. STM32反馈到ESP32 snapshot。
6. LVGL到CommMgr_ESP。
7. Wi-Fi扫描、GOT_IP和指数退避。
8. MQTT CONNECT、订阅和事件回调。
9. ESP32本地测试STARTING→RECORDING→STOPPING。
10. PSRAM→MCTD→逐块PUBACK→complete。
11. Qt按ID/index重组→完整性检查→PNG/CSV。

## 十七、面试最后一句

> 这个项目最核心的不是简单把三个平台连起来，而是按实时性和可靠性分层：STM32
> 保证控制周期，ESP32保证现场状态机和采样缓存，MQTT负责异步传输，Qt负责端到端
> 数据检查和分析；同时我能明确指出Broker确认、Qt接收和文件落盘是三个不同的
> 成功层级。
