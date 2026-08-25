# MotorControl_v2 面试技术点详解

> 基于 `jzq20000919/MotorControl_v2` 当前 `main` 分支整理。目标不是把 MCSDK 源码全部背下来，而是让你能从“原理 → 当前工程代码 → 函数调用链 → 面试表达”四个层次讲清楚项目。
>
> **范围说明：** 本文已按当前工程同步 Wi-Fi、MQTT、ESP32 本地 PID 测试、PSRAM 高速采样、QoS 1 分块上传、PUBACK 确认、失败保留与重新发送实现。文中会明确区分“当前已经实现”“协议本身保证”“当前仍可改进”，面试时不要把 Broker 确认误说成 Qt 端到端确认，也不要把局域网明文 MQTT 说成已经具备生产级安全性。

## 一、先记住整个 V2 工程的总架构

STM32 端应用层：

```text
CAN_STM / USART_STM / MCP_STM
             ↓
        CommMgr_STM
             ↓
        MotorMgr_STM
             ↓
           MCSDK
             ↓
  ADC / Encoder / FOC / PWM
             ↓
        三相逆变桥 → 电机
```

ESP32 端：

```text
LVGL / 网络层
      ↓
 CommMgr_ESP
   ↙      ↘
CAN_ESP  USART_ESP
   ↓        ↓
       STM32
```

网络测试端到端架构：

```text
Qt 上位机 MQTT Client
        ↕ TCP / MQTT 3.1.1
 Mosquitto Broker（当前默认 192.168.10.7:1883）
        ↕ TCP / MQTT
ESP32-S3：Wi-Fi + MQTT Manager + Motor Gateway
        ↕ CAN 500 kbit/s
STM32G431：MCSDK / FOC / 电机
```

这里要特别记住：Broker 是独立消息服务器，Qt 和 ESP32 都是 MQTT Client；
STM32 不直接联网，硬实时 FOC 也不经过 MQTT。

STM32 启动主链：

```text
main()
↓
HAL_Init()
↓
SystemClock_Config()
↓
MX_GPIO_Init() / MX_DMA_Init() / MX_ADCx_Init() / MX_TIMx_Init() ...
↓
MX_MotorControl_Init()
↓
MCboot()
↓
FOC_Init()
↓
初始化电流采样、编码器、PID、位置控制、速度转矩控制等
↓
CommMgr_STM_Init()
↓
while(1)
└─ CommMgr_STM_Tick()
```

高频 FOC 链：

```text
ADC1_2_IRQHandler()
↓
TSK_HighFrequencyTask()
↓
FOC_HighFrequencyTask()
↓
ENC_CalcAngle()
↓
FOC_CurrControllerM1()
↓
PWMC_GetPhaseCurrents()
↓
MCM_Clarke()
↓
MCM_Park()
↓
PI_Controller(Iq) + PI_Controller(Id)
↓
Circle_Limitation()
↓
MCM_Rev_Park()
↓
PWMC_SetPhaseVoltage()
↓
SVPWM / TIM1 PWM
```

当前关键参数：

- MCU：STM32G431。
- PWM：16 kHz。
- 电流环：16 kHz，`REGULATION_EXECUTION_RATE = 1`。
- 中频/位置控制任务：1 kHz。
- MCSDK SysTick：2 kHz。
- 电机极对数：7。
- 编码器：1024 PPR，TIM 编码器四倍频后约 4096 count/rev。
- 最大应用速度：±2600 rpm。
- CAN：500 kbit/s。
- USART3 默认：115200 bit/s。
- Wi-Fi：ESP32-S3 2.4 GHz STA，关闭省电以降低时延抖动。
- MQTT Broker 默认：`192.168.10.7:1883`，ESP32 outbox 8 KiB。
- 本地测试：7 s，速度 500 RPM / 位置 90°，CAN反馈约 2 ms/组。
- PSRAM测试缓冲：3600 × 16 byte = 57.6 KiB。
- MQTT测试块：40点/块，最大二进制 Payload 820 byte。

---

## 二、MCU、CubeMX 与工程构建

### 1. STM32G431 基本架构与外设资源

STM32G431 是 Cortex-M4F 内核 MCU，适合实时电机控制。对本项目真正重要的不是背完整数据手册，而是知道哪些外设被用来完成哪项任务。

当前 `Src/main.c` 中实际使用：

- `ADC1 + ADC2`：三相电流注入采样，以及母线电压等常规采样。
- `OPAMP1/2/3`：电流采样前端内部运放资源。
- `TIM1`：高级定时器，产生三相互补 PWM，并提供 ADC 触发时刻。
- `TIM3`：编码器接口，读取 MT6701 AB 相。
- `CORDIC`：MCSDK 可利用硬件加速三角函数等运算。
- `FDCAN1`：STM32 与 ESP32 的 CAN 通信。
- `USART3 + DMA`：STM32 与 ESP32 的串口通信。
- `USART2 + DMA`：保留给 MCSDK ASPEP/MCP。
- `DAC1`：MCSDK 调试/观测用途。

**函数调用链：**

```text
main()
↓
MX_ADC1_Init()
MX_ADC2_Init()
MX_OPAMP1_Init()/2/3
MX_TIM1_Init()
MX_TIM3_Init()
MX_FDCAN1_Init()
MX_USART2_UART_Init()
MX_USART3_UART_Init()
```

面试时可以说：STM32G431 负责全部硬实时控制，ESP32 不参与电流环和 PWM 计算，因此网络或 UI 卡顿不会直接改变 FOC 控制周期。

### 2. STM32CubeMX `.ioc` 工程配置与代码生成

`.ioc` 是硬件配置的“源文件”。GPIO、ADC、TIM、FDCAN、USART、DMA、时钟和 NVIC 等都应尽量在 CubeMX 中配置，再由 CubeMX 生成 `MX_xxx_Init()`。

本项目重构后的原则是：

```text
CubeMX 管硬件初始化
User/ 管应用逻辑
MCSDK 管电机控制算法
```

例如 CAN：

```text
CubeMX
└─ MX_FDCAN1_Init()       ← 时钟、引脚、位时序、消息 RAM

User/can_STM.c
└─ CAN_STM_Init()         ← 过滤器、启动、协议状态
```

USART3 同理：

```text
CubeMX
└─ MX_USART3_UART_Init() + MX_DMA_Init()

User/usart_STM.c
└─ USART_STM_Init()
   └─ HAL_UART_Receive_DMA()
```

因此不要在 User 层重新写 GPIO/时钟/外设底层初始化，否则很容易与 `.ioc` 生成代码重复甚至冲突。

### 3. CMake 工程的编译、链接、烧录与复位流程

当前 `CMakePresets.json` 定义了 `Debug` 和 `Release`，使用 Ninja，Release 输出目录为：

```text
build/Release/
```

工程目标名是：

```text
MCSDK_FOC_v2
```

`CMakeLists.txt` 在链接完成后用 `objcopy` 自动生成：

```text
MCSDK_FOC_v2.hex
```

标准流程：

```powershell
cmake --preset Release
cmake --build --preset Release
STM32_Programmer_CLI.exe -c port=SWD -d build/Release/MCSDK_FOC_v2.hex -rst
```

单独复位：

```powershell
STM32_Programmer_CLI.exe -c port=SWD -rst
```

VS Code Task 的思想是把这些命令固化到 `.vscode/tasks.json`，以后通过 `Ctrl+Shift+B` 一键编译/烧录。

**工程链：**

```text
CMakePresets.json
↓
CMake 配置
↓
CMakeLists.txt 收集 Src/User/MCSDK 源码
↓
arm-none-eabi-gcc 编译
↓
链接器脚本生成 ELF
↓
objcopy 生成 HEX
↓
STM32CubeProgrammer CLI
↓
ST-Link / SWD
↓
Flash
↓
-rst 复位运行
```

---

## 三、电机、逆变器、PWM 与 FOC 基础

### 4. PMSM / BLDC 电机基本原理

PMSM 和 BLDC 都是永磁同步类电机，转子带永久磁体，定子三相绕组产生旋转磁场。FOC 的核心目标是：让定子电流形成的磁场始终以合适角度作用于转子磁场，从而平滑地产生转矩。

本工程使用有感 FOC，编码器直接提供转子位置，因此不需要依赖反电动势估算转子角度。

当前电机参数在 `Inc/pmsm_motor_parameters.h`：

- 极对数 `POLE_PAIR_NUM = 7`
- 定子电阻 `RS = 2.55 Ω`
- 电感 `LS = 0.00086 H`
- 最大速度 2600 rpm
- 额定电流 2 A

项目中的控制对象不是“直接控制转速”，而是最终通过控制三相电压 → 三相电流 → 电磁转矩 → 机械运动。

### 5. 三相逆变桥、MOSFET 与栅极驱动原理

STM32 的 PWM 不能直接驱动电机，而是控制三相逆变桥的 6 个 MOSFET。

```text
STM32 TIM1 PWM
↓
栅极驱动芯片
↓
高侧/低侧 MOSFET
↓
U/V/W 三相电压
↓
电机三相电流
```

每一相都有一个高侧 MOSFET 和一个低侧 MOSFET。通过改变三相桥臂的占空比，可以合成所需的空间电压矢量。

MCSDK 最终通过：

```text
PWMC_SetPhaseVoltage()
```

把 $V_{\alpha\beta}$ 电压目标转成 PWM 比较值，再写入 TIM1。

### 6. PWM、互补 PWM、死区时间

PWM 用占空比控制平均相电压。三相逆变器中，同一个桥臂的高低侧通常使用互补 PWM。

理想情况：

```text
高侧 ON → 低侧 OFF
高侧 OFF → 低侧 ON
```

但 MOSFET 关断不是瞬间完成，所以高低侧切换时必须留一小段时间，让两只管子都关闭，这就是死区。

当前工程：

```c
#define PWM_FREQUENCY 16000
#define SW_DEADTIME_NS 1000
```

即 PWM 16 kHz，软件参数对应 1000 ns 死区。

如果没有死区，高低侧短时间同时导通会形成母线直通，电流非常大，可能损坏 MOSFET。

**代码落点：**

```text
drive_parameters.h
↓
parameters_conversion.h
↓
DEAD_TIME_COUNTS / PWM_PERIOD_CYCLES
↓
TIM1 + MCSDK PWM driver
```

### 7. Clarke 变换

Clarke 变换把三相静止坐标中的电流：

$$i_a,i_b,i_c$$

转换为二维静止坐标：

$$i_\alpha,i_\beta$$

因为三相平衡系统满足：

$$i_a+i_b+i_c=0$$

所以实际上只有两个独立变量。

本工程不是你自己写 Clarke，而是 MCSDK：

```c
Ialphabeta = MCM_Clarke(Iab);
```

**实际调用链：**

```text
ADC 注入采样完成
↓
ADC1_2_IRQHandler()
↓
TSK_HighFrequencyTask()
↓
FOC_HighFrequencyTask()
↓
FOC_CurrControllerM1()
↓
PWMC_GetPhaseCurrents()
↓
MCM_Clarke()
```

### 8. Park 变换

Park 变换把静止的 $\alpha\beta$ 坐标系旋转到跟随转子磁场旋转的 $dq$ 坐标系。

```text
Iα/Iβ + 转子电角度
↓ Park
Id / Iq
```

本项目：

```c
hElAngle = SPD_GetElAngle(speedHandle);
Iqd = MCM_Park(Ialphabeta, hElAngle);
```

这样三相交流电流就变成相对稳定的 $I_d$、$I_q$，于是可以用普通 PI 控制器控制。

### 9. 反 Park 变换

电流 PI 输出的是旋转坐标系中的：

$$V_d,V_q$$

但逆变器需要的是静止坐标系电压，因此需要反 Park：

```c
Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
```

**实际链：**

```text
Iq/Id PI
↓
Vq/Vd
↓
Circle_Limitation()
↓
MCM_Rev_Park()
↓
Vα/Vβ
↓
PWMC_SetPhaseVoltage()
```

### 10. SVPWM 基本原理与六个扇区

三相逆变桥共有 8 种基本开关状态：6 个有效电压矢量 + 2 个零矢量。六个有效矢量把空间分成六个 60° 扇区。

SVPWM 根据目标 $V_\alpha,V_\beta$：

1. 判断目标矢量在哪个扇区。
2. 选相邻两个有效矢量。
3. 计算两个有效矢量的作用时间。
4. 剩余时间分配给零矢量。
5. 生成三相 CCR，占空比送到 TIM1。

当前工程中你看不到一个 User 层 `SVPWM()` 函数，因为它封装在 MCSDK PWM Current Feedback 驱动内部。

从应用角度记调用链即可：

```text
MCM_Rev_Park()
↓
Vα/Vβ
↓
PWMC_SetPhaseVoltage()
↓
R3_2 对应的 PWM 算法
↓
TIM1 CCR
↓
三相互补 PWM
```

### 11. FOC 整体控制流程

本项目 FOC 最核心源码在 `Src/mc_tasks_foc.c`。

完整高频链：

```text
TIM1 触发 ADC 注入采样
↓
ADC1/ADC2 转换完成
↓
ADC1_2_IRQHandler()
↓
TSK_HighFrequencyTask()
↓
FOC_HighFrequencyTask()
↓
ENC_CalcAngle()              读取/更新编码器角度
↓
FOC_CurrControllerM1()
├─ PWMC_GetPhaseCurrents()   获取相电流
├─ MCM_Clarke()              abc → αβ
├─ MCM_Park()                αβ → dq
├─ PI_Controller(Iq)
├─ PI_Controller(Id)
├─ Circle_Limitation()       限制电压矢量幅值
├─ MCM_Rev_Park()            dq → αβ
└─ PWMC_SetPhaseVoltage()    生成 PWM
```

这是你面试必须能脱稿讲出来的一条链。

### 12. $I_d$、$I_q$ 分别代表什么

在 PMSM 的转子同步坐标系中：

- $I_d$：沿转子磁链方向的电流，主要影响磁链。
- $I_q$：与转子磁链正交的电流，主要产生转矩。

表面式 PMSM 的常规控制通常令：

$$I_{d,ref}\approx0$$

再根据需要调节：

$$I_{q,ref}$$

因此面试中可以近似说：

> $I_q$ 是“转矩电流”，$I_d$ 是“励磁/磁链电流”。

当前工程通过：

```c
MC_GetIqdMotor1_F()
MC_GetIqdrefMotor1_F()
```

由 `MotorMgr_STM_GetState()` 读取，并转换成 mA 供通信层使用。

### 13. 三相电流采样与 ADC 注入采样

FOC 电流环必须在确定的 PWM 时刻采样，否则开关噪声和不可观测区会导致电流值失真。

当前 `main.c` 中 ADC1/ADC2 的 injected conversion 外部触发来自：

```text
TIM1 CC4
```

例如：

```c
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
```

高频链：

```text
TIM1 PWM
↓
MCSDK 根据扇区设置采样点
↓
TIM1 CC4 触发 ADC injected conversion
↓
ADC1/ADC2 完成
↓
ADC1_2_IRQHandler()
↓
TSK_HighFrequencyTask()
```

`PWMC_GetPhaseCurrents()` 会从 MCSDK 当前电流采样拓扑中获得重构后的相电流。

### 14. 电流零点偏置和电流标定

运放和 ADC 即使真实相电流为 0，也通常不会读到理想零值，这就是 offset。

如果不校准：

```text
真实 0 A
↓
ADC 却认为 +0.2 A
↓
Clarke/Park 后 Id/Iq 有假电流
↓
PI 会主动补偿不存在的误差
↓
电机抖动/发热/噪声
```

当前 MCSDK 状态机启动时会进行 polarization/offset calibration。

**代码链：**

```text
MC_StartMotor1()
↓
MCI_StartMotor()
↓
状态机 IDLE
↓
PWMC_CurrentReadingCalibr(..., CRC_START)
↓
OFFSET_CALIB
↓
PWMC_CurrentReadingCalibr(..., CRC_EXEC)
↓
校准完成
↓
CHARGE_BOOT_CAP
```

所以“上电不动作时 $I_d/I_q$ 波动”首先要排查电流零偏、运放噪声、ADC 触发点、采样比例，而不是第一反应就调速度 PID。

### 15. 电流环 PI 控制

电流环是整个 FOC 最内层、最快的闭环。

误差：

$$e_q=I_{q,ref}-I_q$$

$$e_d=I_{d,ref}-I_d$$

PI 输出：

$$V_q=K_{pq}e_q+K_{iq}\int e_qdt$$

$$V_d=K_{pd}e_d+K_{id}\int e_ddt$$

当前代码直接是：

```c
Vqd.q = PI_Controller(pPIDIq[M1], FOCVars[M1].Iqdref.q - Iqd.q);
Vqd.d = PI_Controller(pPIDId[M1], FOCVars[M1].Iqdref.d - Iqd.d);
```

参数：

```text
Iq/Id Kp numerator = 3633
Iq/Id Ki numerator = 2693
Kp divisor = 128
Ki divisor = 512
```

注意 MCSDK 是定点缩放形式，不能简单把 `3633` 当成传统浮点 $K_p=3633$。

**链：**

```text
Iqdref
↓
Iqd 实测
↓
误差
↓
PI_Controller(PIDIqHandle_M1 / PIDIdHandle_M1)
↓
Vqd
↓
SVPWM
```

### 16. 速度环 PI 控制

速度环比电流环慢。速度环不直接输出 PWM，而是产生转矩需求，最终表现为 $I_{q,ref}$。

基本关系：

```text
速度目标 - 实际速度
↓
速度 PI
↓
转矩参考
≈ Iq_ref
↓
Iq 电流环
↓
Vq
```

MCSDK 中相关对象：

- `PIDSpeedHandle_M1`
- `SpeednTorqCtrlM1`
- `STC_CalcTorqueReference()`
- `FOC_CalcCurrRef()`

在速度模式中，中频任务更新速度控制；电流参考再被 16 kHz 高频任务消费。

参数在 `drive_parameters.h` 中：

```c
PID_SPEED_KP_DEFAULT
PID_SPEED_KI_DEFAULT
SP_KPDIV
SP_KIDIV
```

### 17. 位置环 PID 控制

当前工程位置控制由 MCSDK `trajectory_ctrl` 模块完成。

位置误差：

$$e_\theta=\theta_{ref}-\theta$$

位置 PID：

```c
hTorqueRef_Pos = PID_Controller(pHandle->PIDPosRegulator, wError);
```

然后：

```c
STC_SetControlMode(pHandle->pSTC, MCM_TORQUE_MODE);
STC_ExecRamp(pHandle->pSTC, hTorqueRef_Pos, 0);
```

也就是说当前 V2 的 MCSDK 位置控制器 **输出的是转矩参考，最终对应 $I_q$ 参考**。

**实际链：**

```text
TC_PositionRegulation(&PosCtrlM1)
↓
位置参考 Theta
-
编码器机械角
↓
PID_Controller(PID_PosParamsM1)
↓
TorqueRef
↓
STC torque mode
↓
FOC_CalcCurrRef()
↓
Iq_ref
↓
电流环
```

当前位置 PID：

```text
Kp = 48 / 1024
Ki = 4 / 32768
Kd = 8 / 16
```

这里同样是 MCSDK 定点分子/除数表示。

### 18. 电流环、速度环、位置环三级串级控制关系

**理论上常见的三级串级是：**

```text
位置环
↓ 输出速度参考
速度环
↓ 输出 Iq_ref
电流环
↓ 输出 Vq/Vd
PWM
```

但是面试时你必须说明：**当前 V2 的 MCSDK 位置模式并不是严格按这个三级链运行。**

从 `TC_PositionRegulation()` 源码看，当前位置 PID 直接生成 `hTorqueRef_Pos`，并把 STC 切到 `MCM_TORQUE_MODE`：

```text
位置 PID
↓
TorqueRef / Iq_ref
↓
Iq 电流环
```

速度模式则是：

```text
速度 PI
↓
TorqueRef / Iq_ref
↓
Iq 电流环
```

因此更准确地说，本项目存在三个控制器，但运行时根据模式选择：

```text
速度模式：速度 PI → 电流 PI
位置模式：位置 PID → 电流 PI
```

这比机械地说“始终三级串级”更符合当前源码。

### 19. PID 参数 $K_p$、$K_i$ 的作用和基本整定方法

$K_p$：误差一出现就立即产生控制作用。

- 太小：响应慢、软。
- 太大：振荡、噪声放大、超调。

$K_i$：累计长期误差，消除静差。

- 太小：稳态误差消得慢。
- 太大：积分饱和、低频振荡、恢复慢。

位置环还有 $K_d$：根据误差变化趋势增加阻尼，但容易放大噪声。

本项目建议整定顺序：

```text
先电流环
↓
再速度环
↓
最后位置环
```

不要电流环还不稳定就调速度/位置环。

当前工程还提供运行时临时修改：

```text
MotorMgr_STM_SetPidGain()
↓
根据 controller 选择：
PIDSpeedHandle_M1
PID_PosParamsM1
PIDIqHandle_M1
PIDIdHandle_M1
↓
PID_SetKP()/PID_SetKI()/PID_SetKD()
↓
清积分、清上次误差
```

而且函数明确要求电机不能处于 RUN，避免运行中突然改增益导致危险跳变。

---

## 四、MCSDK、状态机、编码器和控制模式

### 20. MCSDK 的整体架构和作用

MCSDK 不是一个单一“FOC 函数”，而是一整套电机控制框架，包括：

- PWM 与电流采样驱动。
- Clarke/Park/反 Park。
- 电流 PI。
- Speed & Torque Controller。
- Encoder feedback。
- Position trajectory controller。
- State machine。
- Fault management。
- Motor Control API。

初始化：

```text
MX_MotorControl_Init()
↓
MCboot()
↓
FOC_Init()
├─ R3_2_Init()
├─ PID_HandleInit(PIDSpeedHandle_M1)
├─ ENC_Init()
├─ EAC_Init()
├─ PID_HandleInit(PID_PosParamsM1)
├─ TC_Init()
├─ STC_Init()
├─ PID_HandleInit(PIDIqHandle_M1)
└─ PID_HandleInit(PIDIdHandle_M1)
```

User 层原则：不要到处直接调用这些内部句柄，而是统一走 `MotorMgr_STM`。

### 21. MCSDK 状态机：IDLE、RUN、FAULT 等状态

当前 `TSK_MediumFrequencyTaskM1()` 是理解状态机最关键的函数。

主要状态：

```text
IDLE
↓ 启动
OFFSET_CALIB
↓
CHARGE_BOOT_CAP
↓
ALIGNMENT
↓
WAIT_STOP_MOTOR
↓
RUN
↓ 停止
STOP
↓
IDLE
```

故障：

```text
检测到故障
↓
FAULT_NOW
↓
FAULT_OVER
↓ ACK
IDLE
```

其中：

- `IDLE`：待机。
- `OFFSET_CALIB`：电流零偏校准。
- `CHARGE_BOOT_CAP`：自举电容准备。
- `ALIGNMENT`：编码器电角度对齐。
- `RUN`：正常闭环运行。
- `STOP`：执行停止过程。
- `FAULT_NOW`：当前故障仍存在。
- `FAULT_OVER`：故障条件消失，但等待人工确认。

### 22. MCSDK Motor Control API：启动、停止、速度、位置

项目应用层主要用：

```c
MC_StartMotor1()
MC_StopMotor1()
MC_AcknowledgeFaultMotor1()
MC_ProgramSpeedRampMotor1_F()
MC_ProgramPositionCommandMotor1()
MC_GetSTMStateMotor1()
MC_GetAverageMecSpeedMotor1_F()
MC_GetIqdMotor1_F()
MC_GetIqdrefMotor1_F()
MC_GetCurrentPosition1()
MC_GetTargetPosition1()
```

启动：

```text
MotorMgr_STM_Start()
↓
MC_ProgramSpeedRampMotor1_F(0, 0)
↓
MC_StartMotor1()
↓
MCI_StartMotor()
↓
状态机开始启动流程
```

这里先放一个 0 rpm 安全参考，是因为当前代码明确要求启动前先有速度/转矩/电流参考。

### 23. `MotorMgr_STM` 为什么作为 MCSDK 的统一应用层接口

如果 CAN、UART、MCP、按键都直接调用 MCSDK，会出现：

- 模式切换逻辑重复。
- 限速规则不一致。
- PID 修改规则不一致。
- 位置轨迹状态难统一。
- 后续换 MCSDK API 时要改很多文件。

当前设计：

```text
CAN_STM ─┐
USART_STM├→ CommMgr_STM → MotorMgr_STM → MCSDK
MCP_STM ─┘
```

`MotorMgr_STM` 负责：

- 当前控制模式。
- RUN 状态切换处理。
- ±2600 rpm 限速。
- 速度 Ramp。
- 位置轨迹。
- 最近单圈路径。
- PID 临时设置。
- 启停/故障确认。
- 将 MCSDK 状态整理成统一 `MotorState_STM`。

### 24. 速度模式和位置模式如何切换

入口：

```c
MotorMgr_STM_SetMode()
```

切到位置：

```text
SetMode(POSITION)
↓
若正在 RUN
↓
MotorMgr_STM_HoldCurrentPosition()
↓
读取 MC_GetCurrentPosition1()
↓
清位置 PID 积分/历史
↓
清旧轨迹
↓
Theta = 当前实际位置
↓
PositionControlRegulation = true
```

切到速度：

```text
SetMode(SPEED)
↓
MotorMgr_STM_EnterSpeedMode()
↓
PositionControlRegulation = false
↓
清位置 PID/轨迹
↓
若 RUN，先下发 0 rpm
```

这样做的目的叫 **bumpless transfer（无扰切换）**：避免模式切换瞬间沿用旧积分、旧轨迹导致电机突然跳动。

### 25. 速度 Ramp 为什么需要

如果目标从 0 rpm 瞬间变 2600 rpm：

```text
速度误差瞬间巨大
↓
Iq_ref 打满
↓
电流限幅/母线电压饱和
↓
机械冲击
```

Ramp 把目标随时间平滑变化。

当前 CAN：

```c
MOTOR_CAN_SPEED_RAMP_MS = 150
```

调用：

```text
CAN_STM_ExecuteCommand()
↓
CommMgr_STM_SetSpeed(...,150ms)
↓
MotorMgr_STM_SetSpeed()
↓
MC_ProgramSpeedRampMotor1_F()
↓
MCI_ExecSpeedRamp_F()
```

UART 同样使用 150 ms。

### 26. 位置轨迹控制与最近单圈位置控制

MCSDK `TC_MoveCommand()` 会生成一条带加速、巡航、减速的平滑轨迹，而不是把角度目标一步跳过去。

本项目 `MotorMgr_STM_SetPosition()`：

```text
检查 RUN + POSITION mode
↓
限制 duration 100 ms ~ 120 s
↓
清旧误差历史
↓
MC_ProgramPositionCommandMotor1(target rad, duration s)
↓
MCSDK trajectory controller
```

最近单圈：假设当前 350°，目标 10°。

不能简单走：

```text
350 → 10 = -340°
```

最近路径应该：

```text
350 → 370 = +20°
```

`MotorMgr_STM_SetNearestSingleTurnPosition()` 把差值限制在：

```text
[-18000, +18000] cdeg
```

即 ±180°。

CAN 位置速度规则当前约 180°/s，最短 200 ms；UART 当前采用固定最短/轨迹规则。

### 27. MT6701 / ABZ 编码器原理

MT6701 可以输出 ABZ 增量编码器信号：

- A、B 相相差 90°，用于判断方向和计数。
- Z 相每圈出现一次，可用于零点标记，但当前核心速度/位置反馈主要依赖 AB。

当前参数：

```c
M1_ENCODER_PPR = 1024
```

PPR 表示每圈每相脉冲数。

### 28. TIM 编码器模式和四倍频计数

STM32 TIM Encoder Mode 可以同时识别 A、B 两相的上升沿和下降沿，因此一个 1024 PPR 编码器得到：

$$1024\times4=4096$$

个计数/机械圈。

当前 MCSDK：

```text
TIM3
↓
ENCODER_M1
↓
ENC_Init()
↓
ENC_CalcAngle()
ENC_CalcAvrgMecSpeedUnit()
```

`parameters_conversion.h` 中：

```c
#define M1_PULSE_NBR ((4 * M1_ENCODER_PPR) - 1)
```

这就是四倍频在当前工程中的直接体现。

### 29. 机械角、电角度、极对数关系

机械角是转子物理转过的角度；电角度表示磁场在一个电周期中的位置。

关系：

$$\theta_e=p\theta_m$$

其中 $p$ 为极对数。

当前：

$$p=7$$

因此转子机械转 1 圈：

$$360^\circ$$

电角度经历：

$$7\times360^\circ$$

FOC 的 Park/反 Park 必须使用电角度，而位置控制更多关心机械角。

代码：

```text
ENCODER_M1
↓
SPD_GetElAngle() → FOC Park
SPD_GetMecAngle() → Position regulation
```

### 30. 编码器速度计算与位置计算

位置本质上由累计计数得到：

$$\theta_m=\frac{count}{4096}\times360^\circ$$

速度本质上是单位时间的角度变化：

$$\omega\approx\frac{\Delta\theta}{\Delta t}$$

MCSDK 不让 User 层手动重复计算，而由 encoder 组件负责：

```text
高频：ENC_CalcAngle(&ENCODER_M1)
中频：ENC_CalcAvrgMecSpeedUnit(&ENCODER_M1,...)
```

速度读取给应用：

```c
MC_GetAverageMecSpeedMotor1_F()
```

位置读取：

```c
MC_GetCurrentPosition1()
```

---

## 五、STM32 CAN / USART 通信

### 31. FDCAN / CAN 总线基本原理

CAN 是多主机差分总线，特点：

- 两根差分线 CAN_H/CAN_L。
- 不需要主从轮询才能发送。
- 帧带 ID，不是传统“设备地址”。
- 有硬件仲裁、CRC、ACK、错误检测和 Bus-Off。

当前项目用 STM32G431 的 FDCAN 外设工作在 **Classic CAN** 模式，500 kbit/s。

```text
ESP32 TWAI
↕ CAN 收发器
CAN_H/CAN_L
↕ CAN 收发器
STM32 FDCAN1
```

### 32. CAN 标准帧、ID、DLC、数据域

当前使用标准 11-bit ID、经典 CAN、8 字节数据。

协议：

```text
0x100 ESP32 → STM32：命令
0x180 STM32 → ESP32：状态
0x181 STM32 → ESP32：速度/位置参考
0x182 STM32 → ESP32：Id/Iq 电流
```

命令帧基本结构：

```text
byte0  协议版本
byte1  sequence
byte2  command
byte3.. 参数
```

发送函数：

```text
CAN_STM_Send()
↓
HAL_FDCAN_AddMessageToTxFifoQ()
```

### 33. CAN 仲裁机制

当多个节点同时发送时，CAN 根据 ID 位逐位仲裁。

显性位 0 会覆盖隐性位 1，所以 ID 数值越小，通常优先级越高。

仲裁失败不是报错，失败节点会停止本次发送，等总线空闲后重试。

在当前只有 ESP32 与 STM32 的场景中，仲裁冲突不复杂，但理解它有助于回答“为什么 CAN 适合多节点实时控制网络”。

### 34. CAN 过滤器

STM32 只需要接收命令帧 `0x100`。

当前：

```text
CAN_STM_Init()
↓
FDCAN_FilterTypeDef
↓
FilterID1 = MOTOR_CAN_ID_COMMAND = 0x100
FilterID2 = 0x7FF
↓
HAL_FDCAN_ConfigFilter()
↓
通过后放 RX FIFO0
```

并通过：

```c
HAL_FDCAN_ConfigGlobalFilter(... FDCAN_REJECT ...)
```

拒绝其他不需要的帧。

ESP32 则只需要反馈帧：

```text
0x180 ~ 0x183
```

其 TWAI 掩码过滤器在 `CAN_ESP_Init()` 中配置。

### 35. CAN 接收 FIFO 和中断/回调

STM32 当前采用：

```text
FDCAN 硬件收到帧
↓
过滤器
↓
RX FIFO0
↓
CAN_STM_Tick()
↓
CAN_STM_ProcessRx()
↓
HAL_FDCAN_GetRxFifoFillLevel()
↓
HAL_FDCAN_GetRxMessage()
↓
协议校验
↓
CAN_STM_ExecuteCommand()
```

ESP32 当前 CAN 接收则更典型地使用 ISR + RTOS Queue：

```text
TWAI 收到帧
↓
CAN_ESP_rx_callback()      ISR，只复制数据
↓
xQueueSendFromISR()
↓
CAN_ESP_rx_task()          普通任务上下文
↓
CAN_ESP_parse_status()/references()/electrical()
```

ISR 中不做 JSON、UI、复杂解析，这是实时系统常见设计。

### 36. CAN Bus-Off 是什么以及如何恢复

CAN 控制器发现自己持续发送错误，会累计 TEC。错误严重到一定程度后进入 Bus-Off，主动退出总线，避免一个坏节点拖垮整条总线。

STM32：

```text
CAN_STM_Tick()
↓
CAN_STM_ServiceBus()
↓
HAL_FDCAN_GetProtocolStatus()
↓
status.BusOff == 1
↓
清 link active
↓
HAL_FDCAN_Stop()
↓
HAL_FDCAN_Start()
```

ESP32：

```text
CAN_ESP_tx_task()
↓
CAN_ESP_service_bus_state()
↓
twai_node_get_info()
↓
TWAI_ERROR_BUS_OFF
↓
twai_node_recover()
```

常见 Bus-Off 根因：

- 对端没上电。
- 波特率不一致。
- CAN_H/CAN_L 接反。
- 终端电阻错误。
- 收发器供电/使能错误。
- 没有其他节点 ACK。

### 37. STM32 与 ESP32 的 CAN 命令/遥测协议

共享协议头：

```text
STM32: User/motor_can_protocol.h
ESP32: main/comm/motor_can_protocol.h
```

两份必须保持字节级一致。

命令链，以速度为例：

```text
LVGL
↓
CommMgr_ESP_SetSpeedRPM()
↓
CAN_ESP_SetSpeedRPM()
↓
s_pending_speed_rpm + s_speed_dirty
↓
CAN_ESP_tx_task()
↓
CAN_ESP_transmit(SET_SPEED)
↓ CAN 0x100
STM32 CAN_STM_ProcessRx()
↓
CAN_STM_ExecuteCommand()
↓
CommMgr_STM_SetSpeed()
↓
MotorMgr_STM_SetSpeed()
↓
MC_ProgramSpeedRampMotor1_F()
```

反馈链：

```text
MotorMgr_STM_GetState()
↓
CAN_STM_SendStatus()/SendReferences()/SendElectrical()
↓ 0x180/181/182
ESP32 CAN_ESP_rx_callback()
↓ Queue
CAN_ESP_rx_task()
↓
解析到 s_snapshot
↓
CommMgr_ESP_GetState()
↓
LVGL
```

### 38. USART 基本原理

UART/USART 是点对点异步串行通信。常见参数：

```text
115200, 8N1
```

含义：

- 115200 bit/s。
- 8 数据位。
- 无校验。
- 1 停止位。

UART 没有 CAN 那样的 ID、仲裁和内置帧边界，所以应用层必须自己定义：

- 帧头。
- 长度。
- 类型。
- Payload。
- CRC。

### 39. USART3 + DMA 循环接收

STM32 端初始化：

```text
main()
↓
MX_DMA_Init()
↓
MX_USART3_UART_Init()
↓
CommMgr_STM_Init()
↓
USART_STM_Init()
↓
HAL_UART_Receive_DMA(&huart3, rxDmaBuffer, 128)
```

DMA 在后台持续把 USART3 收到的字节写入 128 字节循环缓冲区。

主循环：

```text
CommMgr_STM_Tick()
↓
USART_STM_Process()
↓
USART_STM_ReadDma()
↓
读取 DMA 当前 head
↓
从 rxTail 追到 head
↓
USART_STM_ParseByte()
```

这样主 CPU 不必每收到 1 字节就阻塞等待。

### 40. DMA 为什么比轮询更适合持续通信

轮询：

```c
while (...) {
    检查串口有没有字节;
}
```

会浪费 CPU，并可能因为其他任务延迟而丢数据。

DMA：

```text
USART 硬件
↓
DMA 自动搬运
↓
RAM 循环缓冲区
↓
CPU 有空时批量解析
```

这对于同时运行 FOC、通信和上位机反馈的项目更合理。

### 41. UART 数据帧设计

当前帧：

```text
0    0xA5
1    0x5A
2    protocol version
3    frame type
4    sequence
5    payload length
6..  payload
末2  CRC16
```

命令 Payload 固定 5 字节：

```text
command 1 byte
value   4 bytes
```

解析状态机：

```text
USART_STM_ParseByte()
↓
找 0xA5
↓
找 0x5A
↓
收 header
↓
读取 payload length
↓
等完整帧
↓
CRC
↓
版本/type/length
↓
USART_STM_HandleCommand()
```

### 42. CRC 校验的作用

串口没有 CAN 那样完善的硬件帧 CRC，所以项目自己加入 CRC-16/Modbus。

当前算法：

```text
初值 0xFFFF
多项式 0xA001
```

STM32 和 ESP32 两边使用同一算法。

目的：检测传输过程中比特翻转、丢字节、错帧等。

```text
收到完整帧
↓
计算 CRC
↓
与帧尾 CRC 比较
├─ 相同 → 解析命令
└─ 不同 → crcErrors++，丢弃
```

### 43. CAN 与 USART 为什么需要统一管理

没有统一管理时：

```text
CAN 同时发 1000 rpm
UART 同时发 -1000 rpm
```

电机目标会互相覆盖。

ESP32 当前使用 `CommMgr_ESP` 做“控制权仲裁”：

```text
SelectCAN()
↓
CAN_ESP_SetControlEnabled(true)
USART_ESP_SetControlEnabled(false)
```

或者：

```text
SelectUSART()
↓
USART true
CAN false
```

因此任意时刻只有一个通信通道有权控制电机。

STM32 的 `CommMgr_STM` 则统一不同协议进入 MotorMgr 的入口。

### 44. `CommMgr_STM` 的作用

`CommMgr_STM` 很薄，但架构意义很重要。

初始化：

```text
CommMgr_STM_Init()
├─ MotorMgr_STM_Init()
├─ MCP_STM_Init()
├─ CAN_STM_Init()
└─ USART_STM_Init()
```

循环维护：

```text
CommMgr_STM_Tick()
├─ MotorMgr_STM_Tick()
├─ CAN_STM_Tick()
└─ USART_STM_Process()
```

控制转发：

```text
CommMgr_STM_SetSpeed()
→ MotorMgr_STM_SetSpeed()
```

它让 CAN/USART 不需要知道 MCSDK 内部句柄和状态机细节。

### 45. STM32 端完整数据流：通信指令 → CommMgr → MotorMgr → MCSDK → 电机

以 CAN 速度命令为例：

```text
ESP32 CAN_ESP_transmit()
↓
CAN 0x100
↓
STM32 FDCAN RX FIFO0
↓
CAN_STM_ProcessRx()
↓
CAN_STM_ExecuteCommand()
↓
CommMgr_STM_SetSpeed()
↓
MotorMgr_STM_SetSpeed()
├─ 检查 RUN
├─ 检查 SPEED mode
├─ 限幅 ±2600 rpm
└─ duration 限幅
↓
MC_ProgramSpeedRampMotor1_F()
↓
MCI_ExecSpeedRamp_F()
↓
中频速度控制
↓
Iq_ref
↓
16 kHz 电流环
↓
SVPWM
↓
TIM1
↓
逆变器
↓
电机
```

### 46. STM32 遥测数据流：MCSDK → MotorMgr → CommMgr → CAN/UART → ESP32

状态采集统一发生在：

```text
MotorMgr_STM_GetState()
```

里面读取：

```text
MC_GetSTMStateMotor1()
MC_GetCurrentFaultsMotor1()
MC_GetAverageMecSpeedMotor1_F()
MC_GetMecSpeedReferenceMotor1_F()
MC_GetCurrentPosition1()
MC_GetTargetPosition1()
MC_GetIqdMotor1_F()
MC_GetIqdrefMotor1_F()
```

CAN：

```text
CAN_STM_Tick()
↓
CommMgr_STM_GetMotorState()
↓
CAN_STM_SendStatus()
CAN_STM_SendReferences()
CAN_STM_SendElectrical()
```

USART：

```text
USART_STM_Process()
↓ 每 20 ms
USART_STM_SendTelemetry()
↓
HAL_UART_Transmit()
```

---

## 六、ESP32、FreeRTOS、LVGL、Wi-Fi

### 47. ESP32 / ESP-IDF 基本工程结构

当前：

```text
ESP_HMI/main/
├─ main.c
├─ bsp/       硬件板级支持
├─ comm/      CAN / USART / CommMgr
├─ network/   Wi-Fi 与网络相关
└─ ui/        LVGL 界面
```

入口：

```text
app_main()
↓
CommMgr_ESP_Init()
↓
板级 LCD/触摸初始化
↓
网络组件初始化
↓
创建 LVGL UI
```

ESP32 的定位：人机界面 + 通信网关，不承担 FOC 实时闭环。

### 48. FreeRTOS 任务的基本概念

ESP-IDF 本身运行在 FreeRTOS 上。任务可以理解为多个“并发执行的 while(1)”。

当前通信中：

CAN：

```text
CAN_ESP_rx_task
CAN_ESP_tx_task
```

USART：

```text
USART_ESP_rx_task
USART_ESP_tx_task
```

任务之间通过：

- Queue。
- 共享状态 + critical section。
- `vTaskDelay()` / `vTaskDelayUntil()`。

进行协作。

关键思想：中断只做最少工作，复杂处理放到任务。

### 49. ESP32 CAN 与 USART 通信模块

CAN 模块：

```text
CAN_ESP_Init()
├─ CAN 收发器 GPIO 自检
├─ xQueueCreate()
├─ twai_new_node_onchip()
├─ twai_node_config_mask_filter()
├─ 注册 RX/error callback
├─ twai_node_enable()
├─ CAN_ESP_rx_task
└─ CAN_ESP_tx_task
```

USART 模块：

```text
USART_ESP_Init()
├─ uart_driver_install()
├─ uart_param_config()
├─ uart_set_pin()
├─ xQueueCreate()
├─ USART_ESP_rx_task
└─ USART_ESP_tx_task
```

二者最终都暴露类似 API：

```text
SetMode
SetSpeed
SetPosition
Start
Stop
AcknowledgeFault
GetSnapshot
```

这为 `CommMgr_ESP` 统一管理打下基础。

### 50. `CommMgr_ESP` 的作用以及通信仲裁

`CommMgr_ESP` 是 ESP32 侧最关键的应用通信抽象层。

它维护：

```c
static CommMgr_ESP_transport_t s_transport;
```

可能状态：

```text
NONE
USART
CAN
```

CAN 选择：

```text
CommMgr_ESP_SelectCAN()
↓
若未初始化 → CAN_ESP_Init()
↓
s_transport = CAN
↓
CAN_ESP_SetControlEnabled(true)
USART_ESP_SetControlEnabled(false)
```

控制接口：

```text
LVGL
↓
CommMgr_ESP_SetSpeedRPM()
├─ USART active → USART_ESP_SetSpeedRPM()
└─ CAN active   → CAN_ESP_SetSpeedRPM()
```

UI 因此完全不需要知道当前实际走 CAN 还是 UART。

### 51. LVGL 的基本概念

LVGL 是嵌入式 GUI 库。基本对象包括：

- `lv_obj_t`：所有 UI 对象基类。
- label。
- button。
- slider。
- dropdown。
- textarea。
- event callback。

当前 UI 分为：

```text
MENU
FEEDBACK
USART
CAN
WI-FI
MQTT
PID TEST
SPEED
POSITION
```

UI 创建主要在：

```text
motor_ui.c
```

交互行为主要在：

```text
motor_ui_events.c
```

这样把“画界面”和“响应按钮”拆开。

### 52. LVGL 页面、按钮、事件回调

典型模式：

```text
创建 button
↓
lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, ...)
↓
用户触摸
↓
LVGL 触发 callback
```

例如 CAN 连接：

```text
按钮点击
↓
motor_ui_can_connect_event()
↓
CommMgr_ESP_SelectCAN()
```

速度模式：

```text
motor_ui_speed_mode_event()
↓
CommMgr_ESP_SetMode(SPEED)
↓
CommMgr_ESP_Start()
```

### 53. LVGL UI 如何向 STM32 下发电机控制命令

速度滑块：

```text
用户拖动 slider
↓
motor_ui_speed_slider_event()
↓ LV_EVENT_RELEASED
pending_speed_rpm = slider value
↓
CommMgr_ESP_SetMode(SPEED)
↓
CommMgr_ESP_SetSpeedRPM()
↓
当前 transport
├─ CAN_ESP_SetSpeedRPM()
└─ USART_ESP_SetSpeedRPM()
↓
TX task
↓
STM32
```

位置同理：

```text
motor_ui_position_slider_event()
↓
CommMgr_ESP_SetMode(POSITION)
↓
CommMgr_ESP_SetPositionCdeg()
```

这是 ESP32 端面试最应该讲清楚的一条链。

### 54. ESP32 Wi-Fi 基本流程

#### 54.1 Wi-Fi 到底是什么

Wi-Fi 是基于 IEEE 802.11 的无线局域网技术，解决设备如何通过无线链路接入
局域网。它本身不等于互联网，也不等于 MQTT。

把分层关系背清楚：

```text
MQTT：应用层消息协议
TCP：可靠字节流传输
IP：寻址和路由
Wi-Fi / 802.11：无线局域网链路
射频：2.4 GHz 电磁波
```

本工程 ESP32-S3 使用 2.4 GHz Wi-Fi，以 STA 模式加入无线路由器或热点；
Mosquitto Broker、Qt 和 ESP32 需要在 IP 层能够互相访问。

#### 54.2 AP、STA、SSID、BSSID、信道、RSSI

- **AP（Access Point）**：接入点，例如无线路由器或手机热点。
- **STA（Station）**：接入 AP 的终端，本工程 ESP32 就是 STA。
- **SSID**：用户看到的 Wi-Fi 名称，同名 SSID 可能对应多个 AP。
- **BSSID**：一个具体 AP 射频接口的 MAC 地址，比 SSID 更精确。
- **Channel**：无线信道；信道拥挤会造成竞争、重传和时延抖动。
- **RSSI**：接收信号强度，单位 dBm，越接近 0 通常越强，例如 -45 dBm
  通常强于 -75 dBm；它只反映接收功率，不直接等于吞吐量和稳定性。

当前扫描结果保存 SSID、RSSI 和是否加密，按 RSSI 从强到弱排序；相同 SSID 会
去重。因此 UI 面向易用性，没有让用户按 BSSID 选择同名 AP。

#### 54.3 从扫描到真正可联网的阶段

```text
初始化 NVS、esp_netif、事件循环和 Wi-Fi driver
↓
设置 STA 模式并启动
↓
扫描 AP，用户选择 SSID 和输入密码
↓
802.11 Authentication（认证）
↓
Association（关联）
↓
DHCP 获取 IPv4 地址
↓
IP 层可用
↓
TCP 连接 Broker
↓
MQTT CONNECT / CONNACK
```

“已经连上 AP”和“已经拿到 IP”不是同一件事。本工程收到
`WIFI_EVENT_STA_CONNECTED` 时只显示 `waiting for IP`，直到
`IP_EVENT_STA_GOT_IP` 才把 `snapshot.connected` 置为 true。MQTT 页面也只有
在这个状态下才允许连接 Broker。

#### 54.4 当前工程初始化链

```text
app_main()
↓
wifi_manager_init()
├─ xSemaphoreCreateMutex()             状态锁
├─ esp_timer_create()                  重连定时器
├─ nvs_flash_init()                    Wi-Fi 配置存储基础
├─ esp_netif_init()                    TCP/IP 网络接口层
├─ esp_event_loop_create_default()     默认事件循环
├─ esp_netif_create_default_wifi_sta()
├─ esp_wifi_init()
├─ 注册 WIFI_EVENT / IP_EVENT
├─ esp_wifi_set_storage(WIFI_STORAGE_FLASH)
├─ esp_wifi_set_mode(WIFI_MODE_STA)
├─ esp_wifi_start()
└─ esp_wifi_set_ps(WIFI_PS_NONE)
```

工程先分配整屏 DMA 显示缓冲区，再启动网络栈，以减少网络组件分配内存造成的
碎片对 LCD 注册的影响。这说明嵌入式系统的初始化顺序也是资源设计。

#### 54.5 当前异步扫描实现

```text
motor_ui_wifi_scan_event()
↓
wifi_manager_scan_async()
↓
esp_wifi_scan_start(NULL, false)       false 表示非阻塞
↓
WIFI_EVENT_SCAN_DONE
↓
wifi_manager_store_scan_results()
├─ esp_wifi_scan_get_ap_num()
├─ esp_wifi_scan_get_ap_records()
├─ 按 RSSI 降序
├─ 按 SSID 去重
└─ 最多保留 12 个 AP
↓
更新 snapshot / scan_generation
↓
LVGL 50 ms 定时器发现版本变化并刷新下拉列表
```

异步扫描的意义是按钮回调不阻塞等待几秒；`ESP_OK` 只代表扫描已启动，最终结果
通过事件返回。

#### 54.6 当前异步连接实现

```text
motor_ui_wifi_connect_event()
↓
检查已选择网络；加密网络密码至少 8 字符
↓
wifi_manager_connect()
├─ 校验 SSID ≤ 32 字节、密码 ≤ 63 字节
├─ 填充 wifi_config_t
├─ 停止旧重连定时器
├─ esp_wifi_disconnect()               清理旧连接
├─ esp_wifi_set_config(WIFI_IF_STA)
└─ esp_wifi_connect()
↓
WIFI_EVENT_STA_CONNECTED
↓
IP_EVENT_STA_GOT_IP
↓
保存 IPv4 并将 connected=true
```

UI 不直接操作 `esp_wifi_*`，而通过 `wifi_manager`。`wifi_manager_connect()` 返回
`ESP_OK` 只表示请求已提交，不表示密码正确或已经得到 IP，这是异步 API 的关键
语义。

#### 54.7 DHCP、IPv4、子网、ARP 和网关

DHCP Server（通常是路由器）给 ESP32 分配 IPv4 地址、子网掩码、默认网关和
DNS。当前 Broker 默认地址是 `192.168.10.7:1883`。若 ESP32 地址也是
`192.168.10.x/24`，它会判断 Broker 在同一子网，通过 ARP 查询 Broker 主机的
MAC 地址，再直接建立局域网 TCP 连接，不需要经过公网。

#### 54.8 断线重连为什么使用指数退避

当前意外断线后的延时约为：

```text
1 s → 2 s → 4 s → 8 s → 10 s（封顶）
```

调用链：

```text
WIFI_EVENT_STA_DISCONNECTED
↓
wifi_manager_schedule_reconnect_locked(reason)
↓
esp_timer_start_once()
↓
wifi_manager_reconnect_timer_callback()
↓
esp_wifi_connect()
```

指数退避避免 AP 长时间离线时高速重试，占用 CPU、无线信道和日志。用户主动点击
DISCONNECT 时设置 `s_user_disconnect`，不自动重连；切换网络前预期产生的断线
事件由 `s_ignore_next_disconnect` 忽略，避免错误恢复旧连接。

#### 54.9 为什么关闭 Wi-Fi 省电模式

当前调用：

```c
esp_wifi_set_ps(WIFI_PS_NONE);
```

关闭省电通常能降低休眠唤醒带来的时延和抖动，适合有外部供电、重视测试稳定性
的 HMI；代价是功耗更高。面试时要讲清“延迟和功耗的权衡”。

#### 54.10 为什么事件回调不直接改 LVGL

Wi-Fi 事件来自 ESP-IDF 事件任务，LVGL 对象应在受控 UI 上下文中访问。当前设计：

```text
Wi-Fi event callback
↓
mutex 下更新 wifi_manager_snapshot_t
↓
revision / scan_generation++
↓
LVGL 50 ms timer
↓
复制 snapshot 并刷新控件
```

这是“线程安全状态快照 + 版本号”模式，既避免跨任务直接改 UI，也避免状态没有
变化时重复重建列表。

#### 54.11 Wi-Fi 故障按层定位

```text
扫描不到 AP
→ 检查 2.4 GHz、天线、距离、信道、Wi-Fi driver

扫到但关联失败
→ 检查密码、认证方式、PMF、disconnect reason

已关联但无 IP
→ 检查 DHCP Server、地址池、esp_netif/IP_EVENT

有 IP 但 Broker 连不上
→ 检查 Broker IP/端口、防火墙、路由、Mosquitto监听地址

MQTT 经常断
→ 检查 RSSI/干扰、TCP写超时、Broker负载、outbox、keepalive
```

Wi-Fi `reason` 是 802.11 层线索；MQTT socket/TCP 错误是更上层线索，不能把
所有“断联”都归因于 MQTT 协议。

#### 54.12 当前 Wi-Fi 实现的边界与改进

- 当前面向局域网实验环境，不是完整量产配网方案。
- UI 源码存在预填的明文测试密码；正式产品应删除，改用安全配网和受保护的
  凭据存储，并禁止日志打印密码。
- 同名 SSID 去重后不能指定 BSSID，可增加 BSSID/信道显示。
- 当前系统时间初值来自编译时间，没有 SNTP；TLS 证书校验和准确日志需要校时。
- 可增加 RSSI 周期监控、断线原因映射、静态 IP 和网络健康统计。

### 55. TCP/IP 在 ESP32 中由谁实现

不是你自己从零写 TCP/IP 协议栈。

ESP-IDF 使用网络协议栈（底层通常由 lwIP 等组件实现），负责：

- IP。
- ARP。
- ICMP。
- TCP。
- UDP。
- DHCP 等。

你的应用层通常只使用 socket 或更高层客户端 API。

层次：

```text
应用层
↓
MQTT / HTTP / 自定义协议
↓
TCP/UDP API
↓
ESP-IDF 网络栈
↓
Wi-Fi driver
↓
802.11 无线链路
```

#### 55.1 TCP 和 UDP 的区别

TCP 面向连接，保证有序、可靠、去重，有重传、流量控制和拥塞控制；UDP 是无连接
数据报，不保证到达、顺序或去重，但开销更低。MQTT 通常运行在 TCP 上，本工程
也是如此。

#### 55.2 TCP 可靠不等于应用一定成功

本项目存在两段 TCP：

```text
ESP32 TCP ↔ Broker
Broker TCP ↔ Qt
```

ESP32 收到 PUBACK，能确认 Broker 收到了 QoS 1 PUBLISH，但不能证明 Qt 已保存
PNG。真正端到端确认需要 Qt 完成重组和落盘后再发布应用层 `saved_ack`。当前还
没有这一级确认，主动说明这个边界能体现对分层可靠性的理解。

#### 55.3 TCP 为什么要处理半包和粘包

TCP 是字节流，一次 `read()` 可能得到半个 MQTT 包、一个整包或多个包。Qt 的
`MqttClient::processInput()` 先把字节追加到 `inputBuffer_`，解析 MQTT
Remaining Length，完整后才调用 `processPacket()`，并循环处理多个包。这就是
处理 TCP 半包和粘包的正确方式。

---

## 七、MQTT 核心原理与当前工程实现

### 56. MQTT 基本原理

#### 56.1 MQTT 解决什么问题

MQTT 是轻量级发布/订阅消息协议，通常运行在 TCP 上。它把“谁产生消息”和
“谁消费消息”通过 Broker 解耦：发布者不必知道订阅者 IP，订阅者也不必与每个
设备建立一条专用业务连接。

```text
Qt Client ──发布/订阅──┐
                       │
                   Mosquitto Broker
                       │
ESP32 Client ─发布/订阅┘
```

本工程中 ESP32 和 Qt 都是 Client，Mosquitto 才是 Broker。一个 Client 可以
同时充当 Publisher 和 Subscriber。

#### 56.2 Broker、Publisher、Subscriber、Topic

- **Broker**：维护客户端连接、订阅关系并转发消息。
- **Publisher**：向某个 Topic 发布消息。
- **Subscriber**：订阅 Topic 并接收匹配消息。
- **Topic**：由 `/` 分层的 UTF-8 主题名，例如 `motor/control/telemetry`。
- **Payload**：消息正文，Broker 通常不理解业务含义；可以是 JSON、文本或二进制。

Topic 不是 IP 地址，也不是函数名。Broker 根据订阅过滤器转发。协议支持：

- `+`：匹配一层，例如 `motor/+/status`。
- `#`：匹配后续多层，例如 `motor/#`。

当前工程使用精确主题，没有使用通配符，这样权限和业务边界更明确。

#### 56.3 发布/订阅为什么适合本项目

```text
Qt 发布 PID 命令
→ Broker
→ ESP32 订阅后处理

ESP32 发布状态/测试数据
→ Broker
→ Qt 订阅后重组和保存
```

优点：

- Qt 与 ESP32 不需要彼此固定监听端口。
- 可用 MQTTX 等第三方客户端观察同一 Topic，便于调试。
- 控制、遥测、ACK、测试数据可以分主题管理。
- 后续可增加日志服务或数据库订阅者，而不改变 ESP32 目标地址。

代价：

- 多了 Broker 运维和一个故障点。
- 端到端路径包含两段 TCP，Broker ACK 不等于最终订阅者处理成功。
- 消息可能重复，业务需要 ID、索引和幂等设计。

#### 56.4 MQTT 3.1.1 报文结构

Qt 当前手写的是 MQTT 3.1.1，因为 CONNECT 的 Protocol Level 为 `0x04`。
MQTT 控制报文由三部分组成：

```text
Fixed Header
├─ Packet Type + Flags
└─ Remaining Length（1~4 字节变长编码）

Variable Header
└─ Topic、Packet Identifier 等，随报文类型变化

Payload
└─ JSON、文本或二进制测试数据
```

常见 Packet Type：

| 报文 | 方向 | 作用 |
| --- | --- | --- |
| CONNECT | Client → Broker | 建立 MQTT 会话 |
| CONNACK | Broker → Client | 返回连接结果 |
| SUBSCRIBE | Client → Broker | 订阅主题 |
| SUBACK | Broker → Client | 确认订阅结果 |
| PUBLISH | 双向 | 发布业务消息 |
| PUBACK | QoS1 接收方 → 发送方 | 确认 QoS1 PUBLISH |
| PINGREQ/PINGRESP | Client/Broker | Keep Alive 保活 |
| DISCONNECT | Client → Broker | 正常关闭会话 |

#### 56.5 QoS 0、1、2 必须会比较

| QoS | 语义 | 典型代价 | 本工程用途 |
| --- | --- | --- | --- |
| 0 | At most once，最多一次 | 最低，不确认，可能丢 | 250 ms普通遥测、即时错误提示 |
| 1 | At least once，至少一次 | PUBLISH/PUBACK，可能重复 | 命令、ACK、状态、测试二进制 |
| 2 | Exactly once，恰好一次 | 四步握手，状态和开销最大 | 当前未使用 |

“QoS 1 至少一次”意味着消息不会因为协议设计而静默当成成功，但重传可能让接收方
看到重复消息，所以不能说 QoS1 就绝不重复。本工程测试数据带 `test_id` 和
`startIndex`，Qt 用 `receivedSamples_[index]` 去重，重复分块不会重复计数。

最终交付 QoS 还受发布 QoS、订阅请求 QoS和 Broker 能力共同影响。订阅 QoS1
不是把发布者的 QoS0 自动升级成端到端可靠交付。

#### 56.6 PUBACK 到底证明了什么

ESP32 发布 QoS1 到 Broker：

```text
ESP32 PUBLISH(QoS1, packetId=N)
↓
Broker 接收
↓
Broker PUBACK(packetId=N)
```

PUBACK 证明 Broker 已接收该 PUBLISH，不能直接证明：

- Qt 当时在线。
- Qt 已收到 Broker 转发。
- Qt 已重组全部样本。
- PNG/CSV 已成功落盘。

当前 ESP32 页面上的 `UPLOAD SUCCESS` 是 **Broker 确认成功**。如果要达到真正
端到端成功，应增加：

```text
Qt 完整重组 + 文件 fsync/关闭成功
↓
Qt 发布 motor/control/test/saved_ack
↓
ESP32 收到 matching test_id 后才释放 PSRAM
```

这是当前方案最值得在高级追问中主动说明的改进点。

#### 56.7 Retain、Session、LWT 分别是什么

- **Retained Message**：Broker 为 Topic 保存最后一条 retained 消息，新订阅者
  立即得到它。当前发布的 retain 标志为 0，测试分块不会被长期保留。
- **Clean Session**：客户端连接时是否丢弃旧会话。Qt CONNECT Flags 为 `0x02`，
  即 Clean Session；断线期间 Broker 不为它保留一个持久订阅会话。
- **LWT（Last Will and Testament）**：Client 异常掉线时由 Broker 代发“遗嘱”
  消息。当前没有配置，可增加 `motor/device/<id>/online=false`。

为什么测试数据不使用 retained：新打开的 Qt 不应突然收到上一次实验的最后一个
分块并误认为新测试。设备在线状态则很适合 retained + LWT。

#### 56.8 Keep Alive 和自动重连不是一回事

Keep Alive 用于发现死连接：一段时间没有其他报文时 Client 发送 PINGREQ，Broker
回复 PINGRESP。它不能修复连接；断开后还需要重连策略。

当前参数：

- ESP32 MQTT Keep Alive：30 s。
- ESP32 MQTT reconnect timeout：3 s。
- Qt MQTT Keep Alive：20 s，每 10 s 主动发 PINGREQ。
- Wi-Fi 自身另有 1/2/4/8/10 s 指数退避重连。

所以恢复链是分层的：先 Wi-Fi/IP 恢复，再 TCP/MQTT 恢复。

#### 56.9 Client ID 为什么必须唯一

Broker 用 Client ID 标识会话。如果两个在线客户端使用同一 ID，很多 Broker 会
断开旧连接，表现为双方交替掉线。

当前：

```text
ESP32: esp32s3-motor-<STA MAC>
Qt:    qt-pid-test-<进程 PID>
```

ESP32 使用 MAC 形成设备唯一 ID；Qt 使用进程 PID 避免同一台电脑多开时直接冲突。

#### 56.10 MQTT 与 HTTP、WebSocket、直接 TCP 怎么选

- MQTT：发布/订阅、长连接、小头部、设备状态与遥测方便。
- HTTP：请求/响应清晰，适合配置、文件、REST API，但高频双向状态不如 MQTT 自然。
- WebSocket：浏览器友好，提供全双工帧，业务协议仍需设计。
- 直接 TCP：控制最自由，但需要自己设计连接、路由、心跳、重连和消息格式。

本工程需要 Qt、ESP32、调试工具多方观察消息，因此 MQTT 的 Broker 解耦很合适；
但高速 CAN 采样不应该逐点实时走 MQTT，所以采用本地缓存后批量上传。

### 57. ESP32 `mqtt_manager` 的当前实现

#### 57.1 初始化与连接线程

```text
app_main()
↓
mqtt_manager_init()
├─ 读取 STA MAC 生成 Client ID
├─ 创建状态 mutex
├─ 创建 client API mutex
├─ 创建长度 4 的 connect/disconnect command queue
└─ 创建 mqtt_manager worker task
```

MQTT 页面默认 URI：

```text
mqtt://192.168.10.7:1883
```

点击 CONNECT：

```text
motor_ui_mqtt_connect_event()
├─ 先检查 Wi-Fi 已取得 IP
└─ mqtt_manager_connect_async(uri)
    ↓ command queue
mqtt_manager_worker()
├─ 停止/销毁旧 client
├─ esp_mqtt_client_init()
├─ esp_mqtt_client_register_event()
└─ esp_mqtt_client_start()
```

把连接/销毁放到 worker，而不是 LVGL 点击回调，是为了避免网络操作阻塞 UI。

#### 57.2 当前 ESP-MQTT 配置

```text
Keep Alive             30 s
Reconnect timeout       3 s
Outbox limit            8192 bytes
Client ID               STA MAC 派生
URI                      mqtt:// 或 mqtts:// 形式校验
```

当前实际默认使用 `mqtt://` 明文 1883，没有配置用户名/密码和 TLS 证书校验，因此
只适合受控局域网实验。虽然 URI 校验接受 `mqtts://`，但生产级 TLS 还需要 CA
证书/证书包、Broker 证书域名校验、准确系统时间和凭据策略，不能只改协议前缀。

#### 57.3 MQTT 事件驱动状态机

```text
MQTT_EVENT_BEFORE_CONNECT
→ connecting=true

MQTT_EVENT_CONNECTED
→ connected=true
→ 清 PUBACK 历史
→ 订阅 motor/hmi/test/rx（QoS1）
→ 订阅 motor/control/command（QoS1）

MQTT_EVENT_DATA
→ 拼接分片
→ 完整后 received_messages++
→ 调用注册的业务 callback

MQTT_EVENT_PUBLISHED
→ 记录 msg_id，供 PUBACK 查询

MQTT_EVENT_ERROR / DISCONNECTED
→ 保存 TCP/拒绝码诊断
→ connected=false，等待 ESP-MQTT 重连
```

ESP-MQTT 的 `MQTT_EVENT_PUBLISHED` 对 QoS1 表示对应消息已收到 Broker PUBACK。
网关保存最近 32 个 message ID，上传状态机轮询精确 ID。

#### 57.4 为什么收到的 MQTT 数据要复制

ESP-MQTT 事件中的 topic/data 缓冲区只在回调期间有效，而且较大消息可能分多次
事件到达。当前代码用：

```text
current_data_offset
data_len
total_data_len
```

将内容复制到 `s_snapshot.last_payload`，完整后再调用业务回调。不能把 event->data
指针保存下来稍后使用，否则回调返回后可能成为悬空指针。

当前控制 JSON 最大保存 511 字节，适合短命令，不适合通过这个文本回调传测试二进制；
测试二进制是 ESP32 发布出去，由 Qt 的 QByteArray 按显式长度接收。

#### 57.5 为什么有两个互斥锁

- `s_lock`：保护 snapshot、client 指针、回调和 PUBACK 历史。
- `s_client_api_lock`：串行化 publish 与 stop/destroy。

如果 worker 正在销毁 client，另一个任务同时调用 publish，可能访问已经失效的
client。单独的 API 锁让“取 client → enqueue”和“置空 → stop → destroy”互斥；
又避免长时间持有普通状态锁造成事件回调死锁。

#### 57.6 当前三类发布 API

```text
mqtt_manager_publish()
→ QoS1 文本/JSON，进入 outbox

mqtt_manager_publish_qos0()
→ QoS0 即发即弃，store=false

mqtt_manager_publish_binary_qos1_tracked()
→ 显式长度二进制 + QoS1 + 返回 message_id
```

QoS0 不能用 `strlen` 处理二进制零字节；测试数据必须传指针和长度。普通遥测设置
`store=false`，弱网时允许丢弃旧遥测，避免挤占 8 KiB outbox。

#### 57.7 Outbox 是什么，为什么曾导致间歇上传失败

Outbox 保存待发送/待确认的 MQTT 消息。原问题链：

```text
网络短暂变慢
↓
测试分块和普通消息持续 enqueue
↓
8 KiB outbox 逐渐占满
↓
ESP-MQTT 写超时 / enqueue失败 / keepalive受阻
↓
MQTT断开
```

更严重的是旧逻辑只要 `enqueue` 成功就增加 `publish_index`，所有块“入队”后立刻
发送 `complete` 并释放 PSRAM；入队不等于 Broker 已确认，断线时会假成功和数据
丢失。

当前修复：

- QoS0 遥测不存 outbox。
- 测试上传每次只允许一个块在途。
- 必须等该 message ID 的 PUBACK 才推进索引。
- `complete` 也等 PUBACK。
- 30 s 无进展进入可重发失败状态，保留 PSRAM。

#### 57.8 MQTT 错误日志怎么看

管理器会区分：

- TCP transport error：socket errno、TLS/transport 错误。
- connection refused：Broker CONNACK 拒绝码。
- disconnected：连接关闭并准备重连。

网关失败日志还输出：

```text
test_id
progress=已PUBACK样本/总样本
pending_mid
connected
mqtt_status
PSRAM retained
```

`Writing didn't complete in specified timeout` 表示底层传输在时限内没有写完，常见
诱因包括 Wi-Fi 抖动、Broker/网络拥塞、发送缓冲压力；它不是“JSON 格式错误”的
同义词。定位时要同时看 Wi-Fi reason、MQTT error、Broker日志和 outbox进度。

### 58. 当前工程的 Topic、消息和命令协议

#### 58.1 正式业务 Topic

| Topic | 方向 | QoS/格式 | 作用 |
| --- | --- | --- | --- |
| `motor/control/command` | Qt → ESP32 | QoS1 JSON | PID、停止及通用命令 |
| `motor/control/telemetry` | ESP32 → Qt | QoS0 JSON | 250 ms链路/运行状态 |
| `motor/control/ack` | ESP32 → Qt | QoS1 JSON | 命令接受/拒绝 |
| `motor/control/test/status` | ESP32 → Qt | 通常QoS1 JSON | 测试阶段、点数、周期 |
| `motor/control/test/data` | ESP32 → Qt | QoS1二进制 | 测试结束后的高速样本 |

此外 MQTT 页面保留联调主题：

```text
订阅 motor/hmi/test/rx
发布 motor/hmi/test/ping
发布 motor/hmi/test/wifi
发布 motor/hmi/test/motor
```

#### 58.2 控制命令 JSON

通用结构：

```json
{"id":123,"cmd":"stop","value":0}
```

`id` 用于将 ACK 与命令关联；`cmd` 是命令名。网关当前识别：

- `claim`
- `set_mode`
- `set_speed`
- `set_position`
- `set_pid`
- `start`
- `stop`
- `ack_fault`
- `zero_position`

`run_test` 和 `retry_upload` 只允许 ESP32 本地页面调用；从 MQTT 远程发送会被拒绝，
这是为了把电机测试的启动权放在设备现场。

PID 命令示例：

```json
{"id":124,"cmd":"set_pid","controller":1,"kp":48,"ki":4,"kd":8}
```

controller：0速度、1位置、2 Iq、3 Id。速度/Iq/Id 使用 Kp/Ki，位置使用
Kp/Ki/Kd。电机运行中拒绝改 PID；停止且 CAN 在线时立即临时下发 STM32，否则
只缓存到 ESP32，下一次本地测试前统一应用。

#### 58.3 ACK JSON

```json
{"id":124,"ok":true,"message":"PID applied and cached temporarily"}
```

网络层“发布成功”不等于命令业务执行成功，所以需要业务 ACK：例如 MQTT 消息
到了 ESP32，但 STM32 link offline，网关应返回 `ok=false`。

#### 58.4 普通遥测 JSON

```json
{
  "version":2,
  "transport":2,
  "can_online":true,
  "link_active":true,
  "running":false,
  "motor_fault":false,
  "mode":0,
  "faults":0,
  "sample_sequence":1000
}
```

250 ms 遥测只表达低速状态，不承载曲线点。测试采样/上传期间暂停常规遥测，上传
失败等待重发时恢复遥测。

#### 58.5 测试状态 JSON

```json
{
  "id":1000000001,
  "stage":"recording",
  "mode":0,
  "samples":3500,
  "sample_period_us":2000,
  "message":"Target applied; recording CAN feedback"
}
```

当前主要阶段：

```text
accepted → recording → sending → complete
                         └→ error（失败，ESP页面可重发）
```

Qt 兼容解析 `publishing`/`aborted`，但当前 ESP32 正常状态机主要发布上述阶段。

#### 58.6 为什么状态用 JSON、测试数据用二进制

JSON 可读、可用 MQTTX 调试、字段易扩展，适合低频小消息；但每个数字都变成文本，
体积大且解析开销高。约 3500 个采样点若全部 JSON 化，会显著增加 PSRAM、网络和
Qt 解析压力。

所以：

```text
控制/状态：JSON，可读性优先
测试曲线：定长小端二进制，效率优先
```

#### 58.7 `MCTD` 二进制分块格式

每块最大 40 个样本：

```text
20-byte header
0..3    magic "MCTD"
4       version = 1
5       mode: 0速度 / 1位置
6       record_size = 20
7       last-chunk flag
8..11   test_id, uint32 little-endian
12..15  start_index, uint32 little-endian
16..17  count, uint16 little-endian
18..19  total, uint16 little-endian

每条 20-byte record
0..3    relative time_us
4..5    measured_speed_rpm
6..7    reference_speed_rpm
8..9    current_position_cdeg
10..11  target_position_cdeg
12..13  iq_ma
14..15  id_ma
16..17  iq_reference_ma
18..19  id_reference_ma
```

最大 MQTT Payload：

$$20+40\times20=820\ \text{bytes}$$

模式不使用的速度或位置字段填 0。整数统一小端，Qt 用 `readU16Le/readS16Le/
readU32Le` 显式解析，避免结构体 padding、端序和编译器 ABI 差异。

### 59. ESP32 电机 MQTT 网关与本地 PID 测试

#### 59.1 为什么 ESP32 做网关

```text
STM32：FOC、采样、PWM、编码器、保护
ESP32：LVGL、Wi-Fi、MQTT、PSRAM、CAN/UART网关
Qt：PID编辑、数据重组、分析报告
```

网络抖动不能进入 STM32 16 kHz 电流环。ESP32 把网络命令转换为 CommMgr 接口，
再由 CAN/UART 控制 STM32；反向把 STM32 遥测变成 MQTT 消息。

#### 59.2 为什么测试在 ESP32 页面启动

当前 Qt 不再发送 `run_test`。操作者必须在 ESP32 的 `PID TEST` 页面点击：

- `SPEED CONTROL`：固定 500 RPM。
- `POSITION CONTROL`：固定 90.00°。
- 测试时间固定 7 s。

这样测试启动需要现场操作，且采集状态机与 CAN 链路在同一设备上，不依赖 Qt 按钮
到达时刻。Qt 只负责测试前下发临时 PID，以及测试后的接收和保存。

#### 59.3 测试状态机

```text
IDLE
↓ 本地按钮
STARTING
├─ 检查 MQTT online、电机已停止
├─ 只从 PSRAM 分配测试缓冲
├─ 自动选择 CAN
├─ 检查 link/fault
├─ 应用缓存 PID
├─ 设置速度/位置模式
├─ START
└─ 等待 STM32 回显 RUN + 正确模式
↓
RECORDING
├─ 下发 500 RPM 或 90°
├─ 新 sample_sequence 到达时记录
├─ 检查 link/fault/RUN/command_rejected
└─ 相对时间达到 7 s 后 STOP
↓
STOPPING
└─ 等待 motor_running=false
↓
SENDING
├─ 40点/块，50 ms节流
├─ 一次一个 in-flight QoS1块
├─ 每块等 PUBACK 后推进
└─ complete 也等 PUBACK
↓
IDLE + 页面 UPLOAD SUCCESS + 释放PSRAM

任何上传故障
↓
SEND_FAILED + PSRAM retained + RESEND可用
```

#### 59.4 PSRAM 样本为什么是 16 字节而线上是 20 字节

PSRAM 内部只存当前模式需要的主变量：

```text
time_us                 4
primary_measured        2
primary_reference       2
Iq / Id                 4
Iq_ref / Id_ref         4
合计                    16 bytes
```

最多 3600 点：

$$3600\times16=57600\ \text{bytes}$$

线上为了让 Qt 使用统一记录格式，同时保留速度字段和位置字段，所以扩展成 20 字节。
这是“内部紧凑存储、外部稳定协议”的设计。

#### 59.5 为什么不用内部 RAM，也不回退

```c
heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
```

测试数据必须来自 PSRAM。分配失败就不启动电机，避免 57.6 KiB 突然挤占内部 SRAM，
影响 LVGL、Wi-Fi、任务栈和 DMA。终端会区分：

- PSRAM 未初始化/未注册/被 sdkconfig 禁用：total=0。
- 总空闲空间不足：free < required。
- 堆碎片：free 足够但 largest block 不足。
- 能力或堆完整性异常。

#### 59.6 为什么采样时不实时发 MQTT

STM32 每约 2 ms 提供一组完整 CAN 遥测，约 500 sample/s。若采一个点立刻发一个
MQTT JSON：

- Wi-Fi 时延会反向干扰采集任务调度。
- 500条/s产生大量 TCP/MQTT头部和 JSON 开销。
- 网络抖动会改变采样间隔甚至丢点。
- MQTT outbox 容易积压。

当前方案：

```text
测试中：CAN反馈 → ESP32 PSRAM，完全不上传曲线
测试后：电机停止 → PSRAM数据分块上传
```

因此采样时间分辨率由 CAN反馈的 `sample_timestamp_us/sample_sequence` 决定，而
不是由 Wi-Fi 速度决定。

#### 59.7 `sample_sequence` 为什么比固定 RTOS delay 更准确

网关只在 `snapshot.sample_sequence` 变化时记录新点，并使用 CAN 完整样本携带的
`sample_timestamp_us` 计算相对时间。若 gateway task 在同一个样本上循环多次，
不会重复记录；若任务调度存在抖动，时间轴仍来源于样本实际到达时刻。

#### 59.8 一次一个在途块为什么更稳定

最大块 820 字节，outbox 只有 8192 字节。如果连续 enqueue 而不看 PUBACK，弱网
很快积压多块。当前状态机：

```text
enqueue block N → 得到 message_id
↓
等待 MQTT_EVENT_PUBLISHED 中相同 message_id
↓
publish_index += count
↓ 50 ms
enqueue block N+1
```

约 3500 点需要约 88 块，单纯 50 ms 节流的理论最短发送时间约 4.4 s，再加实际
PUBACK和最终 complete；上传比测试慢是允许的，因为电机已经停止。

#### 59.9 RESEND 的完整语义

上传出现断线、enqueue持续失败、数据 PUBACK超时或 complete PUBACK超时：

- 不调用通用 reset，不释放 PSRAM。
- 页面显示具体错误，启用 `RESEND`。
- 速度/位置新测试按钮保持禁用，防止覆盖唯一缓冲区。
- Wi-Fi/MQTT恢复后点击 RESEND。
- 使用原 test ID，从 index 0 重新发送完整数据集。
- Qt 按 test ID + index 重组并去重。
- 所有块和 complete 获得 Broker PUBACK 后才释放。

从 0 重发而不是只发“ESP32认为失败的块”，是因为 ESP32只知道 Broker ACK，不能
知道 Qt究竟收到了哪些块；全量重发配合幂等索引更简单可靠。

注意：如果 Broker 已确认所有消息而 Qt 当时离线，ESP32仍可能释放 PSRAM；手动
RESEND只覆盖 ESP32检测到的上传失败。要彻底覆盖此情况，需要前述 Qt应用层落盘
确认。

#### 59.10 PID 临时值、Qt保存值和工程默认值的区别

```text
工程默认值
→ 固件源码/MCSDK生成值，设备重启后的基准

ESP32运行时缓存
→ MQTT set_pid 更新，下一次本地测试前应用，不写STM32 Flash

Qt QSettings保存值
→ PC端程序下次启动自动加载，只改变Qt默认输入
```

“保存 PID 参数”不是把参数写入 STM32 Flash；“恢复工程默认 PID”是 Qt 删除本地
保存配置并把当前工程默认分子重新放到界面，连接且停机时可临时下发。

---

## 八、Qt 上位机

### 60. Qt 6 Widgets 基本结构

当前 Qt 工程主要是传统 Widgets 桌面 GUI。

入口：

```text
main.cpp
↓
QApplication
↓
MainWindow
↓
show()
↓
Qt event loop
```

`MainWindow` 中创建：

- Broker/连接区域。
- PID 参数编辑。
- ESP32 本地速度/位置测试接收状态。
- 停止按钮。
- 测试状态。
- 输出文件夹。

即 Qt 主要负责“操作者界面 + 测试组织 + 数据后处理”。

### 61. Qt 信号与槽机制

Qt 不需要自己写一个 while(1) 不断检测按钮。

典型：

```cpp
connect(applyPidButton_, &QPushButton::clicked,
        this, &MainWindow::applyPidParameters);
```

含义：

```text
“临时应用 PID”按钮 clicked 信号
↓
Qt event loop
↓
MainWindow::applyPidParameters() 槽函数
```

类似还有：

```text
stopButton clicked         → emergencyStop()
applyPidButton clicked     → applyPidParameters()
savePidButton clicked      → savePidParameters()
MqttClient messageReceived → processMqttMessage()
```

### 62. Qt MQTT/TCP 网络通信

当前 Qt 没有依赖 Qt MQTT 模块，而是用 `QTcpSocket` 实现了一个最小 MQTT 3.1.1
Client：

```text
MainWindow UI
↓
MqttClient
├─ CONNECT / CONNACK
├─ SUBSCRIBE
├─ PUBLISH QoS1
├─ 接收 PUBLISH
├─ 对入站 QoS1发送 PUBACK
├─ PINGREQ
└─ DISCONNECT
↓
QTcpSocket
↓
Mosquitto Broker
```

这样 UI 不手写 TCP 报文字节，`MqttClient` 也不理解 PID 和电机字段，职责分离清楚。

连接默认值：

```text
Host 192.168.10.7
Port 1883
Client ID qt-pid-test-<process id>
```

TCP connected 后才发送 MQTT CONNECT；收到成功 CONNACK 后 `mqttConnected_=true`，
然后 MainWindow 订阅 telemetry、ack、test/status、test/data 四个主题。

#### Qt 如何处理 MQTT 变长长度和 TCP 半包

发送时 `encodeRemainingLength()` 按 MQTT 的 7-bit + continuation bit 规则编码。
接收时：

```text
QTcpSocket::readyRead
↓
readAll()追加到 inputBuffer_
↓
解析1~4字节 Remaining Length
↓
不完整：保留等待下一次 readyRead
完整：取出一包并继续循环
```

这比假设“一次 readyRead 就是一条 MQTT 消息”更可靠。

#### Qt 当前实现的边界

- 能给收到的 QoS1 PUBLISH 返回 PUBACK。
- 当前没有处理出站 PUBLISH 对应的 PUBACK，也没有维护未确认重传表；
  `socket.write()` 成功只表示进入 Qt socket 缓冲区。
- 当前发 SUBSCRIBE 后没有解析 SUBACK，订阅失败主要通过后续“收不到消息”暴露。
- 当前没有自动重连、持久 Session、LWT、TLS、用户名密码和证书校验。
- 这是适合受控实验环境的轻量 Client；生产化可改用成熟 MQTT 库或补齐完整状态机。

面试时既要能讲手写协议体现了对 MQTT/TCP 帧的理解，也要诚实说明完整 MQTT Client
还应处理 PUBACK/SUBACK、重发、重连和安全。

### 63. Qt 上位机如何发送电机控制命令

当前测试由 ESP32 页面启动，Qt 已经没有速度/位置测试启动按钮。Qt 的网络控制重点
是测试前临时 PID 和紧急停止。

PID 临时下发：

```text
用户编辑速度/位置/Iq/Id增益分子
↓
点击“临时应用 PID”
↓
applyPidParameters()
↓
sendAllPidParameters()
↓
连续构造4条 set_pid JSON
↓
mqtt_.publishQos1(motor/control/command)
↓
Broker
↓
ESP32 网关
↓
缓存；CAN在线且电机停止时立即下发
↓
STM32 MotorMgr_STM_SetPidGain()
```

停止：

```text
emergencyStop()
↓
{"id":N,"cmd":"stop","value":0}
↓
ESP32 CommMgr_ESP_Stop()
↓
STM32
```

Qt 窗口关闭时，如果测试未结束或遥测显示电机运行，也会尽力发布一次 stop，但网络
已经断开时不能把这个行为当作安全急停。真正的安全停机必须由 STM32故障保护、
硬件急停或独立安全链保证。

#### Qt 的三个 PID 按钮区别

- **临时应用 PID**：MQTT 下发至 ESP32/STM32运行时，不改固件默认值。
- **保存 PID 参数**：写入 PC 的 `QSettings`，下次启动 Qt 自动载入。
- **恢复工程默认 PID**：删除 `QSettings` 的 pid 组，恢复当前工程默认分子；连接且
  停机时也临时下发。

默认值及固定分频：

| 控制器 | 分子 Kp/Ki/Kd | 固定分频 |
| --- | --- | --- |
| 速度 | 2144/5/0 | 2048/16384/Kd禁用 |
| 位置 | 48/4/8 | 1024/32768/16 |
| Iq | 3633/2693/0 | 128/512/Kd禁用 |
| Id | 3633/2693/0 | 128/512/Kd禁用 |

因此 UI 数字是 MCSDK 定点增益分子，不是未经缩放的浮点 PID。

当前 Qt 对“发布四条 PID 命令”只检查 socket 写入返回值，不逐条等待业务 ACK 后再
发送下一条。这在局域网实验中简单，但更严谨的实现应按 command id 跟踪 ACK、
超时和失败重试。

### 64. Qt 如何接收并解析电机遥测

当前程序存在两类思想：

1. 常规状态消息：解析结构化数据，更新链路/运行状态。
2. 高速测试数据：按照固定二进制记录格式解析。

二进制测试数据函数：

```text
processTestData(payload)
↓
检查 magic "MCTD"
↓
检查 version / record size
↓
读取 id/startIndex/count/total
↓
逐记录 readU32Le/readS16Le/readU16Le
↓
填 MotorSample
```

每个 `MotorSample` 包括：

- 时间。
- 实测/参考速度。
- 实测/目标位置。
- Iq/Id。
- Iq_ref/Id_ref。

#### 测试数据如何乱序重组和去重

第一块到达时：

```text
expectedSampleCount_ = total
samples_.resize(total)
receivedSamples_.fill(false, total)
```

每个 record 的目标位置由 `startIndex + index` 决定，而不是简单 append：

```text
samples_[destination] = sample
if (!receivedSamples_[destination]) {
    receivedSamples_[destination] = true
    receivedSampleCount_++
}
```

所以 QoS1 重发造成的重复块不会重复计数，分块到达顺序变化也能放回正确位置。
同时检查：

- magic/version/record size。
- `startIndex + count <= total`。
- payload 长度足够。
- 同一 test ID 的 total/mode 一致。

#### 状态消息与二进制消息为何分开解析

`processMqttMessage()` 先判断 test/data；它可能包含任意零字节，不能交给 JSON
解析器。其他正式业务 Topic 使用 `QJsonDocument::fromJson()`，解析失败会更新 UI
错误。先按 Topic 路由再按格式解析，是协议适配层的基本设计。

### 65. Qt 如何记录速度、位置、$I_q/I_d$ 数据

当前 Qt 不是简单“来一个点画一个点”，而是支持测试数据集整理和图像保存。

接收：

```text
processTestData()
↓
MotorSample
↓
samples_[index]
↓
receivedSamples_ 标记是否收到
```

完成：

```text
finalizeReceivedTest()
↓
检查 receivedSampleCount == expectedSampleCount
↓
saveTestImage()
↓
MotorPlotRenderer::savePng()
├─ speed/position tracking
├─ 启动瞬态
├─ error
└─ Iq / Iq_ref
↓
MotorPlotRenderer::saveCsv()
```

只有 `receivedSampleCount_ == expectedSampleCount_` 才保存完整报告；否则提示数据
不完整。默认输出目录是用户 Documents 下的 `MotorControl_PID_Tests`。

速度和位置报告都包含 Iq/Iq_ref；速度测试额外分析速度，位置测试额外分析位置。
CSV 每个原始采样点一行，不降采样、不滤波、不删除尖峰，并保存：

- 时间。
- 实测/参考速度及误差。
- 实测/目标位置及最短角误差。
- Iq/Iq_ref、Id/Id_ref。

速度指标包括采样周期、10%~90%上升时间、超调、调节时间、MAE、RMSE、最大误差；
位置指标使用最短角度误差处理 0°/360° 环绕。

因此 Qt 的价值包括：

```text
+ PID参数管理
+ 数据完整性重组
+ 曲线生成
+ PNG/CSV归档
+ 性能指标计算
```

为什么删除实时曲线界面而保存文件：现场测试更关心可重复实验和结果归档；文件可
用于版本对比、论文/报告和离线分析，也不会让 GUI 重绘干扰网络接收。

---

## 九、整机数据链、故障、实时性与架构

### 66. 从 UI 点击按钮到电机动作的完整数据链路

以 LVGL CAN 速度控制为例：

```text
用户拖动速度 Slider
↓
motor_ui_speed_slider_event()
↓
CommMgr_ESP_SetMode(SPEED)
↓
CommMgr_ESP_SetSpeedRPM(target)
↓
CAN_ESP_SetSpeedRPM()
↓
s_pending_speed_rpm / s_speed_dirty
↓
CAN_ESP_tx_task()
↓
CAN_ESP_transmit()
↓
TWAI / CAN 收发器
↓
STM32 FDCAN1
↓
CAN_STM_ProcessRx()
↓
CAN_STM_ExecuteCommand()
↓
CommMgr_STM_SetSpeed()
↓
MotorMgr_STM_SetSpeed()
↓
MC_ProgramSpeedRampMotor1_F()
↓
速度 PI
↓
Iq_ref
↓
Iq PI
↓
SVPWM
↓
TIM1
↓
MOSFET 三相桥
↓
电机转动
```

### 67. 从电机采样到 Qt 曲线显示的完整数据链路

底层实时数据首先产生于 STM32：

```text
三相电流
↓
ADC injected
↓
FOC_CurrControllerM1()
↓
FOCVars.Iqd / Iqdref
```

机械量：

```text
MT6701 AB
↓
TIM3 encoder
↓
ENCODER_M1
↓
速度 / 位置
```

统一状态：

```text
MotorMgr_STM_GetState()
↓
CommMgr_STM_GetMotorState()
↓
CAN_STM 每约2 ms发送 0x180/0x181/0x182
↓
ESP32 CAN ISR → Queue → RX task
↓
CommMgr_ESP_GetState()
↓
sample_sequence变化
↓
mqtt_gateway_record_sample()
↓
16-byte紧凑样本写入PSRAM
↓ 7秒结束且电机停止
MCTD二进制40点/块
↓ QoS1，每块等待Broker PUBACK
Mosquitto Broker
↓
Qt MqttClient
↓
processTestData()
├─ test ID校验
├─ index重组
└─ receivedSamples去重
↓
完整性检查
↓
MotorPlotRenderer
↓
PNG分析报告 + CSV原始数据
```

关键点是采样和上传解耦：PSRAM 中的时间戳来自 CAN反馈，不来自 MQTT 到达时间；
因此弱网会延长上传时间，但不会把 2 ms 采样变成不均匀的网络采样。

### 68. 欠压、过流、堵转、编码器异常等常见电机故障

当前参数：

```text
欠压阈值 8 V
过压阈值 24 V
过温 90 ℃
```

安全任务：

```text
MC_RunMotorControlTasks()
↓
TSK_SafetyTask()
↓
TSK_SafetyTask_PWMOFF()
├─ NTC_CalcAvTemp()
├─ PWMC_IsFaultOccurred()
├─ RVBS_CalcAvVbus()
└─ MCI_FaultProcessing()
```

若存在故障：

```text
PWMC_SwitchOffPWM()
↓
FOC_Clear()
↓
状态机 FAULT_NOW / FAULT_OVER
```

编码器异常通常表现为速度反馈错误、角度突变、方向错误或启动 alignment 失败。

堵转往往表现为：

```text
speed_ref 很高
speed_measured ≈ 0
Iq_ref / Iq 上升
持续大电流/保护触发
```

### 69. 为什么上电后 $I_q/I_d$ 仍可能存在波动

即使目标为 0，也不代表 ADC 数字量绝对为 0。

可能来源：

1. 电流采样 offset 不准。
2. 运放零漂。
3. ADC 噪声。
4. PWM 开关噪声。
5. 采样点不在合适窗口。
6. 编码器电角度抖动，使 Park 变换后的 dq 产生波动。
7. PI 增益过大，对微小噪声过度响应。

定位顺序：

```text
先关闭/停止电机
↓
检查原始 ADC/相电流 offset
↓
检查校准是否完成
↓
检查 Ia/Ib/Ic
↓
再看 Id/Iq
↓
检查编码器角度
↓
最后再调整电流 PI
```

不要一看到 Iq 波动就先调速度 PID。

### 70. 为什么 $I_q$ 跟不上 $I_{q,ref}$

$I_{q,ref}$ 是“想要的转矩电流”，$I_q$ 是实际电流。

跟不上常见原因：

- 母线电压不足。
- 电机高速时反电动势增大，可用电压余量不足。
- 电流 PI 不合适。
- 电流测量比例/offset 错误。
- 电压矢量被 `Circle_Limitation()` 限制。
- 电源本身限流。
- 逆变器压降。

代码链：

```text
Iq_ref - Iq
↓
PI_Controller()
↓
Vq
↓
Circle_Limitation()
```

如果 PI 想给更大的 $V_q$，但已经触碰母线能够合成的最大电压，继续增大 $I_{q,ref}$ 也无法让实际 $I_q$ 追上。

### 71. 母线电压限制对最大转矩和最高转速的影响

电机速度越高，反电动势越高：

$$E\propto \omega$$

电流变化需要电压余量：

$$V \approx RI+L\frac{dI}{dt}+E$$

母线电压低时，高速区：

```text
反电动势占掉大量电压
↓
可用于推动电流的电压变少
↓
Iq 跟踪能力下降
↓
负载一加就掉速
```

所以你之前看到的“$I_{q,ref}$ 上升但 $I_q$ 跟不上、速度明显掉”非常符合电压饱和/电源能力不足需要排查的现象。

### 72. 控制周期、PWM 频率、电流环频率、速度环频率关系

当前工程：

```text
PWM_FREQUENCY = 16 kHz
REGULATION_EXECUTION_RATE = 1
```

因此电流 FOC 调节频率约：

$$f_i=16\text{ kHz}$$

SysTick：

```text
SYS_TICK_FREQUENCY = 2000 Hz
```

位置/中频任务：

```text
POSITION_LOOP_FREQUENCY_HZ = 1000 Hz
MEDIUM_FREQUENCY_TASK_RATE = 1000 Hz
```

基本原则：

```text
电流环最快
速度环/位置控制更慢
UI/通信更慢
```

原因：内部闭环必须比外部闭环快，外环才能把内环近似看成“快速执行器”。

### 73. 实时控制代码为什么不能被通信和 UI 阻塞

如果 FOC 16 kHz，每周期约：

$$T=62.5\ \mu s$$

意味着电流采样、变换、PI、SVPWM 必须在下一个控制周期前完成。

因此：

- FOC 在 ADC 中断高频执行。
- CAN/UART 主要在主循环或 ESP32 RTOS task。
- UI 在 ESP32，不在 STM32 高频中断。
- ISR 中避免打印、复杂解析和长时间等待。

ESP32 CAN 代码中甚至专门避免 TX task 因持续 ACK 超时而占满 CPU，使用 `vTaskDelay()` 强制让出时间片，这就是实时/并发设计意识。

### 74. HAL 层、MCSDK 层、User 应用层区别

HAL：

```text
HAL_FDCAN_...
HAL_UART_...
HAL_ADC_...
```

解决“怎么操作 STM32 外设”。

MCSDK：

```text
FOC
PID
Encoder
State machine
Motor Control API
```

解决“怎么控制电机”。

User：

```text
MotorMgr_STM
CommMgr_STM
CAN_STM
USART_STM
```

解决“你的产品逻辑怎么组织”。

正确依赖：

```text
User 应用
↓
MCSDK / HAL
↓
MCU 硬件
```

### 75. 为什么模块化重构，而不是所有代码写 `main.c`

巨大 `main.c` 的问题：

- 功能耦合。
- 修改 CAN 可能影响电机代码。
- 无法快速定位责任。
- 多人开发冲突严重。
- 面试时无法讲架构。

当前：

```text
main.c
只初始化 + Tick

MotorMgr
只管电机应用逻辑

CommMgr
只管统一通信入口

CAN/USART
只管各自协议
```

因此出现问题可以按层定位：

```text
电机本体不动？ → MCSDK/MotorMgr
只有 CAN 不行？ → CAN_STM/CAN_ESP
只有 UART 不行？ → USART
UI 按钮不生效？ → UI/CommMgr_ESP
```

### 76. `CAN_STM / USART_STM → CommMgr_STM → MotorMgr_STM → MCSDK` 必须讲清楚

这条链的核心不是“多写几层函数”，而是 **依赖方向单一**。

```text
协议适配层
CAN_STM / USART_STM
只负责收发、解析、CRC/CAN ID、链路状态
↓
CommMgr_STM
统一电机命令入口
↓
MotorMgr_STM
统一模式、限幅、轨迹、PID、启停规则
↓
MCSDK
真正执行电机控制
```

面试回答：

> 我没有让 CAN 和 UART 直接操作 MCSDK，而是通过 CommMgr 和 MotorMgr 做两层隔离，这样不同通信接口共享完全相同的电机业务规则，避免出现 CAN 限速一套、UART 限速另一套的问题。

### 77. `LVGL / 网络 → CommMgr_ESP → CAN_ESP / USART_ESP` 必须讲清楚

ESP32 同样遵守：

```text
UI / 网络业务层
↓
CommMgr_ESP
↓
当前选中的唯一 transport
├─ CAN_ESP
└─ USART_ESP
```

优点：

- LVGL 不关心底层 CAN 还是 UART。
- 能实现通信控制权互斥。
- 后续增加 USB/RS485 时只需要增加 transport adapter。
- 网络层也不应该绕过 CommMgr 直接控制 CAN。

### 78. STM32、ESP32、Qt 三个平台分别承担什么职责

STM32：**实时控制器**

- 电流采样。
- 编码器。
- FOC。
- 电流/速度/位置控制。
- PWM。
- 故障保护。
- CAN/UART 电机协议。

ESP32：**HMI + 网关**

- LVGL。
- 触摸。
- CAN/UART 适配。
- 通信仲裁。
- Wi-Fi。
- 网络网关。

Qt：**PC 上位机 / 测试工具**

- 操作界面。
- 参数配置。
- 自动测试组织。
- 数据接收。
- 曲线和图像保存。

一句话：

```text
STM32 管“实时控制”
ESP32 管“人机与连接”
Qt 管“实验与分析”
```

### 79. 项目出现过的实际问题，以及如何定位 CAN、电流环、速度环、位置环

#### CAN 问题

典型现象：CAN 之前能连，修改后不能连；或者出现 Bus-Off。

定位：

```text
1. 检查物理层：VCC/VIO、TXD/RXD、CAN_H/L、终端电阻
2. 检查两端 500 kbit/s
3. 看 ESP32 TWAI error flags / ACK error
4. 看 STM32 CAN_STM_ServiceBus() 是否 Bus-Off
5. 看 0x100 是否收到
6. 看 0x180/181/182 是否返回
7. 最后才查上层 CommMgr/MotorMgr
```

当前代码已经加入：

- ESP32 收发器 GPIO 自检。
- ACK/BIT/FORM/STUFF 等错误日志。
- Bus-Off recovery。
- STM32 Bus-Off stop/start recovery。
- link timeout。

#### 电流环问题

现象：未动作时 Id/Iq 有数百 mA 波动，或 Iq 跟不上 Iq_ref。

定位顺序：

```text
ADC 原始采样
↓
offset/calibration
↓
相电流 Ia/Ib/Ic
↓
编码器电角度
↓
Id/Iq
↓
Iq_ref/Id_ref
↓
Vq/Vd 是否饱和
↓
母线电压
↓
最后调 PI
```

#### 速度环问题

现象：给定速度达不到、加负载后掉速。

先看：

```text
speed_ref
speed
Iq_ref
Iq
```

如果：

```text
speed 下降
Iq_ref 明显上升
但 Iq 跟不上
```

说明速度 PI 已经“知道要加转矩”，问题更可能在电流/电压/电源能力，而不是速度 PI 没工作。

#### 位置环问题

先确认：

```text
编码器位置是否正确
↓
模式是否 POSITION
↓
MCSDK 是否 RUN
↓
targetPosition 是否更新
↓
PositionControlRegulation 是否打开
↓
位置 PID 输出是否产生 TorqueRef/Iq_ref
```

当前 `MotorMgr_STM_HoldCurrentPosition()` 和模式切换代码专门清理 PID 积分和旧轨迹，就是为了解决“切换模式/刚进入 RUN 后位置突然跳”的风险。

### 80. 用 1 分钟完整介绍项目技术架构和自己的工作内容

可以按下面逻辑讲，不需要逐字背：

> 这个项目是一套基于 STM32G431、ESP32-S3 和 Qt 的 PMSM 有感 FOC 控制与 PID 实验平台。STM32 使用 ST MCSDK，TIM1 产生 16 kHz 三相互补 PWM，ADC1/ADC2 在 PWM 同步时刻进行注入电流采样，MT6701 AB 编码器由 TIM3 读取；高频任务完成 Clarke、Park、Id/Iq PI、反 Park 和 SVPWM。应用层通过 MotorMgr_STM 统一模式、限幅、轨迹和 PID，再由 CommMgr_STM 对接 CAN/USART。ESP32 使用 ESP-IDF、FreeRTOS 和 LVGL 做现场 HMI，并通过 CommMgr_ESP 仲裁 CAN/UART；网络侧以 STA 模式连接 Wi-Fi，通过 Mosquitto Broker 与 Qt 使用 MQTT 通信。为了不让网络抖动破坏采样，7 秒速度/位置测试由 ESP32 本地启动，约 2 ms 的 CAN反馈先以 16 字节紧凑结构写入 PSRAM，停机后再按 MCTD 二进制分块以 QoS1 上传，每块和 complete 都等待 Broker PUBACK；失败时保留 PSRAM，可在页面 RESEND。Qt 负责临时 PID 参数、按 ID和索引重组数据，并保存 PNG分析报告和 CSV。整个设计把硬实时控制、通信网关和实验分析分层，同时对 Bus-Off、Wi-Fi/MQTT断线、outbox压力和数据完整性做了诊断与恢复。

---

# 十、面试前真正需要背熟的函数链

如果时间非常紧，优先把下面 11 条背熟。

## A. STM32 启动

```text
main()
→ HAL_Init()
→ SystemClock_Config()
→ MX_xxx_Init()
→ MX_MotorControl_Init()
→ MCboot()
→ FOC_Init()
→ CommMgr_STM_Init()
→ while(1) CommMgr_STM_Tick()
```

## B. FOC 高频电流环

```text
ADC1_2_IRQHandler()
→ TSK_HighFrequencyTask()
→ FOC_HighFrequencyTask()
→ ENC_CalcAngle()
→ FOC_CurrControllerM1()
→ PWMC_GetPhaseCurrents()
→ MCM_Clarke()
→ MCM_Park()
→ PI_Controller(Iq/Id)
→ Circle_Limitation()
→ MCM_Rev_Park()
→ PWMC_SetPhaseVoltage()
```

## C. MCSDK 启动状态机

```text
MotorMgr_STM_Start()
→ MC_StartMotor1()
→ IDLE
→ OFFSET_CALIB
→ CHARGE_BOOT_CAP
→ ALIGNMENT
→ WAIT_STOP_MOTOR
→ RUN
```

## D. CAN 控制 STM32

```text
CAN RX 0x100
→ CAN_STM_ProcessRx()
→ CAN_STM_ExecuteCommand()
→ CommMgr_STM_xxx()
→ MotorMgr_STM_xxx()
→ MCSDK API
```

## E. STM32 CAN 反馈

```text
MCSDK
→ MotorMgr_STM_GetState()
→ CommMgr_STM_GetMotorState()
→ CAN_STM_SendStatus/References/Electrical()
→ 0x180/0x181/0x182
→ ESP32
```

## F. ESP32 UI 控制

```text
LVGL event
→ motor_ui_xxx_event()
→ CommMgr_ESP_xxx()
→ CAN_ESP 或 USART_ESP
→ TX task
→ STM32
```

## G. ESP32 CAN 接收

```text
TWAI hardware
→ CAN_ESP_rx_callback() [ISR]
→ FreeRTOS Queue
→ CAN_ESP_rx_task()
→ parse_status/references/electrical()
→ snapshot
→ CommMgr_ESP_GetState()
→ LVGL
```

## H. Qt 数据处理

```text
MqttClient::readyRead
→ processInput()处理TCP半包/粘包
→ processPacket()/processPublish()
→ MainWindow::processMqttMessage()
→ processTestData(MCTD)
→ MotorSample[]
→ finalizeReceivedTest()
→ saveTestImage()
→ MotorPlotRenderer::savePng()/saveCsv()
```

## I. ESP32 Wi-Fi连接与重连

```text
wifi_manager_init()
→ NVS / esp_netif / event loop / Wi-Fi STA
→ scan_async()
→ WIFI_EVENT_SCAN_DONE
→ connect()
→ WIFI_EVENT_STA_CONNECTED
→ IP_EVENT_STA_GOT_IP

意外断线
→ WIFI_EVENT_STA_DISCONNECTED
→ esp_timer
→ 1/2/4/8/10s指数退避重连
```

## J. ESP32 MQTT连接

```text
LVGL CONNECT
→ mqtt_manager_connect_async()
→ command queue
→ mqtt_manager_worker()
→ esp_mqtt_client_init/start
→ MQTT_EVENT_CONNECTED
→ subscribe command/test-rx
```

## K. 本地 PID测试与可靠上传

```text
ESP32 PID TEST按钮
→ mqtt_motor_gateway_start_local_test()
→ STARTING
→ 自动CAN + 应用PID + START + 下目标
→ RECORDING：CAN新sequence写PSRAM
→ 7秒 STOPPING
→ SENDING：40点/块 + QoS1 + PUBACK
→ complete PUBACK
→ UPLOAD SUCCESS + 释放PSRAM

失败
→ SEND_FAILED + 保留PSRAM
→ RESEND从index 0全量重发
```

---

# 十一、当前工程最值得面试时强调的 10 个设计点

1. **实时控制与 UI/网络彻底分离**：FOC 全部留在 STM32。
2. **MotorMgr 是 MCSDK 唯一应用入口**：不同通信协议共享统一业务规则。
3. **CommMgr 做通信抽象和仲裁**：ESP32 可在 CAN/USART 间切换，但只有一个有控制权。
4. **ISR 最小化 + RTOS Queue**：ESP32 CAN ISR 只复制帧，解析放任务。
5. **通信有工程化诊断**：CRC、sequence、link timeout、Bus-Off recovery、错误计数。
6. **控制模式切换处理运行期历史**：清积分、清轨迹、保持当前位置，避免切换冲击。
7. **采样与网络上传解耦**：CAN高速反馈先存PSRAM，Wi-Fi只影响上传耗时，不改变采样时间轴。
8. **可靠上传不把enqueue当成功**：一次一块，按message ID等待PUBACK，complete同样确认。
9. **失败数据可恢复**：上传错误保留PSRAM并提供RESEND，使用test ID和index实现幂等重组。
10. **诚实区分可靠性层级**：当前UPLOAD SUCCESS是Broker确认，生产化还应增加Qt落盘ACK和TLS安全。

---

# 十二、Wi-Fi / MQTT 高频面试问答

下面每题先背“第一句话”，再用工程细节展开。

## 1. Wi-Fi、TCP、MQTT是什么关系？

第一句话：**Wi-Fi负责无线链路，IP负责寻址，TCP提供可靠字节流，MQTT在TCP上
提供发布/订阅消息语义。**

本工程 ESP32 先以 STA 连接 AP并通过 DHCP取得 IP，再建立到
`192.168.10.7:1883` 的 TCP连接，最后完成 MQTT CONNECT/CONNACK。

## 2. 连上 Wi-Fi 为什么还可能连不上 MQTT？

第一句话：**连上 AP只说明 802.11关联成功，MQTT还依赖 IP、路由、TCP端口和
Broker服务。**

应依次检查 GOT_IP、同网段/网关、Broker IP、1883监听、防火墙、TCP连接和
CONNACK拒绝码。

## 3. AP和STA是什么？本工程是哪种？

AP提供无线接入，STA加入AP。本工程 ESP32是 STA，不创建自己的热点；路由器或
手机热点是 AP。

## 4. SSID与BSSID有什么区别？

SSID是网络名称，多个 AP可同名；BSSID标识具体 AP无线接口。当前 UI按 SSID
去重，易用但无法精确选择同名 AP，生产环境可显示 BSSID和信道。

## 5. RSSI越大越好吗？

RSSI以负 dBm表示，越接近 0通常信号越强。但强信号不代表一定低延迟，信道拥塞、
干扰、AP负载和重传率同样重要。

## 6. DHCP做什么？

DHCP自动分配 IP、掩码、网关和 DNS。本工程只在 `IP_EVENT_STA_GOT_IP` 后认为
Wi-Fi网络真正可用，而不是在刚收到 STA_CONNECTED时。

## 7. 为什么扫描和连接都做成异步？

无线扫描、认证、DHCP可能耗时数秒，不能阻塞 LVGL事件线程。API返回只表示请求
提交，结果由 WIFI_EVENT/IP_EVENT回调更新状态快照。

## 8. 为什么事件回调不直接更新 LVGL？

事件回调与 LVGL不在同一任务上下文，直接改对象有线程安全风险。工程只在回调中
更新 mutex保护的 snapshot，LVGL定时器按 revision刷新。

## 9. Wi-Fi重连为什么是指数退避？

短暂掉线要快速恢复，长期离线又不能高频重试。工程使用约
1/2/4/8/10秒退避，最大10秒；主动断开则关闭重连。

## 10. 为什么关闭 Wi-Fi省电？

`WIFI_PS_NONE` 减少休眠唤醒造成的网络时延抖动，适合外部供电的电机 HMI；代价
是功耗提高，这是稳定性与功耗的权衡。

## 11. MQTT为什么需要 Broker？

Broker维护连接、订阅表并转发消息，使发布者和订阅者在地址、数量和上线时间上
解耦。本工程 Qt与ESP32都连 Mosquitto，不直接互连。

## 12. Topic是什么？

Topic是分层消息路由名，不是IP或队列对象。例如 Qt向
`motor/control/command` 发布，ESP32订阅后处理；ESP32向
`motor/control/test/data` 发布，Qt订阅后重组。

## 13. QoS 0、1、2怎么选？

QoS0最多一次、最低开销；QoS1至少一次、有PUBACK但可能重复；QoS2恰好一次、
握手最重。工程让可刷新状态走QoS0，让命令和测试数据走QoS1，不使用QoS2。

## 14. 为什么遥测用QoS0，测试数据用QoS1？

250 ms遥测是“新值覆盖旧值”，丢一条很快有下一条，QoS0避免积压；测试曲线每个
样本都影响报告完整性，需要QoS1和索引去重。

## 15. QoS1为什么还要去重？

发送方未收到PUBACK会用相同 packet ID重传，Broker或订阅者可能再次看到消息。
所以Qt按test ID、startIndex和receivedSamples位图实现幂等接收。

## 16. PUBACK代表最终业务成功吗？

不代表。它只确认相邻MQTT端点已收到QoS1 PUBLISH。ESP32收到的是Broker
PUBACK，不证明Qt已保存文件；当前可进一步增加Qt落盘ACK。

## 17. Keep Alive有什么作用？

空闲时用PINGREQ/PINGRESP发现半开或失效连接。它是连接健康检测，不是自动重连；
断线后仍要重新建立TCP和MQTT会话。

## 18. Client ID冲突会怎样？

Broker通常会断开同ID旧连接，两个客户端可能交替上线掉线。ESP32用STA MAC派生
唯一ID，Qt用进程PID区分多实例。

## 19. Retained Message适合测试数据吗？

不适合。Retain会让新订阅者立即收到旧的最后一块，可能误组旧测试。设备在线状态
适合retain，测试分块不retain。

## 20. Clean Session和持久会话有什么区别？

Clean Session断线后清理订阅/离线队列；持久会话可保留。Qt当前显式Clean
Session，所以Qt离线期间不能依赖Broker保存实验数据。

## 21. LWT是什么？当前有吗？

LWT是客户端异常掉线时Broker代发的遗嘱消息，可表达设备离线。当前没有配置，
可作为在线状态生产化改进。

## 22. 为什么Qt要处理TCP半包和粘包？

TCP没有消息边界。一次readyRead可能得到半包或多包；Qt根据MQTT Remaining
Length在inputBuffer中等待完整报文并循环解析。

## 23. 为什么状态用JSON，测试数据用二进制？

JSON可读易调试，适合低频控制/状态；二进制体积小、解析确定，适合约3500点曲线。
工程使用统一小端20字节记录，避免直接发送C结构体的padding/ABI问题。

## 24. Outbox是什么？

Outbox保存待发送或待确认消息。弱网时生产速度高于发送速度就会积压，达到8 KiB
上限后enqueue失败，甚至让keepalive写不出去。

## 25. 间歇上传失败的根因和修复是什么？

第一句话：**网络写超时是触发条件，旧代码把enqueue误当成已送达并持续挤压
outbox，才导致假成功和数据丢失。**

修复是QoS0不存outbox、一次一个测试块、按message ID等PUBACK、complete也确认，
失败保留PSRAM并允许RESEND。

## 26. 为什么不实时把每个CAN点发MQTT？

实时逐点上传会把Wi-Fi时延耦合进采样任务，还会产生大量头部/JSON和outbox压力。
工程先按CAN sample_sequence写PSRAM，停机后上传，保证采样时间轴由CAN决定。

## 27. 为什么测试数据放PSRAM而不是内部RAM？

3600点紧凑样本需要57600字节。内部RAM还要供任务栈、LVGL、Wi-Fi和DMA使用；
强制PSRAM分配失败即拒绝测试，避免不可预测地挤占关键内部内存。

## 28. 为什么PSRAM结构16字节、线上记录20字节？

PSRAM只存当前模式需要的主实测/参考量，追求容量；线上协议固定保留速度和位置
两组字段，追求跨模式统一与向后兼容。

## 29. 为什么重新发送从第0点开始？

ESP32知道Broker确认进度，却不知道Qt接收进度。全量重发加索引幂等比维护两端
缺块协商更简单，数据量约70 KiB，可接受。

## 30. RESEND时为什么禁止新测试？

当前只有一份PSRAM测试缓冲。允许新测试会覆盖尚未成功上传的数据，所以失败数据
未处理前锁定速度/位置按钮，成功或重启后才释放。

## 31. 为什么测试必须在ESP32页面启动？

把高能电机实验的启动权放在现场，并让启动、CAN状态机和采样处于同一设备，避免
Qt网络时延决定实验起点。Qt只做参数和结果处理。

## 32. MQTT命令到了ESP32，为什么还要业务ACK？

Broker交付成功只说明消息到设备，不说明CAN在线、参数合法或STM32接受。ACK用
command id返回ok/message，把传输成功与业务成功分开。

## 33. Qt当前MQTT实现有哪些不足？

手写客户端能CONNECT、SUBSCRIBE、PUBLISH、解析入站PUBLISH和返回PUBACK，但
未跟踪出站PUBACK、未解析SUBACK、无自动重连/TLS/LWT/持久会话。这是实验版边界。

## 34. 当前MQTT安全吗？

当前默认1883明文，无Broker认证、TLS和Topic ACL，只适合可信局域网。生产化要用
TLS、CA校验、账号/设备证书、最小Topic权限、凭据保护和命令防重放。

## 35. 1883和8883的区别？

1883通常是明文MQTT，8883通常是MQTT over TLS约定端口，但端口本身不产生安全；
必须真的配置TLS证书、验证和Broker监听。

## 36. ESP32网关为什么不直接让网络层调用CAN？

网络层走CommMgr_ESP，复用与LVGL相同的模式、控制权和链路规则；否则网络和UI可能
同时写不同目标，破坏统一仲裁。

## 37. 如何保证测试结束才上传？

状态机只有在STOPPING观察到`motor_running=false`后进入SENDING；测试期间暂停普通
遥测，电机控制和数据上传时间域明确分开。

## 38. 如果上传时CAN断了怎么办？

电机已经停止且数据已在PSRAM，上传不再依赖CAN；当前SENDING阶段不因CAN离线
丢弃数据。这体现“执行阶段故障”和“上传阶段故障”分开处理。

## 39. 发送成功、Broker成功、Qt成功如何区分？

```text
enqueue成功：进入本地发送系统
PUBACK成功：Broker收到QoS1消息
Qt完整重组：所有index收到
文件保存成功：PNG/CSV生成
```

当前ESP32确认到第二层，Qt自己检查第三、四层；还没有第四层回传ESP32的闭环。

## 40. 如果让你继续改进，优先做什么？

优先顺序：Qt落盘ACK与ESP32延迟释放PSRAM；Qt出站PUBACK/SUBACK和自动重连；
MQTTS与ACL；设备在线LWT；缺块位图/选择性重传；运行统计与故障注入测试。

---

# 十三、Wi-Fi / MQTT 分层排障清单

## 1. 最重要原则：从底到顶，不要跳层

```text
供电/天线
↓
802.11扫描、认证、关联
↓
DHCP / IP / ARP / 路由
↓
TCP 192.168.10.7:1883
↓
MQTT CONNECT / CONNACK / SUBSCRIBE
↓
Topic与JSON/二进制协议
↓
ESP32 Gateway状态机
↓
CAN / STM32 / 电机
```

## 2. 现象—检查点速查

| 现象 | 优先检查 | 不要先做什么 |
| --- | --- | --- |
| ESP32扫不到热点 | 2.4GHz、距离、天线、driver日志 | 不要先改MQTT |
| Wi-Fi反复断 | reason、RSSI、信道、供电、AP负载 | 不要只看Broker日志 |
| 已连接但无IP | DHCP、地址池、IP_EVENT | 不要调QoS |
| MQTT refused | Broker监听、认证、CONNACK码、Client ID | 不要调CAN |
| MQTT写超时 | Wi-Fi质量、outbox、Broker负载、发送速率 | 不要认为JSON一定错 |
| Qt连上但无遥测 | 订阅Topic/SUBACK、防火墙、ESP32 connected | 不要先调电机PID |
| 点击测试无动作 | MQTT在线、PSRAM、CAN link、fault、RUN回显 | 不要先看PNG |
| 上传中断 | ESP日志progress/pending_mid、Broker PUBACK | 不要启动新测试覆盖数据 |
| Qt提示数据不完整 | test ID、total、index、complete时机、Qt是否中途掉线 | 不要补零伪造曲线 |
| 曲线异常但点数完整 | CAN采样、单位/端序、控制环和传感器 | 不要归因于MQTT时延 |

## 3. Windows/Broker侧常用检查

```powershell
ping 192.168.10.7
Test-NetConnection 192.168.10.7 -Port 1883
```

如果安装 Mosquitto客户端，可用：

```powershell
mosquitto_sub -h 192.168.10.7 -p 1883 -t "motor/#" -v
mosquitto_pub -h 192.168.10.7 -p 1883 `
  -t "motor/hmi/test/rx" -m "hello"
```

观察Broker日志时重点看：

- ESP32和Qt是否使用不同Client ID连接。
- PUBLISH的Topic、QoS和message ID。
- PUBACK是否返回。
- 谁主动关闭连接。
- 是否反复出现同一Client ID顶替。

## 4. ESP32终端日志如何形成证据链

```text
WIFI_MANAGER: Connected ... IP ...
↓
MQTT_MANAGER: Connected to mqtt://...
↓
MQTT_MOTOR: test accepted / recording
↓
sample_count增长
↓
chunk PUBACK progress=N/total
↓
upload confirmed by broker
```

失败时记录原始日志，不只截最后一句。至少同时保留：时间、Wi-Fi reason、MQTT
error、测试ID、progress、pending_mid、Broker对应时间段日志。

## 5. 为什么曲线异常通常不是MQTT造成的

当前曲线点先在PSRAM固定下来，MQTT只搬运。只要Qt完整重组，上传时延不会改变
record内的`time_us`、速度、电流和位置值。MQTT更可能造成“收不到/不完整/重复”，
而不是让已完整收到的Iq曲线物理形状变差。曲线数值异常应回到CAN、单位、采样和
控制环定位。

---

# 十四、当前网络方案的安全性、边界与生产化路线

## 1. 当前已具备

- Wi-Fi STA扫描、连接、DHCP状态和指数退避重连。
- MQTT异步连接、Keep Alive、自动重连事件和错误诊断。
- 控制/遥测/ACK/测试状态/二进制数据分Topic。
- 控制参数校验、电机运行中拒绝改PID、现场启动测试。
- PSRAM高速采样、二进制分块、QoS1 PUBACK、失败重发。
- Qt索引去重、完整性检查、PNG/CSV归档。

## 2. 当前不能夸大的地方

- 1883明文不是生产级安全通信。
- Broker PUBACK不是Qt落盘ACK。
- Qt手写MQTT Client不是完整协议栈。
- Qt的QSettings保存不是STM32参数固化。
- RESEND数据在重启后消失，因为PSRAM是易失存储。
- MQTT/CAN不是功能安全急停链路。
- 当前只有一份测试缓冲，不支持多实验排队。

## 3. 生产化改进优先级

### P0：安全与端到端正确性

1. 使用MQTTS，校验CA/域名，增加SNTP。
2. Broker账号或设备证书认证，按Client限制Topic ACL。
3. Qt落盘成功后发送带test ID和hash的应用ACK，ESP32再释放PSRAM。
4. 物理急停/独立安全链，不依赖Wi-Fi。

### P1：可靠性

1. Qt跟踪PUBACK/SUBACK、自动重连和超时重发。
2. 增加数据集CRC32/SHA-256，校验整包内容而不只检查点数。
3. Qt回传缺失index范围，支持选择性重传。
4. 需要断电恢复时，把测试数据临时写入文件系统/外部Flash，并做磨损管理。

### P2：可观测性和扩展性

1. LWT + retained在线状态。
2. 发布固件版本、设备ID、RSSI、重连次数和上传耗时。
3. Topic增加device ID，例如 `motor/<deviceId>/...`，支持多设备。
4. 为协议增加schema version和向后兼容策略。
5. 建立弱网、断Broker、重复包、乱序包、PSRAM不足、CAN掉线的故障注入测试。

## 4. 面试时如何评价自己的实现

推荐表达：

> 我先保证控制实时性，把高速样本本地缓存，再解决网络传输可靠性。实现中我没有
> 把enqueue当成送达，而是按QoS1 message ID等待Broker PUBACK；上传失败保留
> PSRAM并全量幂等重发。与此同时我明确知道PUBACK只到Broker，当前实验版还缺
> Qt落盘后的应用ACK和TLS，这两项是我生产化时的第一优先级。

这种回答同时展示实现能力、协议边界意识和后续设计判断，比简单说“用了MQTT，
所以数据不会丢”更专业。
