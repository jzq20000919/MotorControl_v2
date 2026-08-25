# QtMqttSimpleControl

Qt 6 Widgets MQTT 电机 PID 测试程序。程序通过 ESP32 MQTT 网关临时调整
STM32 MCSDK 的速度、位置、Iq 和 Id 调节器，并执行固定 7 秒跟踪测试。

## PID 参数

界面中的数值是当前 MCSDK 工程使用的增益分子，分频保持固件生成值不变：

| 控制器 | 工程默认值 Kp/Ki/Kd | 固定分频 |
| --- | --- | --- |
| 速度 | 2144 / 5 / 0 | 2048 / 16384 / 禁用 |
| 位置 | 48 / 4 / 8 | 1024 / 32768 / 16 |
| Iq 电流 | 3633 / 2693 / 0 | 128 / 512 / 禁用 |
| Id 电流 | 3633 / 2693 / 0 | 128 / 512 / 禁用 |

- `临时应用 PID`：只在电机停止时通过 MQTT 下发；不写 STM32 Flash，也不改变
  生成代码中的默认值。
- `保存 PID 参数`：使用 Qt `QSettings` 保存当前界面值，下次启动程序自动载入。
  只有点击该按钮才会改变已保存配置。
- `恢复工程默认 PID`：删除已保存配置，恢复上表的当前工程值；连接且停机时也会
  立即临时下发默认值。

## ESP32 本地 7 秒测试

Qt 不再发送 `run_test`。测试从 ESP32 的 `PID TEST` 页面启动：

1. 测试前可在 Qt 修改 PID，并点击 `临时应用 PID` 更新 ESP32 运行时缓存；
2. 在 ESP32 页面点击 `SPEED CONTROL` 或 `POSITION CONTROL`；
3. ESP32 自动选择 CAN，应用缓存 PID 并设置控制模式；
4. ESP32 启动电机，确认进入 RUN 后下发固定测试目标；
5. STM32 每 2 ms 发送一组 CAN 遥测，ESP32 在 PSRAM 中记录完整 7 秒数据；
6. ESP32 自动停止电机，随后才通过 MQTT QoS 1 分块回传记录；
7. Qt 自动识别本地测试 ID，校验并重组完整数据集，然后生成 PNG 分析报告和
   同名 CSV 原始数据文件。

实验运行期间不通过 MQTT 连续传输曲线数据，因此 Wi-Fi/MQTT 抖动不会降低
采样时间分辨率。当前设置的目标采样周期是 2 ms，约为每秒 500 组完整样本；
每组包含速度、位置、Iq、Id 及对应参考值。Qt 会显示最终点数和实际平均采样周期。

程序不再显示曲线界面。默认保存目录是用户文档目录下的
`MotorControl_PID_Tests`，可在界面中修改和打开。速度与位置报告均为
1800×2000 PNG，包含总体跟踪、0.5/1.0 秒启动瞬态、误差和 Iq/Iq_ref
四个面板。目标曲线先以细虚线绘制，实测曲线后以粗实线绘制。电流图 Y 轴使用
1%～99% percentile 加 10% 边距，原始尖峰仍保留并允许在绘图区边缘裁切。
误差图绘制全部时间点，但使用启动瞬态窗口之后的数据确定对称 Y 轴，使稳态小误差
可见；被裁切的启动峰值仍保留在 CSV 和 Maximum error 指标中。

速度报告显示由采样数据计算的最终目标、点数、时间戳中位采样周期、10%～90%
上升时间、方向归一化超调、±2%（最小 5 RPM）持续调节时间、最后 1 秒 MAE、
参考达到最终目标 90% 后的 RMSE 和最大绝对误差。位置报告采用最短角度误差，
正确处理 0°/360° 环绕。

CSV 不降采样、不滤波、不删除尖峰，每个 MQTT 重组后的原始采样点保存一行，
包含时间、速度/参考/误差、位置/目标/最短角误差、Iq/Iq_ref 和 Id/Id_ref。

MQTT 主题：

- 命令：`motor/control/command`
- 遥测：`motor/control/telemetry`
- 确认：`motor/control/ack`
- 测试阶段：`motor/control/test/status`
- 测试二进制数据：`motor/control/test/data`

## 构建

```powershell
cmake -S QtMqttSimpleControl -B QtMqttSimpleControl/build `
  -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/mingw_64
cmake --build QtMqttSimpleControl/build
```

程序只依赖 Qt 6 的 `Network`、`Widgets` 和 QtGui 内置图像绘制能力，不依赖
Qt Charts。
