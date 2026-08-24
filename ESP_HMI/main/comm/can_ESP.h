#ifndef CAN_ESP_H
#define CAN_ESP_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_can_protocol.h"

/** @brief CAN 电机链路的最新遥测与通信诊断快照。 */
typedef struct
{
    bool link_active;                /**< 最近链路超时窗口内收到有效遥测。 */
    bool bus_off;                    /**< CAN 控制器当前处于 Bus-Off 状态。 */
    bool transceiver_fault;          /**< 本地 CAN 收发器自检失败。 */
    bool motor_running;              /**< 电机当前处于运行状态。 */
    bool motor_fault;                /**< 电机当前存在故障。 */
    bool command_rejected;           /**< 电机板拒绝了最近一条命令。 */
    MotorCan_Mode_t mode;            /**< 当前速度或位置控制模式。 */
    uint16_t faults;                 /**< 电机故障位集合。 */
    int16_t measured_speed_rpm;      /**< 实测机械转速，单位 rpm。 */
    int16_t reference_speed_rpm;     /**< 机械转速参考值，单位 rpm。 */
    uint16_t current_position_cdeg;  /**< 当前单圈位置，单位 0.01°。 */
    uint16_t target_position_cdeg;   /**< 目标单圈位置，单位 0.01°。 */
    int16_t position_error_cdeg;     /**< 位置误差，单位 0.01°。 */
    int16_t iq_ma;                   /**< q 轴实测电流，单位 mA。 */
    int16_t id_ma;                   /**< d 轴实测电流，单位 mA。 */
    int16_t iq_reference_ma;         /**< q 轴电流参考值，单位 mA。 */
    int16_t id_reference_ma;         /**< d 轴电流参考值，单位 mA。 */
    uint32_t sample_sequence;        /**< 每收到一组完整遥测后递增。 */
    int64_t sample_timestamp_us;     /**< 完整样本到达 ESP32 的单调时间戳。 */
    uint32_t received_frames;        /**< 累计有效接收帧数。 */
    uint32_t transmitted_frames;     /**< 累计成功发送帧数。 */
    uint32_t transmit_errors;        /**< 累计发送错误次数。 */
} CAN_ESP_snapshot_t;

/** @brief 安装并启动 ESP32 TWAI 控制器及 CAN RX/TX 任务。 @return 成功返回 ESP_OK。 */
esp_err_t CAN_ESP_Init(void);
/** @brief 停止 CAN 任务、卸载 TWAI 并释放队列。 */
void CAN_ESP_Deinit(void);
/** @brief 返回 CAN 传输通道是否已完整初始化。 @return 已初始化返回 true。 */
bool CAN_ESP_IsInitialized(void);
/** @brief 不等待新帧，直接复制最新已解码 CAN 遥测。 @param[out] snapshot 接收快照的对象。 */
void CAN_ESP_GetSnapshot(CAN_ESP_snapshot_t *snapshot);
/** @brief 授予或撤销该传输通道发送电机命令的控制权。 @param enabled true 表示授予控制权。 */
void CAN_ESP_SetControlEnabled(bool enabled);

/** @brief 排队一个 CAN 控制模式命令。 @param mode 目标控制模式。 */
void CAN_ESP_SetMode(MotorCan_Mode_t mode);
/** @brief 保存最新速度目标，供 CAN 周期发送。 @param speed_rpm 目标转速，单位 rpm。 */
void CAN_ESP_SetSpeedRPM(int16_t speed_rpm);
/** @brief 保存最新位置目标，供 CAN 周期发送。 @param position_cdeg 目标位置，单位 0.01°。 */
void CAN_ESP_SetPositionCdeg(uint16_t position_cdeg);
/** @brief 排队临时设置一个 PID 增益。 */
void CAN_ESP_SetPidGain(uint8_t controller, uint8_t term, int16_t value);
/** @brief 排队一个 CAN 电机启动命令。 */
void CAN_ESP_Start(void);
/** @brief 排队一个 CAN 电机停止命令。 */
void CAN_ESP_Stop(void);
/** @brief 排队一个 CAN 故障确认命令。 */
void CAN_ESP_AcknowledgeFault(void);
/** @brief 排队一个 CAN 位置清零命令。 */
void CAN_ESP_ZeroPosition(void);

#endif /* CAN_ESP_H */
