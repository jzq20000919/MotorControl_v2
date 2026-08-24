#ifndef USART_ESP_H
#define USART_ESP_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_uart_protocol.h"

/** @brief UART 电机链路的最新遥测与通信诊断快照。 */
typedef struct
{
    bool link_active;                 /**< 最近链路超时窗口内收到有效遥测。 */
    bool reconnecting;                /**< UART 正在执行异步重连。 */
    bool motor_running;               /**< 电机当前处于运行状态。 */
    bool motor_fault;                 /**< 电机当前存在故障。 */
    bool command_rejected;            /**< 电机板拒绝了最近一条命令。 */
    MotorUart_Mode_t mode;            /**< 当前速度或位置控制模式。 */
    uint16_t faults;                  /**< 电机故障位集合。 */
    int16_t measured_speed_rpm;       /**< 实测机械转速，单位 rpm。 */
    int16_t reference_speed_rpm;      /**< 机械转速参考值，单位 rpm。 */
    uint16_t current_position_cdeg;   /**< 当前单圈位置，单位 0.01°。 */
    uint16_t target_position_cdeg;    /**< 目标单圈位置，单位 0.01°。 */
    int16_t position_error_cdeg;      /**< 位置误差，单位 0.01°。 */
    int16_t iq_ma;                    /**< q 轴实测电流，单位 mA。 */
    int16_t id_ma;                    /**< d 轴实测电流，单位 mA。 */
    int16_t iq_reference_ma;          /**< q 轴电流参考值，单位 mA。 */
    int16_t uq_mv;                    /**< q 轴电压指令，单位 mV。 */
    int16_t ud_mv;                    /**< d 轴电压指令，单位 mV。 */
    uint32_t received_bytes;          /**< UART 累计接收字节数。 */
    uint32_t received_frames;         /**< 累计有效接收帧数。 */
    uint32_t transmitted_frames;      /**< 累计成功发送帧数。 */
    uint32_t transmit_errors;         /**< 累计发送错误次数。 */
    uint32_t crc_errors;              /**< 累计 CRC 校验错误次数。 */
    uint32_t protocol_errors;         /**< 累计协议格式错误次数。 */
    uint32_t baud_rate;               /**< 当前 UART 波特率。 */
    uint32_t reconnect_count;         /**< 累计重连成功或执行次数。 */
    uint32_t reconnect_errors;        /**< 累计重连失败次数。 */
} USART_ESP_snapshot_t;

/** @brief 安装 UART1、创建 RX/TX 任务并初始化协议状态。 @return 成功返回 ESP_OK。 */
esp_err_t USART_ESP_Init(void);
/** @brief 停止 UART 任务、删除队列并释放 UART 驱动。 */
void USART_ESP_Deinit(void);
/** @brief 返回 UART 传输通道是否已完整初始化。 @return 已初始化返回 true。 */
bool USART_ESP_IsInitialized(void);
/** @brief 请求异步重新配置 UART。 @param baud_rate 新波特率。 @return 请求入队成功返回 ESP_OK。 */
esp_err_t USART_ESP_RequestReconnect(uint32_t baud_rate);
/** @brief 不等待新帧，直接复制最新已解析 UART 遥测。 @param[out] snapshot 接收快照的对象。 */
void USART_ESP_GetSnapshot(USART_ESP_snapshot_t *snapshot);
/** @brief 授予或撤销该通道发送电机命令的控制权。 @param enabled true 表示授予控制权。 */
void USART_ESP_SetControlEnabled(bool enabled);

/** @brief 排队一个 UART 控制模式命令。 @param mode 目标控制模式。 */
void USART_ESP_SetMode(MotorUart_Mode_t mode);
/** @brief 保存最新速度目标，供 UART 周期发送。 @param speed_rpm 目标转速，单位 rpm。 */
void USART_ESP_SetSpeedRPM(int16_t speed_rpm);
/** @brief 保存最新位置目标，供 UART 周期发送。 @param position_cdeg 目标位置，单位 0.01°。 */
void USART_ESP_SetPositionCdeg(uint16_t position_cdeg);
/** @brief 排队一个 UART 电机启动命令。 */
void USART_ESP_Start(void);
/** @brief 排队一个 UART 电机停止命令。 */
void USART_ESP_Stop(void);
/** @brief 排队一个 UART 故障确认命令。 */
void USART_ESP_AcknowledgeFault(void);
/** @brief 排队一个 UART 位置清零命令。 */
void USART_ESP_ZeroPosition(void);

#endif /* USART_ESP_H */
