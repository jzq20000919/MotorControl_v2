#ifndef MOTOR_TYPES_STM_H
#define MOTOR_TYPES_STM_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 应用层统一控制模式；数值与现有 CAN、UART、MCP 协议保持一致。 */
typedef enum
{
  MOTOR_MODE_STM_SPEED = 0,
  MOTOR_MODE_STM_POSITION = 1
} MotorMode_STM;

/** @brief 可通过上位机临时调整的调节器编号。 */
typedef enum
{
  MOTOR_PID_STM_SPEED = 0,
  MOTOR_PID_STM_POSITION = 1,
  MOTOR_PID_STM_IQ = 2,
  MOTOR_PID_STM_ID = 3
} MotorPidController_STM;

/** @brief PID 调节器中的增益项编号。 */
typedef enum
{
  MOTOR_PID_TERM_STM_KP = 0,
  MOTOR_PID_TERM_STM_KI = 1,
  MOTOR_PID_TERM_STM_KD = 2
} MotorPidTerm_STM;

/** @brief 由 MotorMgr_STM 统一采集、供所有通信通道只读使用的电机状态。 */
typedef struct
{
  MotorMode_STM mode;          /**< 当前控制模式。 */
  uint8_t motorState;          /**< MCSDK MCI_State_t 的数值。 */
  uint8_t positionStatus;      /**< MCSDK 位置轨迹控制状态。 */
  bool motorRunning;           /**< 电机状态机当前是否为 RUN。 */
  uint16_t currentFaults;      /**< 当前故障位。 */
  uint16_t occurredFaults;     /**< 历史故障位。 */
  int16_t speedReferenceRpm;   /**< 转速参考，单位 rpm。 */
  int16_t measuredSpeedRpm;    /**< 实测转速，单位 rpm。 */
  int32_t targetPositionCdeg;  /**< 多圈目标位置，单位 0.01°。 */
  int32_t currentPositionCdeg; /**< 多圈实测位置，单位 0.01°。 */
  int32_t iqMa;                /**< q 轴实测电流，单位 mA。 */
  int32_t idMa;                /**< d 轴实测电流，单位 mA。 */
  int32_t iqReferenceMa;       /**< q 轴电流参考，单位 mA。 */
  int32_t idReferenceMa;       /**< d 轴电流参考，单位 mA。 */
} MotorState_STM;

#endif /* MOTOR_TYPES_STM_H */
