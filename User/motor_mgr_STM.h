#ifndef MOTOR_MGR_STM_H
#define MOTOR_MGR_STM_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_types_STM.h"

/** @brief 初始化电机应用状态，并记录 MCSDK 当前状态作为后续跃迁检测基线。 */
void MotorMgr_STM_Init(void);
/** @brief 检测 MCSDK 运行状态跃迁，并在进入 RUN 时恢复已验证的模式运行语义。 */
void MotorMgr_STM_Tick(void);
/** @brief 复制统一电机状态快照。 @param[out] state 快照接收对象，不能为 NULL。 */
void MotorMgr_STM_GetState(MotorState_STM *state);
/** @brief 切换速度或位置控制模式。 @return 参数及运行条件允许时返回 true。 */
bool MotorMgr_STM_SetMode(MotorMode_STM mode);
/** @brief 下发速度斜坡，目标单位 rpm、时长单位 ms。 */
bool MotorMgr_STM_SetSpeed(int16_t targetRpm, uint32_t durationMs);
/** @brief 下发多圈绝对位置轨迹，目标单位 0.01°、时长单位 ms。 */
bool MotorMgr_STM_SetPosition(int32_t targetCdeg, uint32_t durationMs);
/** @brief 将单圈目标换算为最近多圈目标；角速度为零时使用固定最短时长。 */
bool MotorMgr_STM_SetNearestSingleTurnPosition(int32_t targetCdeg,
                                               uint32_t minimumDurationMs,
                                               uint32_t speedCdegPerSecond);
/** @brief 启动电机；已处于 RUN 时按成功处理。 */
bool MotorMgr_STM_Start(void);
/** @brief 请求电机停止。 */
bool MotorMgr_STM_Stop(void);
/** @brief 请求确认并清除可恢复故障。 */
bool MotorMgr_STM_AcknowledgeFault(void);
/** @brief 在位置模式中清空当前轨迹历史并锁定实测位置。 */
bool MotorMgr_STM_HoldPosition(void);

#endif /* MOTOR_MGR_STM_H */
