#ifndef COMM_MGR_STM_H
#define COMM_MGR_STM_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_types_STM.h"

/** @brief 初始化统一命令入口及 CAN、USART、MCP 三个通信适配器。 */
void CommMgr_STM_Init(void);
/** @brief 运行电机状态跃迁检测及 CAN/USART 的非阻塞周期任务。 */
void CommMgr_STM_Tick(void);
/** @brief 获取供通信适配器编码使用的统一电机状态快照。 */
void CommMgr_STM_GetMotorState(MotorState_STM *state);
/** @brief 统一请求控制模式切换。 */
bool CommMgr_STM_SetMode(MotorMode_STM mode);
/** @brief 统一请求速度斜坡，单位分别为 rpm 和 ms。 */
bool CommMgr_STM_SetSpeed(int16_t targetRpm, uint32_t durationMs);
/** @brief 统一请求多圈位置轨迹，单位分别为 0.01° 和 ms。 */
bool CommMgr_STM_SetPosition(int32_t targetCdeg, uint32_t durationMs);
/** @brief 统一请求最近单圈位置轨迹。 */
bool CommMgr_STM_SetNearestPosition(int32_t targetCdeg, uint32_t minimumDurationMs,
                                    uint32_t speedCdegPerSecond);
/** @brief 统一请求临时修改一个 PID 增益。 */
bool CommMgr_STM_SetPidGain(MotorPidController_STM controller,
                            MotorPidTerm_STM term, int16_t value);
/** @brief 统一请求启动电机。 */
bool CommMgr_STM_Start(void);
/** @brief 统一请求停止电机。 */
bool CommMgr_STM_Stop(void);
/** @brief 统一请求确认电机故障。 */
bool CommMgr_STM_AcknowledgeFault(void);
/** @brief 统一请求保持当前位置。 */
bool CommMgr_STM_HoldPosition(void);

#endif /* COMM_MGR_STM_H */
