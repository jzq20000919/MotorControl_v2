#include "comm_mgr_STM.h"

#include "can_STM.h"
#include "mcp_STM.h"
#include "motor_mgr_STM.h"
#include "usart_STM.h"

/** @brief 按 MotorMgr、MCP、CAN、USART 的依赖顺序初始化应用层。 */
void CommMgr_STM_Init(void)
{
  MotorMgr_STM_Init();
  MCP_STM_Init();
  (void)CAN_STM_Init();
  (void)USART_STM_Init();
}

/** @brief 执行不阻塞主循环的状态维护、CAN 服务与 USART DMA 解析。 */
void CommMgr_STM_Tick(void)
{
  MotorMgr_STM_Tick();
  CAN_STM_Tick();
  USART_STM_Process();
}

/** @brief 转发统一状态读取，隔离通信层与 MCSDK API。 */
void CommMgr_STM_GetMotorState(MotorState_STM *state) { MotorMgr_STM_GetState(state); }
/** @brief 转发模式命令到唯一的电机管理层。 */
bool CommMgr_STM_SetMode(MotorMode_STM mode) { return MotorMgr_STM_SetMode(mode); }
/** @brief 转发速度命令到唯一的电机管理层。 */
bool CommMgr_STM_SetSpeed(int16_t targetRpm, uint32_t durationMs) { return MotorMgr_STM_SetSpeed(targetRpm, durationMs); }
/** @brief 转发绝对位置命令到唯一的电机管理层。 */
bool CommMgr_STM_SetPosition(int32_t targetCdeg, uint32_t durationMs) { return MotorMgr_STM_SetPosition(targetCdeg, durationMs); }
/** @brief 转发最近单圈位置命令到唯一的电机管理层。 */
bool CommMgr_STM_SetNearestPosition(int32_t targetCdeg, uint32_t minimumDurationMs,
                                    uint32_t speedCdegPerSecond)
{
  return MotorMgr_STM_SetNearestSingleTurnPosition(targetCdeg, minimumDurationMs, speedCdegPerSecond);
}
/** @brief 转发临时 PID 增益到唯一的电机管理层。 */
bool CommMgr_STM_SetPidGain(MotorPidController_STM controller,
                            MotorPidTerm_STM term, int16_t value)
{
  return MotorMgr_STM_SetPidGain(controller, term, value);
}
/** @brief 转发启动命令到唯一的电机管理层。 */
bool CommMgr_STM_Start(void) { return MotorMgr_STM_Start(); }
/** @brief 转发停止命令到唯一的电机管理层。 */
bool CommMgr_STM_Stop(void) { return MotorMgr_STM_Stop(); }
/** @brief 转发故障确认命令到唯一的电机管理层。 */
bool CommMgr_STM_AcknowledgeFault(void) { return MotorMgr_STM_AcknowledgeFault(); }
/** @brief 转发位置保持命令到唯一的电机管理层。 */
bool CommMgr_STM_HoldPosition(void) { return MotorMgr_STM_HoldPosition(); }
