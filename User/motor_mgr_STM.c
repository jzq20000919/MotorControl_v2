#include "motor_mgr_STM.h"

#include <limits.h>

#include "mc_api.h"
#include "mc_config.h"

#define MOTOR_MGR_STM_RAD_TO_CDEG              (5729.577951308232F)
#define MOTOR_MGR_STM_CDEG_TO_RAD              (0.000174532925199433F)
#define MOTOR_MGR_STM_MIN_POSITION_DURATION_MS (100UL)
#define MOTOR_MGR_STM_MAX_POSITION_DURATION_MS (120000UL)
#define MOTOR_MGR_STM_MAX_SPEED_DURATION_MS    (60000UL)
#define MOTOR_MGR_STM_MAX_SPEED_RPM            (2600)

static MCI_State_t s_previousMotorState = IDLE;
static MotorMode_STM s_mode = MOTOR_MODE_STM_POSITION;

/** @brief 将浮点遥测量饱和转换为 int32_t，避免越界转换产生未定义结果。 */
static int32_t MotorMgr_STM_FloatToS32(float value)
{
  if (value > 2147483000.0F) return INT32_MAX;
  if (value < -2147483000.0F) return INT32_MIN;
  return (int32_t)value;
}

/** @brief 将任意多圈位置归一化到 [0, 36000) 的单圈 0.01° 表示。 */
static int32_t MotorMgr_STM_NormalizeCdeg(int32_t positionCdeg)
{
  int32_t normalized = positionCdeg % 36000;
  return normalized < 0 ? normalized + 36000 : normalized;
}

/**
 * @brief 清除位置轨迹历史并把调节器锁定在当前实测位置。
 * @note 保留已验证的 PID 增益，仅复位运行期积分/轨迹状态，避免对齐前坐标残留。
 */
static void MotorMgr_STM_HoldCurrentPosition(void)
{
  const float currentPosition = MC_GetCurrentPosition1();
  PID_HandleInit(&PID_PosParamsM1);
  PosCtrlM1.MovementDuration = 0.0F;
  PosCtrlM1.AngleStep = 0.0F;
  PosCtrlM1.Jerk = 0.0F;
  PosCtrlM1.CruiseSpeed = 0.0F;
  PosCtrlM1.Acceleration = 0.0F;
  PosCtrlM1.Omega = 0.0F;
  PosCtrlM1.OmegaPrev = 0.0F;
  PosCtrlM1.Theta = currentPosition;
  PosCtrlM1.ThetaPrev = currentPosition;
  PosCtrlM1.ReceivedTh = 0U;
  PosCtrlM1.TcTick = 0U;
  PosCtrlM1.ElapseTime = 0.0F;
  PosCtrlM1.PositionControlRegulation = true;
  PosCtrlM1.PositionCtrlStatus = TC_READY_FOR_COMMAND;
}

/** @brief 关闭位置调节并用零速斜坡把 STC 安全切换回速度模式。 */
static void MotorMgr_STM_EnterSpeedMode(void)
{
  PosCtrlM1.PositionControlRegulation = false;
  PosCtrlM1.PositionCtrlStatus = TC_READY_FOR_COMMAND;
  PosCtrlM1.MovementDuration = 0.0F;
  PID_HandleInit(&PID_PosParamsM1);
  if (MC_GetSTMStateMotor1() == RUN) MC_ProgramSpeedRampMotor1_F(0.0F, 0U);
}

/** @brief 初始化应用模式和电机状态跃迁检测基线，不修改任何静态电机参数。 */
void MotorMgr_STM_Init(void)
{
  s_mode = MOTOR_MODE_STM_POSITION;
  s_previousMotorState = MC_GetSTMStateMotor1();
}

/** @brief 在电机首次进入 RUN 时按当前模式建立无突变的运行期控制状态。 */
void MotorMgr_STM_Tick(void)
{
  const MCI_State_t currentState = MC_GetSTMStateMotor1();
  if ((currentState == RUN) && (s_previousMotorState != RUN))
  {
    if (s_mode == MOTOR_MODE_STM_POSITION) MotorMgr_STM_HoldCurrentPosition();
    else MotorMgr_STM_EnterSpeedMode();
  }
  s_previousMotorState = currentState;
}

/** @brief 一次性读取 MCSDK 状态，形成供通信层使用的一致快照。 */
void MotorMgr_STM_GetState(MotorState_STM *state)
{
  qd_f_t current;
  qd_f_t reference;
  if (state == NULL) return;
  current = MC_GetIqdMotor1_F();
  reference = MC_GetIqdrefMotor1_F();
  state->mode = s_mode;
  state->motorState = (uint8_t)MC_GetSTMStateMotor1();
  state->positionStatus = (uint8_t)MC_GetControlPositionStatusMotor1();
  state->motorRunning = MC_GetSTMStateMotor1() == RUN;
  state->currentFaults = MC_GetCurrentFaultsMotor1();
  state->occurredFaults = MC_GetOccurredFaultsMotor1();
  state->speedReferenceRpm = (int16_t)MC_GetMecSpeedReferenceMotor1_F();
  state->measuredSpeedRpm = (int16_t)MC_GetAverageMecSpeedMotor1_F();
  state->targetPositionCdeg = MotorMgr_STM_FloatToS32(MC_GetTargetPosition1() * MOTOR_MGR_STM_RAD_TO_CDEG);
  state->currentPositionCdeg = MotorMgr_STM_FloatToS32(MC_GetCurrentPosition1() * MOTOR_MGR_STM_RAD_TO_CDEG);
  state->iqMa = MotorMgr_STM_FloatToS32(current.q * 1000.0F);
  state->idMa = MotorMgr_STM_FloatToS32(current.d * 1000.0F);
  state->iqReferenceMa = MotorMgr_STM_FloatToS32(reference.q * 1000.0F);
  state->idReferenceMa = MotorMgr_STM_FloatToS32(reference.d * 1000.0F);
}

/** @brief 切换控制模式，同时保留旧工程已验证的运行期无突变处理。 */
bool MotorMgr_STM_SetMode(MotorMode_STM mode)
{
  if ((mode != MOTOR_MODE_STM_SPEED) && (mode != MOTOR_MODE_STM_POSITION)) return false;
  if (mode == s_mode) return true;
  s_mode = mode;
  if (mode == MOTOR_MODE_STM_POSITION)
  {
    if (MC_GetSTMStateMotor1() == RUN) MotorMgr_STM_HoldCurrentPosition();
  }
  else MotorMgr_STM_EnterSpeedMode();
  return true;
}

/** @brief 按旧工程相同边界限幅后，调用 MCSDK 速度斜坡接口。 */
bool MotorMgr_STM_SetSpeed(int16_t targetRpm, uint32_t durationMs)
{
  if ((MC_GetSTMStateMotor1() != RUN) || (s_mode != MOTOR_MODE_STM_SPEED)) return false;
  if (targetRpm > MOTOR_MGR_STM_MAX_SPEED_RPM) targetRpm = MOTOR_MGR_STM_MAX_SPEED_RPM;
  if (targetRpm < -MOTOR_MGR_STM_MAX_SPEED_RPM) targetRpm = -MOTOR_MGR_STM_MAX_SPEED_RPM;
  if (durationMs > MOTOR_MGR_STM_MAX_SPEED_DURATION_MS) durationMs = MOTOR_MGR_STM_MAX_SPEED_DURATION_MS;
  MC_ProgramSpeedRampMotor1_F((float)targetRpm, (uint16_t)durationMs);
  return true;
}

/** @brief 复位待替换轨迹的误差历史，并下发多圈绝对位置命令。 */
bool MotorMgr_STM_SetPosition(int32_t targetCdeg, uint32_t durationMs)
{
  if ((MC_GetSTMStateMotor1() != RUN) || (s_mode != MOTOR_MODE_STM_POSITION)) return false;
  if (durationMs < MOTOR_MGR_STM_MIN_POSITION_DURATION_MS) durationMs = MOTOR_MGR_STM_MIN_POSITION_DURATION_MS;
  if (durationMs > MOTOR_MGR_STM_MAX_POSITION_DURATION_MS) durationMs = MOTOR_MGR_STM_MAX_POSITION_DURATION_MS;
  PosCtrlM1.PositionCtrlStatus = TC_READY_FOR_COMMAND;
  PID_PosParamsM1.wPrevProcessVarError = 0;
  MC_ProgramPositionCommandMotor1((float)targetCdeg * MOTOR_MGR_STM_CDEG_TO_RAD,
                                  (float)durationMs / 1000.0F);
  return true;
}

/** @brief 统一计算最近单圈路径，消除 CAN 与 UART 各自重复的多圈换算。 */
bool MotorMgr_STM_SetNearestSingleTurnPosition(int32_t targetCdeg,
                                               uint32_t minimumDurationMs,
                                               uint32_t speedCdegPerSecond)
{
  const int32_t current = MotorMgr_STM_FloatToS32(MC_GetCurrentPosition1() * MOTOR_MGR_STM_RAD_TO_CDEG);
  int32_t delta = MotorMgr_STM_NormalizeCdeg(targetCdeg) - MotorMgr_STM_NormalizeCdeg(current);
  uint32_t durationMs = minimumDurationMs;
  if (delta > 18000) delta -= 36000;
  else if (delta < -18000) delta += 36000;
  if (speedCdegPerSecond != 0U)
  {
    const uint32_t distance = (uint32_t)(delta < 0 ? -delta : delta);
    durationMs = (distance * 1000UL) / speedCdegPerSecond;
    if (durationMs < minimumDurationMs) durationMs = minimumDurationMs;
  }
  return MotorMgr_STM_SetPosition(current + delta, durationMs);
}

/** @brief 启动电机，并把已处于 RUN 的幂等请求视为成功。 */
bool MotorMgr_STM_Start(void) { return (MC_GetSTMStateMotor1() == RUN) ? true : MC_StartMotor1(); }
/** @brief 将停止请求原样交给 MCSDK。 */
bool MotorMgr_STM_Stop(void) { return MC_StopMotor1(); }
/** @brief 将故障确认请求原样交给 MCSDK。 */
bool MotorMgr_STM_AcknowledgeFault(void) { return MC_AcknowledgeFaultMotor1(); }

/** @brief 仅在位置模式 RUN 时锁定当前实测位置。 */
bool MotorMgr_STM_HoldPosition(void)
{
  if ((MC_GetSTMStateMotor1() != RUN) || (s_mode != MOTOR_MODE_STM_POSITION)) return false;
  MotorMgr_STM_HoldCurrentPosition();
  return true;
}
