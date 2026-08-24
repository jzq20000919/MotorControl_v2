/**
  ******************************************************************************
  * @file    can_STM.c
  * @brief   ESP32 HMI command and telemetry bridge over FDCAN1.
  ******************************************************************************
  */
#include "can_STM.h"

#include <stdint.h>
#include <string.h>

#include "comm_mgr_STM.h"
#include "main.h"
#include "motor_can_protocol.h"

extern FDCAN_HandleTypeDef hfdcan1; /**< CubeMX 生成并初始化的 FDCAN1 句柄。 */

#define MOTOR_CAN_TELEMETRY_PERIOD_MS       (20UL)
#define MOTOR_CAN_LINK_TIMEOUT_MS           (300UL)
#define MOTOR_CAN_INIT_RETRY_MS              (500UL)
#define MOTOR_CAN_BUS_OFF_RECOVERY_MS        (100UL)
#define MOTOR_CAN_SPEED_RAMP_MS             (150UL)
#define MOTOR_CAN_POSITION_MIN_DURATION_MS  (200UL)
#define MOTOR_CAN_POSITION_CDEG_PER_SECOND  (18000UL)

static bool CAN_STM_Ready;                      /**< 为 true 时 FDCAN 已初始化、过滤器已配置且节点已启动。 */
static uint32_t CAN_STM_LastCommandTick;        /**< 最近收到有效 ESP32 命令的 HAL 毫秒节拍。 */
static uint32_t CAN_STM_LastTelemetryTick;      /**< 最近一次发送三组遥测帧的 HAL 毫秒节拍。 */
static uint32_t CAN_STM_LastInitAttemptTick;    /**< 最近尝试初始化 FDCAN 的 HAL 毫秒节拍。 */
static uint32_t CAN_STM_LastRecoveryTick;       /**< 最近尝试从 Bus-Off 恢复的 HAL 毫秒节拍。 */
static uint8_t CAN_STM_LastSequence;            /**< 最近接收命令帧中的序列号，用于状态回显与诊断。 */
static uint8_t CAN_STM_LastCommand;             /**< 最近接收命令帧中的命令码，用于状态回显。 */
static bool CAN_STM_CommandRejected;            /**< 为 true 时表示最近一条非 PING 命令未被应用层接受。 */

/** @brief 将 32 位遥测量饱和限制到经典 CAN 帧使用的 16 位有符号范围。 */
static int16_t CAN_STM_ClampS16(int32_t value)
{
  if (value > INT16_MAX)
  {
    return INT16_MAX;
  }
  if (value < INT16_MIN)
  {
    return INT16_MIN;
  }
  return (int16_t)value;
}

/** @brief 将多圈位置归一化到 [0, 36000) 的单圈 0.01° 表示。 */
static int32_t CAN_STM_NormalizeCdeg(int32_t positionCdeg)
{
  /* 位置对一圈 36000 个 0.01° 单位取模后的临时结果。 */
  int32_t normalized = positionCdeg % 36000;
  if (normalized < 0)
  {
    normalized += 36000;
  }
  return normalized;
}

/** @brief 判断最近 300 ms 内是否收到过有效 ESP32 命令。 */
static bool CAN_STM_LinkActive(uint32_t now)
{
  return (CAN_STM_LastCommandTick != 0UL) &&
         ((now - CAN_STM_LastCommandTick) <= MOTOR_CAN_LINK_TIMEOUT_MS);
}

/** @brief 以经典 CAN 标准帧发送固定 8 字节负载；TX FIFO 满时直接返回失败。 */
static bool CAN_STM_Send(uint32_t identifier, const uint8_t data[8])
{
  /* 当前待发送经典 CAN 数据帧的 FDCAN 帧头配置。 */
  FDCAN_TxHeaderTypeDef header = {0};

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0UL)
  {
    return false;
  }

  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  return HAL_FDCAN_AddMessageToTxFifoQ(
           &hfdcan1, &header, data) == HAL_OK;
}

/** @brief 按旧 CAN 的 180°/s 规则请求最近单圈位置目标。 */
static bool CAN_STM_SetNearestSingleTurnPosition(int32_t targetSingleTurnCdeg)
{
  return CommMgr_STM_SetNearestPosition(targetSingleTurnCdeg,
                                        MOTOR_CAN_POSITION_MIN_DURATION_MS,
                                        MOTOR_CAN_POSITION_CDEG_PER_SECOND);
}

/** @brief 解码命令帧并只通过 CommMgr_STM 请求电机动作。 */
static bool CAN_STM_ExecuteCommand(const uint8_t data[8])
{
  /* 命令帧 byte2 中携带的协议命令码。 */
  const MotorCan_Command_t command = (MotorCan_Command_t)data[2];

  switch (command)
  {
    case MOTOR_CAN_CMD_NOP:
    case MOTOR_CAN_CMD_PING:
      return true;

    case MOTOR_CAN_CMD_SET_MODE:
      return CommMgr_STM_SetMode((MotorMode_STM)data[3]);

    case MOTOR_CAN_CMD_SET_SPEED_RPM:
      return CommMgr_STM_SetSpeed(
        MotorCan_ReadS16(&data[3]), MOTOR_CAN_SPEED_RAMP_MS);

    case MOTOR_CAN_CMD_SET_POSITION_CDEG:
      return CAN_STM_SetNearestSingleTurnPosition(MotorCan_ReadS32(&data[3]));

    case MOTOR_CAN_CMD_START:
      return CommMgr_STM_Start();

    case MOTOR_CAN_CMD_STOP:
      return CommMgr_STM_Stop();

    case MOTOR_CAN_CMD_ACK_FAULT:
      return CommMgr_STM_AcknowledgeFault();

    case MOTOR_CAN_CMD_ZERO_POSITION:
    default:
      /* Changing the estimator angle is unsafe; reject the legacy command. */
      return false;
  }
}

/** @brief 排空 RX FIFO0，校验帧格式/版本并记录最近命令及拒绝状态。 */
static void CAN_STM_ProcessRx(void)
{
  while (HAL_FDCAN_GetRxFifoFillLevel(
           &hfdcan1, FDCAN_RX_FIFO0) > 0UL)
  {
    FDCAN_RxHeaderTypeDef header; /* 从 RX FIFO0 读出的帧格式、ID 和长度信息。 */
    uint8_t data[8];              /* 从 RX FIFO0 读出的 8 字节命令负载。 */

    if (HAL_FDCAN_GetRxMessage(
          &hfdcan1, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      break;
    }
    if ((header.Identifier != MOTOR_CAN_ID_COMMAND) ||
        (header.IdType != FDCAN_STANDARD_ID) ||
        (header.RxFrameType != FDCAN_DATA_FRAME) ||
        (header.FDFormat != FDCAN_CLASSIC_CAN) ||
        (header.DataLength != FDCAN_DLC_BYTES_8) ||
        (data[0] != MOTOR_CAN_PROTOCOL_VERSION))
    {
      continue;
    }

    CAN_STM_LastCommandTick = HAL_GetTick();
    CAN_STM_LastSequence = data[1];
    CAN_STM_LastCommand = data[2];
    if ((MotorCan_Command_t)data[2] != MOTOR_CAN_CMD_PING)
    {
      CAN_STM_CommandRejected = !CAN_STM_ExecuteCommand(data);
    }
    else
    {
      (void)CAN_STM_ExecuteCommand(data);
    }
  }
}

/** @brief 查询 FDCAN 协议状态，并按 100 ms 退避执行 Bus-Off 停止/重启恢复。 */
static bool CAN_STM_ServiceBus(uint32_t now)
{
  /* FDCAN 控制器当前 Bus-Off、错误被动等协议状态。 */
  FDCAN_ProtocolStatusTypeDef status = {0};

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK)
  {
    return false;
  }
  if (status.BusOff == 0U)
  {
    return true;
  }

  /*
   * M_CAN enters INIT when the transmit error counter reaches Bus-Off.
   * The HAL handle does not recover its state automatically, so explicitly
   * stop/start the node.  Clear the link first: telemetry must wait until a
   * fresh ESP32 command proves that another active node can acknowledge it.
   */
  CAN_STM_LastCommandTick = 0UL;
  if ((now - CAN_STM_LastRecoveryTick) < MOTOR_CAN_BUS_OFF_RECOVERY_MS)
  {
    return false;
  }
  CAN_STM_LastRecoveryTick = now;

  if ((HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) ||
      (HAL_FDCAN_Start(&hfdcan1) != HAL_OK))
  {
    CAN_STM_Ready = false;
    return false;
  }

  CAN_STM_LastTelemetryTick = now;
  return false;
}

/** @brief 编码并发送 0x180 状态、链路、命令回显和故障遥测帧。 */
static void CAN_STM_SendStatus(uint32_t now)
{
  MotorState_STM state;
  uint8_t data[8] = {0}; /* ID 0x180 状态遥测帧的 8 字节负载。 */
  uint8_t flags = 0U;    /* 控制模式、运行、故障、链路和拒绝状态的位集合。 */
  CommMgr_STM_GetMotorState(&state);

  if (state.mode == MOTOR_MODE_STM_POSITION)
  {
    flags |= MOTOR_CAN_STATUS_POSITION_MODE;
  }
  if (state.motorRunning)
  {
    flags |= MOTOR_CAN_STATUS_MOTOR_RUNNING;
  }
  if (state.currentFaults != 0U)
  {
    flags |= MOTOR_CAN_STATUS_MOTOR_FAULT;
  }
  if (CAN_STM_LinkActive(now))
  {
    flags |= MOTOR_CAN_STATUS_LINK_ACTIVE;
  }
  if (CAN_STM_CommandRejected)
  {
    flags |= MOTOR_CAN_STATUS_COMMAND_REJECTED;
  }

  data[0] = MOTOR_CAN_PROTOCOL_VERSION;
  data[1] = CAN_STM_LastSequence;
  data[2] = CAN_STM_LastCommand;
  data[3] = flags;
  MotorCan_WriteS16(
    &data[4], state.measuredSpeedRpm);
  MotorCan_WriteU16(&data[6], state.currentFaults);
  (void)CAN_STM_Send(MOTOR_CAN_ID_STATUS, data);
}

/** @brief 编码并发送 0x181 速度参考、单圈位置及位置误差遥测帧。 */
static void CAN_STM_SendReferences(void)
{
  MotorState_STM state;
  uint8_t data[8] = {0}; /* ID 0x181 速度与位置参考遥测帧的负载。 */
  CommMgr_STM_GetMotorState(&state);

  MotorCan_WriteS16(&data[0], state.speedReferenceRpm);
  MotorCan_WriteU16(&data[2], (uint16_t)CAN_STM_NormalizeCdeg(state.currentPositionCdeg));
  MotorCan_WriteU16(&data[4], (uint16_t)CAN_STM_NormalizeCdeg(state.targetPositionCdeg));
  MotorCan_WriteS16(&data[6], CAN_STM_ClampS16(state.targetPositionCdeg - state.currentPositionCdeg));
  (void)CAN_STM_Send(MOTOR_CAN_ID_REFERENCES, data);
}

/** @brief 编码并发送 0x182 q/d 轴实测与参考电流遥测帧。 */
static void CAN_STM_SendElectrical(void)
{
  MotorState_STM state;
  uint8_t data[8] = {0}; /* ID 0x182 d/q 轴电流遥测帧的负载。 */
  CommMgr_STM_GetMotorState(&state);

  MotorCan_WriteS16(&data[0], CAN_STM_ClampS16(state.iqMa));
  MotorCan_WriteS16(&data[2], CAN_STM_ClampS16(state.idMa));
  MotorCan_WriteS16(&data[4], CAN_STM_ClampS16(state.iqReferenceMa));
  MotorCan_WriteS16(&data[6], CAN_STM_ClampS16(state.idReferenceMa));
  (void)CAN_STM_Send(MOTOR_CAN_ID_ELECTRICAL, data);
}

/**
 * @brief 在 CubeMX 已初始化的 FDCAN1 上安装命令过滤器并启动协议控制器。
 * @note 时钟、PA11/PB9、500 kbit/s 位时序和消息 RAM 数量全部由 CubeMX 管理。
 */
bool CAN_STM_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};          /* 仅允许命令 ID 0x100 进入 FIFO0 的过滤器。 */

  CAN_STM_Ready = false;
  CAN_STM_LastInitAttemptTick = HAL_GetTick();
  CAN_STM_LastCommandTick = 0UL;
  CAN_STM_LastTelemetryTick = 0UL;
  CAN_STM_LastRecoveryTick = 0UL;
  CAN_STM_LastSequence = 0U;
  CAN_STM_LastCommand = (uint8_t)MOTOR_CAN_CMD_NOP;
  CAN_STM_CommandRejected = false;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤器就放入FIFO
  filter.FilterID1 = MOTOR_CAN_ID_COMMAND;
  filter.FilterID2 = 0x7FFU;
  if ((HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) ||
      (HAL_FDCAN_ConfigGlobalFilter(
         &hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
         FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
      (HAL_FDCAN_Start(&hfdcan1) != HAL_OK))
  {
    return false;
  }

  CAN_STM_Ready = true;
  return true;
}

/** @brief 执行初始化重试、Bus-Off 服务、命令接收和 20 ms 遥测周期。 */
void CAN_STM_Tick(void)
{
  /* 本轮周期处理开始时读取的 HAL 毫秒节拍。 */
  uint32_t now = HAL_GetTick();

  if (!CAN_STM_Ready)
  {
    if ((now - CAN_STM_LastInitAttemptTick) >= MOTOR_CAN_INIT_RETRY_MS)
    {
      (void)CAN_STM_Init();
    }
    return;
  }

  if (!CAN_STM_ServiceBus(now))
  {
    return;
  }
  CAN_STM_ProcessRx();
  now = HAL_GetTick();
  if (CAN_STM_LinkActive(now) &&
      ((now - CAN_STM_LastTelemetryTick) >= MOTOR_CAN_TELEMETRY_PERIOD_MS))
  {
    CAN_STM_LastTelemetryTick = now;
    CAN_STM_SendStatus(now);
    CAN_STM_SendReferences();
    CAN_STM_SendElectrical();
  }
}
