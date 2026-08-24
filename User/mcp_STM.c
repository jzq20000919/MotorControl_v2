#include "mcp_STM.h"

#include <string.h>

#include "comm_mgr_STM.h"
#include "mcp.h"
#include "usart_STM.h"

#define MCP_STM_PROTOCOL_VERSION (3U)
#define MCP_STM_CALLBACK_ID       (0U)
#define MCP_STM_TELEMETRY_MAGIC   (0x31434F46UL)
#define MCP_STM_UART_DIAG_MAGIC   (0x31443355UL)
#define MCP_STM_TELEMETRY_SIZE    (88U)

/** @brief MCP 扩展命令码；数值保持与既有 Qt 上位机协议完全一致。 */
typedef enum
{
  MCP_STM_CMD_GET_TELEMETRY = 0,
  MCP_STM_CMD_SET_MODE = 1,
  MCP_STM_CMD_SET_SPEED_RPM = 2,
  MCP_STM_CMD_SET_POSITION = 3,
  MCP_STM_CMD_START_MOTOR = 4,
  MCP_STM_CMD_STOP_MOTOR = 5,
  MCP_STM_CMD_ACK_FAULT = 6,
  MCP_STM_CMD_ZERO_POSITION = 7,
  MCP_STM_CMD_HOLD_POSITION = 8
} MCP_STM_Command;

/** @brief 按小端序写入 16 位无符号字段。 */
static void MCP_STM_WriteU16(uint8_t *buffer, uint16_t value)
{
  buffer[0] = (uint8_t)value;
  buffer[1] = (uint8_t)(value >> 8U);
}

/** @brief 按小端序写入 16 位有符号字段。 */
static void MCP_STM_WriteS16(uint8_t *buffer, int16_t value)
{
  MCP_STM_WriteU16(buffer, (uint16_t)value);
}

/** @brief 按小端序写入 32 位无符号字段。 */
static void MCP_STM_WriteU32(uint8_t *buffer, uint32_t value)
{
  buffer[0] = (uint8_t)value;
  buffer[1] = (uint8_t)(value >> 8U);
  buffer[2] = (uint8_t)(value >> 16U);
  buffer[3] = (uint8_t)(value >> 24U);
}

/** @brief 按小端序写入 32 位有符号字段。 */
static void MCP_STM_WriteS32(uint8_t *buffer, int32_t value)
{
  MCP_STM_WriteU32(buffer, (uint32_t)value);
}

/** @brief 从小端负载读取 16 位有符号参数。 */
static int16_t MCP_STM_ReadS16(const uint8_t *buffer)
{
  return (int16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8U));
}

/** @brief 从小端负载读取 32 位无符号参数。 */
static uint32_t MCP_STM_ReadU32(const uint8_t *buffer)
{
  return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8U) |
         ((uint32_t)buffer[2] << 16U) | ((uint32_t)buffer[3] << 24U);
}

/** @brief 从小端负载读取 32 位有符号参数。 */
static int32_t MCP_STM_ReadS32(const uint8_t *buffer)
{
  return (int32_t)MCP_STM_ReadU32(buffer);
}

/**
 * @brief 构建固定 88 字节 MCP 遥测。
 * @note 0~47 字节来自统一 MotorState；48~87 字节保留专用 USART 诊断布局。
 */
static uint16_t MCP_STM_BuildTelemetry(uint8_t *buffer, int16_t capacity)
{
  MotorState_STM state;
  USART_STM_Diagnostics_t diagnostics;
  if ((buffer == NULL) || (capacity < (int16_t)MCP_STM_TELEMETRY_SIZE)) return 0U;
  CommMgr_STM_GetMotorState(&state);
  USART_STM_GetDiagnostics(&diagnostics);
  (void)memset(buffer, 0, MCP_STM_TELEMETRY_SIZE);
  MCP_STM_WriteU32(&buffer[0], MCP_STM_TELEMETRY_MAGIC);
  buffer[4] = MCP_STM_PROTOCOL_VERSION;
  buffer[5] = (uint8_t)state.mode;
  buffer[6] = state.motorState;
  buffer[7] = state.positionStatus;
  MCP_STM_WriteU16(&buffer[8], state.currentFaults);
  MCP_STM_WriteU16(&buffer[10], state.occurredFaults);
  MCP_STM_WriteS32(&buffer[12], state.iqMa);
  MCP_STM_WriteS32(&buffer[16], state.idMa);
  MCP_STM_WriteS32(&buffer[20], state.iqReferenceMa);
  MCP_STM_WriteS32(&buffer[24], state.idReferenceMa);
  MCP_STM_WriteS16(&buffer[36], state.speedReferenceRpm);
  MCP_STM_WriteS16(&buffer[38], state.measuredSpeedRpm);
  MCP_STM_WriteS32(&buffer[40], state.targetPositionCdeg);
  MCP_STM_WriteS32(&buffer[44], state.currentPositionCdeg);
  MCP_STM_WriteU32(&buffer[48], MCP_STM_UART_DIAG_MAGIC);
  buffer[52] = diagnostics.initialized ? 1U : 0U;
  buffer[53] = diagnostics.linkActive ? 1U : 0U;
  buffer[54] = diagnostics.initStage;
  buffer[55] = diagnostics.lastTxStatus;
  MCP_STM_WriteU32(&buffer[56], diagnostics.uartError);
  MCP_STM_WriteU32(&buffer[60], diagnostics.receivedBytes);
  MCP_STM_WriteU32(&buffer[64], diagnostics.validCommandFrames);
  MCP_STM_WriteU32(&buffer[68], diagnostics.crcErrors);
  MCP_STM_WriteU32(&buffer[72], diagnostics.protocolErrors);
  MCP_STM_WriteU32(&buffer[76], diagnostics.telemetryAttempts);
  MCP_STM_WriteU32(&buffer[80], diagnostics.telemetrySent);
  MCP_STM_WriteU32(&buffer[84], diagnostics.telemetryErrors);
  return MCP_STM_TELEMETRY_SIZE;
}

/** @brief 解析 MCP 扩展命令并通过 CommMgr_STM 执行，成功时回送最新遥测。 */
static uint8_t MCP_STM_Callback(uint16_t rxLength, uint8_t *rxBuffer,
                                int16_t txSyncFreeSpace, uint16_t *txLength,
                                uint8_t *txBuffer)
{
  uint8_t result = MCP_CMD_OK;
  if ((rxLength < 1U) || (rxBuffer == NULL) || (txLength == NULL) || (txBuffer == NULL)) return MCP_CMD_NOK;
  switch ((MCP_STM_Command)rxBuffer[0])
  {
    case MCP_STM_CMD_GET_TELEMETRY:
      break;
    case MCP_STM_CMD_SET_MODE:
      result = ((rxLength >= 2U) && CommMgr_STM_SetMode((MotorMode_STM)rxBuffer[1])) ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_SET_SPEED_RPM:
      result = ((rxLength >= 7U) && CommMgr_STM_SetSpeed(MCP_STM_ReadS16(&rxBuffer[1]), MCP_STM_ReadU32(&rxBuffer[3]))) ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_SET_POSITION:
      result = ((rxLength >= 9U) && CommMgr_STM_SetPosition(MCP_STM_ReadS32(&rxBuffer[1]), MCP_STM_ReadU32(&rxBuffer[5]))) ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_START_MOTOR:
      result = CommMgr_STM_Start() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_STOP_MOTOR:
      result = CommMgr_STM_Stop() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_ACK_FAULT:
      result = CommMgr_STM_AcknowledgeFault() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_HOLD_POSITION:
      result = CommMgr_STM_HoldPosition() ? MCP_CMD_OK : MCP_CMD_NOK;
      break;
    case MCP_STM_CMD_ZERO_POSITION:
    default:
      result = MCP_CMD_UNKNOWN;
      break;
  }
  if (result == MCP_CMD_OK)
  {
    *txLength = MCP_STM_BuildTelemetry(txBuffer, txSyncFreeSpace);
    if (*txLength == 0U) result = MCP_ERROR_NO_TXSYNC_SPACE;
  }
  else *txLength = 0U;
  return result;
}

/** @brief 向 MCSDK MCP 注册用户回调 0，保持既有上位机命令号不变。 */
void MCP_STM_Init(void)
{
  (void)MCP_RegisterCallBack(MCP_STM_CALLBACK_ID, MCP_STM_Callback);
}
