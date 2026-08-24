/*
 * @file usart_STM.c
 * @brief ESP32 HMI 专用 USART3 通信适配器。
 * PC10 is TX and PC11 is RX.  USART2/ASPEP remains dedicated to Qt/Motor Pilot.
 */
#include "usart_STM.h"

#include <limits.h>
#include <string.h>

#include "comm_mgr_STM.h"
#include "motor_uart_protocol.h"
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart3;          /**< CubeMX 生成并初始化的 USART3 句柄。 */
extern DMA_HandleTypeDef hdma_usart3_rx;   /**< CubeMX 生成的 USART3 循环接收 DMA 句柄。 */

#define MOTOR_UART_LINK_TIMEOUT_MS       (300U)
#define MOTOR_UART_TELEMETRY_PERIOD_MS   (20U)
#define MOTOR_UART_RX_DMA_SIZE             (128U)

typedef struct
{
  uint8_t rxDmaBuffer[MOTOR_UART_RX_DMA_SIZE];
  uint16_t rxTail;
  uint8_t parser[MOTOR_UART_MAX_FRAME_SIZE];
  uint8_t parserLength;
  uint8_t telemetrySequence;
  uint8_t lastCommandSequence;
  uint32_t lastCommandTick;
  uint32_t nextTelemetryTick;
  bool initialized;
  bool commandRejected;
  uint8_t initStage;
  uint8_t lastTxStatus;
  uint32_t uartError;
  uint32_t receivedBytes;
  uint32_t validCommandFrames;
  uint32_t crcErrors;
  uint32_t protocolErrors;
  uint32_t telemetryAttempts;
  uint32_t telemetrySent;
  uint32_t telemetryErrors;
} USART_STM_State_t;

static USART_STM_State_t motorUart;

/** @brief 计算协议使用的 CRC16/Modbus 校验值。 */
static uint16_t USART_STM_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

/** @brief 将 32 位遥测量饱和限制到 UART 协议的 16 位有符号字段范围。 */
static int16_t USART_STM_ClampS16(int32_t value)
{
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return (int16_t)value;
}

/** @brief 将多圈位置归一化到 [0, 36000) 的单圈 0.01° 表示。 */
static int32_t USART_STM_NormalizeCdeg(int32_t cdeg)
{
  cdeg %= 36000;
  return cdeg < 0 ? cdeg + 36000 : cdeg;
}

/** @brief 判断最近 300 ms 内是否收到过有效命令帧。 */
static bool USART_STM_LinkActive(void)
{
  return (motorUart.lastCommandTick != 0U) &&
         ((HAL_GetTick() - motorUart.lastCommandTick) <= MOTOR_UART_LINK_TIMEOUT_MS);
}

/** @brief 按旧 UART 固定 1000 ms 轨迹请求最近单圈位置目标。 */
static bool USART_STM_SetNearestSingleTurnPosition(int32_t targetCdeg)
{
  return CommMgr_STM_SetNearestPosition(targetCdeg, 1000U, 0U);
}

/** @brief 解码命令负载，并只通过 CommMgr_STM 请求电机动作。 */
static bool USART_STM_HandleCommand(const uint8_t *payload)
{
  const int32_t value = MotorUart_ReadS32(&payload[1]);
  switch ((MotorUart_Command_t)payload[0])
  {
    case MOTOR_UART_CMD_NOP:
    case MOTOR_UART_CMD_PING:
      return true;
    case MOTOR_UART_CMD_SET_MODE:
      if ((value != MOTOR_UART_MODE_SPEED) && (value != MOTOR_UART_MODE_POSITION)) return false;
      return CommMgr_STM_SetMode((MotorMode_STM)value);
    case MOTOR_UART_CMD_SET_SPEED_RPM:
      return CommMgr_STM_SetSpeed(USART_STM_ClampS16(value), 150U);
    case MOTOR_UART_CMD_SET_POSITION_CDEG:
      return USART_STM_SetNearestSingleTurnPosition(value);
    case MOTOR_UART_CMD_START:
      return CommMgr_STM_Start();
    case MOTOR_UART_CMD_STOP:
      return CommMgr_STM_Stop();
    case MOTOR_UART_CMD_ACK_FAULT:
      return CommMgr_STM_AcknowledgeFault();
    case MOTOR_UART_CMD_ZERO_POSITION:
    default:
      return false;
  }
}

/** @brief 接收并解析 USART_STM_ParseByte 对应的数据或通信帧。 */
static void USART_STM_ParseByte(uint8_t byte)
{
  if (motorUart.parserLength == 0U)
  {
    if (byte == MOTOR_UART_SOF0) motorUart.parser[motorUart.parserLength++] = byte;
    return;
  }
  if (motorUart.parserLength == 1U)
  {
    if (byte == MOTOR_UART_SOF1) motorUart.parser[motorUart.parserLength++] = byte;
    else motorUart.parserLength = byte == MOTOR_UART_SOF0 ? 1U : 0U;
    return;
  }
  if (motorUart.parserLength >= MOTOR_UART_MAX_FRAME_SIZE)
  {
    motorUart.parserLength = 0U;
    return;
  }
  motorUart.parser[motorUart.parserLength++] = byte;
  if (motorUart.parserLength < 6U) return;

  const uint8_t length = motorUart.parser[5];
  const uint16_t frameLength = (uint16_t)(8U + length);
  if (length > MOTOR_UART_MAX_PAYLOAD)
  {
    motorUart.protocolErrors++;
    motorUart.parserLength = 0U;
    return;
  }
  if (motorUart.parserLength < frameLength) return;

  if (MotorUart_ReadU16(&motorUart.parser[6U + length]) !=
      USART_STM_Crc16(&motorUart.parser[2], (uint16_t)(4U + length)))
  {
    motorUart.crcErrors++;
  }
  else if ((motorUart.parser[2] == MOTOR_UART_PROTOCOL_VERSION) &&
           (motorUart.parser[3] == MOTOR_UART_FRAME_COMMAND) &&
           (length == MOTOR_UART_COMMAND_PAYLOAD_SIZE))
  {
    motorUart.commandRejected = !USART_STM_HandleCommand(&motorUart.parser[6]);
    motorUart.lastCommandSequence = motorUart.parser[4];
    motorUart.lastCommandTick = HAL_GetTick();
    motorUart.validCommandFrames++;
  }
  else
  {
    motorUart.protocolErrors++;
  }
  motorUart.parserLength = 0U;
}

/** @brief 根据 DMA 剩余计数读取循环缓冲区新增字节，并逐字节解析。 */
static void USART_STM_ReadDma(void)
{
  const uint16_t head = (uint16_t)(
    MOTOR_UART_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx));

  while (motorUart.rxTail != head)
  {
    motorUart.receivedBytes++;
    USART_STM_ParseByte(motorUart.rxDmaBuffer[motorUart.rxTail]);
    motorUart.rxTail++;
    if (motorUart.rxTail >= MOTOR_UART_RX_DMA_SIZE)
    {
      motorUart.rxTail = 0U;
    }
  }
}

/** @brief 编码并发送固定 24 字节负载的兼容遥测帧，同时累计发送诊断。 */
static void USART_STM_SendTelemetry(void)
{
  MotorState_STM state;
  uint8_t frame[32] = {0U};
  uint8_t *payload = &frame[6];
  uint8_t flags = 0U;
  CommMgr_STM_GetMotorState(&state);
  if (state.motorRunning) flags |= MOTOR_UART_STATUS_MOTOR_RUNNING;
  if (state.currentFaults != 0U) flags |= MOTOR_UART_STATUS_MOTOR_FAULT;
  if (motorUart.commandRejected) flags |= MOTOR_UART_STATUS_COMMAND_REJECTED;
  if (USART_STM_LinkActive()) flags |= MOTOR_UART_STATUS_LINK_ACTIVE;
  payload[0] = flags;
  payload[1] = state.mode == MOTOR_MODE_STM_POSITION ? MOTOR_UART_MODE_POSITION : MOTOR_UART_MODE_SPEED;
  MotorUart_WriteU16(&payload[2], state.currentFaults);
  MotorUart_WriteS16(&payload[4], state.measuredSpeedRpm);
  MotorUart_WriteS16(&payload[6], state.speedReferenceRpm);
  MotorUart_WriteU16(&payload[8], (uint16_t)USART_STM_NormalizeCdeg(state.currentPositionCdeg));
  MotorUart_WriteU16(&payload[10], (uint16_t)USART_STM_NormalizeCdeg(state.targetPositionCdeg));
  MotorUart_WriteS16(&payload[12], USART_STM_ClampS16(state.targetPositionCdeg - state.currentPositionCdeg));
  MotorUart_WriteS16(&payload[14], USART_STM_ClampS16(state.iqMa));
  MotorUart_WriteS16(&payload[16], USART_STM_ClampS16(state.idMa));
  MotorUart_WriteS16(&payload[18], USART_STM_ClampS16(state.iqReferenceMa));
  frame[0] = MOTOR_UART_SOF0;
  frame[1] = MOTOR_UART_SOF1;
  frame[2] = MOTOR_UART_PROTOCOL_VERSION;
  frame[3] = MOTOR_UART_FRAME_TELEMETRY;
  frame[4] = motorUart.telemetrySequence++;
  frame[5] = MOTOR_UART_TELEMETRY_PAYLOAD_SIZE;
  MotorUart_WriteU16(&frame[30], USART_STM_Crc16(&frame[2], 28U));
  motorUart.telemetryAttempts++;
  const HAL_StatusTypeDef status =
    HAL_UART_Transmit(&huart3, frame, sizeof(frame), 2U);
  motorUart.lastTxStatus = (uint8_t)status;
  motorUart.uartError = HAL_UART_GetError(&huart3);
  if (status == HAL_OK)
  {
    motorUart.telemetrySent++;
  }
  else
  {
    motorUart.telemetryErrors++;
  }
}

/**
 * @brief 在 CubeMX 已初始化的 USART3 上启动循环 DMA 接收。
 * @note 波特率、PC10/PC11、DMA1 Channel3 和 FIFO 设置全部由 CubeMX 管理。
 */
bool USART_STM_Init(void)
{
  memset(&motorUart, 0, sizeof(motorUart));
  motorUart.initStage = 6U;
  motorUart.lastTxStatus = (uint8_t)HAL_UART_Receive_DMA(
    &huart3, motorUart.rxDmaBuffer, MOTOR_UART_RX_DMA_SIZE);
  if (motorUart.lastTxStatus != (uint8_t)HAL_OK)
  {
    motorUart.uartError = HAL_UART_GetError(&huart3);
    return false;
  }
  motorUart.nextTelemetryTick = HAL_GetTick() + MOTOR_UART_TELEMETRY_PERIOD_MS;
  motorUart.initialized = true;
  motorUart.initStage = 7U;
  return true;
}

/** @brief 解析循环 DMA 新数据，并按 20 ms 周期发送兼容遥测。 */
void USART_STM_Process(void)
{
  if (!motorUart.initialized) return;
  USART_STM_ReadDma();
  const uint32_t now = HAL_GetTick();
  /*
   * 保留已验证工程的上电遥测行为。收到有效 ESP32 命令前，LINK_ACTIVE 位仍为 0，
   * 但 ESP32 接收端可以据此区分回传线路缺失与命令内容错误。
   */
  if ((int32_t)(now - motorUart.nextTelemetryTick) >= 0)
  {
    motorUart.nextTelemetryTick = now + MOTOR_UART_TELEMETRY_PERIOD_MS;
    USART_STM_SendTelemetry();
  }
}

/** @brief 复制 USART 初始化、链路、CRC 和收发计数诊断。 */
void USART_STM_GetDiagnostics(USART_STM_Diagnostics_t *diagnostics)
{
  if (diagnostics == NULL) return;
  diagnostics->initialized = motorUart.initialized;
  diagnostics->linkActive = USART_STM_LinkActive();
  diagnostics->initStage = motorUart.initStage;
  diagnostics->lastTxStatus = motorUart.lastTxStatus;
  diagnostics->uartError = motorUart.uartError;
  diagnostics->receivedBytes = motorUart.receivedBytes;
  diagnostics->validCommandFrames = motorUart.validCommandFrames;
  diagnostics->crcErrors = motorUart.crcErrors;
  diagnostics->protocolErrors = motorUart.protocolErrors;
  diagnostics->telemetryAttempts = motorUart.telemetryAttempts;
  diagnostics->telemetrySent = motorUart.telemetrySent;
  diagnostics->telemetryErrors = motorUart.telemetryErrors;
}
