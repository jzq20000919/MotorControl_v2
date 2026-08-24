#ifndef USART_STM_H
#define USART_STM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool initialized;
  bool linkActive;
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
} USART_STM_Diagnostics_t;

/** @brief 初始化 USART3 与循环 RX DMA。 @return 外设及 DMA 均启动成功时返回 true。 */
bool USART_STM_Init(void);
/** @brief 非阻塞解析 DMA 接收数据并按周期发送遥测。 */
void USART_STM_Process(void);
/** @brief 复制 USART 链路与协议诊断计数。 @param[out] diagnostics 诊断接收对象。 */
void USART_STM_GetDiagnostics(USART_STM_Diagnostics_t *diagnostics);

#endif /* USART_STM_H */
