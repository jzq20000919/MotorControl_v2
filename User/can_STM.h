#ifndef CAN_STM_H
#define CAN_STM_H

#include <stdbool.h>

/** @brief 初始化 FDCAN1、命令过滤器和通信诊断状态。 @return 外设启动成功返回 true。 */
bool CAN_STM_Init(void);
/** @brief 处理接收命令、Bus-Off 恢复、初始化重试及 20 ms 遥测发送。 */
void CAN_STM_Tick(void);

#endif /* CAN_STM_H */
