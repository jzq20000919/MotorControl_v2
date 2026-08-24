/* ESP32 HMI 与 STM32 电机板共用的 USART3 协议；须与电机工程中的副本保持一致。 */

#ifndef MOTOR_UART_PROTOCOL_H
#define MOTOR_UART_PROTOCOL_H

#include <stdint.h>

/** @brief UART 电机协议版本号。 */
#define MOTOR_UART_PROTOCOL_VERSION       (1U)
/** @brief UART 电机链路默认波特率。 */
#define MOTOR_UART_BAUD_RATE              (115200U)
/** @brief UART 帧头第一个同步字节。 */
#define MOTOR_UART_SOF0                   (0xA5U)
/** @brief UART 帧头第二个同步字节。 */
#define MOTOR_UART_SOF1                   (0x5AU)
/** @brief UART 单帧允许的最大负载字节数。 */
#define MOTOR_UART_MAX_PAYLOAD            (32U)
/** @brief 帧头、长度与 CRC 等非负载字段占用的字节数。 */
#define MOTOR_UART_FRAME_OVERHEAD         (8U)
/** @brief 包含协议开销的 UART 完整帧最大字节数。 */
#define MOTOR_UART_MAX_FRAME_SIZE         \
  (MOTOR_UART_MAX_PAYLOAD + MOTOR_UART_FRAME_OVERHEAD)

/** @brief UART 协议帧类型。 */
typedef enum
{
  MOTOR_UART_FRAME_COMMAND = 1,   /**< ESP32 发往电机板的命令帧。 */
  MOTOR_UART_FRAME_TELEMETRY = 2  /**< 电机板发往 ESP32 的遥测帧。 */
} MotorUart_FrameType_t;

/** @brief UART 电机控制命令码。 */
typedef enum
{
  MOTOR_UART_CMD_NOP = 0,               /**< 空操作命令。 */
  MOTOR_UART_CMD_SET_MODE = 1,          /**< 设置速度或位置控制模式。 */
  MOTOR_UART_CMD_SET_SPEED_RPM = 2,     /**< 设置机械转速目标，单位 rpm。 */
  MOTOR_UART_CMD_SET_POSITION_CDEG = 3, /**< 设置位置目标，单位 0.01°。 */
  MOTOR_UART_CMD_START = 4,             /**< 启动电机。 */
  MOTOR_UART_CMD_STOP = 5,              /**< 停止电机。 */
  MOTOR_UART_CMD_ACK_FAULT = 6,         /**< 确认并复位电机故障。 */
  MOTOR_UART_CMD_ZERO_POSITION = 7,     /**< 请求将当前位置设为零点。 */
  MOTOR_UART_CMD_PING = 8               /**< 链路心跳命令。 */
} MotorUart_Command_t;

/** @brief UART 控制模式编码。 */
typedef enum
{
  MOTOR_UART_MODE_SPEED = 0,   /**< 速度闭环控制模式。 */
  MOTOR_UART_MODE_POSITION = 1 /**< 位置闭环控制模式。 */
} MotorUart_Mode_t;

/** @brief 遥测状态中“电机正在运行”的位掩码。 */
#define MOTOR_UART_STATUS_MOTOR_RUNNING       (1U << 0)
/** @brief 遥测状态中“电机存在故障”的位掩码。 */
#define MOTOR_UART_STATUS_MOTOR_FAULT         (1U << 1)
/** @brief 遥测状态中“上一命令被拒绝”的位掩码。 */
#define MOTOR_UART_STATUS_COMMAND_REJECTED    (1U << 2)
/** @brief 遥测状态中“控制链路在线”的位掩码。 */
#define MOTOR_UART_STATUS_LINK_ACTIVE         (1U << 3)

/** @brief UART 命令负载的固定字节数。 */
#define MOTOR_UART_COMMAND_PAYLOAD_SIZE        (5U)
/** @brief UART 遥测负载的固定字节数。 */
#define MOTOR_UART_TELEMETRY_PAYLOAD_SIZE      (24U)

/**
 * @brief 从小端字节流读取无符号 16 位整数。
 * @param data 指向至少 2 字节输入数据的指针。
 * @return 解码后的无符号 16 位值。
 */
static inline uint16_t MotorUart_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0]) |
                    ((uint16_t)data[1] << 8U));
}

/**
 * @brief 从小端字节流读取有符号 16 位整数。
 * @param data 指向至少 2 字节输入数据的指针。
 * @return 解码后的有符号 16 位值。
 */
static inline int16_t MotorUart_ReadS16(const uint8_t *data)
{
  return (int16_t)MotorUart_ReadU16(data);
}

/**
 * @brief 从小端字节流读取无符号 32 位整数。
 * @param data 指向至少 4 字节输入数据的指针。
 * @return 解码后的无符号 32 位值。
 */
static inline uint32_t MotorUart_ReadU32(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

/**
 * @brief 从小端字节流读取有符号 32 位整数。
 * @param data 指向至少 4 字节输入数据的指针。
 * @return 解码后的有符号 32 位值。
 */
static inline int32_t MotorUart_ReadS32(const uint8_t *data)
{
  return (int32_t)MotorUart_ReadU32(data);
}

/**
 * @brief 将无符号 16 位整数写入小端字节流。
 * @param[out] data 指向至少 2 字节输出缓冲区的指针。
 * @param value 待编码的无符号 16 位值。
 */
static inline void MotorUart_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/**
 * @brief 将有符号 16 位整数写入小端字节流。
 * @param[out] data 指向至少 2 字节输出缓冲区的指针。
 * @param value 待编码的有符号 16 位值。
 */
static inline void MotorUart_WriteS16(uint8_t *data, int16_t value)
{
  MotorUart_WriteU16(data, (uint16_t)value);
}

/**
 * @brief 将无符号 32 位整数写入小端字节流。
 * @param[out] data 指向至少 4 字节输出缓冲区的指针。
 * @param value 待编码的无符号 32 位值。
 */
static inline void MotorUart_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief 将有符号 32 位整数写入小端字节流。
 * @param[out] data 指向至少 4 字节输出缓冲区的指针。
 * @param value 待编码的有符号 32 位值。
 */
static inline void MotorUart_WriteS32(uint8_t *data, int32_t value)
{
  MotorUart_WriteU32(data, (uint32_t)value);
}

#endif /* MOTOR_UART_PROTOCOL_H */
