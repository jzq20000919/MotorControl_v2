/* ESP32 HMI 与 STM32 电机板共用的经典 CAN 协议；须与电机工程中的副本保持一致。 */

#ifndef MOTOR_CAN_PROTOCOL_H
#define MOTOR_CAN_PROTOCOL_H

#include <stdint.h>

/** @brief CAN 电机协议版本号。 */
#define MOTOR_CAN_PROTOCOL_VERSION          (1U)
/** @brief CAN 总线默认比特率，单位 bit/s。 */
#define MOTOR_CAN_BITRATE                   (500000U)

/** @brief ESP32 发往电机板的命令帧标准标识符。 */
#define MOTOR_CAN_ID_COMMAND                (0x100U)
/** @brief 电机运行状态遥测帧标准标识符。 */
#define MOTOR_CAN_ID_STATUS                 (0x180U)
/** @brief 速度与位置参考值遥测帧标准标识符。 */
#define MOTOR_CAN_ID_REFERENCES             (0x181U)
/** @brief 电流与电压遥测帧标准标识符。 */
#define MOTOR_CAN_ID_ELECTRICAL             (0x182U)

/** @brief 经典 CAN 数据帧固定字节数。 */
#define MOTOR_CAN_FRAME_SIZE                (8U)

/** @brief CAN 电机控制命令码。 */
typedef enum
{
  MOTOR_CAN_CMD_NOP = 0,               /**< 空操作命令。 */
  MOTOR_CAN_CMD_SET_MODE = 1,          /**< 设置速度或位置控制模式。 */
  MOTOR_CAN_CMD_SET_SPEED_RPM = 2,     /**< 设置机械转速目标，单位 rpm。 */
  MOTOR_CAN_CMD_SET_POSITION_CDEG = 3, /**< 设置位置目标，单位 0.01°。 */
  MOTOR_CAN_CMD_START = 4,             /**< 启动电机。 */
  MOTOR_CAN_CMD_STOP = 5,              /**< 停止电机。 */
  MOTOR_CAN_CMD_ACK_FAULT = 6,         /**< 确认并复位电机故障。 */
  MOTOR_CAN_CMD_ZERO_POSITION = 7,     /**< 请求将当前位置设为零点。 */
  MOTOR_CAN_CMD_PING = 8,              /**< 链路心跳命令。 */
  MOTOR_CAN_CMD_SET_PID_GAIN = 9       /**< 临时设置一个 PID 增益。 */
} MotorCan_Command_t;

/** @brief CAN 控制模式编码。 */
typedef enum
{
  MOTOR_CAN_MODE_SPEED = 0,   /**< 速度闭环控制模式。 */
  MOTOR_CAN_MODE_POSITION = 1 /**< 位置闭环控制模式。 */
} MotorCan_Mode_t;

/** @brief 状态帧中“当前为位置模式”的位掩码。 */
#define MOTOR_CAN_STATUS_POSITION_MODE      (1U << 0)
/** @brief 状态帧中“电机正在运行”的位掩码。 */
#define MOTOR_CAN_STATUS_MOTOR_RUNNING      (1U << 1)
/** @brief 状态帧中“电机存在故障”的位掩码。 */
#define MOTOR_CAN_STATUS_MOTOR_FAULT        (1U << 2)
/** @brief 状态帧中“控制链路在线”的位掩码。 */
#define MOTOR_CAN_STATUS_LINK_ACTIVE        (1U << 3)
/** @brief 状态帧中“上一命令被拒绝”的位掩码。 */
#define MOTOR_CAN_STATUS_COMMAND_REJECTED   (1U << 4)

/**
 * @brief 从小端字节流读取无符号 16 位整数。
 * @param data 指向至少 2 字节输入数据的指针。
 * @return 解码后的无符号 16 位值。
 */
static inline uint16_t MotorCan_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0]) |
                    ((uint16_t)data[1] << 8U));
}

/**
 * @brief 从小端字节流读取有符号 16 位整数。
 * @param data 指向至少 2 字节输入数据的指针。
 * @return 解码后的有符号 16 位值。
 */
static inline int16_t MotorCan_ReadS16(const uint8_t *data)
{
  return (int16_t)MotorCan_ReadU16(data);
}

/**
 * @brief 从小端字节流读取无符号 32 位整数。
 * @param data 指向至少 4 字节输入数据的指针。
 * @return 解码后的无符号 32 位值。
 */
static inline uint32_t MotorCan_ReadU32(const uint8_t *data)
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
static inline int32_t MotorCan_ReadS32(const uint8_t *data)
{
  return (int32_t)MotorCan_ReadU32(data);
}

/**
 * @brief 将无符号 16 位整数写入小端字节流。
 * @param[out] data 指向至少 2 字节输出缓冲区的指针。
 * @param value 待编码的无符号 16 位值。
 */
static inline void MotorCan_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/**
 * @brief 将有符号 16 位整数写入小端字节流。
 * @param[out] data 指向至少 2 字节输出缓冲区的指针。
 * @param value 待编码的有符号 16 位值。
 */
static inline void MotorCan_WriteS16(uint8_t *data, int16_t value)
{
  MotorCan_WriteU16(data, (uint16_t)value);
}

/**
 * @brief 将无符号 32 位整数写入小端字节流。
 * @param[out] data 指向至少 4 字节输出缓冲区的指针。
 * @param value 待编码的无符号 32 位值。
 */
static inline void MotorCan_WriteU32(uint8_t *data, uint32_t value)
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
static inline void MotorCan_WriteS32(uint8_t *data, int32_t value)
{
  MotorCan_WriteU32(data, (uint32_t)value);
}

#endif /* MOTOR_CAN_PROTOCOL_H */
