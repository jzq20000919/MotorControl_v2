#include "mqtt_motor_gateway.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "comm_mgr_ESP.h"
#include "mqtt_manager.h"

#define MQTT_GATEWAY_QUEUE_LENGTH       8U
#define MQTT_GATEWAY_COMMAND_MAX_LEN    255U
#define MQTT_GATEWAY_TASK_STACK_SIZE    5120U
#define MQTT_GATEWAY_TASK_PRIORITY      5U
#define MQTT_GATEWAY_TELEMETRY_PERIOD_MS 250U
#define MQTT_GATEWAY_DEFAULT_UART_BAUD  115200U
#define MQTT_GATEWAY_SPEED_LIMIT_RPM    2600

typedef struct
{
    char payload[MQTT_GATEWAY_COMMAND_MAX_LEN + 1U];
} mqtt_gateway_command_t;

static const char *TAG = "MQTT_MOTOR";
static QueueHandle_t s_command_queue;
static TaskHandle_t s_gateway_task;

/** @brief 将有符号远程命令值限制在闭区间安全范围内。 */
static int32_t mqtt_gateway_clamp_i32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/**
 * @brief 为一次远程控制请求发布 JSON 确认消息。
 * @param command_id 客户端提供的关联 ID；未提供时为零。
 * @param accepted 命令通过网关校验时为 true，否则为 false。
 * @param message 写入 JSON 负载的简短诊断消息。
 */
static void mqtt_gateway_publish_ack(
    uint32_t command_id,
    bool accepted,
    const char *message)
{
    char payload[192];
    snprintf(payload, sizeof(payload), "{\"id\":%lu,\"ok\":%s,\"message\":\"%s\"}", (unsigned long)command_id, accepted ? "true" : "false", message != NULL ? message : "");
    (void)mqtt_manager_publish(MQTT_MOTOR_ACK_TOPIC, payload);
}

/**
 * @brief 在允许影响电机的命令前验证 STM32 链路在线。
 * @return 当前 CommMgr_ESP 状态快照表示链路在线时返回 true。
 */
static bool mqtt_gateway_require_link(
    const CommMgr_ESP_State *snapshot,
    uint32_t command_id)
{
    if (snapshot->link_active) {
        return true;
    }
    mqtt_gateway_publish_ack(command_id, false, "STM32 link offline");
    return false;
}

/**
 * @brief 从网关约束的 JSON 格式中提取简单整数字段。
 * @return @p key 存在且能够解析为十进制有符号整数时返回 true。
 */
static bool mqtt_gateway_json_int(
    const char *json,
    const char *key,
    int32_t *value)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *cursor = strstr(json, pattern);
    if (cursor == NULL) {
        return false;
    }
    cursor = strchr(cursor + strlen(pattern), ':');
    if (cursor == NULL) {
        return false;
    }
    cursor++;
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    char *end = NULL;
    const long parsed = strtol(cursor, &end, 10);
    if (end == cursor) {
        return false;
    }
    *value = (int32_t)parsed;
    return true;
}

/**
 * @brief 从网关约束的 JSON 格式中提取简单带引号字符串字段。
 * @return @p key 存在且字段值能够完整写入目标缓冲区时返回 true。
 */
static bool mqtt_gateway_json_string(
    const char *json,
    const char *key,
    char *value,
    size_t value_size)
{
    if (value_size == 0U) {
        return false;
    }
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *cursor = strstr(json, pattern);
    if (cursor == NULL) {
        return false;
    }
    cursor = strchr(cursor + strlen(pattern), ':');
    if (cursor == NULL) {
        return false;
    }
    cursor++;
    while (*cursor == ' ' || *cursor == '\t') {
        cursor++;
    }
    if (*cursor++ != '\"') {
        return false;
    }
    const char *end = strchr(cursor, '\"');
    if (end == NULL) {
        return false;
    }
    size_t length = (size_t)(end - cursor);
    if (length >= value_size) {
        length = value_size - 1U;
    }
    memcpy(value, cursor, length);
    value[length] = '\0';
    return true;
}

/**
 * @brief 校验远程 JSON 命令，并映射为 CommMgr_ESP API 调用。
 *
 * 此处被有意设计为唯一能够通过 MQTT 发出电机动作的路径；它会检查链路是否
 * 可用、限制速度范围并对角度目标进行归一化。
 * @param payload 以空字符结尾的完整 JSON 命令消息。
 */
static void mqtt_gateway_process_command(const char *payload)
{
    char command[32];
    int32_t id_value = 0;
    int32_t value = 0;
    const bool has_value =
        mqtt_gateway_json_int(payload, "value", &value);
    (void)mqtt_gateway_json_int(payload, "id", &id_value);
    const uint32_t command_id = id_value > 0 ? (uint32_t)id_value : 0U;

    if (!mqtt_gateway_json_string(payload, "cmd", command, sizeof(command))) {
        mqtt_gateway_publish_ack(command_id, false, "Missing command");
        return;
    }

    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);

    if (strcmp(command, "claim") == 0) {
        esp_err_t result = ESP_OK;
        if (snapshot.transport == COMM_MGR_ESP_NONE) {
            result = CommMgr_ESP_SelectUSART(MQTT_GATEWAY_DEFAULT_UART_BAUD);
        }
        mqtt_gateway_publish_ack(command_id, result == ESP_OK, result == ESP_OK ? "Gateway ready" : "UART activation failed");
    } else if (strcmp(command, "set_mode") == 0) {
        if (!has_value) {
            mqtt_gateway_publish_ack(command_id, false, "Missing mode value");
        } else if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_SetMode(value == 0 ? COMM_MGR_ESP_MODE_SPEED : COMM_MGR_ESP_MODE_POSITION);
            mqtt_gateway_publish_ack(command_id, true, "Mode accepted");
        }
    } else if (strcmp(command, "set_speed") == 0) {
        if (!has_value) {
            mqtt_gateway_publish_ack(command_id, false, "Missing speed value");
        } else if (mqtt_gateway_require_link(&snapshot, command_id)) {
            const int32_t speed = mqtt_gateway_clamp_i32(value, -MQTT_GATEWAY_SPEED_LIMIT_RPM, MQTT_GATEWAY_SPEED_LIMIT_RPM);
            CommMgr_ESP_SetMode(COMM_MGR_ESP_MODE_SPEED);
            CommMgr_ESP_SetSpeedRPM((int16_t)speed);
            mqtt_gateway_publish_ack(command_id, true, "Speed accepted");
        }
    } else if (strcmp(command, "set_position") == 0) {
        if (!has_value) {
            mqtt_gateway_publish_ack(command_id, false, "Missing position value");
        } else if (mqtt_gateway_require_link(&snapshot, command_id)) {
            int32_t position = value % 36000;
            if (position < 0) {
                position += 36000;
            }
            CommMgr_ESP_SetMode(COMM_MGR_ESP_MODE_POSITION);
            CommMgr_ESP_SetPositionCdeg((uint16_t)position);
            mqtt_gateway_publish_ack(command_id, true, "Position accepted");
        }
    } else if (strcmp(command, "start") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_Start();
            mqtt_gateway_publish_ack(command_id, true, "Start accepted");
        }
    } else if (strcmp(command, "stop") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_Stop();
            mqtt_gateway_publish_ack(command_id, true, "Stop accepted");
        }
    } else if (strcmp(command, "ack_fault") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_AcknowledgeFault();
            mqtt_gateway_publish_ack(command_id, true, "Fault reset accepted");
        }
    } else if (strcmp(command, "zero_position") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_ZeroPosition();
            mqtt_gateway_publish_ack(command_id, true, "Zero accepted");
        }
    } else {
        mqtt_gateway_publish_ack(command_id, false, "Unknown command");
    }

}

/**
 * @brief MQTT 回调：将控制消息复制到网关工作队列。
 *
 * 此回调运行在 MQTT 事件上下文中，因此不会直接操作 CommMgr_ESP。队列已满时
 * 丢弃最旧请求，以优先保留操作者的最新命令。
 */
static void mqtt_gateway_message_callback(
    const char *topic,
    const char *payload,
    void *context)
{
    (void)context;
    if (topic == NULL || payload == NULL ||
        strcmp(topic, MQTT_MANAGER_CONTROL_TOPIC) != 0 ||
        s_command_queue == NULL) {
        return;
    }

    mqtt_gateway_command_t command;
    strlcpy(command.payload, payload, sizeof(command.payload));
    if (xQueueSend(s_command_queue, &command, 0U) != pdTRUE) {
        mqtt_gateway_command_t discarded;
        (void)xQueueReceive(s_command_queue, &discarded, 0U);
        (void)xQueueSend(s_command_queue, &command, 0U);
    }
}

/**
 * @brief 将统一电机状态快照序列化为 JSON 遥测发布消息。
 */
static void mqtt_gateway_publish_telemetry(void)
{
    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);

    char payload[512];
    snprintf(payload, sizeof(payload), "{\"version\":1,\"transport\":%u,\"uart_online\":%s," "\"can_online\":%s,\"link_active\":%s,\"running\":%s," "\"motor_fault\":%s,\"command_rejected\":%s,\"mode\":%u," "\"faults\":%u,\"speed_rpm\":%d,\"speed_ref_rpm\":%d," "\"position_cdeg\":%u,\"target_cdeg\":%u," "\"position_error_cdeg\":%d,\"iq_ma\":%d,\"id_ma\":%d," "\"iq_ref_ma\":%d,\"id_ref_ma\":%d,\"uq_mv\":%d," "\"ud_mv\":%d,\"rx_frames\":%lu,\"tx_frames\":%lu," "\"tx_errors\":%lu}", (unsigned)snapshot.transport, snapshot.uart_link_active ? "true" : "false", snapshot.can_link_active ? "true" : "false", snapshot.link_active ? "true" : "false", snapshot.motor_running ? "true" : "false", snapshot.motor_fault ? "true" : "false", snapshot.command_rejected ? "true" : "false", (unsigned)snapshot.mode, (unsigned)snapshot.faults, snapshot.measured_speed_rpm, snapshot.reference_speed_rpm, (unsigned)snapshot.current_position_cdeg, (unsigned)snapshot.target_position_cdeg, snapshot.position_error_cdeg, snapshot.iq_ma, snapshot.id_ma, snapshot.iq_reference_ma, snapshot.id_reference_ma, snapshot.uq_mv, snapshot.ud_mv, (unsigned long)snapshot.received_frames, (unsigned long)snapshot.transmitted_frames, (unsigned long)snapshot.transmit_errors);
    (void)mqtt_manager_publish_qos0(MQTT_MOTOR_TELEMETRY_TOPIC, payload);
}

/**
 * @brief 处理排队的远程命令，并周期发布电机遥测。
 * @param argument 未使用的任务参数。
 */
static void mqtt_gateway_task(void *argument)
{
    (void)argument;
    TickType_t last_publish = xTaskGetTickCount();
    mqtt_gateway_command_t command;

    while (true) {
        if (xQueueReceive(s_command_queue, &command, pdMS_TO_TICKS(20U)) == pdTRUE) {
            mqtt_gateway_process_command(command.payload);
        }
        if (xTaskGetTickCount() - last_publish >=
            pdMS_TO_TICKS(MQTT_GATEWAY_TELEMETRY_PERIOD_MS)) {
            last_publish = xTaskGetTickCount();
            mqtt_gateway_publish_telemetry();
        }
    }
}

/**
 * @brief 创建 MQTT 电机网关队列、注册回调并创建工作任务。
 * @return 成功时返回 ESP_OK；队列或任务分配失败时返回 ESP_ERR_NO_MEM。
 */
esp_err_t mqtt_motor_gateway_init(void)
{
    if (s_gateway_task != NULL) {
        return ESP_OK;
    }
    s_command_queue = xQueueCreate(MQTT_GATEWAY_QUEUE_LENGTH, sizeof(mqtt_gateway_command_t));
    if (s_command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    mqtt_manager_set_message_callback(mqtt_gateway_message_callback, NULL);
    if (xTaskCreate(mqtt_gateway_task, "mqtt_motor", MQTT_GATEWAY_TASK_STACK_SIZE, NULL, MQTT_GATEWAY_TASK_PRIORITY, &s_gateway_task) != pdPASS) {
        mqtt_manager_set_message_callback(NULL, NULL);
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "MQTT motor gateway ready");
    return ESP_OK;
}
