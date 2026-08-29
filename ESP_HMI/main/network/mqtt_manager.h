#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/** @brief MQTT Broker URI 的最大有效字符数。 */
#define MQTT_MANAGER_URI_MAX_LEN 95U
/** @brief MQTT 电机控制命令订阅主题。 */
#define MQTT_MANAGER_CONTROL_TOPIC "motor/control/command"

/**
 * @brief 完整 MQTT 入站消息回调类型。
 * @param topic 收到消息的主题字符串。
 * @param payload 收到的消息负载字符串。
 * @param context 注册回调时提供的用户上下文。
 */
typedef void (*mqtt_manager_message_callback_t)(
    const char *topic,
    const char *payload,
    void *context);

/** @brief MQTT 管理器的线程安全连接快照。 */
typedef struct
{
    bool initialized;                         /**< 管理任务和同步对象已初始化。 */
    bool connecting;                          /**< MQTT 客户端正在连接 Broker。 */
    bool connected;                           /**< MQTT 客户端已连接 Broker。 */
    uint32_t revision;                        /**< 状态变化时递增的版本号。 */
    char broker_uri[MQTT_MANAGER_URI_MAX_LEN + 1U]; /**< 当前 Broker URI。 */
    char status[96];                          /**< 面向界面显示的状态文本。 */
} mqtt_manager_snapshot_t;

/** @brief 初始化互斥锁、队列和 MQTT 管理工作任务。 @return 成功返回 ESP_OK。 */
esp_err_t mqtt_manager_init(void);
/** @brief 排队一个非阻塞连接请求。 @param broker_uri 目标 Broker URI。 @return 请求入队结果。 */
esp_err_t mqtt_manager_connect_async(const char *broker_uri);
/** @brief 排队一个非阻塞 MQTT 客户端断开请求。 @return 请求入队结果。 */
esp_err_t mqtt_manager_disconnect_async(void);
/**
 * @brief 使用管理器默认服务质量策略发布消息。
 * @param topic 目标主题。
 * @param payload 以空字符结尾的消息负载。
 * @return 消息成功提交时返回 ESP_OK。
 */
esp_err_t mqtt_manager_publish(
    const char *topic,
    const char *payload);
/** @brief 发布 QoS 1 文本并返回可用于查询 PUBACK 的消息 ID。 */
esp_err_t mqtt_manager_publish_tracked(
    const char *topic,
    const char *payload,
    int *message_id);
/**
 * @brief 发布一条即发即弃的 QoS 0 消息。
 * @param topic 目标主题。
 * @param payload 以空字符结尾的消息负载。
 * @return 消息成功提交时返回 ESP_OK。
 */
esp_err_t mqtt_manager_publish_qos0(
    const char *topic,
    const char *payload);
/** @brief 发布 QoS 1 二进制数据并返回可用于查询 PUBACK 的消息 ID。 */
esp_err_t mqtt_manager_publish_binary_qos1_tracked(
    const char *topic,
    const void *payload,
    size_t payload_length,
    int *message_id);
/** @brief 查询指定 QoS 1 消息是否已经收到 Broker PUBACK。 */
bool mqtt_manager_is_message_delivered(int message_id);
/**
 * @brief 注册接收完整 MQTT 入站消息的应用回调。
 * @param callback 消息回调；传 NULL 可取消注册。
 * @param context 原样传给回调的用户上下文。
 */
void mqtt_manager_set_message_callback(
    mqtt_manager_message_callback_t callback,
    void *context);
/** @brief 复制最新的线程安全 MQTT 连接快照。 @param[out] snapshot 接收快照的对象。 */
void mqtt_manager_get_snapshot(mqtt_manager_snapshot_t *snapshot);

#endif /* MQTT_MANAGER_H */
