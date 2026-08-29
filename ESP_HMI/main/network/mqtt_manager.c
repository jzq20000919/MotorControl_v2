#include "mqtt_manager.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#define MQTT_MANAGER_COMMAND_QUEUE_LENGTH 4U
#define MQTT_MANAGER_WORKER_STACK_SIZE 4096U
#define MQTT_MANAGER_WORKER_PRIORITY 4U
#define MQTT_MANAGER_OUTBOX_LIMIT_BYTES 8192U
#define MQTT_MANAGER_DELIVERY_HISTORY 32U
#define MQTT_MANAGER_TOPIC_MAX_LEN 63U
#define MQTT_MANAGER_PAYLOAD_MAX_LEN 511U

typedef enum
{
    MQTT_MANAGER_COMMAND_CONNECT = 0,
    MQTT_MANAGER_COMMAND_DISCONNECT,
} mqtt_manager_command_type_t;

typedef struct
{
    mqtt_manager_command_type_t type;
    char broker_uri[MQTT_MANAGER_URI_MAX_LEN + 1U];
} mqtt_manager_command_t;

static const char *TAG = "MQTT_MANAGER";

static SemaphoreHandle_t s_lock;
/*
 * 用于串行化客户端 API 调用与客户端停止/销毁操作。该锁必须独立于 s_lock：
 * ESP-MQTT 持有自身 API 锁时会分发回调，而回调会在 s_lock 保护下更新
 * s_snapshot。
 */
static SemaphoreHandle_t s_client_api_lock;
static QueueHandle_t s_command_queue;
static esp_mqtt_client_handle_t s_client;
static mqtt_manager_snapshot_t s_snapshot;
static char s_active_uri[MQTT_MANAGER_URI_MAX_LEN + 1U];
static char s_client_id[32];
static char s_last_error[80];
static char s_incoming_topic[MQTT_MANAGER_TOPIC_MAX_LEN + 1U];
static char s_incoming_payload[MQTT_MANAGER_PAYLOAD_MAX_LEN + 1U];
static mqtt_manager_message_callback_t s_message_callback;
static void *s_message_callback_context;
static int s_delivered_message_ids[MQTT_MANAGER_DELIVERY_HISTORY];
static uint8_t s_delivered_message_cursor;

/** @brief 清空当前 MQTT 客户端对应的 PUBACK 历史。调用者必须持有 s_lock。 */
static void mqtt_manager_clear_delivery_history_locked(void)
{
    memset(s_delivered_message_ids, 0, sizeof(s_delivered_message_ids));
    s_delivered_message_cursor = 0U;
}

/** @brief 获取保护 MQTT 状态和回调注册信息的互斥锁。 */
static void mqtt_manager_lock(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

/** @brief 释放保护 MQTT 状态和回调注册信息的互斥锁。 */
static void mqtt_manager_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

/** @brief 串行化 ESP-MQTT 客户端 API 调用和客户端销毁操作。 */
static void mqtt_manager_client_api_lock(void)
{
    if (s_client_api_lock != NULL) {
        xSemaphoreTake(s_client_api_lock, portMAX_DELAY);
    }
}

/** @brief 释放 ESP-MQTT 客户端 API 串行化互斥锁。 */
static void mqtt_manager_client_api_unlock(void)
{
    if (s_client_api_lock != NULL) {
        xSemaphoreGive(s_client_api_lock);
    }
}

/** @brief 设置状态文本并递增版本号；调用者必须持有 s_lock。 */
static void mqtt_manager_set_status_locked(const char *status)
{
    strlcpy(s_snapshot.status, status, sizeof(s_snapshot.status));
    s_snapshot.revision++;
}

/** @brief 将非空终止的 ESP-MQTT 事件文本复制为安全 C 字符串。 */
static void mqtt_manager_copy_event_text(
    char *destination,
    size_t destination_size,
    const char *source,
    int source_length)
{
    if (destination_size == 0U) {
        return;
    }
    if (source == NULL || source_length <= 0) {
        destination[0] = '\0';
        return;
    }
    size_t length = (size_t)source_length;
    if (length >= destination_size) {
        length = destination_size - 1U;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
}

/**
 * @brief 将 ESP-MQTT 生命周期/数据事件转换为管理器状态和回调。
 *
 * MQTT 库的数据缓冲区仅在回调执行期间有效，因此事件负载文本必须先复制，
 * 再提供给管理器外部使用。
 */
static void mqtt_manager_event_handler(
    void *handler_argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)handler_argument;
    (void)event_base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_BEFORE_CONNECT:
        mqtt_manager_lock();
        s_snapshot.connecting = true;
        mqtt_manager_set_status_locked("Connecting to broker...");
        mqtt_manager_unlock();
        break;

    case MQTT_EVENT_CONNECTED:
        mqtt_manager_lock();
        s_snapshot.connecting = false;
        s_snapshot.connected = true;
        s_last_error[0] = '\0';
        mqtt_manager_clear_delivery_history_locked();
        mqtt_manager_set_status_locked("Connected");
        mqtt_manager_unlock();
        if (event != NULL) {
            (void)esp_mqtt_client_subscribe(event->client, MQTT_MANAGER_CONTROL_TOPIC, 1);
        }
        ESP_LOGI(TAG, "Connected to %s", s_active_uri);
        break;

    case MQTT_EVENT_DISCONNECTED:
        mqtt_manager_lock();
        s_snapshot.connected = false;
        s_snapshot.connecting = true;
        if (s_last_error[0] != '\0') {
            snprintf(s_snapshot.status, sizeof(s_snapshot.status), "%s; retrying in 3s", s_last_error);
            s_snapshot.revision++;
        } else {
            mqtt_manager_set_status_locked("MQTT link lost; retrying in 3s");
        }
        mqtt_manager_unlock();
        ESP_LOGW(TAG, "MQTT disconnected%s%s", s_last_error[0] != '\0' ? ": " : "", s_last_error);
        break;

    case MQTT_EVENT_PUBLISHED:
        mqtt_manager_lock();
        if (event != NULL && event->msg_id > 0) {
            s_delivered_message_ids[s_delivered_message_cursor] =
                event->msg_id;
            s_delivered_message_cursor = (uint8_t)(
                (s_delivered_message_cursor + 1U) %
                MQTT_MANAGER_DELIVERY_HISTORY);
        }
        mqtt_manager_unlock();
        break;

    case MQTT_EVENT_DATA: {
        if (event == NULL) {
            break;
        }
        bool message_complete = false;
        mqtt_manager_message_callback_t callback = NULL;
        void *callback_context = NULL;
        char completed_topic[MQTT_MANAGER_TOPIC_MAX_LEN + 1U];
        char completed_payload[MQTT_MANAGER_PAYLOAD_MAX_LEN + 1U];
        mqtt_manager_lock();
        if (event->current_data_offset == 0) {
            mqtt_manager_copy_event_text(
                s_incoming_topic, sizeof(s_incoming_topic),
                event->topic, event->topic_len);
            s_incoming_payload[0] = '\0';
        }
        if (event->data != NULL && event->data_len > 0 &&
            event->current_data_offset <
                (int)sizeof(s_incoming_payload) - 1) {
            size_t offset = (size_t)event->current_data_offset;
            size_t length = (size_t)event->data_len;
            const size_t available =
                sizeof(s_incoming_payload) - 1U - offset;
            if (length > available) {
                length = available;
            }
            memcpy(s_incoming_payload + offset, event->data, length);
            s_incoming_payload[offset + length] = '\0';
        }
        if (event->current_data_offset + event->data_len >=
            event->total_data_len) {
            strlcpy(completed_topic, s_incoming_topic,
                    sizeof(completed_topic));
            strlcpy(completed_payload, s_incoming_payload,
                    sizeof(completed_payload));
            callback = s_message_callback;
            callback_context = s_message_callback_context;
            message_complete = true;
        }
        mqtt_manager_unlock();
        if (message_complete && callback != NULL) {
            callback(completed_topic, completed_payload, callback_context);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        mqtt_manager_lock();
        s_snapshot.connected = false;
        if (event != NULL && event->error_handle != NULL) {
            const esp_mqtt_error_codes_t *error = event->error_handle;
            if (error->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                snprintf(s_last_error, sizeof(s_last_error), "MQTT TCP sock=%d tls=0x%X stack=0x%X", error->esp_transport_sock_errno, (unsigned int)error->esp_tls_last_esp_err, (unsigned int)error->esp_tls_stack_err);
            } else if (
                error->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                snprintf(s_last_error, sizeof(s_last_error), "MQTT refused code=%d", (int)error->connect_return_code);
            } else {
                snprintf(s_last_error, sizeof(s_last_error), "MQTT error type=%d", (int)error->error_type);
            }
        } else {
            strlcpy(s_last_error, "MQTT connection error", sizeof(s_last_error));
        }
        strlcpy(s_snapshot.status, s_last_error, sizeof(s_snapshot.status));
        s_snapshot.revision++;
        mqtt_manager_unlock();
        ESP_LOGE(TAG, "%s", s_last_error);
        break;

    default:
        break;
    }
}

/**
 * @brief 停止并销毁活动 MQTT 客户端，避免与发布调用竞争。
 */
static void mqtt_manager_stop_client(void)
{
    mqtt_manager_client_api_lock();
    mqtt_manager_lock();
    esp_mqtt_client_handle_t client = s_client;
    s_client = NULL;
    mqtt_manager_unlock();

    if (client != NULL) {
        (void)esp_mqtt_client_stop(client);
        (void)esp_mqtt_client_destroy(client);
    }
    mqtt_manager_client_api_unlock();
}

/**
 * @brief 串行执行 MQTT 连接/断开命令的工作任务。
 * @param argument 未使用的任务参数。
 */
static void mqtt_manager_worker(void *argument)
{
    (void)argument;
    mqtt_manager_command_t command;

    while (true) {
        if (xQueueReceive(s_command_queue, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        mqtt_manager_stop_client();

        if (command.type == MQTT_MANAGER_COMMAND_DISCONNECT) {
            mqtt_manager_lock();
            s_snapshot.connecting = false;
            s_snapshot.connected = false;
            mqtt_manager_set_status_locked("Disconnected");
            mqtt_manager_unlock();
            continue;
        }

        strlcpy(s_active_uri, command.broker_uri, sizeof(s_active_uri));
        const esp_mqtt_client_config_t configuration = {
            .broker.address.uri = s_active_uri,
            .credentials.client_id = s_client_id,
            .session.keepalive = 30,
            .network.reconnect_timeout_ms = 3000,
            .outbox.limit = MQTT_MANAGER_OUTBOX_LIMIT_BYTES,
        };
        esp_mqtt_client_handle_t client =
            esp_mqtt_client_init(&configuration);
        if (client == NULL) {
            mqtt_manager_lock();
            s_snapshot.connecting = false;
            mqtt_manager_set_status_locked("MQTT client init failed");
            mqtt_manager_unlock();
            continue;
        }

        esp_err_t result = esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_manager_event_handler, NULL);
        if (result == ESP_OK) {
            mqtt_manager_lock();
            s_client = client;
            mqtt_manager_unlock();
            result = esp_mqtt_client_start(client);
        }
        if (result != ESP_OK) {
            mqtt_manager_lock();
            if (s_client == client) {
                s_client = NULL;
            }
            s_snapshot.connecting = false;
            snprintf(s_snapshot.status, sizeof(s_snapshot.status), "MQTT start failed: %s", esp_err_to_name(result));
            s_snapshot.revision++;
            mqtt_manager_unlock();
            (void)esp_mqtt_client_destroy(client);
        }
    }
}

/**
 * @brief 创建管理器同步对象、唯一客户端 ID 及工作任务。
 * @return 成功时返回 ESP_OK，否则返回 MAC 地址查询或资源分配错误码。
 */
esp_err_t mqtt_manager_init(void)
{
    if (s_lock != NULL) {
        return ESP_OK;
    }//检查锁是否存在，如果存在就可能初始化过了，则不需要初始化

    uint8_t station_mac[6];//创建一个mac变量存放ESP32的mac地址
    const esp_err_t mac_result =esp_read_mac(station_mac, ESP_MAC_WIFI_STA);//读取WIFI-STA的mac地址
    if (mac_result != ESP_OK) {
        return mac_result;
    }

    s_lock = xSemaphoreCreateMutex();//创建一个锁
    s_client_api_lock = xSemaphoreCreateMutex();
    s_command_queue = xQueueCreate(MQTT_MANAGER_COMMAND_QUEUE_LENGTH, sizeof(mqtt_manager_command_t));//创建RTOS消息队列，存放命令
    if (s_lock == NULL || s_client_api_lock == NULL ||
        s_command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    snprintf(s_client_id, sizeof(s_client_id), "esp32s3-motor-%02X%02X%02X%02X%02X%02X", station_mac[0], station_mac[1], station_mac[2], station_mac[3], station_mac[4], station_mac[5]);//生成s_client_id
    s_snapshot.initialized = true;//MQTT准备好了，未开始连接
    strlcpy(s_snapshot.status, "Ready - enter broker URI", sizeof(s_snapshot.status));
    ESP_LOGI(TAG, "MQTT client ID: %s", s_client_id);

    if (xTaskCreate(mqtt_manager_worker, "mqtt_manager", MQTT_MANAGER_WORKER_STACK_SIZE, NULL, MQTT_MANAGER_WORKER_PRIORITY, NULL) != pdPASS) {
        s_snapshot.initialized = false;
        strlcpy(s_snapshot.status, "MQTT worker allocation failed", sizeof(s_snapshot.status));
        return ESP_ERR_NO_MEM;
    }//创建MQTT worker RTOS Task
    return ESP_OK;
}

/**
 * @brief 校验并排队一个 MQTT Broker 连接请求。
 * @return 请求成功入队时返回 ESP_OK；实际连接结果会异步上报。
 */
esp_err_t mqtt_manager_connect_async(const char *broker_uri)
{
    if (broker_uri == NULL ||
        (strncmp(broker_uri, "mqtt://", 7U) != 0 &&
         strncmp(broker_uri, "mqtts://", 8U) != 0) ||
        strlen(broker_uri) > MQTT_MANAGER_URI_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_command_t command = {
        .type = MQTT_MANAGER_COMMAND_CONNECT,
    };
    strlcpy(command.broker_uri, broker_uri, sizeof(command.broker_uri));

    mqtt_manager_lock();
    if (!s_snapshot.initialized) {
        mqtt_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_snapshot.connecting = true;
    s_snapshot.connected = false;
    strlcpy(s_snapshot.broker_uri, broker_uri, sizeof(s_snapshot.broker_uri));
    mqtt_manager_set_status_locked("Connection queued");
    mqtt_manager_unlock();

    if (xQueueSend(s_command_queue, &command, 0U) != pdTRUE) {
        mqtt_manager_lock();
        s_snapshot.connecting = false;
        mqtt_manager_set_status_locked("MQTT command queue busy");
        mqtt_manager_unlock();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/** @brief 为管理工作任务排队一个客户端关闭请求。 */
esp_err_t mqtt_manager_disconnect_async(void)
{
    const mqtt_manager_command_t command = {
        .type = MQTT_MANAGER_COMMAND_DISCONNECT,
    };
    if (s_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueSend(s_command_queue, &command, 0U) == pdTRUE
        ? ESP_OK
        : ESP_ERR_TIMEOUT;
}

/**
 * @brief 通过活动 MQTT 客户端排队一条 QoS 1 发布消息。
 * @return 当前没有活动 Broker 连接时返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t mqtt_manager_publish(
    const char *topic,
    const char *payload)
{
    return mqtt_manager_publish_tracked(topic, payload, NULL);
}

esp_err_t mqtt_manager_publish_tracked(
    const char *topic,
    const char *payload,
    int *published_message_id)
{
    if (published_message_id != NULL) {
        *published_message_id = 0;
    }
    if (topic == NULL || topic[0] == '\0' || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_client_api_lock();
    mqtt_manager_lock();
    if (!s_snapshot.connected || s_client == NULL) {
        mqtt_manager_unlock();
        mqtt_manager_client_api_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    esp_mqtt_client_handle_t client = s_client;
    mqtt_manager_unlock();
    const int queued_message_id =
        esp_mqtt_client_enqueue(client, topic, payload, 0, 1, 0, true);
    if (queued_message_id >= 0 && published_message_id != NULL) {
        *published_message_id = queued_message_id;
    }
    mqtt_manager_client_api_unlock();
    return queued_message_id >= 0 ? ESP_OK : ESP_FAIL;
}

/** @brief 通过活动客户端排队一条 QoS 0 遥测发布消息。 */
esp_err_t mqtt_manager_publish_qos0(
    const char *topic,
    const char *payload)
{
    if (topic == NULL || topic[0] == '\0' || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_client_api_lock();
    mqtt_manager_lock();
    if (!s_snapshot.connected || s_client == NULL) {
        mqtt_manager_unlock();
        mqtt_manager_client_api_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    esp_mqtt_client_handle_t client = s_client;
    mqtt_manager_unlock();
    /* QoS 0 遥测不得滞留在 outbox，否则弱网时会挤占测试数据空间。 */
    const int message_id = esp_mqtt_client_enqueue(
        client, topic, payload, 0, 0, 0, false);
    mqtt_manager_client_api_unlock();
    return message_id >= 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_manager_publish_binary_qos1_tracked(
    const char *topic,
    const void *payload,
    size_t payload_length,
    int *published_message_id)
{
    if (published_message_id != NULL) {
        *published_message_id = 0;
    }
    if (topic == NULL || topic[0] == '\0' || payload == NULL ||
        payload_length == 0U || payload_length > INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    mqtt_manager_client_api_lock();
    mqtt_manager_lock();
    if (!s_snapshot.connected || s_client == NULL) {
        mqtt_manager_unlock();
        mqtt_manager_client_api_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    esp_mqtt_client_handle_t client = s_client;
    mqtt_manager_unlock();
    const int queued_message_id = esp_mqtt_client_enqueue(
        client, topic, (const char *)payload, (int)payload_length,
        1, 0, true);
    if (queued_message_id >= 0 && published_message_id != NULL) {
        *published_message_id = queued_message_id;
    }
    mqtt_manager_client_api_unlock();
    return queued_message_id >= 0 ? ESP_OK : ESP_FAIL;
}

bool mqtt_manager_is_message_delivered(int message_id)
{
    if (message_id <= 0 || s_lock == NULL) {
        return false;
    }
    bool delivered = false;
    mqtt_manager_lock();
    for (size_t index = 0U; index < MQTT_MANAGER_DELIVERY_HISTORY; ++index) {
        if (s_delivered_message_ids[index] == message_id) {
            delivered = true;
            break;
        }
    }
    mqtt_manager_unlock();
    return delivered;
}

/**
 * @brief 注册用于接收完整 MQTT 入站消息的应用回调。
 * @note 回调必须快速返回，不得阻塞 ESP-MQTT 事件任务。
 */
void mqtt_manager_set_message_callback(
    mqtt_manager_message_callback_t callback,
    void *context)
{
    mqtt_manager_lock();
    s_message_callback = callback;
    s_message_callback_context = context;
    mqtt_manager_unlock();
}

/** @brief 在管理器互斥锁保护下复制当前 MQTT 连接状态。 */
void mqtt_manager_get_snapshot(mqtt_manager_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    mqtt_manager_lock();
    *snapshot = s_snapshot;
    mqtt_manager_unlock();
}
