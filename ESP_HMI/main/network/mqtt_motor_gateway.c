#include "mqtt_motor_gateway.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comm_mgr_ESP.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_manager.h"

#define MQTT_GATEWAY_QUEUE_LENGTH          8U
#define MQTT_GATEWAY_COMMAND_MAX_LEN       511U
#define MQTT_GATEWAY_TASK_STACK_SIZE       6144U
#define MQTT_GATEWAY_TASK_PRIORITY         5U
#define MQTT_GATEWAY_TELEMETRY_PERIOD_MS   250U
#define MQTT_GATEWAY_DEFAULT_UART_BAUD     115200U
#define MQTT_GATEWAY_SPEED_LIMIT_RPM       2600
#define MQTT_GATEWAY_TEST_DURATION_MS      7000U
#define MQTT_GATEWAY_TEST_MAX_SAMPLES      3600U
#define MQTT_GATEWAY_TEST_CHUNK_SAMPLES    40U
#define MQTT_GATEWAY_TEST_HEADER_SIZE      20U
#define MQTT_GATEWAY_TEST_RECORD_SIZE      20U
#define MQTT_GATEWAY_TEST_SEND_PERIOD_US   50000LL
#define MQTT_GATEWAY_TEST_START_TIMEOUT_US 5000000LL
#define MQTT_GATEWAY_TEST_STOP_TIMEOUT_US  3000000LL
#define MQTT_GATEWAY_TEST_SEND_TIMEOUT_US  30000000LL

typedef struct
{
    char payload[MQTT_GATEWAY_COMMAND_MAX_LEN + 1U];
    bool local;
} mqtt_gateway_command_t;

typedef struct
{
    uint32_t time_us;
    int16_t primary_measured;
    int16_t primary_reference;
    int16_t iq_ma;
    int16_t id_ma;
    int16_t iq_reference_ma;
    int16_t id_reference_ma;
} mqtt_test_sample_t;

_Static_assert(sizeof(mqtt_test_sample_t) == 16U,
               "Compact test sample must remain 16 bytes");

typedef enum
{
    MQTT_TEST_IDLE = 0,
    MQTT_TEST_STARTING,
    MQTT_TEST_RECORDING,
    MQTT_TEST_STOPPING,
    MQTT_TEST_SENDING,
    MQTT_TEST_SEND_FAILED
} mqtt_test_stage_t;

typedef struct
{
    mqtt_test_stage_t stage;
    uint32_t command_id;
    CommMgr_ESP_mode_t mode;
    int32_t target;
    uint32_t duration_ms;
    mqtt_test_sample_t *samples;
    int64_t deadline_us;
    int64_t first_sample_timestamp_us;
    uint32_t last_sample_sequence;
    uint16_t sample_count;
    uint16_t publish_index;
    uint16_t pending_sample_count;
    int pending_message_id;
    int completion_message_id;
    int64_t next_publish_us;
    int64_t reject_check_after_us;
    uint32_t last_sample_time_us;
    bool start_issued;
    bool stop_requested;
    uint16_t enqueue_failure_count;
    uint16_t retry_count;
} mqtt_test_state_t;

static const char *TAG = "MQTT_MOTOR";
static QueueHandle_t s_command_queue;
static TaskHandle_t s_gateway_task;
static mqtt_test_state_t s_test;
static mqtt_motor_test_snapshot_t s_test_snapshot;
static portMUX_TYPE s_test_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_next_local_test_id = 1000000000UL;

static const int16_t s_default_pid[4][3] = {
    {2144, 5, 0},
    {48, 4, 8},
    {3633, 2693, 0},
    {3633, 2693, 0}
};

static void mqtt_gateway_update_test_snapshot(
    mqtt_motor_test_ui_state_t state,
    const char *message)
{
    portENTER_CRITICAL(&s_test_snapshot_lock);
    s_test_snapshot.state = state;
    if (s_test.command_id != 0U) {
        s_test_snapshot.mode = (mqtt_motor_local_test_mode_t)s_test.mode;
        s_test_snapshot.command_id = s_test.command_id;
        s_test_snapshot.duration_ms = s_test.duration_ms;
        s_test_snapshot.sample_count = s_test.sample_count;
        s_test_snapshot.uploaded_sample_count = s_test.publish_index;
    }
    s_test_snapshot.resend_available =
        state == MQTT_MOTOR_TEST_UI_UPLOAD_FAILED &&
        s_test.samples != NULL && s_test.sample_count > 0U;
    strlcpy(s_test_snapshot.message,
            message != NULL ? message : "",
            sizeof(s_test_snapshot.message));
    s_test_snapshot.revision++;
    portEXIT_CRITICAL(&s_test_snapshot_lock);
}

/** @brief 将已收到 PUBACK 的采样进度同步到 PID TEST 页面。 */
static void mqtt_gateway_update_upload_progress(void)
{
    portENTER_CRITICAL(&s_test_snapshot_lock);
    s_test_snapshot.uploaded_sample_count = s_test.publish_index;
    s_test_snapshot.revision++;
    portEXIT_CRITICAL(&s_test_snapshot_lock);
}

static void mqtt_gateway_update_pid_snapshot(
    uint8_t controller,
    int16_t kp,
    int16_t ki,
    int16_t kd)
{
    portENTER_CRITICAL(&s_test_snapshot_lock);
    s_test_snapshot.pid[controller][0] = kp;
    s_test_snapshot.pid[controller][1] = ki;
    if (controller == COMM_MGR_ESP_PID_POSITION) {
        s_test_snapshot.pid[controller][2] = kd;
    }
    s_test_snapshot.revision++;
    portEXIT_CRITICAL(&s_test_snapshot_lock);
}

/** @brief 将 MQTT 预先缓存的九个有效 PID 增益排队到当前 CAN 通道。 */
static void mqtt_gateway_apply_cached_pid(void)
{
    int16_t pid[4][3];
    portENTER_CRITICAL(&s_test_snapshot_lock);
    memcpy(pid, s_test_snapshot.pid, sizeof(pid));
    portEXIT_CRITICAL(&s_test_snapshot_lock);

    for (uint8_t controller = 0U; controller < 4U; controller++) {
        CommMgr_ESP_SetPidGain(
            (CommMgr_ESP_pid_controller_t)controller,
            COMM_MGR_ESP_PID_KP, pid[controller][0]);
        CommMgr_ESP_SetPidGain(
            (CommMgr_ESP_pid_controller_t)controller,
            COMM_MGR_ESP_PID_KI, pid[controller][1]);
        if (controller == COMM_MGR_ESP_PID_POSITION) {
            CommMgr_ESP_SetPidGain(
                COMM_MGR_ESP_PID_POSITION,
                COMM_MGR_ESP_PID_KD, pid[controller][2]);
        }
    }
}

static int32_t mqtt_gateway_clamp_i32(
    int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void mqtt_gateway_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static void mqtt_gateway_write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)(value >> 24U);
}

static void mqtt_gateway_publish_ack(
    uint32_t command_id, bool accepted, const char *message)
{
    char payload[192];
    snprintf(payload, sizeof(payload),
             "{\"id\":%lu,\"ok\":%s,\"message\":\"%s\"}",
             (unsigned long)command_id,
             accepted ? "true" : "false",
             message != NULL ? message : "");
    (void)mqtt_manager_publish(MQTT_MOTOR_ACK_TOPIC, payload);
}

static bool mqtt_gateway_publish_test_status_for(
    uint32_t command_id,
    CommMgr_ESP_mode_t mode,
    uint16_t sample_count,
    uint32_t last_sample_time_us,
    const char *stage,
    const char *message)
{
    const uint32_t sample_period_us = sample_count > 1U
        ? last_sample_time_us / (sample_count - 1U) : 0U;
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"id\":%lu,\"stage\":\"%s\",\"mode\":%u,"
             "\"samples\":%u,\"sample_period_us\":%lu,"
             "\"message\":\"%s\"}",
             (unsigned long)command_id,
             stage,
             (unsigned)mode,
             (unsigned)sample_count,
             (unsigned long)sample_period_us,
             message != NULL ? message : "");
    return mqtt_manager_publish(
               MQTT_MOTOR_TEST_STATUS_TOPIC, payload) == ESP_OK;
}

static bool mqtt_gateway_publish_test_status(
    const char *stage, const char *message)
{
    return mqtt_gateway_publish_test_status_for(
        s_test.command_id, s_test.mode, s_test.sample_count,
        s_test.last_sample_time_us, stage, message);
}

/** @brief 即发即弃地通知 Qt 上传失败，避免错误状态滞留到重发阶段。 */
static void mqtt_gateway_publish_upload_error_qos0(const char *message)
{
    const uint32_t sample_period_us = s_test.sample_count > 1U
        ? s_test.last_sample_time_us / (s_test.sample_count - 1U) : 0U;
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"id\":%lu,\"stage\":\"error\",\"mode\":%u,"
             "\"samples\":%u,\"sample_period_us\":%lu,"
             "\"message\":\"%s\"}",
             (unsigned long)s_test.command_id,
             (unsigned)s_test.mode,
             (unsigned)s_test.sample_count,
             (unsigned long)sample_period_us,
             message != NULL ? message : "MQTT upload failed");
    (void)mqtt_manager_publish_qos0(
        MQTT_MOTOR_TEST_STATUS_TOPIC, payload);
}

/** @brief 发布可跟踪 PUBACK 的当前测试状态。 */
static esp_err_t mqtt_gateway_publish_test_status_tracked(
    const char *stage, const char *message, int *message_id)
{
    const uint32_t sample_period_us = s_test.sample_count > 1U
        ? s_test.last_sample_time_us / (s_test.sample_count - 1U) : 0U;
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"id\":%lu,\"stage\":\"%s\",\"mode\":%u,"
             "\"samples\":%u,\"sample_period_us\":%lu,"
             "\"message\":\"%s\"}",
             (unsigned long)s_test.command_id,
             stage,
             (unsigned)s_test.mode,
             (unsigned)s_test.sample_count,
             (unsigned long)sample_period_us,
             message != NULL ? message : "");
    return mqtt_manager_publish_tracked(
        MQTT_MOTOR_TEST_STATUS_TOPIC, payload, message_id);
}

static bool mqtt_gateway_require_link(
    const CommMgr_ESP_State *snapshot, uint32_t command_id)
{
    if (snapshot->link_active) {
        return true;
    }
    mqtt_gateway_publish_ack(command_id, false, "STM32 link offline");
    return false;
}

static bool mqtt_gateway_json_int(
    const char *json, const char *key, int32_t *value)
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
    if (end == cursor || parsed < INT32_MIN || parsed > INT32_MAX) {
        return false;
    }
    *value = (int32_t)parsed;
    return true;
}

static bool mqtt_gateway_json_string(
    const char *json, const char *key, char *value, size_t value_size)
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
    if (*cursor++ != '"') {
        return false;
    }
    const char *end = strchr(cursor, '"');
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

static void mqtt_gateway_reset_test(void)
{
    if (s_test.samples != NULL) {
        heap_caps_free(s_test.samples);
    }
    memset(&s_test, 0, sizeof(s_test));
    s_test.stage = MQTT_TEST_IDLE;
}

static void mqtt_gateway_fail_test(const char *message)
{
    CommMgr_ESP_Stop();
    mqtt_gateway_publish_upload_error_qos0(message);
    mqtt_gateway_update_test_snapshot(MQTT_MOTOR_TEST_UI_ERROR, message);
    mqtt_gateway_reset_test();
}

/**
 * @brief 标记 MQTT 上传失败但保留 PSRAM 数据，供页面上的 RESEND 使用。
 *
 * 与测试执行阶段错误不同，电机此时已经停止，CAN 状态不影响已记录的数据。
 * 因此不能调用 mqtt_gateway_reset_test() 释放样本。
 */
static void mqtt_gateway_fail_upload(const char *message)
{
    mqtt_manager_snapshot_t mqtt_snapshot;
    mqtt_manager_get_snapshot(&mqtt_snapshot);
    ESP_LOGE(TAG,
             "MQTT upload failed: %s; test_id=%lu progress=%u/%u "
             "pending_mid=%d connected=%u mqtt_status=%s; PSRAM retained",
             message != NULL ? message : "unknown error",
             (unsigned long)s_test.command_id,
             (unsigned)s_test.publish_index,
             (unsigned)s_test.sample_count,
             s_test.pending_message_id,
             mqtt_snapshot.connected ? 1U : 0U,
             mqtt_snapshot.status);
    s_test.stage = MQTT_TEST_SEND_FAILED;
    s_test.pending_message_id = 0;
    s_test.pending_sample_count = 0U;
    s_test.completion_message_id = 0;
    mqtt_gateway_publish_upload_error_qos0(message);
    mqtt_gateway_update_test_snapshot(
        MQTT_MOTOR_TEST_UI_UPLOAD_FAILED, message);
}

static void mqtt_gateway_begin_test_send(int64_t now_us)
{
    if (s_test.sample_count == 0U) {
        mqtt_gateway_fail_test("No CAN samples recorded");
        return;
    }
    s_test.stage = MQTT_TEST_SENDING;
    s_test.publish_index = 0U;
    s_test.pending_sample_count = 0U;
    s_test.pending_message_id = 0;
    s_test.completion_message_id = 0;
    s_test.enqueue_failure_count = 0U;
    s_test.next_publish_us = now_us + MQTT_GATEWAY_TEST_SEND_PERIOD_US;
    s_test.deadline_us = now_us + MQTT_GATEWAY_TEST_SEND_TIMEOUT_US;
    mqtt_gateway_update_test_snapshot(
        MQTT_MOTOR_TEST_UI_UPLOADING, "Test complete; uploading MQTT data");
    (void)mqtt_gateway_publish_test_status(
        "sending", "Sending recorded CAN samples");
}

/** @brief 从第一个样本重新上传保留在 PSRAM 中的完整数据集。 */
static void mqtt_gateway_retry_upload_internal(uint32_t command_id)
{
    if (s_test.stage != MQTT_TEST_SEND_FAILED ||
        s_test.samples == NULL || s_test.sample_count == 0U) {
        ESP_LOGW(TAG, "Resend rejected: no failed PSRAM dataset retained");
        mqtt_gateway_publish_ack(
            command_id, false, "No failed dataset is available to resend");
        return;
    }

    mqtt_manager_snapshot_t mqtt_snapshot;
    mqtt_manager_get_snapshot(&mqtt_snapshot);
    if (!mqtt_snapshot.connected) {
        ESP_LOGW(TAG,
                 "Resend deferred: MQTT is offline; test_id=%lu samples=%u",
                 (unsigned long)s_test.command_id,
                 (unsigned)s_test.sample_count);
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_UPLOAD_FAILED,
            "MQTT offline - reconnect then press RESEND");
        mqtt_gateway_publish_ack(command_id, false, "MQTT is offline");
        return;
    }

    s_test.stage = MQTT_TEST_SENDING;
    s_test.publish_index = 0U;
    s_test.pending_sample_count = 0U;
    s_test.pending_message_id = 0;
    s_test.completion_message_id = 0;
    s_test.enqueue_failure_count = 0U;
    s_test.retry_count++;
    const int64_t now_us = esp_timer_get_time();
    s_test.next_publish_us = now_us + MQTT_GATEWAY_TEST_SEND_PERIOD_US;
    s_test.deadline_us = now_us + MQTT_GATEWAY_TEST_SEND_TIMEOUT_US;

    ESP_LOGI(TAG,
             "Resending retained PSRAM dataset: test_id=%lu samples=%u "
             "attempt=%u",
             (unsigned long)s_test.command_id,
             (unsigned)s_test.sample_count,
             (unsigned)s_test.retry_count);
    mqtt_gateway_update_test_snapshot(
        MQTT_MOTOR_TEST_UI_UPLOADING,
        "Resending retained PSRAM data");
    mqtt_gateway_publish_ack(command_id, true, "Resend accepted");
    (void)mqtt_gateway_publish_test_status(
        "sending", "Resending complete retained dataset");
}

static esp_err_t mqtt_gateway_publish_test_chunk(void)
{
    const uint16_t remaining =
        (uint16_t)(s_test.sample_count - s_test.publish_index);
    const uint16_t count = remaining > MQTT_GATEWAY_TEST_CHUNK_SAMPLES
        ? MQTT_GATEWAY_TEST_CHUNK_SAMPLES : remaining;
    uint8_t packet[MQTT_GATEWAY_TEST_HEADER_SIZE +
                   MQTT_GATEWAY_TEST_CHUNK_SAMPLES *
                       MQTT_GATEWAY_TEST_RECORD_SIZE];
    memset(packet, 0, sizeof(packet));
    packet[0] = 'M';
    packet[1] = 'C';
    packet[2] = 'T';
    packet[3] = 'D';
    packet[4] = 1U;
    packet[5] = (uint8_t)s_test.mode;
    packet[6] = MQTT_GATEWAY_TEST_RECORD_SIZE;
    if ((uint16_t)(s_test.publish_index + count) == s_test.sample_count) {
        packet[7] = 1U;
    }
    mqtt_gateway_write_u32(&packet[8], s_test.command_id);
    mqtt_gateway_write_u32(&packet[12], s_test.publish_index);
    mqtt_gateway_write_u16(&packet[16], count);
    mqtt_gateway_write_u16(&packet[18], s_test.sample_count);

    for (uint16_t index = 0U; index < count; index++) {
        const mqtt_test_sample_t *sample =
            &s_test.samples[s_test.publish_index + index];
        uint8_t *record = &packet[MQTT_GATEWAY_TEST_HEADER_SIZE +
                                  index * MQTT_GATEWAY_TEST_RECORD_SIZE];
        mqtt_gateway_write_u32(&record[0], sample->time_us);
        if (s_test.mode == COMM_MGR_ESP_MODE_SPEED) {
            mqtt_gateway_write_u16(
                &record[4], (uint16_t)sample->primary_measured);
            mqtt_gateway_write_u16(
                &record[6], (uint16_t)sample->primary_reference);
        } else {
            mqtt_gateway_write_u16(
                &record[8], (uint16_t)sample->primary_measured);
            mqtt_gateway_write_u16(
                &record[10], (uint16_t)sample->primary_reference);
        }
        mqtt_gateway_write_u16(&record[12], (uint16_t)sample->iq_ma);
        mqtt_gateway_write_u16(&record[14], (uint16_t)sample->id_ma);
        mqtt_gateway_write_u16(
            &record[16], (uint16_t)sample->iq_reference_ma);
        mqtt_gateway_write_u16(
            &record[18], (uint16_t)sample->id_reference_ma);
    }

    const size_t packet_size = MQTT_GATEWAY_TEST_HEADER_SIZE +
        (size_t)count * MQTT_GATEWAY_TEST_RECORD_SIZE;
    int message_id = 0;
    const esp_err_t result = mqtt_manager_publish_binary_qos1_tracked(
        MQTT_MOTOR_TEST_DATA_TOPIC, packet, packet_size, &message_id);
    if (result != ESP_OK || message_id <= 0) {
        return result != ESP_OK ? result : ESP_FAIL;
    }
    s_test.pending_message_id = message_id;
    s_test.pending_sample_count = count;
    return ESP_OK;
}

static bool mqtt_gateway_record_sample(const CommMgr_ESP_State *snapshot)
{
    if (snapshot->sample_sequence == s_test.last_sample_sequence) {
        return true;
    }
    s_test.last_sample_sequence = snapshot->sample_sequence;
    if (s_test.sample_count >= MQTT_GATEWAY_TEST_MAX_SAMPLES) {
        return false;
    }
    if (s_test.first_sample_timestamp_us == 0LL) {
        s_test.first_sample_timestamp_us = snapshot->sample_timestamp_us;
    }
    int64_t relative =
        snapshot->sample_timestamp_us - s_test.first_sample_timestamp_us;
    if (relative < 0LL) {
        relative = 0LL;
    }

    mqtt_test_sample_t *sample = &s_test.samples[s_test.sample_count];
    memset(sample, 0, sizeof(*sample));
    sample->time_us = relative > UINT32_MAX
        ? UINT32_MAX : (uint32_t)relative;
    if (s_test.mode == COMM_MGR_ESP_MODE_SPEED) {
        sample->primary_measured = snapshot->measured_speed_rpm;
        sample->primary_reference = snapshot->reference_speed_rpm;
    } else {
        sample->primary_measured =
            (int16_t)snapshot->current_position_cdeg;
        sample->primary_reference =
            (int16_t)snapshot->target_position_cdeg;
    }
    sample->iq_ma = snapshot->iq_ma;
    sample->id_ma = snapshot->id_ma;
    sample->iq_reference_ma = snapshot->iq_reference_ma;
    sample->id_reference_ma = snapshot->id_reference_ma;
    s_test.last_sample_time_us = sample->time_us;
    s_test.sample_count++;
    if ((s_test.sample_count % 50U) == 0U) {
        portENTER_CRITICAL(&s_test_snapshot_lock);
        s_test_snapshot.sample_count = s_test.sample_count;
        portEXIT_CRITICAL(&s_test_snapshot_lock);
    }
    return true;
}

static void mqtt_gateway_service_test(void)
{
    if (s_test.stage == MQTT_TEST_IDLE) {
        return;
    }
    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);
    const int64_t now_us = esp_timer_get_time();

    switch (s_test.stage) {
    case MQTT_TEST_STARTING:
        if (!s_test.start_issued) {
            if (snapshot.transport != COMM_MGR_ESP_CAN) {
                if (CommMgr_ESP_SelectCAN() != ESP_OK) {
                    mqtt_gateway_fail_test("CAN activation failed");
                }
                break;
            }
            if (snapshot.motor_fault) {
                mqtt_gateway_fail_test("STM32 has an active motor fault");
                break;
            }
            if (snapshot.link_active && !snapshot.motor_running) {
                mqtt_gateway_apply_cached_pid();
                CommMgr_ESP_SetMode(s_test.mode);
                CommMgr_ESP_Start();
                s_test.start_issued = true;
                s_test.deadline_us =
                    now_us + MQTT_GATEWAY_TEST_START_TIMEOUT_US;
                /* 等待 MODE/START 至少经过若干 CAN TX 周期再判断回显。 */
                s_test.reject_check_after_us = now_us + 200000LL;
            } else if (now_us >= s_test.deadline_us) {
                mqtt_gateway_fail_test("CAN link did not become ready");
            }
            break;
        }
        if (!snapshot.link_active) {
            mqtt_gateway_fail_test("CAN link lost during startup");
        } else if (snapshot.motor_fault) {
            mqtt_gateway_fail_test("Motor fault occurred during startup");
        } else if (snapshot.command_rejected &&
                   now_us >= s_test.reject_check_after_us) {
            mqtt_gateway_fail_test("STM32 rejected mode or START");
        } else if (snapshot.motor_running && snapshot.mode == s_test.mode) {
            if (s_test.mode == COMM_MGR_ESP_MODE_SPEED) {
                CommMgr_ESP_SetSpeedRPM((int16_t)s_test.target);
            } else {
                CommMgr_ESP_SetPositionCdeg((uint16_t)s_test.target);
            }
            s_test.stage = MQTT_TEST_RECORDING;
            s_test.last_sample_sequence = snapshot.sample_sequence;
            s_test.first_sample_timestamp_us = 0LL;
            s_test.deadline_us = now_us +
                (int64_t)s_test.duration_ms * 1000LL + 2000000LL;
            s_test.reject_check_after_us = now_us + 200000LL;
            (void)mqtt_gateway_publish_test_status(
                "recording", "Target applied; recording CAN feedback");
        } else if (now_us >= s_test.deadline_us) {
            mqtt_gateway_fail_test("Motor did not enter RUN state");
        }
        break;

    case MQTT_TEST_RECORDING:
        if (!snapshot.link_active) {
            mqtt_gateway_fail_test("CAN link lost during test");
            break;
        }
        if (snapshot.motor_fault) {
            mqtt_gateway_fail_test("STM32 fault during test");
            break;
        }
        if (!snapshot.motor_running) {
            mqtt_gateway_fail_test("Motor left RUN during test");
            break;
        }
        if (snapshot.command_rejected &&
            now_us >= s_test.reject_check_after_us) {
            mqtt_gateway_fail_test("STM32 rejected test target");
            break;
        }
        if (!mqtt_gateway_record_sample(&snapshot)) {
            mqtt_gateway_fail_test("PSRAM sample buffer overflow");
            break;
        }
        if (s_test.sample_count > 0U &&
            s_test.last_sample_time_us >= s_test.duration_ms * 1000U) {
            CommMgr_ESP_Stop();
            s_test.stage = MQTT_TEST_STOPPING;
            s_test.deadline_us = now_us + MQTT_GATEWAY_TEST_STOP_TIMEOUT_US;
        } else if (s_test.sample_count >= MQTT_GATEWAY_TEST_MAX_SAMPLES) {
            mqtt_gateway_fail_test("PSRAM sample buffer overflow");
        } else if (now_us >= s_test.deadline_us) {
            mqtt_gateway_fail_test("CAN sampling timed out");
        }
        break;

    case MQTT_TEST_STOPPING:
        if (!snapshot.link_active) {
            mqtt_gateway_fail_test("CAN link lost while stopping");
        } else if (snapshot.motor_fault) {
            mqtt_gateway_fail_test("STM32 fault while stopping");
        } else if (!snapshot.motor_running) {
            mqtt_gateway_begin_test_send(now_us);
        } else if (now_us >= s_test.deadline_us) {
            mqtt_gateway_fail_test("Motor stop timed out");
        }
        break;

    case MQTT_TEST_SENDING: {
        mqtt_manager_snapshot_t mqtt_snapshot;
        mqtt_manager_get_snapshot(&mqtt_snapshot);

        /* 每次只允许一个数据分块等待 PUBACK，避免 8 KiB outbox 被填满。 */
        if (s_test.pending_message_id > 0) {
            if (mqtt_manager_is_message_delivered(
                    s_test.pending_message_id)) {
                s_test.publish_index = (uint16_t)(
                    s_test.publish_index + s_test.pending_sample_count);
                ESP_LOGD(TAG,
                         "MQTT chunk PUBACK mid=%d progress=%u/%u",
                         s_test.pending_message_id,
                         (unsigned)s_test.publish_index,
                         (unsigned)s_test.sample_count);
                s_test.pending_message_id = 0;
                s_test.pending_sample_count = 0U;
                s_test.enqueue_failure_count = 0U;
                s_test.next_publish_us =
                    now_us + MQTT_GATEWAY_TEST_SEND_PERIOD_US;
                s_test.deadline_us =
                    now_us + MQTT_GATEWAY_TEST_SEND_TIMEOUT_US;
                mqtt_gateway_update_upload_progress();
            } else if (now_us >= s_test.deadline_us) {
                mqtt_gateway_fail_upload(
                    mqtt_snapshot.connected
                        ? "MQTT data PUBACK timed out - press RESEND"
                        : "MQTT disconnected during upload - press RESEND");
            }
            break;
        }

        if (s_test.publish_index < s_test.sample_count) {
            if (now_us >= s_test.next_publish_us) {
                esp_err_t publish_result = ESP_ERR_INVALID_STATE;
                if (mqtt_snapshot.connected) {
                    publish_result = mqtt_gateway_publish_test_chunk();
                }
                if (publish_result != ESP_OK) {
                    s_test.enqueue_failure_count++;
                    if (s_test.enqueue_failure_count == 1U ||
                        (s_test.enqueue_failure_count % 20U) == 0U) {
                        ESP_LOGW(TAG,
                                 "MQTT chunk enqueue failed: %s; "
                                 "progress=%u/%u connected=%u status=%s",
                                 esp_err_to_name(publish_result),
                                 (unsigned)s_test.publish_index,
                                 (unsigned)s_test.sample_count,
                                 mqtt_snapshot.connected ? 1U : 0U,
                                 mqtt_snapshot.status);
                    }
                } else {
                    s_test.enqueue_failure_count = 0U;
                    s_test.deadline_us =
                        now_us + MQTT_GATEWAY_TEST_SEND_TIMEOUT_US;
                }
                s_test.next_publish_us =
                    now_us + MQTT_GATEWAY_TEST_SEND_PERIOD_US;
            }
            if (s_test.stage == MQTT_TEST_SENDING &&
                now_us >= s_test.deadline_us) {
                mqtt_gateway_fail_upload(
                    mqtt_snapshot.connected
                        ? "MQTT outbox remained busy - press RESEND"
                        : "MQTT reconnect timed out - press RESEND");
            }
            break;
        }

        /* complete 状态同样必须收到 PUBACK 后才允许释放 PSRAM。 */
        if (s_test.completion_message_id > 0) {
            if (mqtt_manager_is_message_delivered(
                    s_test.completion_message_id)) {
                ESP_LOGI(TAG,
                         "MQTT test upload confirmed by broker: "
                         "test_id=%lu samples=%u retries=%u",
                         (unsigned long)s_test.command_id,
                         (unsigned)s_test.sample_count,
                         (unsigned)s_test.retry_count);
                mqtt_gateway_update_test_snapshot(
                    MQTT_MOTOR_TEST_UI_UPLOAD_SUCCESS,
                    "Upload successful");
                mqtt_gateway_reset_test();
            } else if (now_us >= s_test.deadline_us) {
                mqtt_gateway_fail_upload(
                    "MQTT completion PUBACK timed out - press RESEND");
            }
            break;
        }

        if (now_us >= s_test.next_publish_us) {
            int message_id = 0;
            const esp_err_t result = mqtt_snapshot.connected
                ? mqtt_gateway_publish_test_status_tracked(
                      "complete",
                      s_test.stop_requested
                          ? "Stopped test dataset sent"
                          : "CAN dataset sent",
                      &message_id)
                : ESP_ERR_INVALID_STATE;
            if (result == ESP_OK && message_id > 0) {
                s_test.completion_message_id = message_id;
                s_test.deadline_us =
                    now_us + MQTT_GATEWAY_TEST_SEND_TIMEOUT_US;
            } else {
                s_test.enqueue_failure_count++;
                if (s_test.enqueue_failure_count == 1U ||
                    (s_test.enqueue_failure_count % 20U) == 0U) {
                    ESP_LOGW(TAG,
                             "MQTT completion enqueue failed: %s; "
                             "connected=%u status=%s",
                             esp_err_to_name(result),
                             mqtt_snapshot.connected ? 1U : 0U,
                             mqtt_snapshot.status);
                }
            }
            s_test.next_publish_us =
                now_us + MQTT_GATEWAY_TEST_SEND_PERIOD_US;
        }
        if (s_test.stage == MQTT_TEST_SENDING &&
            now_us >= s_test.deadline_us) {
            mqtt_gateway_fail_upload(
                "MQTT completion send timed out - press RESEND");
        }
        break;
    }

    case MQTT_TEST_SEND_FAILED:
        /* PSRAM 数据保持不动，直到用户点击 RESEND 或设备重启。 */
        break;

    case MQTT_TEST_IDLE:
    default:
        break;
    }
}

static mqtt_test_sample_t *mqtt_gateway_allocate_test_samples(void)
{
    const size_t bytes = MQTT_GATEWAY_TEST_MAX_SAMPLES *
        sizeof(mqtt_test_sample_t);
    mqtt_test_sample_t *samples = heap_caps_malloc(
        bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (samples != NULL) {
        return samples;
    }

    const size_t total_bytes =
        heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t free_bytes =
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t largest_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (total_bytes == 0U) {
        ESP_LOGE(TAG,
                 "PSRAM allocation failed: PSRAM is not initialized, not "
                 "registered with the heap, or disabled in sdkconfig; "
                 "required=%zu bytes, total=%zu, free=%zu, largest=%zu",
                 bytes, total_bytes, free_bytes, largest_block);
    } else if (free_bytes < bytes) {
        ESP_LOGE(TAG,
                 "PSRAM allocation failed: insufficient free PSRAM; "
                 "required=%zu bytes, total=%zu, free=%zu, largest=%zu",
                 bytes, total_bytes, free_bytes, largest_block);
    } else if (largest_block < bytes) {
        ESP_LOGE(TAG,
                 "PSRAM allocation failed: PSRAM heap is fragmented and no "
                 "contiguous block is large enough; required=%zu bytes, "
                 "total=%zu, free=%zu, largest=%zu",
                 bytes, total_bytes, free_bytes, largest_block);
    } else {
        ESP_LOGE(TAG,
                 "PSRAM allocation failed despite sufficient reported heap; "
                 "check PSRAM/heap integrity and allocation capabilities; "
                 "required=%zu bytes, total=%zu, free=%zu, largest=%zu",
                 bytes, total_bytes, free_bytes, largest_block);
    }
    return NULL;
}

static void mqtt_gateway_start_test(
    const char *payload, uint32_t command_id)
{
    int32_t mode = 0;
    int32_t target = 0;
    int32_t duration_ms = MQTT_GATEWAY_TEST_DURATION_MS;
    if (s_test.stage != MQTT_TEST_IDLE) {
        mqtt_gateway_publish_ack(command_id, false, "Test already running");
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Another test is already running");
        return;
    }
    if (!mqtt_gateway_json_int(payload, "mode", &mode) ||
        !mqtt_gateway_json_int(payload, "target", &target)) {
        mqtt_gateway_publish_ack(command_id, false, "Missing test fields");
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Invalid local test request");
        return;
    }
    (void)mqtt_gateway_json_int(payload, "duration_ms", &duration_ms);
    if ((mode != 0 && mode != 1) ||
        duration_ms != (int32_t)MQTT_GATEWAY_TEST_DURATION_MS ||
        (mode == 0 && (target < -MQTT_GATEWAY_SPEED_LIMIT_RPM ||
                       target > MQTT_GATEWAY_SPEED_LIMIT_RPM))) {
        mqtt_gateway_publish_ack(command_id, false, "Invalid test settings");
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Invalid local test settings");
        return;
    }
    if (mode == 1) {
        target %= 36000;
        if (target < 0) {
            target += 36000;
        }
    }

    mqtt_manager_snapshot_t mqtt_snapshot;
    mqtt_manager_get_snapshot(&mqtt_snapshot);
    if (!mqtt_snapshot.connected) {
        mqtt_gateway_publish_ack(command_id, false, "MQTT is offline");
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Connect MQTT before starting a test");
        return;
    }
    CommMgr_ESP_State motor_snapshot;
    CommMgr_ESP_GetState(&motor_snapshot);
    if (motor_snapshot.motor_running) {
        mqtt_gateway_publish_ack(command_id, false, "Motor is already running");
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Stop the motor before starting a test");
        return;
    }

    mqtt_test_sample_t *samples = mqtt_gateway_allocate_test_samples();
    if (samples == NULL) {
        mqtt_gateway_publish_ack(command_id, false,
                                 "Test PSRAM allocation failed");
        (void)mqtt_gateway_publish_test_status_for(
            command_id,
            mode == 0 ? COMM_MGR_ESP_MODE_SPEED
                      : COMM_MGR_ESP_MODE_POSITION,
            0U, 0U, "error", "Test PSRAM allocation failed");
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Test PSRAM allocation failed");
        return;
    }

    memset(&s_test, 0, sizeof(s_test));
    s_test.command_id = command_id;
    s_test.mode = mode == 0
        ? COMM_MGR_ESP_MODE_SPEED : COMM_MGR_ESP_MODE_POSITION;
    s_test.target = target;
    s_test.duration_ms = (uint32_t)duration_ms;
    s_test.samples = samples;
    s_test.stage = MQTT_TEST_STARTING;
    s_test.deadline_us = esp_timer_get_time() + 3000000LL;
    mqtt_gateway_update_test_snapshot(
        MQTT_MOTOR_TEST_UI_TESTING, "Test in progress");
    mqtt_gateway_publish_ack(command_id, true, "ESP32 test accepted");
    (void)mqtt_gateway_publish_test_status(
        "accepted", "Test command accepted; preparing CAN");
}

static void mqtt_gateway_process_command(
    const char *payload, bool local)
{
    char command[32];
    int32_t id_value = 0;
    int32_t value = 0;
    const bool has_value = mqtt_gateway_json_int(payload, "value", &value);
    (void)mqtt_gateway_json_int(payload, "id", &id_value);
    const uint32_t command_id = id_value > 0 ? (uint32_t)id_value : 0U;

    if (!mqtt_gateway_json_string(
            payload, "cmd", command, sizeof(command))) {
        mqtt_gateway_publish_ack(command_id, false, "Missing command");
        return;
    }
    if (strcmp(command, "run_test") == 0) {
        if (local) {
            mqtt_gateway_start_test(payload, command_id);
        } else {
            mqtt_gateway_publish_ack(
                command_id, false,
                "Start tests from the ESP32 PID TEST page");
        }
        return;
    }
    if (strcmp(command, "retry_upload") == 0) {
        if (local) {
            mqtt_gateway_retry_upload_internal(command_id);
        } else {
            mqtt_gateway_publish_ack(
                command_id, false,
                "Retry uploads from the ESP32 PID TEST page");
        }
        return;
    }

    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);
    if (strcmp(command, "stop") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_Stop();
            if (s_test.stage == MQTT_TEST_STARTING ||
                s_test.stage == MQTT_TEST_RECORDING) {
                s_test.stop_requested = true;
                s_test.stage = MQTT_TEST_STOPPING;
                s_test.deadline_us = esp_timer_get_time() +
                    MQTT_GATEWAY_TEST_STOP_TIMEOUT_US;
            }
            mqtt_gateway_publish_ack(command_id, true, "Stop accepted");
        }
        return;
    }
    if (s_test.stage != MQTT_TEST_IDLE) {
        mqtt_gateway_publish_ack(command_id, false, "ESP32 test in progress");
        return;
    }

    if (strcmp(command, "claim") == 0) {
        esp_err_t result = ESP_OK;
        if (snapshot.transport == COMM_MGR_ESP_NONE) {
            result = CommMgr_ESP_SelectUSART(MQTT_GATEWAY_DEFAULT_UART_BAUD);
        }
        mqtt_gateway_publish_ack(
            command_id, result == ESP_OK,
            result == ESP_OK ? "Gateway ready" : "UART activation failed");
    } else if (strcmp(command, "set_mode") == 0) {
        if (!has_value) {
            mqtt_gateway_publish_ack(command_id, false, "Missing mode value");
        } else if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_SetMode(value == 0
                ? COMM_MGR_ESP_MODE_SPEED : COMM_MGR_ESP_MODE_POSITION);
            mqtt_gateway_publish_ack(command_id, true, "Mode accepted");
        }
    } else if (strcmp(command, "set_speed") == 0) {
        if (!has_value) {
            mqtt_gateway_publish_ack(command_id, false, "Missing speed value");
        } else if (mqtt_gateway_require_link(&snapshot, command_id)) {
            const int32_t speed = mqtt_gateway_clamp_i32(
                value, -MQTT_GATEWAY_SPEED_LIMIT_RPM,
                MQTT_GATEWAY_SPEED_LIMIT_RPM);
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
    } else if (strcmp(command, "set_pid") == 0) {
        int32_t controller = 0;
        int32_t kp = 0;
        int32_t ki = 0;
        int32_t kd = 0;
        const bool has_kd = mqtt_gateway_json_int(payload, "kd", &kd);
        const bool valid =
            mqtt_gateway_json_int(payload, "controller", &controller) &&
            mqtt_gateway_json_int(payload, "kp", &kp) &&
            mqtt_gateway_json_int(payload, "ki", &ki) &&
            controller >= 0 && controller <= 3 &&
            kp >= 0 && kp <= INT16_MAX &&
            ki >= 0 && ki <= INT16_MAX &&
            (controller != COMM_MGR_ESP_PID_POSITION ||
             (has_kd && kd >= 0 && kd <= INT16_MAX));
        if (!valid) {
            mqtt_gateway_publish_ack(command_id, false, "Invalid PID values");
        } else if (snapshot.motor_running) {
            mqtt_gateway_publish_ack(
                command_id, false, "Stop motor before PID update");
        } else {
            mqtt_gateway_update_pid_snapshot(
                (uint8_t)controller, (int16_t)kp, (int16_t)ki,
                (int16_t)kd);
            const CommMgr_ESP_pid_controller_t pid_controller =
                (CommMgr_ESP_pid_controller_t)controller;
            if (snapshot.transport == COMM_MGR_ESP_CAN &&
                snapshot.link_active) {
                CommMgr_ESP_SetPidGain(
                    pid_controller, COMM_MGR_ESP_PID_KP, (int16_t)kp);
                CommMgr_ESP_SetPidGain(
                    pid_controller, COMM_MGR_ESP_PID_KI, (int16_t)ki);
                if (controller == COMM_MGR_ESP_PID_POSITION) {
                    CommMgr_ESP_SetPidGain(
                        pid_controller, COMM_MGR_ESP_PID_KD, (int16_t)kd);
                }
            }
            mqtt_gateway_publish_ack(
                command_id, true,
                snapshot.transport == COMM_MGR_ESP_CAN && snapshot.link_active
                    ? "PID applied and cached temporarily"
                    : "PID cached for the next local test");
        }
    } else if (strcmp(command, "start") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_Start();
            mqtt_gateway_publish_ack(command_id, true, "Start accepted");
        }
    } else if (strcmp(command, "ack_fault") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_AcknowledgeFault();
            mqtt_gateway_publish_ack(
                command_id, true, "Fault reset accepted");
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

static void mqtt_gateway_message_callback(
    const char *topic, const char *payload, void *context)
{
    (void)context;
    if (topic == NULL || payload == NULL ||
        strcmp(topic, MQTT_MANAGER_CONTROL_TOPIC) != 0 ||
        s_command_queue == NULL) {
        return;
    }
    mqtt_gateway_command_t command;
    memset(&command, 0, sizeof(command));
    strlcpy(command.payload, payload, sizeof(command.payload));
    if (xQueueSend(s_command_queue, &command, 0U) != pdTRUE) {
        mqtt_gateway_command_t discarded;
        (void)xQueueReceive(s_command_queue, &discarded, 0U);
        (void)xQueueSend(s_command_queue, &command, 0U);
    }
}

static void mqtt_gateway_publish_telemetry(void)
{
    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);
    char payload[320];
    snprintf(payload, sizeof(payload),
             "{\"version\":2,\"transport\":%u,\"can_online\":%s,"
             "\"link_active\":%s,\"running\":%s,\"motor_fault\":%s,"
             "\"mode\":%u,\"faults\":%u,\"sample_sequence\":%lu}",
             (unsigned)snapshot.transport,
             snapshot.can_link_active ? "true" : "false",
             snapshot.link_active ? "true" : "false",
             snapshot.motor_running ? "true" : "false",
             snapshot.motor_fault ? "true" : "false",
             (unsigned)snapshot.mode,
             (unsigned)snapshot.faults,
             (unsigned long)snapshot.sample_sequence);
    (void)mqtt_manager_publish_qos0(MQTT_MOTOR_TELEMETRY_TOPIC, payload);
}

static void mqtt_gateway_task(void *argument)
{
    (void)argument;
    TickType_t last_publish = xTaskGetTickCount();
    mqtt_gateway_command_t command;

    while (true) {
        if (xQueueReceive(
                s_command_queue, &command,
                pdMS_TO_TICKS(1U)) == pdTRUE) {
            mqtt_gateway_process_command(command.payload, command.local);
        }
        mqtt_gateway_service_test();
        const bool telemetry_allowed =
            s_test.stage == MQTT_TEST_IDLE ||
            s_test.stage == MQTT_TEST_SEND_FAILED;
        if (telemetry_allowed &&
            xTaskGetTickCount() - last_publish >=
            pdMS_TO_TICKS(MQTT_GATEWAY_TELEMETRY_PERIOD_MS)) {
            last_publish = xTaskGetTickCount();
            mqtt_gateway_publish_telemetry();
        } else if (!telemetry_allowed) {
            /* 采样/上传期间暂停遥测；CAN 样本只进入 PSRAM。 */
            last_publish = xTaskGetTickCount();
        }
    }
}

esp_err_t mqtt_motor_gateway_start_local_test(
    mqtt_motor_local_test_mode_t mode)
{
    if (mode != MQTT_MOTOR_LOCAL_TEST_SPEED &&
        mode != MQTT_MOTOR_LOCAL_TEST_POSITION) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_test_snapshot_lock);
    const bool busy =
        s_test_snapshot.state == MQTT_MOTOR_TEST_UI_TESTING ||
        s_test_snapshot.state == MQTT_MOTOR_TEST_UI_UPLOADING ||
        s_test_snapshot.resend_available;
    if (busy) {
        portEXIT_CRITICAL(&s_test_snapshot_lock);
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t command_id = ++s_next_local_test_id;
    s_test_snapshot.state = MQTT_MOTOR_TEST_UI_TESTING;
    s_test_snapshot.mode = mode;
    s_test_snapshot.command_id = command_id;
    s_test_snapshot.duration_ms = MQTT_MOTOR_LOCAL_TEST_DURATION_MS;
    s_test_snapshot.sample_count = 0U;
    s_test_snapshot.uploaded_sample_count = 0U;
    s_test_snapshot.resend_available = false;
    strlcpy(s_test_snapshot.message, "Preparing local test",
            sizeof(s_test_snapshot.message));
    s_test_snapshot.revision++;
    portEXIT_CRITICAL(&s_test_snapshot_lock);

    mqtt_gateway_command_t command = {.local = true};
    const int32_t target = mode == MQTT_MOTOR_LOCAL_TEST_SPEED
        ? MQTT_MOTOR_LOCAL_SPEED_TARGET_RPM
        : (int32_t)MQTT_MOTOR_LOCAL_POSITION_TARGET_CDEG;
    snprintf(command.payload, sizeof(command.payload),
             "{\"id\":%lu,\"cmd\":\"run_test\",\"mode\":%u,"
             "\"target\":%ld,\"duration_ms\":%u}",
             (unsigned long)command_id, (unsigned)mode,
             (long)target, (unsigned)MQTT_MOTOR_LOCAL_TEST_DURATION_MS);
    if (xQueueSendToFront(s_command_queue, &command, 0U) != pdTRUE) {
        mqtt_gateway_update_test_snapshot(
            MQTT_MOTOR_TEST_UI_ERROR, "Test request queue is full");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t mqtt_motor_gateway_retry_upload(void)
{
    if (s_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t command_id = 0U;
    portENTER_CRITICAL(&s_test_snapshot_lock);
    if (!s_test_snapshot.resend_available ||
        s_test_snapshot.command_id == 0U) {
        portEXIT_CRITICAL(&s_test_snapshot_lock);
        return ESP_ERR_INVALID_STATE;
    }
    command_id = s_test_snapshot.command_id;
    s_test_snapshot.state = MQTT_MOTOR_TEST_UI_UPLOADING;
    s_test_snapshot.resend_available = false;
    strlcpy(s_test_snapshot.message, "Resend queued",
            sizeof(s_test_snapshot.message));
    s_test_snapshot.revision++;
    portEXIT_CRITICAL(&s_test_snapshot_lock);

    mqtt_gateway_command_t command = {.local = true};
    snprintf(command.payload, sizeof(command.payload),
             "{\"id\":%lu,\"cmd\":\"retry_upload\"}",
             (unsigned long)command_id);
    if (xQueueSendToFront(s_command_queue, &command, 0U) != pdTRUE) {
        portENTER_CRITICAL(&s_test_snapshot_lock);
        s_test_snapshot.state = MQTT_MOTOR_TEST_UI_UPLOAD_FAILED;
        s_test_snapshot.resend_available = true;
        strlcpy(s_test_snapshot.message, "Resend request queue is full",
                sizeof(s_test_snapshot.message));
        s_test_snapshot.revision++;
        portEXIT_CRITICAL(&s_test_snapshot_lock);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void mqtt_motor_gateway_get_test_snapshot(
    mqtt_motor_test_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_test_snapshot_lock);
    *snapshot = s_test_snapshot;
    portEXIT_CRITICAL(&s_test_snapshot_lock);
}

esp_err_t mqtt_motor_gateway_init(void)
{
    if (s_gateway_task != NULL) {
        return ESP_OK;
    }
    mqtt_gateway_reset_test();
    memset(&s_test_snapshot, 0, sizeof(s_test_snapshot));
    memcpy(s_test_snapshot.pid, s_default_pid, sizeof(s_default_pid));
    s_test_snapshot.state = MQTT_MOTOR_TEST_UI_IDLE;
    s_test_snapshot.duration_ms = MQTT_MOTOR_LOCAL_TEST_DURATION_MS;
    strlcpy(s_test_snapshot.message, "Ready",
            sizeof(s_test_snapshot.message));
    s_test_snapshot.revision = 1U;
    s_command_queue = xQueueCreate(
        MQTT_GATEWAY_QUEUE_LENGTH, sizeof(mqtt_gateway_command_t));
    if (s_command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    mqtt_manager_set_message_callback(mqtt_gateway_message_callback, NULL);
    if (xTaskCreate(
            mqtt_gateway_task, "mqtt_motor",
            MQTT_GATEWAY_TASK_STACK_SIZE, NULL,
            MQTT_GATEWAY_TASK_PRIORITY, &s_gateway_task) != pdPASS) {
        mqtt_manager_set_message_callback(NULL, NULL);
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    const size_t psram_total =
        heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const size_t psram_free =
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (psram_total == 0U) {
        ESP_LOGE(TAG,
                 "MQTT motor test gateway ready, but PSRAM is unavailable; "
                 "check boot log and CONFIG_SPIRAM settings");
    } else {
        ESP_LOGI(TAG,
                 "MQTT motor test gateway ready; PSRAM total=%zu bytes, "
                 "free=%zu bytes; test buffer allocated from PSRAM per test",
                 psram_total, psram_free);
    }
    return ESP_OK;
}
