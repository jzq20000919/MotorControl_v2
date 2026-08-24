#include "mqtt_motor_gateway.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comm_mgr_ESP.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"
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
#define MQTT_GATEWAY_DEFAULT_UART_BAUD      115200U
#define MQTT_GATEWAY_SPEED_LIMIT_RPM       2600
#define MQTT_GATEWAY_TEST_DURATION_MS      7000U
#define MQTT_GATEWAY_TEST_MAX_SAMPLES      3600U
#define MQTT_GATEWAY_TEST_CHUNK_SAMPLES    40U
#define MQTT_GATEWAY_TEST_HEADER_SIZE      20U
#define MQTT_GATEWAY_TEST_RECORD_SIZE      20U
#define MQTT_GATEWAY_TEST_FLASH_BYTES      (64U * 1024U)
#define MQTT_GATEWAY_WRITER_QUEUE_LENGTH   128U
#define MQTT_GATEWAY_WRITER_BATCH_SAMPLES  32U
#define MQTT_GATEWAY_WRITER_STACK_SIZE     3072U
#define MQTT_GATEWAY_WRITER_PRIORITY       4U

typedef struct
{
    char payload[MQTT_GATEWAY_COMMAND_MAX_LEN + 1U];
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
    MQTT_TEST_WRITER_BEGIN = 0,
    MQTT_TEST_WRITER_SAMPLE,
    MQTT_TEST_WRITER_FLUSH
} mqtt_test_writer_command_t;

typedef struct
{
    mqtt_test_writer_command_t command;
    mqtt_test_sample_t sample;
} mqtt_test_writer_item_t;

typedef enum
{
    MQTT_TEST_IDLE = 0,
    MQTT_TEST_ACTIVATING_CAN,
    MQTT_TEST_WAIT_LINK,
    MQTT_TEST_CONFIGURING,
    MQTT_TEST_WAIT_RUNNING,
    MQTT_TEST_RECORDING,
    MQTT_TEST_WAIT_STOP,
    MQTT_TEST_FLUSHING,
    MQTT_TEST_PUBLISHING
} mqtt_test_stage_t;

typedef struct
{
    mqtt_test_stage_t stage;
    uint32_t command_id;
    CommMgr_ESP_mode_t mode;
    int32_t target;
    uint32_t duration_ms;
    int16_t pid[4][3];
    int64_t deadline_us;
    int64_t first_sample_timestamp_us;
    uint32_t last_sample_sequence;
    uint16_t sample_count;
    uint16_t publish_index;
    int64_t next_publish_us;
    bool aborted;
    uint32_t last_sample_time_us;
} mqtt_test_state_t;

static const char *TAG = "MQTT_MOTOR";
static QueueHandle_t s_command_queue;
static TaskHandle_t s_gateway_task;
static QueueHandle_t s_writer_queue;
static TaskHandle_t s_writer_task;
static mqtt_test_state_t s_test;
static portMUX_TYPE s_writer_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_test_flash_offset;
static bool s_writer_flush_complete;
static bool s_writer_error;

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

static void mqtt_gateway_set_writer_state(bool flush_complete, bool error)
{
    portENTER_CRITICAL(&s_writer_lock);
    s_writer_flush_complete = flush_complete;
    if (error) {
        s_writer_error = true;
    }
    portEXIT_CRITICAL(&s_writer_lock);
}

static void mqtt_gateway_get_writer_state(
    bool *flush_complete,
    bool *error)
{
    portENTER_CRITICAL(&s_writer_lock);
    *flush_complete = s_writer_flush_complete;
    *error = s_writer_error;
    portEXIT_CRITICAL(&s_writer_lock);
}

static bool mqtt_gateway_flash_write_samples(
    uint32_t *write_offset,
    const mqtt_test_sample_t *samples,
    uint16_t count)
{
    const uint32_t bytes =
        (uint32_t)count * (uint32_t)sizeof(mqtt_test_sample_t);
    if (*write_offset + bytes > MQTT_GATEWAY_TEST_FLASH_BYTES) {
        return false;
    }
    if (esp_flash_write(NULL, samples,
                        s_test_flash_offset + *write_offset,
                        bytes) != ESP_OK) {
        return false;
    }
    *write_offset += bytes;
    return true;
}

static void mqtt_gateway_writer_task(void *argument)
{
    (void)argument;
    mqtt_test_sample_t batch[MQTT_GATEWAY_WRITER_BATCH_SAMPLES];
    uint16_t batch_count = 0U;
    uint32_t write_offset = 0U;
    mqtt_test_writer_item_t item;

    for (;;) {
        if (xQueueReceive(s_writer_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.command == MQTT_TEST_WRITER_BEGIN) {
            batch_count = 0U;
            write_offset = 0U;
            portENTER_CRITICAL(&s_writer_lock);
            s_writer_flush_complete = false;
            s_writer_error = false;
            portEXIT_CRITICAL(&s_writer_lock);
            continue;
        }
        if (item.command == MQTT_TEST_WRITER_SAMPLE) {
            batch[batch_count++] = item.sample;
            if (batch_count == MQTT_GATEWAY_WRITER_BATCH_SAMPLES) {
                if (!mqtt_gateway_flash_write_samples(
                        &write_offset, batch, batch_count)) {
                    mqtt_gateway_set_writer_state(false, true);
                }
                batch_count = 0U;
            }
            continue;
        }
        if (item.command == MQTT_TEST_WRITER_FLUSH) {
            bool error = false;
            if (batch_count > 0U &&
                !mqtt_gateway_flash_write_samples(
                    &write_offset, batch, batch_count)) {
                error = true;
            }
            batch_count = 0U;
            mqtt_gateway_set_writer_state(true, error);
        }
    }
}

static bool mqtt_gateway_queue_writer_command(
    mqtt_test_writer_command_t command,
    const mqtt_test_sample_t *sample)
{
    mqtt_test_writer_item_t item = {.command = command};
    if (sample != NULL) {
        item.sample = *sample;
    }
    return s_writer_queue != NULL &&
        xQueueSend(s_writer_queue, &item, 0U) == pdTRUE;
}

static void mqtt_gateway_publish_ack(
    uint32_t command_id,
    bool accepted,
    const char *message)
{
    char payload[192];
    snprintf(payload, sizeof(payload),
             "{\"id\":%lu,\"ok\":%s,\"message\":\"%s\"}",
             (unsigned long)command_id,
             accepted ? "true" : "false",
             message != NULL ? message : "");
    (void)mqtt_manager_publish(MQTT_MOTOR_ACK_TOPIC, payload);
}

static void mqtt_gateway_publish_test_status(
    const char *stage,
    const char *message)
{
    uint32_t sample_period_us = 0U;
    if (s_test.sample_count > 1U) {
        sample_period_us =
            s_test.last_sample_time_us / (s_test.sample_count - 1U);
    }
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
    (void)mqtt_manager_publish(MQTT_MOTOR_TEST_STATUS_TOPIC, payload);
}

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
    if (end == cursor || parsed < INT32_MIN || parsed > INT32_MAX) {
        return false;
    }
    *value = (int32_t)parsed;
    return true;
}

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
    memset(&s_test, 0, sizeof(s_test));
    s_test.stage = MQTT_TEST_IDLE;
}

static bool mqtt_gateway_read_test_pid(const char *payload)
{
    static const char *keys[4][3] = {
        {"sp_kp", "sp_ki", NULL},
        {"pos_kp", "pos_ki", "pos_kd"},
        {"iq_kp", "iq_ki", NULL},
        {"id_kp", "id_ki", NULL}
    };
    for (uint8_t controller = 0U; controller < 4U; controller++) {
        for (uint8_t term = 0U; term < 3U; term++) {
            int32_t value = 0;
            if (keys[controller][term] != NULL &&
                !mqtt_gateway_json_int(
                    payload, keys[controller][term], &value)) {
                return false;
            }
            if (value < 0 || value > INT16_MAX) {
                return false;
            }
            s_test.pid[controller][term] = (int16_t)value;
        }
    }
    return true;
}

static void mqtt_gateway_apply_test_configuration(void)
{
    for (uint8_t controller = 0U; controller < 4U; controller++) {
        for (uint8_t term = 0U; term < 3U; term++) {
            CommMgr_ESP_SetPidGain(
                (CommMgr_ESP_pid_controller_t)controller,
                (CommMgr_ESP_pid_term_t)term,
                s_test.pid[controller][term]);
        }
    }
    CommMgr_ESP_SetMode(s_test.mode);
}

static void mqtt_gateway_fail_test(const char *message)
{
    CommMgr_ESP_Stop();
    mqtt_gateway_publish_test_status("error", message);
    mqtt_gateway_reset_test();
}

static void mqtt_gateway_begin_test_publish(void)
{
    if (s_test.sample_count == 0U) {
        mqtt_gateway_fail_test("No CAN samples recorded");
        return;
    }
    s_test.stage = MQTT_TEST_PUBLISHING;
    s_test.publish_index = 0U;
    s_test.next_publish_us = esp_timer_get_time();
    mqtt_gateway_publish_test_status(
        "publishing", "Sending recorded CAN samples");
}

static bool mqtt_gateway_publish_test_chunk(void)
{
    const uint16_t remaining =
        (uint16_t)(s_test.sample_count - s_test.publish_index);
    const uint16_t count = remaining > MQTT_GATEWAY_TEST_CHUNK_SAMPLES
        ? MQTT_GATEWAY_TEST_CHUNK_SAMPLES : remaining;
    uint8_t packet[MQTT_GATEWAY_TEST_HEADER_SIZE +
                   MQTT_GATEWAY_TEST_CHUNK_SAMPLES *
                       MQTT_GATEWAY_TEST_RECORD_SIZE];
    mqtt_test_sample_t stored[MQTT_GATEWAY_TEST_CHUNK_SAMPLES];
    const uint32_t stored_bytes =
        (uint32_t)count * (uint32_t)sizeof(mqtt_test_sample_t);
    if (esp_flash_read(NULL, stored,
                       s_test_flash_offset +
                           (uint32_t)s_test.publish_index *
                               (uint32_t)sizeof(mqtt_test_sample_t),
                       stored_bytes) != ESP_OK) {
        return false;
    }
    memset(packet, 0, sizeof(packet));
    packet[0] = 'M';
    packet[1] = 'C';
    packet[2] = 'T';
    packet[3] = 'D';
    packet[4] = 1U;
    packet[5] = (uint8_t)s_test.mode;
    packet[6] = MQTT_GATEWAY_TEST_RECORD_SIZE;
    packet[7] = (uint8_t)(s_test.aborted ? 2U : 0U);
    if ((uint16_t)(s_test.publish_index + count) == s_test.sample_count) {
        packet[7] |= 1U;
    }
    mqtt_gateway_write_u32(&packet[8], s_test.command_id);
    mqtt_gateway_write_u32(&packet[12], s_test.publish_index);
    mqtt_gateway_write_u16(&packet[16], count);
    mqtt_gateway_write_u16(&packet[18], s_test.sample_count);

    for (uint16_t index = 0U; index < count; index++) {
        const mqtt_test_sample_t *sample =
            &stored[index];
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
    if (mqtt_manager_publish_binary_qos1(
            MQTT_MOTOR_TEST_DATA_TOPIC, packet, packet_size) != ESP_OK) {
        return false;
    }
    s_test.publish_index = (uint16_t)(s_test.publish_index + count);
    return true;
}

static void mqtt_gateway_record_sample(const CommMgr_ESP_State *snapshot)
{
    if (snapshot->sample_sequence == s_test.last_sample_sequence ||
        s_test.sample_count >= MQTT_GATEWAY_TEST_MAX_SAMPLES) {
        return;
    }
    s_test.last_sample_sequence = snapshot->sample_sequence;
    if (s_test.first_sample_timestamp_us == 0LL) {
        s_test.first_sample_timestamp_us = snapshot->sample_timestamp_us;
    }
    int64_t relative =
        snapshot->sample_timestamp_us - s_test.first_sample_timestamp_us;
    if (relative < 0LL) {
        relative = 0LL;
    }
    mqtt_test_sample_t sample = {0};
    sample.time_us = relative > UINT32_MAX
        ? UINT32_MAX : (uint32_t)relative;
    if (s_test.mode == COMM_MGR_ESP_MODE_SPEED) {
        sample.primary_measured = snapshot->measured_speed_rpm;
        sample.primary_reference = snapshot->reference_speed_rpm;
    } else {
        sample.primary_measured =
            (int16_t)snapshot->current_position_cdeg;
        sample.primary_reference =
            (int16_t)snapshot->target_position_cdeg;
    }
    sample.iq_ma = snapshot->iq_ma;
    sample.id_ma = snapshot->id_ma;
    sample.iq_reference_ma = snapshot->iq_reference_ma;
    sample.id_reference_ma = snapshot->id_reference_ma;
    if (!mqtt_gateway_queue_writer_command(
            MQTT_TEST_WRITER_SAMPLE, &sample)) {
        mqtt_gateway_set_writer_state(false, true);
        return;
    }
    s_test.last_sample_time_us = sample.time_us;
    s_test.sample_count++;
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
    case MQTT_TEST_ACTIVATING_CAN:
        if (CommMgr_ESP_SelectCAN() != ESP_OK) {
            mqtt_gateway_fail_test("CAN activation failed");
            break;
        }
        s_test.stage = MQTT_TEST_WAIT_LINK;
        s_test.deadline_us = now_us + 3000000LL;
        mqtt_gateway_publish_test_status(
            "waiting_can", "Waiting for high-rate CAN feedback");
        break;

    case MQTT_TEST_WAIT_LINK:
        if (snapshot.transport == COMM_MGR_ESP_CAN &&
            snapshot.link_active && !snapshot.motor_running) {
            if (snapshot.motor_fault) {
                mqtt_gateway_fail_test("STM32 has an active motor fault");
                break;
            }
            if (esp_flash_erase_region(
                    NULL, s_test_flash_offset,
                    MQTT_GATEWAY_TEST_FLASH_BYTES) != ESP_OK) {
                mqtt_gateway_fail_test("Test flash erase failed");
                break;
            }
            if (!mqtt_gateway_queue_writer_command(
                    MQTT_TEST_WRITER_BEGIN, NULL)) {
                mqtt_gateway_fail_test("Test flash writer unavailable");
                break;
            }
            mqtt_gateway_apply_test_configuration();
            s_test.stage = MQTT_TEST_CONFIGURING;
            s_test.deadline_us = now_us + 150000LL;
            mqtt_gateway_publish_test_status(
                "configuring", "Applying PID and control mode");
        } else if (now_us >= s_test.deadline_us) {
            mqtt_gateway_fail_test("CAN link did not become ready");
        }
        break;

    case MQTT_TEST_CONFIGURING:
        if (now_us >= s_test.deadline_us) {
            if (snapshot.command_rejected || snapshot.mode != s_test.mode) {
                mqtt_gateway_fail_test("STM32 rejected PID or mode setup");
                break;
            }
            CommMgr_ESP_Start();
            s_test.stage = MQTT_TEST_WAIT_RUNNING;
            s_test.deadline_us = now_us + 5000000LL;
            mqtt_gateway_publish_test_status(
                "starting", "Waiting for motor RUN state");
        }
        break;

    case MQTT_TEST_WAIT_RUNNING:
        if (snapshot.motor_running) {
            if (s_test.mode == COMM_MGR_ESP_MODE_SPEED) {
                CommMgr_ESP_SetSpeedRPM((int16_t)s_test.target);
            } else {
                CommMgr_ESP_SetPositionCdeg((uint16_t)s_test.target);
            }
            s_test.stage = MQTT_TEST_RECORDING;
            s_test.last_sample_sequence = snapshot.sample_sequence;
            s_test.first_sample_timestamp_us = 0LL;
            mqtt_gateway_publish_test_status(
                "recording", "Target applied; recording CAN feedback");
        } else if (snapshot.motor_fault) {
            mqtt_gateway_fail_test("Motor fault occurred during startup");
        } else if (snapshot.command_rejected) {
            mqtt_gateway_fail_test("STM32 rejected the START command");
        } else if (now_us >= s_test.deadline_us) {
            mqtt_gateway_fail_test("Motor did not enter RUN state");
        }
        break;

    case MQTT_TEST_RECORDING:
        if (!snapshot.link_active || !snapshot.motor_running) {
            s_test.aborted = true;
            CommMgr_ESP_Stop();
            s_test.stage = MQTT_TEST_WAIT_STOP;
            s_test.deadline_us = now_us + 3000000LL;
            mqtt_gateway_publish_test_status(
                "stopping", "Test ended early; stopping motor");
            break;
        }
        mqtt_gateway_record_sample(&snapshot);
        bool writer_flush_complete = false;
        bool writer_error = false;
        mqtt_gateway_get_writer_state(
            &writer_flush_complete, &writer_error);
        (void)writer_flush_complete;
        if (writer_error) {
            s_test.aborted = true;
            CommMgr_ESP_Stop();
            s_test.stage = MQTT_TEST_WAIT_STOP;
            s_test.deadline_us = now_us + 3000000LL;
            mqtt_gateway_publish_test_status(
                "stopping", "Flash writer error; stopping motor");
            break;
        }
        if (s_test.sample_count >= MQTT_GATEWAY_TEST_MAX_SAMPLES ||
            (s_test.sample_count > 0U &&
             s_test.last_sample_time_us >=
                 s_test.duration_ms * 1000U)) {
            CommMgr_ESP_Stop();
            s_test.stage = MQTT_TEST_WAIT_STOP;
            s_test.deadline_us = now_us + 3000000LL;
            mqtt_gateway_publish_test_status(
                "stopping", "Recording complete; stopping motor");
        }
        break;

    case MQTT_TEST_WAIT_STOP:
        if (!snapshot.motor_running || now_us >= s_test.deadline_us) {
            if (mqtt_gateway_queue_writer_command(
                    MQTT_TEST_WRITER_FLUSH, NULL)) {
                s_test.stage = MQTT_TEST_FLUSHING;
                mqtt_gateway_publish_test_status(
                    "flushing", "Finalizing recorded CAN samples");
            }
        }
        break;

    case MQTT_TEST_FLUSHING: {
        bool flush_complete = false;
        bool writer_error = false;
        mqtt_gateway_get_writer_state(&flush_complete, &writer_error);
        if (writer_error) {
            mqtt_gateway_fail_test("Test flash write failed");
        } else if (flush_complete) {
            mqtt_gateway_begin_test_publish();
        }
        break;
    }

    case MQTT_TEST_PUBLISHING:
        if (s_test.publish_index < s_test.sample_count) {
            if (now_us >= s_test.next_publish_us) {
                const bool queued = mqtt_gateway_publish_test_chunk();
                s_test.next_publish_us = now_us +
                    (queued ? 20000LL : 10000LL);
            }
        } else {
            mqtt_gateway_publish_test_status(
                s_test.aborted ? "aborted" : "complete",
                s_test.aborted
                    ? "Partial CAN dataset sent"
                    : "CAN dataset sent");
            mqtt_gateway_reset_test();
        }
        break;

    case MQTT_TEST_IDLE:
    default:
        break;
    }
}

static void mqtt_gateway_start_test(
    const char *payload,
    uint32_t command_id)
{
    int32_t mode = 0;
    int32_t target = 0;
    int32_t duration_ms = MQTT_GATEWAY_TEST_DURATION_MS;
    if (s_test.stage != MQTT_TEST_IDLE) {
        mqtt_gateway_publish_ack(command_id, false, "Test already running");
        return;
    }
    if (!mqtt_gateway_json_int(payload, "mode", &mode) ||
        !mqtt_gateway_json_int(payload, "target", &target)) {
        mqtt_gateway_publish_ack(command_id, false, "Missing test fields");
        return;
    }
    (void)mqtt_gateway_json_int(payload, "duration_ms", &duration_ms);
    if ((mode != 0 && mode != 1) ||
        duration_ms != (int32_t)MQTT_GATEWAY_TEST_DURATION_MS ||
        (mode == 0 && (target < -MQTT_GATEWAY_SPEED_LIMIT_RPM ||
                       target > MQTT_GATEWAY_SPEED_LIMIT_RPM))) {
        mqtt_gateway_publish_ack(command_id, false, "Invalid test settings");
        return;
    }
    if (mode == 1) {
        target %= 36000;
        if (target < 0) {
            target += 36000;
        }
    }

    memset(&s_test, 0, sizeof(s_test));
    s_test.command_id = command_id;
    s_test.mode = mode == 0
        ? COMM_MGR_ESP_MODE_SPEED : COMM_MGR_ESP_MODE_POSITION;
    s_test.target = target;
    s_test.duration_ms = (uint32_t)duration_ms;
    if (!mqtt_gateway_read_test_pid(payload)) {
        mqtt_gateway_reset_test();
        mqtt_gateway_publish_ack(command_id, false, "Invalid test PID values");
        return;
    }
    /*
     * 先完成 MQTT 命令确认，再由网关任务下一轮初始化 CAN 和分配记录内存。
     * 这样 CAN 硬件异常或内存紧张不会阻塞 ESP-MQTT 的入站 QoS1 处理。
     */
    s_test.stage = MQTT_TEST_ACTIVATING_CAN;
    mqtt_gateway_publish_ack(command_id, true, "ESP32 test accepted");
    mqtt_gateway_publish_test_status(
        "activating_can", "Activating CAN test transport");
}

static void mqtt_gateway_process_command(const char *payload)
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
        mqtt_gateway_start_test(payload, command_id);
        return;
    }

    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);
    if (strcmp(command, "stop") == 0) {
        if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_Stop();
            if (s_test.stage != MQTT_TEST_IDLE) {
                s_test.aborted = true;
                s_test.stage = MQTT_TEST_WAIT_STOP;
                s_test.deadline_us = esp_timer_get_time() + 3000000LL;
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
            result = CommMgr_ESP_SelectUSART(
                MQTT_GATEWAY_DEFAULT_UART_BAUD);
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
        const bool valid =
            mqtt_gateway_json_int(payload, "controller", &controller) &&
            mqtt_gateway_json_int(payload, "kp", &kp) &&
            mqtt_gateway_json_int(payload, "ki", &ki) &&
            mqtt_gateway_json_int(payload, "kd", &kd);
        if (!valid || controller < 0 || controller > 3 ||
            kp < 0 || kp > INT16_MAX || ki < 0 || ki > INT16_MAX ||
            kd < 0 || kd > INT16_MAX) {
            mqtt_gateway_publish_ack(command_id, false, "Invalid PID values");
        } else if (snapshot.motor_running) {
            mqtt_gateway_publish_ack(
                command_id, false, "Stop motor before PID update");
        } else if (mqtt_gateway_require_link(&snapshot, command_id)) {
            CommMgr_ESP_SetPidGain(
                (CommMgr_ESP_pid_controller_t)controller,
                COMM_MGR_ESP_PID_KP, (int16_t)kp);
            CommMgr_ESP_SetPidGain(
                (CommMgr_ESP_pid_controller_t)controller,
                COMM_MGR_ESP_PID_KI, (int16_t)ki);
            CommMgr_ESP_SetPidGain(
                (CommMgr_ESP_pid_controller_t)controller,
                COMM_MGR_ESP_PID_KD, (int16_t)kd);
            mqtt_gateway_publish_ack(
                command_id, true, "PID accepted temporarily");
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
    (void)mqtt_manager_publish_qos0(
        MQTT_MOTOR_TELEMETRY_TOPIC, payload);
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
            mqtt_gateway_process_command(command.payload);
        }
        mqtt_gateway_service_test();
        if (s_test.stage == MQTT_TEST_IDLE &&
            xTaskGetTickCount() - last_publish >=
            pdMS_TO_TICKS(MQTT_GATEWAY_TELEMETRY_PERIOD_MS)) {
            last_publish = xTaskGetTickCount();
            mqtt_gateway_publish_telemetry();
        } else if (s_test.stage != MQTT_TEST_IDLE) {
            /* 测试期间禁止周期 MQTT 遥测；CAN 样本只写本地 Flash。 */
            last_publish = xTaskGetTickCount();
        }
    }
}

esp_err_t mqtt_motor_gateway_init(void)
{
    if (s_gateway_task != NULL) {
        return ESP_OK;
    }
    uint32_t flash_size = 0U;
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK ||
        flash_size < MQTT_GATEWAY_TEST_FLASH_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }
    s_test_flash_offset =
        (flash_size - MQTT_GATEWAY_TEST_FLASH_BYTES) & ~0xFFFU;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL ||
        running->address + running->size > s_test_flash_offset) {
        return ESP_ERR_INVALID_SIZE;
    }
    memset(&s_test, 0, sizeof(s_test));
    s_writer_queue = xQueueCreate(
        MQTT_GATEWAY_WRITER_QUEUE_LENGTH,
        sizeof(mqtt_test_writer_item_t));
    if (s_writer_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(
            mqtt_gateway_writer_task, "test_flash",
            MQTT_GATEWAY_WRITER_STACK_SIZE, NULL,
            MQTT_GATEWAY_WRITER_PRIORITY,
            &s_writer_task) != pdPASS) {
        vQueueDelete(s_writer_queue);
        s_writer_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_command_queue = xQueueCreate(
        MQTT_GATEWAY_QUEUE_LENGTH, sizeof(mqtt_gateway_command_t));
    if (s_command_queue == NULL) {
        vTaskDelete(s_writer_task);
        s_writer_task = NULL;
        vQueueDelete(s_writer_queue);
        s_writer_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    mqtt_manager_set_message_callback(
        mqtt_gateway_message_callback, NULL);
    if (xTaskCreate(
            mqtt_gateway_task, "mqtt_motor",
            MQTT_GATEWAY_TASK_STACK_SIZE, NULL,
            MQTT_GATEWAY_TASK_PRIORITY, &s_gateway_task) != pdPASS) {
        mqtt_manager_set_message_callback(NULL, NULL);
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        vTaskDelete(s_writer_task);
        s_writer_task = NULL;
        vQueueDelete(s_writer_queue);
        s_writer_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "MQTT motor test gateway ready; flash spool at 0x%lx (%u bytes)",
             (unsigned long)s_test_flash_offset,
             (unsigned)MQTT_GATEWAY_TEST_FLASH_BYTES);
    return ESP_OK;
}
