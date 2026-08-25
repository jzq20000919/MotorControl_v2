#ifndef MQTT_MOTOR_GATEWAY_H
#define MQTT_MOTOR_GATEWAY_H

#include <stdint.h>

#include "esp_err.h"

/** @brief 周期发布电机遥测数据的 MQTT 主题。 */
#define MQTT_MOTOR_TELEMETRY_TOPIC "motor/control/telemetry"
/** @brief 发布电机命令执行确认的 MQTT 主题。 */
#define MQTT_MOTOR_ACK_TOPIC       "motor/control/ack"
/** @brief 发布 ESP32 本地测试阶段与结果摘要的主题。 */
#define MQTT_MOTOR_TEST_STATUS_TOPIC "motor/control/test/status"
/** @brief 测试结束后发布二进制采样分块的主题。 */
#define MQTT_MOTOR_TEST_DATA_TOPIC   "motor/control/test/data"

/** @brief ESP32 本地 PID 测试页使用的固定测试参数。 */
#define MQTT_MOTOR_LOCAL_TEST_DURATION_MS       7000U
#define MQTT_MOTOR_LOCAL_SPEED_TARGET_RPM       500
#define MQTT_MOTOR_LOCAL_POSITION_TARGET_CDEG   9000U

typedef enum
{
    MQTT_MOTOR_LOCAL_TEST_SPEED = 0,
    MQTT_MOTOR_LOCAL_TEST_POSITION = 1
} mqtt_motor_local_test_mode_t;

typedef enum
{
    MQTT_MOTOR_TEST_UI_IDLE = 0,
    MQTT_MOTOR_TEST_UI_TESTING,
    MQTT_MOTOR_TEST_UI_UPLOADING,
    MQTT_MOTOR_TEST_UI_UPLOAD_SUCCESS,
    MQTT_MOTOR_TEST_UI_ERROR
} mqtt_motor_test_ui_state_t;

/** @brief 本地 PID 测试页显示所需的只读网关快照。 */
typedef struct
{
    mqtt_motor_test_ui_state_t state;
    mqtt_motor_local_test_mode_t mode;
    uint32_t command_id;
    uint32_t duration_ms;
    uint16_t sample_count;
    int16_t pid[4][3];
    uint32_t revision;
    char message[80];
} mqtt_motor_test_snapshot_t;

/**
 * @brief 启动 MQTT 到 CommMgr_ESP 的命令网关及遥测发布器。
 * @return 工作任务和 MQTT 回调注册成功时返回 ESP_OK。
 */
esp_err_t mqtt_motor_gateway_init(void);

/** @brief 从 ESP32 本地界面排队启动一次速度或位置 PID 测试。 */
esp_err_t mqtt_motor_gateway_start_local_test(
    mqtt_motor_local_test_mode_t mode);

/** @brief 获取当前测试状态和运行时 PID 参数快照。 */
void mqtt_motor_gateway_get_test_snapshot(
    mqtt_motor_test_snapshot_t *snapshot);

#endif /* MQTT_MOTOR_GATEWAY_H */
