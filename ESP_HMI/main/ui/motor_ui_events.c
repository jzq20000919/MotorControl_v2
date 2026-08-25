#include "motor_ui_events.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "comm_mgr_ESP.h"
#include "mqtt_manager.h"
#include "mqtt_motor_gateway.h"

static motor_ui_event_context_t s_context;

/** @brief 保存绘制模块提供的控件引用与共享状态地址。 */
void motor_ui_events_configure(const motor_ui_event_context_t *context)
{
    if (context != NULL) {
        s_context = *context;
    }
}

/** @brief 隐藏 Wi-Fi 密码软键盘。 */
static void ui_hide_wifi_keyboard(void)
{
    if (s_context.wifi_keyboard != NULL) {
        lv_keyboard_set_textarea(s_context.wifi_keyboard, NULL);
        lv_obj_add_flag(s_context.wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

/** @brief 隐藏 MQTT URI 软键盘。 */
static void ui_hide_mqtt_keyboard(void)
{
    if (s_context.mqtt_keyboard != NULL) {
        lv_keyboard_set_textarea(s_context.mqtt_keyboard, NULL);
        lv_obj_add_flag(s_context.mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

/** @brief 清除尚未被遥测确认的速度与位置命令。 */
static void ui_clear_pending_commands(void)
{
    *s_context.speed_command_pending = false;
    *s_context.position_command_pending = false;
}

/** @brief 处理导航按钮点击，并请求绘制模块切换目标页面。 */
void motor_ui_navigation_event(lv_event_t *event)
{
    const ui_page_t page =
        (ui_page_t)(uintptr_t)lv_event_get_user_data(event);
    motor_ui_internal_navigate(page);
}

/** @brief 请求切换到速度模式并启动电机。 */
void motor_ui_speed_mode_event(lv_event_t *event)
{
    (void)event;
    CommMgr_ESP_SetMode(COMM_MGR_ESP_MODE_SPEED);
    CommMgr_ESP_Start();
}

/** @brief 请求切换到位置模式并启动电机。 */
void motor_ui_position_mode_event(lv_event_t *event)
{
    (void)event;
    CommMgr_ESP_SetMode(COMM_MGR_ESP_MODE_POSITION);
    CommMgr_ESP_Start();
}

/** @brief 清除 UI 交互状态并请求停止电机。 */
void motor_ui_stop_event(lv_event_t *event)
{
    (void)event;
    ui_clear_pending_commands();
    *s_context.speed_dragging = false;
    *s_context.position_dragging = false;
    CommMgr_ESP_Stop();
}

/** @brief 请求确认并复位电机故障。 */
void motor_ui_ack_fault_event(lv_event_t *event)
{
    (void)event;
    CommMgr_ESP_AcknowledgeFault();
}

/** @brief 使用下拉框选定的波特率连接 UART 电机链路。 */
void motor_ui_uart_reconnect_event(lv_event_t *event)
{
    (void)event;
    uint16_t selected = lv_dropdown_get_selected(s_context.baud_dropdown);
    if (selected >= s_context.uart_baud_rate_count) {
        selected = 0U;
    }
    const uint32_t baud = s_context.uart_baud_rates[selected];
    if (CommMgr_ESP_SelectUSART(baud) == ESP_OK) {
        ui_clear_pending_commands();
        lv_label_set_text_fmt(s_context.home_state_label, "SERIAL CONNECTING %lu", (unsigned long)baud);
    } else {
        lv_label_set_text(s_context.home_state_label, "SERIAL CONNECT FAILED");
    }
}

/** @brief 初始化并选择 CAN 作为电机控制通道。 */
void motor_ui_can_connect_event(lv_event_t *event)
{
    (void)event;
    if (CommMgr_ESP_SelectCAN() == ESP_OK) {
        ui_clear_pending_commands();
        lv_label_set_text(s_context.can_state_label, "CAN CONNECTING 500K");
    } else {
        lv_label_set_text(s_context.can_state_label, "CAN CONNECT FAILED");
    }
}

/** @brief 断开 UART 电机控制通道。 */
void motor_ui_uart_disconnect_event(lv_event_t *event)
{
    (void)event;
    ui_clear_pending_commands();
    CommMgr_ESP_DisconnectUSART();
    lv_label_set_text(s_context.home_state_label, "USART DISCONNECTED");
}

/** @brief 断开 CAN 电机控制通道。 */
void motor_ui_can_disconnect_event(lv_event_t *event)
{
    (void)event;
    ui_clear_pending_commands();
    CommMgr_ESP_DisconnectCAN();
    lv_label_set_text(s_context.can_state_label, "CAN DISCONNECTED");
}

/** @brief 更新当前选中 Wi-Fi 接入点的详情文本。 */
void motor_ui_events_refresh_wifi_detail(void)
{
    if (s_context.wifi_detail_label == NULL) {
        return;
    }
    if (*s_context.wifi_network_count == 0U) {
        lv_label_set_text(s_context.wifi_detail_label, "No network selected");
        return;
    }

    uint16_t selected =
        lv_dropdown_get_selected(s_context.wifi_network_dropdown);
    if (selected >= *s_context.wifi_network_count) {
        selected = 0U;
    }
    wifi_manager_snapshot_t snapshot;
    wifi_manager_get_snapshot(&snapshot);
    int rssi = 0;
    if (selected < snapshot.ap_count &&
        strcmp(snapshot.aps[selected].ssid, s_context.wifi_network_ssids[selected]) == 0) {
        rssi = snapshot.aps[selected].rssi;
    }
    lv_label_set_text_fmt(s_context.wifi_detail_label, "%s\n%d dBm  %s", s_context.wifi_network_ssids[selected], rssi, s_context.wifi_network_secured[selected] ? "SECURED" : "OPEN");
}

/** @brief Wi-Fi 网络选择变化时刷新选中接入点详情。 */
void motor_ui_wifi_network_event(lv_event_t *event)
{
    (void)event;
    motor_ui_events_refresh_wifi_detail();
}

/** @brief Wi-Fi 密码框获得焦点时显示软键盘。 */
void motor_ui_wifi_password_event(lv_event_t *event)
{
    (void)event;
    lv_keyboard_set_textarea(s_context.wifi_keyboard, s_context.wifi_password_textarea);
    lv_obj_remove_flag(s_context.wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_context.wifi_keyboard);
}

/** @brief Wi-Fi 软键盘确认或取消后隐藏键盘。 */
void motor_ui_wifi_keyboard_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        ui_hide_wifi_keyboard();
    }
}

/** @brief 请求异步扫描 Wi-Fi 接入点。 */
void motor_ui_wifi_scan_event(lv_event_t *event)
{
    (void)event;
    const esp_err_t result = wifi_manager_scan_async();
    if (result != ESP_OK) {
        lv_label_set_text_fmt(s_context.wifi_page_state_label, "Scan unavailable: %s", esp_err_to_name(result));
    }
}

/** @brief 将选定 SSID 和密码提交给 Wi-Fi 管理器。 */
void motor_ui_wifi_connect_event(lv_event_t *event)
{
    (void)event;
    if (*s_context.wifi_network_count == 0U) {
        lv_label_set_text(s_context.wifi_page_state_label, "Scan and select a network");
        return;
    }
    uint16_t selected =
        lv_dropdown_get_selected(s_context.wifi_network_dropdown);
    if (selected >= *s_context.wifi_network_count) {
        selected = 0U;
    }
    const char *password =
        lv_textarea_get_text(s_context.wifi_password_textarea);
    if (s_context.wifi_network_secured[selected] && strlen(password) < 8U) {
        lv_label_set_text(s_context.wifi_page_state_label, "Secure network password must be 8+ characters");
        return;
    }
    if (!s_context.wifi_network_secured[selected]) {
        password = "";
    }
    ui_hide_wifi_keyboard();
    const esp_err_t result = wifi_manager_connect(s_context.wifi_network_ssids[selected], password);
    if (result != ESP_OK) {
        lv_label_set_text_fmt(s_context.wifi_page_state_label, "Connect unavailable: %s", esp_err_to_name(result));
    }
}

/** @brief 请求断开 Wi-Fi 并关闭自动重连。 */
void motor_ui_wifi_disconnect_event(lv_event_t *event)
{
    (void)event;
    ui_hide_wifi_keyboard();
    const esp_err_t result = wifi_manager_disconnect();
    if (result != ESP_OK) {
        lv_label_set_text_fmt(s_context.wifi_page_state_label, "Disconnect failed: %s", esp_err_to_name(result));
    }
}

/** @brief MQTT URI 输入框获得焦点时显示软键盘。 */
void motor_ui_mqtt_uri_event(lv_event_t *event)
{
    (void)event;
    lv_keyboard_set_textarea(s_context.mqtt_keyboard, s_context.mqtt_uri_textarea);
    lv_obj_remove_flag(s_context.mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_context.mqtt_keyboard);
}

/** @brief MQTT 软键盘确认或取消后隐藏键盘。 */
void motor_ui_mqtt_keyboard_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        ui_hide_mqtt_keyboard();
    }
}

/** @brief 校验 Broker URI 并请求异步连接 MQTT。 */
void motor_ui_mqtt_connect_event(lv_event_t *event)
{
    (void)event;
    wifi_manager_snapshot_t wifi_snapshot;
    wifi_manager_get_snapshot(&wifi_snapshot);
    if (!wifi_snapshot.connected) {
        lv_label_set_text(s_context.mqtt_page_state_label, "Connect Wi-Fi before MQTT");
        return;
    }
    ui_hide_mqtt_keyboard();
    const esp_err_t result = mqtt_manager_connect_async(lv_textarea_get_text(s_context.mqtt_uri_textarea));
    if (result != ESP_OK) {
        lv_label_set_text_fmt(s_context.mqtt_page_state_label, "Invalid broker URI: %s", esp_err_to_name(result));
    }
}

/** @brief 请求异步断开 MQTT 客户端。 */
void motor_ui_mqtt_disconnect_event(lv_event_t *event)
{
    (void)event;
    ui_hide_mqtt_keyboard();
    const esp_err_t result = mqtt_manager_disconnect_async();
    if (result != ESP_OK) {
        lv_label_set_text_fmt(s_context.mqtt_page_state_label, "Disconnect failed: %s", esp_err_to_name(result));
    }
}

/** @brief 发布测试消息并在失败时更新 MQTT 页面状态。 */
static void ui_mqtt_publish_test(const char *topic, const char *payload)
{
    if (mqtt_manager_publish(topic, payload) != ESP_OK) {
        lv_label_set_text(s_context.mqtt_page_state_label, "MQTT is offline - connect first");
    }
}

/** @brief 发布 MQTT 连通性测试消息。 */
void motor_ui_mqtt_ping_event(lv_event_t *event)
{
    (void)event;
    ui_mqtt_publish_test("motor/hmi/test/ping", "PING from ESP32-S3");
}

/** @brief 发布当前 Wi-Fi 状态测试消息。 */
void motor_ui_mqtt_wifi_event(lv_event_t *event)
{
    (void)event;
    wifi_manager_snapshot_t snapshot;
    wifi_manager_get_snapshot(&snapshot);
    char payload[128];
    snprintf(payload, sizeof(payload), "ssid=%s ip=%s", snapshot.ssid, snapshot.ip_address);
    ui_mqtt_publish_test("motor/hmi/test/wifi", payload);
}

/** @brief 发布当前电机状态测试消息。 */
void motor_ui_mqtt_motor_event(lv_event_t *event)
{
    (void)event;
    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);
    char payload[160];
    snprintf(payload, sizeof(payload), "running=%u mode=%s speed=%d position=%u.%02u", snapshot.motor_running ? 1U : 0U, snapshot.mode == COMM_MGR_ESP_MODE_SPEED ? "speed" : "position", snapshot.measured_speed_rpm, snapshot.current_position_cdeg / 100U, snapshot.current_position_cdeg % 100U);
    ui_mqtt_publish_test("motor/hmi/test/motor", payload);
}

/** @brief 从 ESP32 本地界面排队启动固定 500 RPM、7 秒速度测试。 */
void motor_ui_pid_speed_test_event(lv_event_t *event)
{
    (void)event;
    const esp_err_t result = mqtt_motor_gateway_start_local_test(
        MQTT_MOTOR_LOCAL_TEST_SPEED);
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_context.pid_test_state_label,
            "START FAILED: %s", esp_err_to_name(result));
    }
}

/** @brief 从 ESP32 本地界面排队启动固定 90°、7 秒位置测试。 */
void motor_ui_pid_position_test_event(lv_event_t *event)
{
    (void)event;
    const esp_err_t result = mqtt_motor_gateway_start_local_test(
        MQTT_MOTOR_LOCAL_TEST_POSITION);
    if (result != ESP_OK) {
        lv_label_set_text_fmt(
            s_context.pid_test_state_label,
            "START FAILED: %s", esp_err_to_name(result));
    }
}

/** @brief 重新排队上传 ESP32 PSRAM 中保留的完整测试数据集。 */
void motor_ui_pid_resend_event(lv_event_t *event)
{
    (void)event;
    const esp_err_t result = mqtt_motor_gateway_retry_upload();
    if (result == ESP_OK) {
        lv_label_set_text(
            s_context.pid_test_state_label, "RESEND QUEUED");
    } else {
        lv_label_set_text_fmt(
            s_context.pid_test_state_label,
            "RESEND FAILED: %s", esp_err_to_name(result));
    }
}

/** @brief 更新速度滑块文本，并在释放时提交最终转速。 */
void motor_ui_speed_slider_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_PRESSED) 
    {
        *s_context.speed_dragging = true;
        return;
    }
    if (code == LV_EVENT_VALUE_CHANGED) {
        const int32_t speed = lv_slider_get_value(s_context.speed_slider);
        lv_label_set_text_fmt(s_context.speed_slider_value, "%ld RPM", (long)speed);
        return;
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (*s_context.speed_dragging) {
            *s_context.pending_speed_rpm =
                (int16_t)lv_slider_get_value(s_context.speed_slider);
            *s_context.speed_command_pending = true;
            *s_context.speed_command_tick = lv_tick_get();
            CommMgr_ESP_SetMode(COMM_MGR_ESP_MODE_SPEED);
            CommMgr_ESP_SetSpeedRPM(*s_context.pending_speed_rpm);
        }
        *s_context.speed_dragging = false;
    }
}

/** @brief 更新位置滑块文本，并在释放时提交最终角度。 */
void motor_ui_position_slider_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_PRESSED) {
        *s_context.position_dragging = true;
    }
    if (code == LV_EVENT_VALUE_CHANGED) {
        const int32_t cdeg = lv_slider_get_value(s_context.position_slider);
        lv_label_set_text_fmt(s_context.position_target_label, "%3ld.%02ld deg", (long)(cdeg / 100L), (long)(cdeg % 100L));
        return;
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (*s_context.position_dragging) {
            *s_context.pending_position_cdeg =
                (uint16_t)lv_slider_get_value(s_context.position_slider);
            *s_context.position_command_pending = true;
            *s_context.position_command_tick = lv_tick_get();
            CommMgr_ESP_SetMode(COMM_MGR_ESP_MODE_POSITION);
            CommMgr_ESP_SetPositionCdeg(*s_context.pending_position_cdeg);
        }
        *s_context.position_dragging = false;
    }
}
