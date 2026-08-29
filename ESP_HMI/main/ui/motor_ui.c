#include "motor_ui.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "board_keys.h"
#include "mqtt_manager.h"
#include "mqtt_motor_gateway.h"
#include "comm_mgr_ESP.h"
#include "motor_ui_events.h"
#include "motor_ui_style.h"
#include "wifi_manager.h"

#define UI_VIEWPORT_TOP           22
#define UI_PAGE_TOP               0
#define UI_PAGE_HEIGHT            218
#define UI_SPEED_LIMIT_RPM        2600
#define UI_PAGE_ANIMATION_MS      160U

static const char *s_page_names[UI_PAGE_COUNT] = {
    "MENU",
    "FEEDBACK",
    "USART",
    "CAN",
    "WI-FI",
    "MQTT",
    "PID TEST",
    "SPEED",
    "POSITION",
};

static lv_obj_t *s_pages[UI_PAGE_COUNT];
static lv_obj_t *s_page_viewport;
static ui_page_t s_current_page;
static lv_obj_t *s_page_label;
static lv_obj_t *s_uart_status_label;
static lv_obj_t *s_can_status_label;
static lv_obj_t *s_wifi_status_label;
static lv_obj_t *s_mqtt_status_label;
static lv_obj_t *s_home_state_label;
static lv_obj_t *s_home_speed_measured_label;
static lv_obj_t *s_home_speed_target_label;
static lv_obj_t *s_home_position_current_label;
static lv_obj_t *s_home_position_target_label;
static lv_obj_t *s_home_current_label;
static lv_obj_t *s_home_current_reference_label;
static lv_obj_t *s_home_voltage_label;
static lv_obj_t *s_home_mode_label;
static lv_obj_t *s_home_run_label;
static lv_obj_t *s_home_fault_label;
static lv_obj_t *s_home_transport_label;
static lv_obj_t *s_can_state_label;
static lv_obj_t *s_link_diag_label;
static lv_obj_t *s_baud_dropdown;
static lv_obj_t *s_reconnect_button;
static lv_obj_t *s_uart_disconnect_button;
static lv_obj_t *s_can_connect_button;
static lv_obj_t *s_can_disconnect_button;
static lv_obj_t *s_wifi_network_dropdown;
static lv_obj_t *s_wifi_password_textarea;
static lv_obj_t *s_wifi_page_state_label;
static lv_obj_t *s_wifi_detail_label;
static lv_obj_t *s_wifi_keyboard;
static lv_obj_t *s_mqtt_uri_textarea;
static lv_obj_t *s_mqtt_page_state_label;
static lv_obj_t *s_mqtt_rx_label;
static lv_obj_t *s_mqtt_keyboard;
static lv_obj_t *s_pid_test_pid_label;
static lv_obj_t *s_pid_test_config_label;
static lv_obj_t *s_pid_test_state_label;
static lv_obj_t *s_pid_speed_test_button;
static lv_obj_t *s_pid_position_test_button;
static lv_obj_t *s_pid_resend_button;
static lv_obj_t *s_stop_button;
static lv_obj_t *s_speed_stop_button;
static lv_obj_t *s_position_stop_button;
static lv_obj_t *s_ack_button;
static lv_obj_t *s_speed_actual_label;
static lv_obj_t *s_speed_reference_label;
static lv_obj_t *s_speed_slider;
static lv_obj_t *s_speed_slider_value;
static lv_obj_t *s_speed_mode_button;
static lv_obj_t *s_speed_mode_button_label;
static lv_obj_t *s_position_current_label;
static lv_obj_t *s_position_target_label;
static lv_obj_t *s_position_slider;
static lv_obj_t *s_position_mode_button;
static lv_obj_t *s_position_mode_button_label;
static lv_obj_t *s_electrical_label;
static bool s_speed_dragging;
static bool s_speed_command_pending;
static int16_t s_pending_speed_rpm;
static uint32_t s_speed_command_tick;
static bool s_position_dragging;
static bool s_position_command_pending;
static uint16_t s_pending_position_cdeg;
static uint32_t s_position_command_tick;
static CommMgr_ESP_State s_previous_snapshot;
static bool s_have_previous_snapshot;
static uint8_t s_key_candidate;
static uint8_t s_key_stable;
static uint8_t s_key_debounce_count;
static bool s_page_animating;
static uint32_t s_wifi_revision = UINT32_MAX;
static uint32_t s_wifi_scan_generation = UINT32_MAX;
static uint32_t s_mqtt_revision = UINT32_MAX;
static uint32_t s_pid_test_revision = UINT32_MAX;
static uint16_t s_wifi_network_count;
static bool s_wifi_network_secured[WIFI_MANAGER_MAX_APS];
static char s_wifi_network_ssids[WIFI_MANAGER_MAX_APS]
                                  [WIFI_MANAGER_SSID_MAX_LEN + 1U];

static const uint32_t s_uart_baud_rates[] = {
    115200U, 230400U, 460800U, 921600U, 1000000U,
    1500000U, 1843200U, 2000000U,
};

/**
 * @brief 创建采用指定文本、颜色和字体的 LVGL 标签。
 * @param parent 标签的父对象。
 * @param text 标签初始文本。
 * @param color 文本使用的主题颜色角色。
 * @param font 标签使用的 LVGL 字体。
 * @return 新创建的标签对象。
 */
static lv_obj_t *ui_create_label(
    lv_obj_t *parent,
    const char *text,
    motor_ui_style_color_t color,
    const lv_font_t *font)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    motor_ui_style_label(label, color, font);
    return label;
}

/** @brief Wi-Fi 文本输入结束后隐藏共用软键盘。 */
static void ui_hide_wifi_keyboard(void)
{
    if (s_wifi_keyboard != NULL) {
        lv_keyboard_set_textarea(s_wifi_keyboard, NULL);
        lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

/** @brief MQTT URI 输入结束后隐藏共用软键盘。 */
static void ui_hide_mqtt_keyboard(void)
{
    if (s_mqtt_keyboard != NULL) {
        lv_keyboard_set_textarea(s_mqtt_keyboard, NULL);
        lv_obj_add_flag(s_mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief 不使用动画，直接显示 @p page 指定的页面。
 * @note 用于初始状态和动画完成后的页面切换，并会校验页面编号。
 */
static void ui_show_page(ui_page_t page)
{
    if (page >= UI_PAGE_COUNT) {
        return;
    }

    if (page != UI_PAGE_WIFI) {
        ui_hide_wifi_keyboard();
    }
    if (page != UI_PAGE_MQTT) {
        ui_hide_mqtt_keyboard();
    }

    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        lv_anim_delete(s_pages[i], NULL);
        lv_obj_set_pos(s_pages[i], 0, UI_PAGE_TOP);
        if (i == page) {
            lv_obj_remove_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    s_page_animating = false;
    s_current_page = page;
    lv_label_set_text(s_page_label, s_page_names[page]);
    lv_obj_invalidate(lv_screen_active());
}

/** @brief LVGL 动画执行函数：更新页面对象的纵向位置。 */
static void ui_page_animation_set_y(void *object, int32_t value)
{
    lv_obj_set_y((lv_obj_t *)object, value);
}

/** @brief 页面滑出动画完成后隐藏旧页面。 */
static void ui_page_animation_out_completed(lv_anim_t *animation)
{
    lv_obj_t *page = lv_anim_get_user_data(animation);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_y(page, UI_PAGE_TOP);
}

/** @brief 新页面滑入动画完成后收尾页面状态。 */
static void ui_page_animation_in_completed(lv_anim_t *animation)
{
    lv_obj_t *page = lv_anim_get_user_data(animation);
    lv_obj_set_y(page, UI_PAGE_TOP);
    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        if (s_pages[i] != page) {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_y(s_pages[i], UI_PAGE_TOP);
        }
    }
    s_page_animating = false;
    lv_obj_invalidate(lv_screen_active());
}

/**
 * @brief 将当前页面以纵向动画切换到 @p page。
 * @param forward 为 true 时按下一页方向切换，为 false 时按上一页方向切换。
 */
static void ui_animate_to_page(ui_page_t page, bool forward)
{
    if (page >= UI_PAGE_COUNT || page == s_current_page ||
        s_page_animating) {
        return;
    }

    /*
     * 在单缓冲 I80 面板上，大号 CAN 标题是唯一仍会残留像素的对象。切换起点
     * 或终点为 CAN 页面时采用原子隐藏/显示并使全屏失效；其他页面仍使用
     * 请求的纵向动画。
     */
    if (s_current_page == UI_PAGE_CAN || page == UI_PAGE_CAN) {
        ui_show_page(page);
        return;
    }

    if (page != UI_PAGE_WIFI) {
        ui_hide_wifi_keyboard();
    }
    if (page != UI_PAGE_MQTT) {
        ui_hide_mqtt_keyboard();
    }

    lv_obj_t *outgoing = s_pages[s_current_page];
    lv_obj_t *incoming = s_pages[page];
    const int32_t incoming_start =
        UI_PAGE_TOP + (forward ? UI_PAGE_HEIGHT : -UI_PAGE_HEIGHT);
    const int32_t outgoing_end =
        UI_PAGE_TOP + (forward ? -UI_PAGE_HEIGHT : UI_PAGE_HEIGHT);

    s_page_animating = true;
    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        lv_anim_delete(s_pages[i], NULL);
        if (s_pages[i] != outgoing && s_pages[i] != incoming) {
            lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_y(s_pages[i], UI_PAGE_TOP);
        }
    }
    lv_obj_set_y(outgoing, UI_PAGE_TOP);
    lv_obj_remove_flag(incoming, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_y(incoming, incoming_start);
    s_current_page = page;
    lv_label_set_text(s_page_label, s_page_names[page]);

    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, outgoing);
    lv_anim_set_exec_cb(&animation, ui_page_animation_set_y);
    lv_anim_set_values(&animation, UI_PAGE_TOP, outgoing_end);
    lv_anim_set_duration(&animation, UI_PAGE_ANIMATION_MS);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_out);
    lv_anim_set_user_data(&animation, outgoing);
    lv_anim_set_completed_cb(&animation, ui_page_animation_out_completed);
    lv_anim_start(&animation);

    lv_anim_init(&animation);
    lv_anim_set_var(&animation, incoming);
    lv_anim_set_exec_cb(&animation, ui_page_animation_set_y);
    lv_anim_set_values(&animation, incoming_start, UI_PAGE_TOP);
    lv_anim_set_duration(&animation, UI_PAGE_ANIMATION_MS);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_out);
    lv_anim_set_user_data(&animation, incoming);
    lv_anim_set_completed_cb(&animation, ui_page_animation_in_completed);
    lv_anim_start(&animation);
}

/** @brief 向事件模块返回当前页面编号。 */
ui_page_t motor_ui_internal_current_page(void)
{
    return s_current_page;
}

/** @brief 接收事件模块的导航请求并执行页面切换。 */
void motor_ui_internal_navigate(ui_page_t page)
{
    ui_animate_to_page(page, page >= s_current_page);
}

/**
 * @brief 创建控制模式按钮并返回其文本标签。
 * @param parent 按钮的父页面。
 * @param text 按钮显示文本。
 * @param callback 点击按钮时调用的事件回调。
 * @param[out] label_out 接收按钮内部标签对象的地址。
 * @return 新创建的按钮对象。
 */
static lv_obj_t *ui_create_mode_button(
    lv_obj_t *parent,
    const char *text,
    lv_event_cb_t callback,
    lv_obj_t **label_out)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 116, 40);
    motor_ui_style_button(button, UI_COLOR_BLUE, 12);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label =
        ui_create_label(button, text, UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(label);
    *label_out = label;
    return button;
}

/**
 * @brief 创建指定尺寸的红色电机停止按钮。
 * @param parent 按钮的父页面。
 * @param width 按钮宽度，单位为像素。
 * @param height 按钮高度，单位为像素。
 * @return 新创建的停止按钮对象。
 */
static lv_obj_t *ui_create_stop_button(
    lv_obj_t *parent,
    int32_t width,
    int32_t height)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, width, height);
    motor_ui_style_button(button, UI_COLOR_RED, 12);
    lv_obj_add_event_cb(button, motor_ui_stop_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = ui_create_label(button, "STOP", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(label);
    return button;
}

/**
 * @brief 创建带统一样式的导航按钮，并绑定目标页面事件。
 * @param parent 按钮所属的页面对象。
 * @param text 按钮显示的标题文本。
 * @param page 点击后切换到的目标页面。
 */
static void ui_create_navigation_button(
    lv_obj_t *parent,
    const char *text,
    ui_page_t page,
    int32_t y)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 286, 36);
    lv_obj_set_pos(button, 1, y);
    motor_ui_style_button(button, UI_COLOR_PANEL_LIGHT, 9);
    lv_obj_add_event_cb(button, motor_ui_navigation_event, LV_EVENT_CLICKED, (void *)(uintptr_t)page);
    lv_obj_t *label = ui_create_label(button, text, UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(label);
}

/** @brief 创建包含全部功能子页面入口的首页。 */
static void ui_create_navigation_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(parent, "PAGE SELECT", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_set_pos(title, 8, 2);

    lv_obj_t *list = lv_obj_create(parent);
    lv_obj_set_size(list, 304, 188);
    lv_obj_set_pos(list, 8, 26);
    motor_ui_style_navigation_list(list);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);

    ui_create_navigation_button(list, "MOTOR FEEDBACK", UI_PAGE_FEEDBACK, 1);
    ui_create_navigation_button(list, "USART", UI_PAGE_UART, 43);
    ui_create_navigation_button(list, "CAN", UI_PAGE_CAN, 85);
    ui_create_navigation_button(list, "WI-FI", UI_PAGE_WIFI, 127);
    ui_create_navigation_button(list, "MQTT", UI_PAGE_MQTT, 169);
    ui_create_navigation_button(list, "PID TEST", UI_PAGE_PID_TEST, 211);
    ui_create_navigation_button(list, "SPEED CONTROL", UI_PAGE_SPEED, 253);
    ui_create_navigation_button(list, "POSITION CONTROL", UI_PAGE_POSITION, 295);
}

/** @brief 创建实时电机反馈/状态页面及其数值标签。 */
static void ui_create_feedback_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(parent, "MOTOR FEEDBACK", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_set_pos(title, 8, 1);

    lv_obj_t *speed_panel = lv_obj_create(parent);
    lv_obj_set_size(speed_panel, 151, 61);
    lv_obj_set_pos(speed_panel, 6, 20);
    motor_ui_style_panel(speed_panel, 8, 6);
    lv_obj_t *label = ui_create_label(speed_panel, "SPEED", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);
    s_home_speed_measured_label = ui_create_label(speed_panel, "0 RPM", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_set_pos(s_home_speed_measured_label, 0, 14);
    s_home_speed_target_label = ui_create_label(speed_panel, "TARGET 0 RPM", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_speed_target_label, 0, 39);

    lv_obj_t *position_panel = lv_obj_create(parent);
    lv_obj_set_size(position_panel, 151, 61);
    lv_obj_set_pos(position_panel, 163, 20);
    motor_ui_style_panel(position_panel, 8, 6);
    label = ui_create_label(position_panel, "POSITION", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);
    s_home_position_current_label = ui_create_label(position_panel, "0.00 deg", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_set_pos(s_home_position_current_label, 0, 14);
    s_home_position_target_label = ui_create_label(position_panel, "TARGET 0.00 deg", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_position_target_label, 0, 39);

    lv_obj_t *electrical_panel = lv_obj_create(parent);
    lv_obj_set_size(electrical_panel, 308, 66);
    lv_obj_set_pos(electrical_panel, 6, 85);
    motor_ui_style_panel(electrical_panel, 8, 6);
    label = ui_create_label(electrical_panel, "ELECTRICAL FEEDBACK", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);
    s_home_current_label = ui_create_label(electrical_panel, "Iq     0 mA\nId     0 mA", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_current_label, 0, 17);
    s_home_current_reference_label = ui_create_label(electrical_panel, "Iq*    0 mA\nId*    0 mA", UI_COLOR_GREEN, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_current_reference_label, 104, 17);
    s_home_voltage_label = ui_create_label(electrical_panel, "Uq     0 mV\nUd     0 mV", UI_COLOR_YELLOW, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_voltage_label, 208, 17);

    lv_obj_t *status_panel = lv_obj_create(parent);
    lv_obj_set_size(status_panel, 308, 57);
    lv_obj_set_pos(status_panel, 6, 155);
    motor_ui_style_panel(status_panel, 8, 6);
    s_home_mode_label = ui_create_label(status_panel, "MODE SPEED", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_mode_label, 0, 1);
    s_home_run_label = ui_create_label(status_panel, "STATE STOPPED", UI_COLOR_YELLOW, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_run_label, 150, 1);
    s_home_transport_label = ui_create_label(status_panel, "LINK NONE", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_transport_label, 0, 25);
    s_home_fault_label = ui_create_label(status_panel, "FAULT 0x0000", UI_COLOR_GREEN, &lv_font_montserrat_12);
    lv_obj_set_pos(s_home_fault_label, 150, 25);
}

/** @brief 创建 UART 通道配置与诊断页面。 */
static void ui_create_uart_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(parent, "USART COMMUNICATION", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 12);
    lv_obj_t *port = ui_create_label(parent, "PORT1  GPIO18 TX / GPIO8 RX", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align_to(port, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

    s_link_diag_label = ui_create_label(parent, "UART RX 0  TX 0  ERR 0\nFAULT 0x0000", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_link_diag_label, LV_ALIGN_CENTER, 0, -5);

    s_baud_dropdown = lv_dropdown_create(parent);
    lv_dropdown_set_options(s_baud_dropdown, "115200\n230400\n460800\n921600\n1000000\n" "1500000\n1843200\n2000000");
    lv_dropdown_set_selected(s_baud_dropdown, 0U);
    lv_obj_set_size(s_baud_dropdown, 108, 36);
    lv_obj_set_pos(s_baud_dropdown, 7, 158);
    motor_ui_style_dropdown(s_baud_dropdown);

    s_reconnect_button = lv_button_create(parent);
    lv_obj_set_size(s_reconnect_button, 94, 36);
    lv_obj_set_pos(s_reconnect_button, 120, 158);
    motor_ui_style_button(s_reconnect_button, UI_COLOR_BLUE, 10);
    lv_obj_add_event_cb(s_reconnect_button, motor_ui_uart_reconnect_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *button_label = ui_create_label(s_reconnect_button, "CONNECT", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(button_label);

    s_uart_disconnect_button = lv_button_create(parent);
    lv_obj_set_size(s_uart_disconnect_button, 94, 36);
    lv_obj_set_pos(s_uart_disconnect_button, 219, 158);
    motor_ui_style_button(s_uart_disconnect_button, UI_COLOR_RED, 10);
    lv_obj_add_event_cb(s_uart_disconnect_button, motor_ui_uart_disconnect_event, LV_EVENT_CLICKED, NULL);
    button_label = ui_create_label(s_uart_disconnect_button, "DISCONNECT", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(button_label);

    s_home_state_label = ui_create_label(parent, "SELECT BAUD AND CONNECT", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_home_state_label, LV_ALIGN_BOTTOM_MID, 0, -7);
}

/** @brief 创建 CAN 通道配置与诊断页面。 */
static void ui_create_can_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(parent, "CAN COMMUNICATION", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 12);
    lv_obj_t *detail = ui_create_label(parent, "J2/PORT1  GPIO5/6  CLASSIC CAN 500K", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align_to(detail, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
    s_stop_button = lv_button_create(parent);
    lv_obj_set_size(s_stop_button, 132, 36);
    lv_obj_set_pos(s_stop_button, 22, 116);
    motor_ui_style_button(s_stop_button, UI_COLOR_RED, 10);
    lv_obj_add_event_cb(s_stop_button, motor_ui_stop_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *stop = ui_create_label(s_stop_button, "STOP", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(stop);

    s_ack_button = lv_button_create(parent);
    lv_obj_set_size(s_ack_button, 132, 36);
    lv_obj_set_pos(s_ack_button, 166, 116);
    motor_ui_style_button(s_ack_button, UI_COLOR_BLUE, 10);
    lv_obj_add_event_cb(s_ack_button, motor_ui_ack_fault_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ack = ui_create_label(s_ack_button, "ACK FAULT", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(ack);

    s_can_connect_button = lv_button_create(parent);
    lv_obj_set_size(s_can_connect_button, 132, 40);
    lv_obj_set_pos(s_can_connect_button, 22, 64);
    motor_ui_style_button(s_can_connect_button, UI_COLOR_CYAN, 12);
    lv_obj_add_event_cb(s_can_connect_button, motor_ui_can_connect_event, LV_EVENT_CLICKED, NULL);
    lv_obj_t *button_label = ui_create_label(s_can_connect_button, "CONNECT", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(button_label);

    s_can_disconnect_button = lv_button_create(parent);
    lv_obj_set_size(s_can_disconnect_button, 132, 40);
    lv_obj_set_pos(s_can_disconnect_button, 166, 64);
    motor_ui_style_button(s_can_disconnect_button, UI_COLOR_RED, 12);
    lv_obj_add_event_cb(s_can_disconnect_button, motor_ui_can_disconnect_event, LV_EVENT_CLICKED, NULL);
    button_label = ui_create_label(s_can_disconnect_button, "DISCONNECT", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_center(button_label);

    s_can_state_label = ui_create_label(parent, "Connecting selects CAN as the active control transport.", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_can_state_label, LV_ALIGN_BOTTOM_MID, 0, -24);
}

/**
 * @brief 创建 Wi-Fi/MQTT 页面共用的操作按钮。
 * @param parent 按钮的父页面。
 * @param text 按钮显示文本。
 * @param color 按钮背景的主题颜色角色。
 * @param x 按钮左上角横坐标。
 * @param y 按钮左上角纵坐标。
 * @param callback 点击按钮时调用的事件回调。
 * @return 新创建的操作按钮对象。
 */
static lv_obj_t *ui_create_wifi_action_button(
    lv_obj_t *parent,
    const char *text,
    motor_ui_style_color_t color,
    int32_t x,
    int32_t y,
    lv_event_cb_t callback)
{
    lv_obj_t *button = lv_button_create(parent);
    lv_obj_set_size(button, 92, 34);
    lv_obj_set_pos(button, x, y);
    motor_ui_style_button(button, color, 10);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = ui_create_label(button, text, UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(label);
    return button;
}

/** @brief 创建 Wi-Fi 扫描、网络选择及密码输入页面。 */
static void ui_create_wifi_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(parent, "WI-FI NETWORK", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_set_pos(title, 8, 4);

    s_wifi_network_dropdown = lv_dropdown_create(parent);
    lv_dropdown_set_options(s_wifi_network_dropdown, "No networks - tap SCAN");
    lv_obj_set_size(s_wifi_network_dropdown, 210, 34);
    lv_obj_set_pos(s_wifi_network_dropdown, 8, 34);
    motor_ui_style_dropdown(s_wifi_network_dropdown);
    lv_obj_add_event_cb(s_wifi_network_dropdown, motor_ui_wifi_network_event, LV_EVENT_VALUE_CHANGED, NULL);

    ui_create_wifi_action_button(parent, "SCAN", UI_COLOR_CYAN, 220, 34, motor_ui_wifi_scan_event);

    s_wifi_password_textarea = lv_textarea_create(parent);
    lv_obj_set_size(s_wifi_password_textarea, 204, 34);
    lv_obj_set_pos(s_wifi_password_textarea, 8, 76);
    lv_textarea_set_one_line(s_wifi_password_textarea, true);
    lv_textarea_set_password_mode(s_wifi_password_textarea, false);
    lv_textarea_set_max_length(s_wifi_password_textarea, 63U);
    lv_textarea_set_text(s_wifi_password_textarea, "13579035076");
    lv_textarea_set_placeholder_text(s_wifi_password_textarea, "Wi-Fi password");
    motor_ui_style_textarea(s_wifi_password_textarea);
    lv_obj_add_event_cb(s_wifi_password_textarea, motor_ui_wifi_password_event, LV_EVENT_CLICKED, NULL);

    ui_create_wifi_action_button(parent, "CONNECT", UI_COLOR_BLUE, 220, 76, motor_ui_wifi_connect_event);
    ui_create_wifi_action_button(parent, "DISCONNECT", UI_COLOR_RED, 220, 118, motor_ui_wifi_disconnect_event);

    s_wifi_detail_label = ui_create_label(parent, "No network selected", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_wifi_detail_label, 10, 120);
    lv_obj_set_width(s_wifi_detail_label, 202);
    lv_label_set_long_mode(s_wifi_detail_label, LV_LABEL_LONG_DOT);

    s_wifi_page_state_label = ui_create_label(parent, "Initializing Wi-Fi", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_wifi_page_state_label, 10, 165);
    lv_obj_set_width(s_wifi_page_state_label, 300);
    lv_label_set_long_mode(s_wifi_page_state_label, LV_LABEL_LONG_WRAP);

    s_wifi_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_set_size(s_wifi_keyboard, 320, 160);
    lv_obj_align(s_wifi_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_wifi_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    motor_ui_style_keyboard(s_wifi_keyboard);
    lv_obj_add_event_cb(s_wifi_keyboard, motor_ui_wifi_keyboard_event, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(s_wifi_keyboard, LV_OBJ_FLAG_HIDDEN);
}

/** @brief 创建 MQTT Broker 配置与测试发布页面。 */
static void ui_create_mqtt_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(parent, "MQTT TEST", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_set_pos(title, 8, 4);

    s_mqtt_uri_textarea = lv_textarea_create(parent);
    lv_obj_set_size(s_mqtt_uri_textarea, 204, 34);
    lv_obj_set_pos(s_mqtt_uri_textarea, 8, 34);
    lv_textarea_set_one_line(s_mqtt_uri_textarea, true);
    lv_textarea_set_max_length(s_mqtt_uri_textarea, MQTT_MANAGER_URI_MAX_LEN);
    lv_textarea_set_text(s_mqtt_uri_textarea, "mqtt://192.168.10.7:1883");
    motor_ui_style_textarea(s_mqtt_uri_textarea);
    lv_obj_add_event_cb(s_mqtt_uri_textarea, motor_ui_mqtt_uri_event, LV_EVENT_CLICKED, NULL);

    ui_create_wifi_action_button(parent, "CONNECT", UI_COLOR_BLUE, 220, 34, motor_ui_mqtt_connect_event);

    s_mqtt_page_state_label = ui_create_label(parent, "Connect Wi-Fi, then connect MQTT", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_mqtt_page_state_label, 10, 75);
    lv_obj_set_width(s_mqtt_page_state_label, 300);
    lv_label_set_long_mode(s_mqtt_page_state_label, LV_LABEL_LONG_DOT);



    s_mqtt_rx_label = ui_create_label(parent, "RX motor/hmi/test/rx\nNo message from MQTTX", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_mqtt_rx_label, 10, 149);
    lv_obj_set_size(s_mqtt_rx_label, 202, 48);
    lv_label_set_long_mode(s_mqtt_rx_label, LV_LABEL_LONG_DOT);

    ui_create_wifi_action_button(parent, "DISCONNECT", UI_COLOR_RED, 220, 153, motor_ui_mqtt_disconnect_event);

    s_mqtt_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_set_size(s_mqtt_keyboard, 320, 160);
    lv_obj_align(s_mqtt_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_mode(s_mqtt_keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
    motor_ui_style_keyboard(s_mqtt_keyboard);
    lv_obj_add_event_cb(s_mqtt_keyboard, motor_ui_mqtt_keyboard_event, LV_EVENT_ALL, NULL);
    lv_obj_add_flag(s_mqtt_keyboard, LV_OBJ_FLAG_HIDDEN);
}

/** @brief 创建由 ESP32 本地启动、MQTT 仅负责上传的 PID 测试页面。 */
static void ui_create_pid_test_page(lv_obj_t *parent)
{
    lv_obj_t *title = ui_create_label(
        parent, "PID TEST", UI_COLOR_TEXT, &lv_font_montserrat_14);
    lv_obj_set_pos(title, 8, 2);

    s_pid_test_pid_label = ui_create_label(
        parent,
        "SPD 2144/5    POS 48/4/8\n"
        "Iq 3633/2693  Id 3633/2693",
        UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_set_pos(s_pid_test_pid_label, 8, 22);

    s_pid_test_config_label = ui_create_label(
        parent, "7.0 s | 500 RPM | 90.00 deg",
        UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_set_pos(s_pid_test_config_label, 8, 72);

    s_pid_speed_test_button = lv_button_create(parent);
    lv_obj_set_size(s_pid_speed_test_button, 148, 42);
    lv_obj_set_pos(s_pid_speed_test_button, 8, 92);
    motor_ui_style_button(s_pid_speed_test_button, UI_COLOR_BLUE, 10);
    lv_obj_add_event_cb(
        s_pid_speed_test_button, motor_ui_pid_speed_test_event,
        LV_EVENT_CLICKED, NULL);
    lv_obj_t *label = ui_create_label(
        s_pid_speed_test_button, "SPEED CONTROL",
        UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(label);

    s_pid_position_test_button = lv_button_create(parent);
    lv_obj_set_size(s_pid_position_test_button, 148, 42);
    lv_obj_set_pos(s_pid_position_test_button, 164, 92);
    motor_ui_style_button(s_pid_position_test_button, UI_COLOR_BLUE, 10);
    lv_obj_add_event_cb(
        s_pid_position_test_button, motor_ui_pid_position_test_event,
        LV_EVENT_CLICKED, NULL);
    label = ui_create_label(
        s_pid_position_test_button, "POSITION CONTROL",
        UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_center(label);

    lv_obj_t *status_panel = lv_obj_create(parent);
    lv_obj_set_size(status_panel, 304, 70);
    lv_obj_set_pos(status_panel, 8, 140);
    motor_ui_style_panel(status_panel, 8, 6);
    label = ui_create_label(
        status_panel, "TEST STATUS", UI_COLOR_MUTED,
        &lv_font_montserrat_12);
    lv_obj_set_pos(label, 0, 0);

    s_pid_resend_button = lv_button_create(status_panel);
    lv_obj_set_size(s_pid_resend_button, 88, 26);
    lv_obj_set_pos(s_pid_resend_button, 198, -2);
    motor_ui_style_button(s_pid_resend_button, UI_COLOR_BLUE, 7);
    lv_obj_add_event_cb(
        s_pid_resend_button, motor_ui_pid_resend_event,
        LV_EVENT_CLICKED, NULL);
    lv_obj_add_state(s_pid_resend_button, LV_STATE_DISABLED);
    label = ui_create_label(
        s_pid_resend_button, "RESEND", UI_COLOR_TEXT,
        &lv_font_montserrat_12);
    lv_obj_center(label);

    s_pid_test_state_label = ui_create_label(
        status_panel, "READY", UI_COLOR_GREEN,
        &lv_font_montserrat_14);
    lv_obj_set_pos(s_pid_test_state_label, 0, 27);
    lv_obj_set_width(s_pid_test_state_label, 286);
    lv_label_set_long_mode(
        s_pid_test_state_label, LV_LABEL_LONG_WRAP);
}

/** @brief 创建速度模式控制、滑块及当前目标标签。 */
static void ui_create_speed_page(lv_obj_t *parent)
{
    lv_obj_t *actual_card = lv_obj_create(parent);
    lv_obj_set_size(actual_card, 144, 68);
    lv_obj_align(actual_card, LV_ALIGN_TOP_LEFT, 8, 10);
    motor_ui_style_panel(actual_card, 14, 10);

    lv_obj_t *actual_title = ui_create_label(actual_card, "ACTUAL SPEED", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(actual_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_speed_actual_label = ui_create_label(actual_card, "0 RPM", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_align(s_speed_actual_label, LV_ALIGN_BOTTOM_LEFT, 0, -3);

    lv_obj_t *reference_card = lv_obj_create(parent);
    lv_obj_set_size(reference_card, 144, 68);
    lv_obj_align(reference_card, LV_ALIGN_TOP_RIGHT, -8, 10);
    motor_ui_style_panel(reference_card, 14, 10);

    lv_obj_t *reference_title = ui_create_label(reference_card, "REFERENCE", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(reference_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_speed_reference_label = ui_create_label(reference_card, "0 RPM", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(s_speed_reference_label, LV_ALIGN_BOTTOM_LEFT, 0, -3);

    s_speed_mode_button = ui_create_mode_button(parent, "ENABLE SPEED", motor_ui_speed_mode_event, &s_speed_mode_button_label);
    lv_obj_align(s_speed_mode_button, LV_ALIGN_CENTER, -66, 3);

    s_speed_stop_button =
        ui_create_stop_button(parent, 116, 40);
    lv_obj_align(s_speed_stop_button, LV_ALIGN_CENTER, 66, 3);

    s_speed_slider_value = ui_create_label(parent, "0 RPM", UI_COLOR_TEXT, &lv_font_montserrat_12);
    lv_obj_align(s_speed_slider_value, LV_ALIGN_BOTTOM_MID, 0, -40);

    s_speed_slider = lv_slider_create(parent);
    lv_obj_set_size(s_speed_slider, 284, 16);
    lv_obj_align(s_speed_slider, LV_ALIGN_BOTTOM_MID, 0, -14);
    lv_slider_set_range(s_speed_slider, -UI_SPEED_LIMIT_RPM, UI_SPEED_LIMIT_RPM);
    lv_slider_set_value(s_speed_slider, 0, LV_ANIM_OFF);
    motor_ui_style_slider(s_speed_slider);
    lv_obj_add_event_cb(s_speed_slider, motor_ui_speed_slider_event, LV_EVENT_ALL, NULL);
}

/** @brief 创建位置模式控制、滑块及目标角度标签。 */
static void ui_create_position_page(lv_obj_t *parent)
{
    lv_obj_t *current_card = lv_obj_create(parent);
    lv_obj_set_size(current_card, 144, 64);
    lv_obj_align(current_card, LV_ALIGN_TOP_LEFT, 8, 8);
    motor_ui_style_panel(current_card, 14, 10);
    lv_obj_t *current_title = ui_create_label(current_card, "CURRENT POSITION", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(current_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_position_current_label = ui_create_label(current_card, "0.00 deg", UI_COLOR_CYAN, &lv_font_montserrat_20);
    lv_obj_align(s_position_current_label, LV_ALIGN_BOTTOM_LEFT, 0, -2);

    lv_obj_t *target_card = lv_obj_create(parent);
    lv_obj_set_size(target_card, 144, 64);
    lv_obj_align(target_card, LV_ALIGN_TOP_RIGHT, -8, 8);
    motor_ui_style_panel(target_card, 14, 10);
    lv_obj_t *target_title = ui_create_label(target_card, "TARGET POSITION", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(target_title, LV_ALIGN_TOP_LEFT, 0, 0);
    s_position_target_label = ui_create_label(target_card, "0.00 deg", UI_COLOR_TEXT, &lv_font_montserrat_20);
    lv_obj_align(s_position_target_label, LV_ALIGN_BOTTOM_LEFT, 0, -2);

    s_position_mode_button = ui_create_mode_button(parent, "ENABLE POS", motor_ui_position_mode_event, &s_position_mode_button_label);
    lv_obj_align(s_position_mode_button, LV_ALIGN_CENTER, -66, -2);

    s_position_stop_button =
        ui_create_stop_button(parent, 116, 40);
    lv_obj_align(s_position_stop_button, LV_ALIGN_CENTER, 66, -2);

    s_electrical_label = ui_create_label(parent, "Iq 0mA  Id 0mA\nIq* 0mA  Id* 0mA", UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_electrical_label, LV_ALIGN_BOTTOM_LEFT, 12, -48);

    s_position_slider = lv_slider_create(parent);
    lv_obj_set_size(s_position_slider, 284, 16);
    lv_obj_align(s_position_slider, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_slider_set_range(s_position_slider, 0, 36000);
    lv_slider_set_value(s_position_slider, 0, LV_ANIM_OFF);
    motor_ui_style_slider(s_position_slider);
    lv_obj_add_event_cb(s_position_slider, motor_ui_position_slider_event, LV_EVENT_ALL, NULL);
}

/**
 * @brief 对板载按键去抖，并将稳定按键映射为首页/上一页/下一页动作。
 */
static void ui_handle_keys(void)
{
    const uint8_t raw = board_keys_read();
    if (raw != s_key_candidate) {
        s_key_candidate = raw;
        s_key_debounce_count = 1U;
        return;
    }
    if (s_key_candidate == s_key_stable) {
        s_key_debounce_count = 0U;
        return;
    }
    if (s_key_debounce_count < 2U) {
        s_key_debounce_count++;
        if (s_key_debounce_count < 2U) {
            return;
        }
    }

    const uint8_t pressed =
        s_key_candidate & (uint8_t)~s_key_stable;
    s_key_stable = s_key_candidate;

    if ((pressed & BOARD_KEY_K0) != 0U) {
        /* K0 始终用于直接返回可滚动的页面选择器。 */
        ui_show_page(UI_PAGE_HOME);
    } else if ((pressed & BOARD_KEY_K1) != 0U) {
        ui_animate_to_page((ui_page_t)( (s_current_page + UI_PAGE_COUNT - 1U) % UI_PAGE_COUNT), false);
    } else if ((pressed & BOARD_KEY_K2) != 0U) {
        ui_animate_to_page((ui_page_t)((s_current_page + 1U) % UI_PAGE_COUNT), true);
    }
}

/**
 * @brief 轮询 CommMgr_ESP、刷新电机控件，并重发尚未确认的目标值。
 *
 * 仅当遥测尚未回显请求状态时，才重试由 UI 管理的待处理命令。本函数不会等待
 * I/O，因此可安全地在 LVGL 周期定时器中运行。
 */
static void ui_update_motor_data(void)
{
    CommMgr_ESP_State snapshot;
    CommMgr_ESP_GetState(&snapshot);

    if (s_speed_command_pending &&
        snapshot.mode == COMM_MGR_ESP_MODE_SPEED &&
        snapshot.reference_speed_rpm == s_pending_speed_rpm) {
        s_speed_command_pending = false;
    } else if (s_speed_command_pending &&
               snapshot.link_active &&
               snapshot.motor_running &&
               snapshot.mode == COMM_MGR_ESP_MODE_SPEED &&
               lv_tick_elaps(s_speed_command_tick) > 120U) {
        CommMgr_ESP_SetSpeedRPM(s_pending_speed_rpm);
        s_speed_command_tick = lv_tick_get();
    }

    if (s_position_command_pending &&
        snapshot.mode == COMM_MGR_ESP_MODE_POSITION &&
        snapshot.target_position_cdeg ==
            (uint16_t)(s_pending_position_cdeg % 36000U)) {
        s_position_command_pending = false;
    } else if (s_position_command_pending &&
               snapshot.link_active &&
               snapshot.motor_running &&
               snapshot.mode == COMM_MGR_ESP_MODE_POSITION &&
               lv_tick_elaps(s_position_command_tick) > 120U) {
        CommMgr_ESP_SetPositionCdeg(s_pending_position_cdeg);
        s_position_command_tick = lv_tick_get();
    }

    if (!s_have_previous_snapshot ||
        snapshot.link_active != s_previous_snapshot.link_active ||
        snapshot.transport != s_previous_snapshot.transport ||
        snapshot.uart_connected != s_previous_snapshot.uart_connected ||
        snapshot.uart_link_active != s_previous_snapshot.uart_link_active ||
        snapshot.can_connected != s_previous_snapshot.can_connected ||
        snapshot.can_link_active != s_previous_snapshot.can_link_active ||
        snapshot.bus_off != s_previous_snapshot.bus_off ||
        snapshot.transceiver_fault != s_previous_snapshot.transceiver_fault ||
        snapshot.motor_running != s_previous_snapshot.motor_running ||
        snapshot.motor_fault != s_previous_snapshot.motor_fault) {
        motor_ui_style_set_text_color(s_uart_status_label, snapshot.uart_link_active ? UI_COLOR_GREEN : UI_COLOR_RED);
        motor_ui_style_set_text_color(s_can_status_label, snapshot.can_link_active ? UI_COLOR_GREEN : UI_COLOR_RED);
        const char *transport_name =
            snapshot.transport == COMM_MGR_ESP_USART ? "USART"
                : (snapshot.transport == COMM_MGR_ESP_CAN ? "CAN" : "NONE");
        lv_label_set_text_fmt(s_home_transport_label, "LINK %s%s", transport_name, snapshot.transport == COMM_MGR_ESP_NONE ? "" : (snapshot.link_active ? " ONLINE" : " OFFLINE"));
        motor_ui_style_set_text_color(s_home_transport_label, snapshot.link_active ? UI_COLOR_GREEN : UI_COLOR_RED);
        lv_label_set_text(s_home_run_label, snapshot.motor_fault ? "STATE FAULT" : (snapshot.motor_running ? "STATE RUNNING" : "STATE STOPPED"));
        motor_ui_style_set_text_color(s_home_run_label, snapshot.motor_fault ? UI_COLOR_RED : (snapshot.motor_running ? UI_COLOR_GREEN : UI_COLOR_YELLOW));
        lv_label_set_text(s_home_state_label, snapshot.transport == COMM_MGR_ESP_NONE ? "SELECT SERIAL OR CAN" : (snapshot.transceiver_fault ? "CHECK VIO TXD RXD" : (snapshot.bus_off ? "CHECK CAN WIRING" : (snapshot.reconnecting ? "SERIAL RECONNECTING" : (!snapshot.link_active ? "WAITING FOR TELEMETRY" : (snapshot.motor_running ? "MOTOR RUNNING" : "MOTOR READY"))))));
    }

    if (!s_have_previous_snapshot ||
        snapshot.received_frames != s_previous_snapshot.received_frames ||
        snapshot.transmitted_frames != s_previous_snapshot.transmitted_frames ||
        snapshot.faults != s_previous_snapshot.faults ||
        snapshot.command_rejected != s_previous_snapshot.command_rejected ||
        snapshot.transmit_errors != s_previous_snapshot.transmit_errors) {
        lv_label_set_text_fmt(s_link_diag_label, "%s RX %lu   TX %lu   ERR %lu\nFAULT 0x%04X%s", snapshot.transport == COMM_MGR_ESP_USART ? "UART" : "CAN", (unsigned long)snapshot.received_frames, (unsigned long)snapshot.transmitted_frames, (unsigned long)snapshot.transmit_errors, snapshot.faults, snapshot.command_rejected ? "  CMD REJECTED" : "");
        lv_label_set_text_fmt(s_home_fault_label, "FAULT 0x%04X%s", snapshot.faults, snapshot.command_rejected ? " CMD" : "");
        motor_ui_style_set_text_color(s_home_fault_label, (snapshot.faults != 0U || snapshot.command_rejected) ? UI_COLOR_RED : UI_COLOR_GREEN);
    }

    if (!s_have_previous_snapshot ||
        snapshot.measured_speed_rpm !=
            s_previous_snapshot.measured_speed_rpm) {
        lv_label_set_text_fmt(s_speed_actual_label, "%d RPM", snapshot.measured_speed_rpm);
        lv_label_set_text_fmt(s_home_speed_measured_label, "%d RPM", snapshot.measured_speed_rpm);
    }

    if (!s_have_previous_snapshot ||
        snapshot.reference_speed_rpm !=
            s_previous_snapshot.reference_speed_rpm) {
        lv_label_set_text_fmt(s_speed_reference_label, "%d RPM", snapshot.reference_speed_rpm);
        lv_label_set_text_fmt(s_home_speed_target_label, "TARGET %d RPM", snapshot.reference_speed_rpm);
        if (!s_speed_dragging && !s_speed_command_pending) {
            lv_slider_set_value(s_speed_slider, snapshot.reference_speed_rpm, LV_ANIM_OFF);
            lv_label_set_text_fmt(s_speed_slider_value, "%d RPM", snapshot.reference_speed_rpm);
        }
    }

    if (!s_have_previous_snapshot ||
        snapshot.current_position_cdeg !=
            s_previous_snapshot.current_position_cdeg) {
        const uint16_t value = snapshot.current_position_cdeg;
        lv_label_set_text_fmt(s_position_current_label, "%3u.%02u deg", value / 100U, value % 100U);
        lv_label_set_text_fmt(s_home_position_current_label, "%u.%02u deg", value / 100U, value % 100U);
    }

    if ((!s_have_previous_snapshot ||
         snapshot.target_position_cdeg !=
             s_previous_snapshot.target_position_cdeg) &&
        !s_position_dragging &&
        !s_position_command_pending) {
        const uint16_t value = snapshot.target_position_cdeg;
        lv_slider_set_value(s_position_slider, value, LV_ANIM_OFF);
        lv_label_set_text_fmt(s_position_target_label, "%3u.%02u deg", value / 100U, value % 100U);
        lv_label_set_text_fmt(s_home_position_target_label, "TARGET %u.%02u deg", value / 100U, value % 100U);
    }

    if (!s_have_previous_snapshot ||
        snapshot.iq_ma != s_previous_snapshot.iq_ma ||
        snapshot.id_ma != s_previous_snapshot.id_ma ||
        snapshot.iq_reference_ma != s_previous_snapshot.iq_reference_ma ||
        snapshot.id_reference_ma != s_previous_snapshot.id_reference_ma) {
        lv_label_set_text_fmt(s_electrical_label, "Iq %5dmA  Id %5dmA\nIq* %4dmA  Id* %4dmA", snapshot.iq_ma, snapshot.id_ma, snapshot.iq_reference_ma, snapshot.id_reference_ma);
        lv_label_set_text_fmt(s_home_current_label, "Iq %5d mA\nId %5d mA", snapshot.iq_ma, snapshot.id_ma);
        lv_label_set_text_fmt(s_home_current_reference_label, "Iq*%5d mA\nId*%5d mA", snapshot.iq_reference_ma, snapshot.id_reference_ma);
    }

    if (!s_have_previous_snapshot ||
        snapshot.uq_mv != s_previous_snapshot.uq_mv ||
        snapshot.ud_mv != s_previous_snapshot.ud_mv) {
        lv_label_set_text_fmt(s_home_voltage_label, "Uq %5d mV\nUd %5d mV", snapshot.uq_mv, snapshot.ud_mv);
    }

    if (!s_have_previous_snapshot ||
        snapshot.mode != s_previous_snapshot.mode) {
        const bool speed_mode =
            snapshot.mode == COMM_MGR_ESP_MODE_SPEED;
        lv_label_set_text(s_speed_mode_button_label, speed_mode ? "SPEED ACTIVE" : "ENABLE SPEED");
        lv_label_set_text(s_position_mode_button_label, speed_mode ? "ENABLE POS" : "POS ACTIVE");
        lv_label_set_text(s_home_mode_label, speed_mode ? "MODE SPEED" : "MODE POSITION");
        motor_ui_style_set_text_color(s_home_mode_label, speed_mode ? UI_COLOR_CYAN : UI_COLOR_GREEN);
    }

    s_previous_snapshot = snapshot;
    s_have_previous_snapshot = true;
}

/** @brief 根据管理器快照版本刷新 Wi-Fi 状态标签和 AP 列表。 */
static void ui_update_wifi_data(void)
{
    wifi_manager_snapshot_t snapshot;
    wifi_manager_get_snapshot(&snapshot);

    if (snapshot.scan_generation != s_wifi_scan_generation) {
        char options[WIFI_MANAGER_MAX_APS * 48U];
        size_t used = 0U;
        options[0] = '\0';
        s_wifi_network_count = snapshot.ap_count;
        if (s_wifi_network_count > WIFI_MANAGER_MAX_APS) {
            s_wifi_network_count = WIFI_MANAGER_MAX_APS;
        }

        for (uint16_t i = 0U; i < s_wifi_network_count; i++) {
            strlcpy(s_wifi_network_ssids[i], snapshot.aps[i].ssid, sizeof(s_wifi_network_ssids[i]));
            s_wifi_network_secured[i] = snapshot.aps[i].secured;
            const int written = snprintf(options + used, sizeof(options) - used, "%s%s  %d dBm%s", i == 0U ? "" : "\n", snapshot.aps[i].ssid, snapshot.aps[i].rssi, snapshot.aps[i].secured ? "  *" : "");
            if (written < 0 || (size_t)written >= sizeof(options) - used) {
                break;
            }
            used += (size_t)written;
        }

        if (s_wifi_network_count == 0U) {
            lv_dropdown_set_options(s_wifi_network_dropdown, "No networks - tap SCAN");
        } else {
            lv_dropdown_set_options(s_wifi_network_dropdown, options);
            lv_dropdown_set_selected(s_wifi_network_dropdown, 0U);
        }
        s_wifi_scan_generation = snapshot.scan_generation;
        motor_ui_events_refresh_wifi_detail();
    }

    if (snapshot.revision == s_wifi_revision) {
        return;
    }

    motor_ui_style_set_text_color(s_wifi_status_label, snapshot.connected ? UI_COLOR_GREEN : UI_COLOR_RED);
    if (snapshot.connected) {
        lv_label_set_text_fmt(s_wifi_page_state_label, "CONNECTED: %s\nIP: %s", snapshot.ssid, snapshot.ip_address);
    } else {
        lv_label_set_text(s_wifi_page_state_label, snapshot.status);
    }
    s_wifi_revision = snapshot.revision;
}

/** @brief 管理器快照版本变化时刷新 MQTT 状态标签。 */
static void ui_update_mqtt_data(void)
{
    mqtt_manager_snapshot_t snapshot;
    mqtt_manager_get_snapshot(&snapshot);
    if (snapshot.revision == s_mqtt_revision) {
        return;
    }

    motor_ui_style_set_text_color(s_mqtt_status_label, snapshot.connected ? UI_COLOR_GREEN : UI_COLOR_RED);
    if (snapshot.connected) {
        lv_label_set_text_fmt(s_mqtt_page_state_label, "CONNECTED  TX %lu  RX %lu", (unsigned long)snapshot.transmitted_messages, (unsigned long)snapshot.received_messages);
    } else {
        lv_label_set_text(s_mqtt_page_state_label, snapshot.status);
    }

    if (snapshot.received_messages > 0U) {
        lv_label_set_text_fmt(s_mqtt_rx_label, "RX %s\n%s", snapshot.last_topic, snapshot.last_payload);
    }
    s_mqtt_revision = snapshot.revision;
}

/** @brief 刷新本地测试页的运行时 PID、测试阶段和按钮可用状态。 */
static void ui_update_pid_test_data(void)
{
    mqtt_motor_test_snapshot_t snapshot;
    mqtt_motor_gateway_get_test_snapshot(&snapshot);
    if (snapshot.revision == s_pid_test_revision) {
        return;
    }

    lv_label_set_text_fmt(
        s_pid_test_pid_label,
        "SPD %d/%d    POS %d/%d/%d\nIq %d/%d  Id %d/%d",
        snapshot.pid[0][0], snapshot.pid[0][1],
        snapshot.pid[1][0], snapshot.pid[1][1], snapshot.pid[1][2],
        snapshot.pid[2][0], snapshot.pid[2][1],
        snapshot.pid[3][0], snapshot.pid[3][1]);
    lv_label_set_text_fmt(
        s_pid_test_config_label,
        "%lu.%01lu s | %d RPM | %u.%02u deg",
        (unsigned long)(snapshot.duration_ms / 1000U),
        (unsigned long)((snapshot.duration_ms % 1000U) / 100U),
        MQTT_MOTOR_LOCAL_SPEED_TARGET_RPM,
        MQTT_MOTOR_LOCAL_POSITION_TARGET_CDEG / 100U,
        MQTT_MOTOR_LOCAL_POSITION_TARGET_CDEG % 100U);

    const bool busy =
        snapshot.state == MQTT_MOTOR_TEST_UI_TESTING ||
        snapshot.state == MQTT_MOTOR_TEST_UI_UPLOADING ||
        snapshot.resend_available;
    if (busy) {
        lv_obj_add_state(s_pid_speed_test_button, LV_STATE_DISABLED);
        lv_obj_add_state(s_pid_position_test_button, LV_STATE_DISABLED);
    } else {
        lv_obj_remove_state(s_pid_speed_test_button, LV_STATE_DISABLED);
        lv_obj_remove_state(s_pid_position_test_button, LV_STATE_DISABLED);
    }
    if (snapshot.resend_available) {
        lv_obj_remove_state(s_pid_resend_button, LV_STATE_DISABLED);
    } else {
        lv_obj_add_state(s_pid_resend_button, LV_STATE_DISABLED);
    }

    const char *state_text = "READY";
    char state_buffer[80];
    motor_ui_style_color_t color = UI_COLOR_MUTED;
    switch (snapshot.state) {
    case MQTT_MOTOR_TEST_UI_TESTING:
        state_text = "TESTING";
        color = UI_COLOR_YELLOW;
        break;
    case MQTT_MOTOR_TEST_UI_UPLOADING:
        snprintf(state_buffer, sizeof(state_buffer),
                 "UPLOADING %u/%u",
                 (unsigned)snapshot.uploaded_sample_count,
                 (unsigned)snapshot.sample_count);
        state_text = state_buffer;
        color = UI_COLOR_CYAN;
        break;
    case MQTT_MOTOR_TEST_UI_UPLOAD_SUCCESS:
        state_text = "UPLOAD SUCCESS";
        color = UI_COLOR_GREEN;
        break;
    case MQTT_MOTOR_TEST_UI_UPLOAD_FAILED:
        state_text = snapshot.message[0] != '\0'
            ? snapshot.message : "UPLOAD FAILED - PRESS RESEND";
        color = UI_COLOR_RED;
        break;
    case MQTT_MOTOR_TEST_UI_ERROR:
        state_text = snapshot.message[0] != '\0'
            ? snapshot.message : "TEST ERROR";
        color = UI_COLOR_RED;
        break;
    case MQTT_MOTOR_TEST_UI_IDLE:
    default:
        break;
    }
    lv_label_set_text(s_pid_test_state_label, state_text);
    motor_ui_style_set_text_color(s_pid_test_state_label, color);
    s_pid_test_revision = snapshot.revision;
}

/**
 * @brief LVGL 周期定时器回调：刷新电机、Wi-Fi 与 MQTT 页面。
 * @param timer LVGL 定时器对象；全部 UI 状态均为模块级变量，因此未使用该参数。
 */
static void ui_timer_callback(lv_timer_t *timer)
{
    (void)timer;
    ui_update_motor_data();
    ui_update_wifi_data();
    ui_update_mqtt_data();
    ui_update_pid_test_data();
}

/** @brief 专用于按键轮询和去抖的 LVGL 周期定时器回调。 */
static void ui_key_timer_callback(lv_timer_t *timer)
{
    (void)timer;
    ui_handle_keys();
}

/**
 * @brief 创建全部常驻 LVGL 页面、共用状态栏及刷新定时器。
 *
 * 所有页面预先创建在不透明视口内，防止滑动动画暴露页面外对象。调用者必须
 * 持有 LVGL 端口锁。
 * @param display UI 使用的显示器；传入 NULL 时终止创建。
 */
void motor_ui_create(lv_display_t *display)
{
    if (display == NULL) {
        return;
    }

    lv_obj_t *screen = lv_screen_active();
    /*
     * ESP 复位后 LCD 控制器可能仍保留上一帧。背光尚未打开时删除所有旧 LVGL
     * 子对象并强制进行不透明全屏重绘，避免旧欢迎页面残留。
     */
    lv_obj_clean(screen);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    motor_ui_style_screen(screen);
    s_page_label = ui_create_label(screen, s_page_names[0], UI_COLOR_MUTED, &lv_font_montserrat_12);
    lv_obj_align(s_page_label, LV_ALIGN_TOP_LEFT, 6, 5);

    s_mqtt_status_label = ui_create_label(screen, "MQTT", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align(s_mqtt_status_label, LV_ALIGN_TOP_RIGHT, -6, 5);
    s_wifi_status_label = ui_create_label(screen, "WI-FI", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align_to(s_wifi_status_label, s_mqtt_status_label, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    s_can_status_label = ui_create_label(screen, "CAN", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align_to(s_can_status_label, s_wifi_status_label, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    s_uart_status_label = ui_create_label(screen, "USART", UI_COLOR_RED, &lv_font_montserrat_12);
    lv_obj_align_to(s_uart_status_label, s_can_status_label, LV_ALIGN_OUT_LEFT_MID, -12, 0);

    /*
     * 将页面动画限制在同一个不透明裁剪视口内。LVGL 使用局部显示缓冲区时，
     * 若直接在根屏幕上移动页面对象，内容区域上下方可能暴露旧扫描线。
     */
    s_page_viewport = lv_obj_create(screen);
    lv_obj_set_size(s_page_viewport, 320, UI_PAGE_HEIGHT);
    lv_obj_set_pos(s_page_viewport, 0, UI_VIEWPORT_TOP);
    motor_ui_style_viewport(s_page_viewport);
    lv_obj_remove_flag(s_page_viewport, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_page_viewport, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        s_pages[i] = lv_obj_create(s_page_viewport);
        lv_obj_set_size(s_pages[i], 320, UI_PAGE_HEIGHT);
        lv_obj_set_pos(s_pages[i], 0, UI_PAGE_TOP);
        motor_ui_style_page(s_pages[i]);
        lv_obj_remove_flag(s_pages[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    ui_create_navigation_page(s_pages[UI_PAGE_HOME]);
    ui_create_feedback_page(s_pages[UI_PAGE_FEEDBACK]);
    ui_create_uart_page(s_pages[UI_PAGE_UART]);
    ui_create_can_page(s_pages[UI_PAGE_CAN]);
    ui_create_wifi_page(s_pages[UI_PAGE_WIFI]);
    ui_create_mqtt_page(s_pages[UI_PAGE_MQTT]);
    ui_create_pid_test_page(s_pages[UI_PAGE_PID_TEST]);
    ui_create_speed_page(s_pages[UI_PAGE_SPEED]);
    ui_create_position_page(s_pages[UI_PAGE_POSITION]);

    const motor_ui_event_context_t event_context = {
        .baud_dropdown = s_baud_dropdown,
        .home_state_label = s_home_state_label,
        .can_state_label = s_can_state_label,
        .wifi_network_dropdown = s_wifi_network_dropdown,
        .wifi_password_textarea = s_wifi_password_textarea,
        .wifi_page_state_label = s_wifi_page_state_label,
        .wifi_detail_label = s_wifi_detail_label,
        .wifi_keyboard = s_wifi_keyboard,
        .mqtt_uri_textarea = s_mqtt_uri_textarea,
        .mqtt_page_state_label = s_mqtt_page_state_label,
        .mqtt_keyboard = s_mqtt_keyboard,
        .pid_test_state_label = s_pid_test_state_label,
        .speed_slider = s_speed_slider,
        .speed_slider_value = s_speed_slider_value,
        .position_slider = s_position_slider,
        .position_target_label = s_position_target_label,
        .speed_dragging = &s_speed_dragging,
        .speed_command_pending = &s_speed_command_pending,
        .pending_speed_rpm = &s_pending_speed_rpm,
        .speed_command_tick = &s_speed_command_tick,
        .position_dragging = &s_position_dragging,
        .position_command_pending = &s_position_command_pending,
        .pending_position_cdeg = &s_pending_position_cdeg,
        .position_command_tick = &s_position_command_tick,
        .wifi_network_count = &s_wifi_network_count,
        .wifi_network_secured = s_wifi_network_secured,
        .wifi_network_ssids = s_wifi_network_ssids,
        .uart_baud_rates = s_uart_baud_rates,
        .uart_baud_rate_count = sizeof(s_uart_baud_rates) /
                                sizeof(s_uart_baud_rates[0]),
    };
    motor_ui_events_configure(&event_context);

    ui_show_page(UI_PAGE_HOME);
    lv_obj_move_foreground(s_page_label);
    lv_obj_move_foreground(s_uart_status_label);
    lv_obj_move_foreground(s_can_status_label);
    lv_obj_move_foreground(s_wifi_status_label);
    lv_obj_move_foreground(s_mqtt_status_label);
    lv_timer_create(ui_timer_callback, 50, NULL);
    lv_timer_create(ui_key_timer_callback, 20, NULL);
    ui_update_motor_data();
    ui_update_wifi_data();
    ui_update_mqtt_data();
    lv_obj_update_layout(screen);
    lv_obj_invalidate(screen);
    /* 由 LVGL 端口任务执行已经标记失效的全屏刷新。 */
}
