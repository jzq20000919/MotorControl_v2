#ifndef MOTOR_UI_EVENTS_H
#define MOTOR_UI_EVENTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lvgl.h"
#include "wifi_manager.h"

/** @brief UI 页面编号；顺序同时用于按键翻页与菜单导航。 */
typedef enum
{
    UI_PAGE_HOME = 0,
    UI_PAGE_FEEDBACK,
    UI_PAGE_UART,
    UI_PAGE_CAN,
    UI_PAGE_WIFI,
    UI_PAGE_MQTT,
    UI_PAGE_SPEED,
    UI_PAGE_POSITION,
    UI_PAGE_COUNT
} ui_page_t;

/** @brief 事件模块与绘制模块之间共享的控件和交互状态。 */
typedef struct
{
    lv_obj_t *baud_dropdown;              /**< 串口波特率下拉框。 */
    lv_obj_t *home_state_label;           /**< 首页电机状态标签。 */
    lv_obj_t *can_state_label;            /**< CAN 连接状态标签。 */
    lv_obj_t *wifi_network_dropdown;      /**< Wi-Fi 网络下拉框。 */
    lv_obj_t *wifi_password_textarea;     /**< Wi-Fi 密码输入框。 */
    lv_obj_t *wifi_page_state_label;      /**< Wi-Fi 页面状态标签。 */
    lv_obj_t *wifi_detail_label;          /**< Wi-Fi 接入点详情标签。 */
    lv_obj_t *wifi_keyboard;              /**< Wi-Fi 软键盘。 */
    lv_obj_t *mqtt_uri_textarea;          /**< MQTT 地址输入框。 */
    lv_obj_t *mqtt_page_state_label;      /**< MQTT 页面状态标签。 */
    lv_obj_t *mqtt_keyboard;              /**< MQTT 软键盘。 */
    lv_obj_t *speed_slider;               /**< 目标速度滑块。 */
    lv_obj_t *speed_slider_value;         /**< 目标速度数值标签。 */
    lv_obj_t *position_slider;            /**< 目标位置滑块。 */
    lv_obj_t *position_target_label;      /**< 目标位置数值标签。 */

    bool *speed_dragging;                 /**< 速度滑块拖动状态。 */
    bool *speed_command_pending;          /**< 速度命令待发送标志。 */
    int16_t *pending_speed_rpm;           /**< 待发送目标速度，单位 rpm。 */
    uint32_t *speed_command_tick;         /**< 最近速度命令时间戳。 */
    bool *position_dragging;              /**< 位置滑块拖动状态。 */
    bool *position_command_pending;       /**< 位置命令待发送标志。 */
    uint16_t *pending_position_cdeg;      /**< 待发送目标位置，单位 0.01°。 */
    uint32_t *position_command_tick;      /**< 最近位置命令时间戳。 */

    uint16_t *wifi_network_count;         /**< 当前扫描到的 Wi-Fi 网络数量。 */
    bool *wifi_network_secured;           /**< 各 Wi-Fi 网络的加密状态数组。 */
    char (*wifi_network_ssids)[WIFI_MANAGER_SSID_MAX_LEN + 1U]; /**< Wi-Fi SSID 数组。 */
    const uint32_t *uart_baud_rates;      /**< 可选串口波特率数组。 */
    size_t uart_baud_rate_count;          /**< 可选串口波特率数量。 */
} motor_ui_event_context_t;

/**
 * @brief 保存绘制模块提供的控件引用与共享状态地址。
 * @param context 由绘制模块初始化的事件上下文。
 */
void motor_ui_events_configure(const motor_ui_event_context_t *context);

/** @brief 使用共享事件上下文刷新当前选中 Wi-Fi 接入点的详情。 */
void motor_ui_events_refresh_wifi_detail(void);

/** @brief 返回绘制模块当前显示的页面。 */
ui_page_t motor_ui_internal_current_page(void);
/** @brief 由事件模块请求绘制模块切换页面。 @param page 目标页面编号。 */
void motor_ui_internal_navigate(ui_page_t page);

/** @brief 处理菜单按钮导航事件。 @param event LVGL 事件对象。 */
void motor_ui_navigation_event(lv_event_t *event);
/** @brief 处理速度模式切换事件。 @param event LVGL 事件对象。 */
void motor_ui_speed_mode_event(lv_event_t *event);
/** @brief 处理位置模式切换事件。 @param event LVGL 事件对象。 */
void motor_ui_position_mode_event(lv_event_t *event);
/** @brief 处理电机停止事件。 @param event LVGL 事件对象。 */
void motor_ui_stop_event(lv_event_t *event);
/** @brief 处理故障确认事件。 @param event LVGL 事件对象。 */
void motor_ui_ack_fault_event(lv_event_t *event);
/** @brief 处理串口重新连接事件。 @param event LVGL 事件对象。 */
void motor_ui_uart_reconnect_event(lv_event_t *event);
/** @brief 处理 CAN 连接事件。 @param event LVGL 事件对象。 */
void motor_ui_can_connect_event(lv_event_t *event);
/** @brief 处理串口断开事件。 @param event LVGL 事件对象。 */
void motor_ui_uart_disconnect_event(lv_event_t *event);
/** @brief 处理 CAN 断开事件。 @param event LVGL 事件对象。 */
void motor_ui_can_disconnect_event(lv_event_t *event);
/** @brief 处理 Wi-Fi 网络选择事件。 @param event LVGL 事件对象。 */
void motor_ui_wifi_network_event(lv_event_t *event);
/** @brief 处理 Wi-Fi 密码框聚焦事件。 @param event LVGL 事件对象。 */
void motor_ui_wifi_password_event(lv_event_t *event);
/** @brief 处理 Wi-Fi 软键盘事件。 @param event LVGL 事件对象。 */
void motor_ui_wifi_keyboard_event(lv_event_t *event);
/** @brief 处理 Wi-Fi 扫描事件。 @param event LVGL 事件对象。 */
void motor_ui_wifi_scan_event(lv_event_t *event);
/** @brief 处理 Wi-Fi 连接事件。 @param event LVGL 事件对象。 */
void motor_ui_wifi_connect_event(lv_event_t *event);
/** @brief 处理 Wi-Fi 断开事件。 @param event LVGL 事件对象。 */
void motor_ui_wifi_disconnect_event(lv_event_t *event);
/** @brief 处理 MQTT 地址输入框聚焦事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_uri_event(lv_event_t *event);
/** @brief 处理 MQTT 软键盘事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_keyboard_event(lv_event_t *event);
/** @brief 处理 MQTT 连接事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_connect_event(lv_event_t *event);
/** @brief 处理 MQTT 断开事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_disconnect_event(lv_event_t *event);
/** @brief 处理 MQTT 测试发布事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_ping_event(lv_event_t *event);
/** @brief 处理 MQTT 页面跳转到 Wi-Fi 页事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_wifi_event(lv_event_t *event);
/** @brief 处理 MQTT 页面跳转到电机页事件。 @param event LVGL 事件对象。 */
void motor_ui_mqtt_motor_event(lv_event_t *event);
/** @brief 处理目标速度滑块事件。 @param event LVGL 事件对象。 */
void motor_ui_speed_slider_event(lv_event_t *event);
/** @brief 处理目标位置滑块事件。 @param event LVGL 事件对象。 */
void motor_ui_position_slider_event(lv_event_t *event);

#endif /* MOTOR_UI_EVENTS_H */
