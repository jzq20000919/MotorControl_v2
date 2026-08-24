#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "board_keys.h"
#include "board_touch.h"
#include "bsp_lcd.h"
#include "bsp_xl9555.h"
#include "comm_mgr_ESP.h"
#include "motor_ui.h"
#include "mqtt_manager.h"
#include "mqtt_motor_gateway.h"
#include "wifi_manager.h"

static const char *TAG = "MOTOR_HMI";

/**
 * @brief 使用固件编译时间为系统时钟设置初始值。
 *
 * 板卡启动时没有 RTC 或网络校时依赖；该初值能让日志具有可用的时间基准，
 * 直到后续加入真正的校时功能。
 */
static void board_set_time_from_build(void)
{
    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char month_name[4] = {0};
    int day = 1;
    int year = 2026;
    int hour = 0;
    int minute = 0;
    int second = 0;

    if (sscanf(__DATE__, "%3s %d %d", month_name, &day, &year) != 3 || sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second) != 3) {
        return;
    }

    const char *month_ptr = strstr(months, month_name);
    if (month_ptr == NULL) {
        return;
    }

    struct tm build_time = {
        .tm_year = year - 1900,
        .tm_mon = (int)((month_ptr - months) / 3),
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second,
        .tm_isdst = -1,
    };
    const time_t epoch = mktime(&build_time);
    const struct timeval now = {
        .tv_sec = epoch,
        .tv_usec = 0,
    };
    (void)settimeofday(&now, NULL);
}

/**
 * @brief 电机 HMI 的 ESP-IDF 应用入口函数。
 *
 * 初始化顺序经过设计：先初始化 XL9555 并熄灭背光，再完成 LCD 与 LVGL
 * 注册；随后初始化网络组件，最后在持有 LVGL 锁时创建 UI 和触摸输入设备。
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting DNESP32S3B Motor HMI");
    board_set_time_from_build();
    CommMgr_ESP_Init();

    ESP_ERROR_CHECK(bsp_xl9555_init());
    board_keys_init();
    ESP_ERROR_CHECK(bsp_xl9555_set_lcd_backlight(false));
    lv_display_t *display = bsp_lcd_init();//lcd_init() 内部会调用 lvgl_port_init()，因此必须在 lvgl_port_init() 之后调用

    /* 先分配整屏 DMA 显示缓冲区，再启动网络栈，避免内存碎片影响显示注册。 */
    const esp_err_t wifi_result = wifi_manager_init();
    if (wifi_result != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi initialization failed: %s", esp_err_to_name(wifi_result));
    }
    const esp_err_t mqtt_result = mqtt_manager_init();
    if (mqtt_result != ESP_OK) {
        ESP_LOGE(TAG, "MQTT initialization failed: %s", esp_err_to_name(mqtt_result));
    }
    const esp_err_t mqtt_gateway_result = mqtt_motor_gateway_init();
    if (mqtt_gateway_result != ESP_OK) {
        ESP_LOGE(TAG, "MQTT motor gateway initialization failed: %s", esp_err_to_name(mqtt_gateway_result));
    }

    ESP_LOGI(TAG, "Creating motor control UI");
    lvgl_port_lock(0);
    motor_ui_create(display);
    const esp_err_t touch_result = board_touch_init(bsp_xl9555_get_i2c_bus(), display, bsp_xl9555_set_touch_reset);
    lvgl_port_unlock();

    if (touch_result != ESP_OK) {
        ESP_LOGE(TAG, "Touch initialization failed: %s", esp_err_to_name(touch_result));
    }

    /* 等待首帧刷新完成后再打开背光，减少启动白屏和闪屏。 */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(bsp_xl9555_set_lcd_backlight(true));
    ESP_LOGI(TAG, "Motor HMI started successfully");
}
