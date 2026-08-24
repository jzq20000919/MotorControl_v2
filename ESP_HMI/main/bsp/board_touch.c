#include "board_touch.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOUCH_I2C_ADDRESS        0x2EU
#define TOUCH_ID_ADDRESS         0x20000080UL
#define TOUCH_EVENT_ADDRESS      0x2000002CUL
#define TOUCH_EVENT_DATA_SIZE    28U
#define TOUCH_I2C_FREQUENCY_HZ   400000U
#define TOUCH_DISPLAY_H_RES      320U
#define TOUCH_DISPLAY_V_RES      240U

static const char *TAG = "BOARD_TOUCH";
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_touch_device;
static board_touch_reset_cb_t s_reset_callback;
static bool s_address_little_endian = true;
static lv_point_t s_last_point;
static lv_indev_t *s_touch_indev;

/**
 * @brief 为 I2C 事务编码 32 位 CHSC5432 寄存器地址。
 *
 * 不同版本的控制器使用相同的寄存器映射，但寄存器地址的字节序可能不同。
 * 初始化期间检测到的字节序会被保存，后续所有读取操作均沿用该字节序。
 *
 * @param address 需要编码的寄存器地址。
 * @param little_endian 为 true 时最低有效字节在前。
 * @param[out] bytes 接收编码结果的四字节缓冲区。
 */
static void touch_make_address(uint32_t address, bool little_endian, uint8_t bytes[4])
{
    if (little_endian) {
        bytes[0] = (uint8_t)(address >> 0U);
        bytes[1] = (uint8_t)(address >> 8U);
        bytes[2] = (uint8_t)(address >> 16U);
        bytes[3] = (uint8_t)(address >> 24U);
    } else {
        bytes[0] = (uint8_t)(address >> 24U);
        bytes[1] = (uint8_t)(address >> 16U);
        bytes[2] = (uint8_t)(address >> 8U);
        bytes[3] = (uint8_t)(address >> 0U);
    }
}

/**
 * @brief 使用指定地址字节序读取触摸控制器寄存器数据。
 *
 * @param address CHSC5432 寄存器地址。
 * @param little_endian 待测试或使用的寄存器地址字节序。
 * @param[out] data 接收读取数据的目标缓冲区。
 * @param length 需要读取的字节数。
 * @return 组合 I2C 事务成功时返回 ESP_OK，否则返回驱动错误码。
 */
static esp_err_t touch_direct_read_order(uint32_t address, bool little_endian, uint8_t *data, size_t length)
{
    uint8_t address_bytes[4];
    touch_make_address(address, little_endian, address_bytes);
    return i2c_master_transmit_receive(s_touch_device, address_bytes, sizeof(address_bytes), data, length, 20);
}

/**
 * @brief 使用启动时选定的字节序读取 CHSC5432 寄存器。
 * @param address 需要读取的寄存器地址。
 * @param[out] data 接收读取数据的目标缓冲区。
 * @param length 请求读取的字节数。
 * @return I2C 主机驱动返回的执行结果。
 */
static esp_err_t touch_direct_read(uint32_t address, uint8_t *data, size_t length)
{
    return touch_direct_read_order(address, s_address_little_endian, data, length);
}

/**
 * @brief 检查候选控制器 ID 是否为无效总线返回值。
 *
 * 全零或全 0xFF 的响应通常表示设备无响应或寄存器地址字节序错误，
 * 不能视为有效的芯片 ID。
 *
 * @param id 从 ID 寄存器读取的字节数组。
 * @param length @p id 中包含的字节数。
 * @return 数值可能有效时返回 true，否则返回 false。
 */
static bool touch_id_is_valid(const uint8_t *id, size_t length)
{
    bool all_zero = true;
    bool all_ff = true;

    for (size_t i = 0; i < length; i++) {
        all_zero = all_zero && (id[i] == 0x00U);
        all_ff = all_ff && (id[i] == 0xFFU);
    }
    return !all_zero && !all_ff;
}

/**
 * @brief 将原始触摸坐标转换到 LVGL 显示坐标系。
 *
 * 编译期的坐标交换和镜像选项必须与 main.c 设置的 LCD 方向一致。
 * 最后的限幅处理可避免异常坐标超出 320×240 显示区域并传入 LVGL。
 *
 * @param[in,out] x 输入原始横坐标并输出转换后的横坐标。
 * @param[in,out] y 输入原始纵坐标并输出转换后的纵坐标。
 */
static void touch_transform(uint16_t *x, uint16_t *y)
{
    uint16_t transformed_x = *x;
    uint16_t transformed_y = *y;

#if BOARD_TOUCH_SWAP_XY
    const uint16_t temporary = transformed_x;
    transformed_x = transformed_y;
    transformed_y = temporary;
#endif

#if BOARD_TOUCH_MIRROR_X
    transformed_x =
        (uint16_t)(TOUCH_DISPLAY_H_RES - 1U - transformed_x);
#endif

#if BOARD_TOUCH_MIRROR_Y
    transformed_y =
        (uint16_t)(TOUCH_DISPLAY_V_RES - 1U - transformed_y);
#endif

    if (transformed_x >= TOUCH_DISPLAY_H_RES) {
        transformed_x = TOUCH_DISPLAY_H_RES - 1U;
    }
    if (transformed_y >= TOUCH_DISPLAY_V_RES) {
        transformed_y = TOUCH_DISPLAY_V_RES - 1U;
    }

    *x = transformed_x;
    *y = transformed_y;
}

/**
 * @brief 读取并解码第一个有效的 CHSC5432 触摸点。
 *
 * 控制器事件报告可能包含多个触点；由于 LVGL 注册的是单指针设备，
 * 本界面只使用第一个有效触点。I2C 事务失败、没有触点或坐标越界时，
 * 均向调用者报告为“未按下”。
 *
 * @param[out] x 解码并转换后的横坐标。
 * @param[out] y 解码并转换后的纵坐标。
 * @return 成功获得有效按压点时返回 true，否则返回 false。
 */
static bool touch_read_point(uint16_t *x, uint16_t *y)
{
    uint8_t buffer[TOUCH_EVENT_DATA_SIZE] = {0U};
    if (touch_direct_read(TOUCH_EVENT_ADDRESS, buffer, sizeof(buffer)) != ESP_OK) {
        return false;
    }

    const uint8_t touch_count = buffer[1] & 0x0FU;
    if (touch_count == 0U || touch_count > 5U) {
        return false;
    }

    /*
     * CHSC5432 厂商针对 DNESP32S3B 横向 320×240 显示屏提供的坐标转换方式。
     */
    uint16_t point_x =((uint16_t)(buffer[5] >> 4U) << 8U) | buffer[3];
    const uint16_t raw_y =((uint16_t)(buffer[5] & 0x0FU) << 8U) | buffer[2];
    uint16_t point_y = raw_y >= TOUCH_DISPLAY_V_RES? 0U: (uint16_t)(TOUCH_DISPLAY_V_RES - raw_y);
    if (point_x >= TOUCH_DISPLAY_H_RES ||point_y >= TOUCH_DISPLAY_V_RES)
    {
        return false;
    }

    touch_transform(&point_x, &point_y);
    *x = point_x;
    *y = point_y;
    return true;
}

/**
 * @brief 向 LVGL 输入子系统提供最新触摸状态。
 *
 * 此回调由 LVGL 输入读取定时器调用，而不是由应用代码直接调用。
 * 释放时保留最后一个有效坐标符合 LVGL 指针输入约定，可使控件收到
 * 连贯的按下和释放事件序列。
 *
 * @param indev 请求数据的 LVGL 输入设备；本驱动未使用该参数。
 * @param[out] data 接收坐标和按压状态的 LVGL 输入数据结构。
 */
static void touch_read_callback(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t x;
    uint16_t y;
    (void)indev;

    if (touch_read_point(&x, &y))
    {
        s_last_point.x = (lv_coord_t)x;
        s_last_point.y = (lv_coord_t)y;
        data->point = s_last_point;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->point = s_last_point;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/**
 * @brief 初始化 CHSC5432，并将其注册为 LVGL 指针输入设备。
 *
 * 调用前必须已创建共享 I2C 主机总线并持有 LVGL port 锁。本函数会复位并
 * 探测触摸控制器、识别寄存器地址字节序，然后创建以 100 Hz 轮询的 LVGL
 * 指针输入设备。
 *
 * @param i2c_bus 板级外设共用的现有 I2C 主机总线句柄。
 * @param display 与触摸坐标关联的 LVGL 显示器。
 * @param reset_callback 控制触摸芯片复位引脚的板级回调函数。
 * @return 控制器和 LVGL 输入设备就绪时返回 ESP_OK，否则返回错误码。
 */
esp_err_t board_touch_init(i2c_master_bus_handle_t i2c_bus, lv_display_t *display, board_touch_reset_cb_t reset_callback)
{
    uint8_t id[4] = {0U};
    esp_err_t result;

    if (i2c_bus == NULL ||
        display == NULL ||
        reset_callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_bus = i2c_bus;
    s_reset_callback = reset_callback;

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_I2C_ADDRESS,
        .scl_speed_hz = TOUCH_I2C_FREQUENCY_HZ,
    };
    result = i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_touch_device);
    if (result != ESP_OK) {
        return result;
    }

    result = s_reset_callback(true);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    result = s_reset_callback(false);
    if (result != ESP_OK) {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(80));

    result = i2c_master_probe(s_i2c_bus, TOUCH_I2C_ADDRESS, 1000);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "CHSC5432 did not respond at I2C address 0x%02X", TOUCH_I2C_ADDRESS);
        return result;
    }

    result = touch_direct_read_order(TOUCH_ID_ADDRESS, true, id, sizeof(id));
    if (result == ESP_OK && touch_id_is_valid(id, sizeof(id))) {
        s_address_little_endian = true;
    } else {
        (void)memset(id, 0, sizeof(id));
        result = touch_direct_read_order(TOUCH_ID_ADDRESS, false, id, sizeof(id));
        if (result != ESP_OK) {
            return result;
        }
        s_address_little_endian = false;
    }

    ESP_LOGI(TAG, "CHSC5432 ID %02X %02X %02X %02X, %s-endian address", id[0], id[1], id[2], id[3], s_address_little_endian ? "little" : "big");

    s_touch_indev = lv_indev_create();
    if (s_touch_indev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_touch_indev, touch_read_callback);
    lv_indev_set_display(s_touch_indev, display);

    lv_timer_t *read_timer =lv_indev_get_read_timer(s_touch_indev);
    if (read_timer != NULL) {
        lv_timer_set_period(read_timer, 10U);
    }

    ESP_LOGI(TAG, "CHSC5432 LVGL input registered at 100 Hz");
    return ESP_OK;
}

/**
 * @brief 返回 board_touch_init() 创建的 LVGL 输入设备。
 * @return 初始化成功后返回指针输入设备；初始化完成前返回 NULL。
 */
lv_indev_t *board_touch_get_indev(void)
{
    return s_touch_indev;
}
