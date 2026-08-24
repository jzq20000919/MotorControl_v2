#ifndef BOARD_TOUCH_H
#define BOARD_TOUCH_H

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "lvgl.h"

/** @brief 非零时在厂商坐标转换后交换触摸 X/Y 轴。 */
#define BOARD_TOUCH_SWAP_XY       0
/** @brief 非零时镜像触摸 X 轴坐标。 */
#define BOARD_TOUCH_MIRROR_X      0
/** @brief 非零时镜像触摸 Y 轴坐标。 */
#define BOARD_TOUCH_MIRROR_Y      0

/**
 * @brief 触摸控制器复位引脚操作回调类型。
 * @param asserted true 表示拉入复位，false 表示释放复位。
 * @return 操作成功返回 ESP_OK，否则返回板级驱动错误码。
 */
typedef esp_err_t (*board_touch_reset_cb_t)(bool asserted);

/**
 * @brief 复位、探测并将板载触摸控制器注册为 LVGL 输入设备。
 * @note 必须在持有 LVGL port 锁时调用。
 * @param i2c_bus 板载外设共用的 I2C 主机总线句柄。
 * @param display 与触摸坐标关联的 LVGL 显示器。
 * @param reset_callback 控制触摸芯片复位状态的回调函数。
 * @return 初始化成功返回 ESP_OK，否则返回具体错误码。
 */
esp_err_t board_touch_init(
    i2c_master_bus_handle_t i2c_bus,
    lv_display_t *display,
    board_touch_reset_cb_t reset_callback);

/**
 * @brief 获取 board_touch_init() 创建的指针输入设备。
 * @return 已注册的 LVGL 输入设备；未初始化时返回 NULL。
 */
lv_indev_t *board_touch_get_indev(void);

#endif /* BOARD_TOUCH_H */
