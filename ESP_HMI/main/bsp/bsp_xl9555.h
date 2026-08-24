#ifndef BSP_XL9555_H
#define BSP_XL9555_H

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief 初始化板级共享 I2C 总线和 XL9555 I/O 扩展器。
 * @return 初始化成功时返回 ESP_OK，否则返回 I2C 或 GPIO 驱动错误码。
 */
esp_err_t bsp_xl9555_init(void);

/**
 * @brief 获取由 XL9555 BSP 创建的共享 I2C 主机总线。
 * @return 初始化成功后返回 I2C 总线句柄，初始化前返回 NULL。
 */
i2c_master_bus_handle_t bsp_xl9555_get_i2c_bus(void);

/**
 * @brief 通过 XL9555 P0.7 控制 LCD 背光。
 * @param enabled 为 true 时打开背光，为 false 时关闭背光。
 * @return 操作成功时返回 ESP_OK，否则返回 I2C 驱动错误码。
 */
esp_err_t bsp_xl9555_set_lcd_backlight(bool enabled);

/**
 * @brief 通过 XL9555 P0.6 控制低有效的触摸控制器复位线。
 * @param asserted 为 true 时保持复位，为 false 时释放复位。
 * @return 操作成功时返回 ESP_OK，否则返回 I2C 驱动错误码。
 */
esp_err_t bsp_xl9555_set_touch_reset(bool asserted);

#endif /* BSP_XL9555_H */
