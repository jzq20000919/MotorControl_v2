#ifndef BSP_LCD_H
#define BSP_LCD_H

#include "lvgl.h"

/**
 * @brief 初始化 I80 总线、ST7789 面板和 LVGL 显示端口。
 * @return 初始化成功后返回 LVGL 显示器对象；注册失败时终止程序。
 */
lv_display_t *bsp_lcd_init(void);

#endif /* BSP_LCD_H */
