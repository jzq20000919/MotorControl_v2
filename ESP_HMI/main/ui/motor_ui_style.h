#ifndef MOTOR_UI_STYLE_H
#define MOTOR_UI_STYLE_H

#include <stdint.h>

#include "lvgl.h"

/** @brief UI 主题颜色角色；具体 RGB 数值仅在样式模块中维护。 */
typedef enum
{
    MOTOR_UI_STYLE_COLOR_BACKGROUND = 0,
    MOTOR_UI_STYLE_COLOR_PANEL,
    MOTOR_UI_STYLE_COLOR_PANEL_LIGHT,
    MOTOR_UI_STYLE_COLOR_TEXT,
    MOTOR_UI_STYLE_COLOR_MUTED,
    MOTOR_UI_STYLE_COLOR_BLUE,
    MOTOR_UI_STYLE_COLOR_CYAN,
    MOTOR_UI_STYLE_COLOR_GREEN,
    MOTOR_UI_STYLE_COLOR_RED,
    MOTOR_UI_STYLE_COLOR_YELLOW,
    MOTOR_UI_STYLE_COLOR_COUNT
} motor_ui_style_color_t;

/* 控件创建代码使用的简洁颜色角色别名，不包含具体 RGB 数值。 */
#define UI_COLOR_BACKGROUND  MOTOR_UI_STYLE_COLOR_BACKGROUND
#define UI_COLOR_PANEL       MOTOR_UI_STYLE_COLOR_PANEL
#define UI_COLOR_PANEL_LIGHT MOTOR_UI_STYLE_COLOR_PANEL_LIGHT
#define UI_COLOR_TEXT        MOTOR_UI_STYLE_COLOR_TEXT
#define UI_COLOR_MUTED       MOTOR_UI_STYLE_COLOR_MUTED
#define UI_COLOR_BLUE        MOTOR_UI_STYLE_COLOR_BLUE
#define UI_COLOR_CYAN        MOTOR_UI_STYLE_COLOR_CYAN
#define UI_COLOR_GREEN       MOTOR_UI_STYLE_COLOR_GREEN
#define UI_COLOR_RED         MOTOR_UI_STYLE_COLOR_RED
#define UI_COLOR_YELLOW      MOTOR_UI_STYLE_COLOR_YELLOW

/** @brief 获取指定主题颜色。 @param color 主题颜色角色。 @return LVGL 颜色值。 */
lv_color_t motor_ui_style_color(motor_ui_style_color_t color);

/** @brief 设置标签的文字颜色和字体。 @param label 标签对象。 @param color 文字颜色角色。 @param font 字体。 */
void motor_ui_style_label(lv_obj_t *label, motor_ui_style_color_t color, const lv_font_t *font);

/** @brief 设置对象的文字颜色。 @param object LVGL 对象。 @param color 文字颜色角色。 */
void motor_ui_style_set_text_color(lv_obj_t *object, motor_ui_style_color_t color);

/** @brief 设置通用面板样式。 @param object 面板对象。 @param radius 圆角半径。 @param padding 内边距。 */
void motor_ui_style_panel(lv_obj_t *object, int32_t radius, int32_t padding);

/** @brief 设置通用按钮样式。 @param button 按钮对象。 @param color 背景颜色角色。 @param radius 圆角半径。 */
void motor_ui_style_button(lv_obj_t *button, motor_ui_style_color_t color, int32_t radius);

/** @brief 设置页面导航列表样式。 @param list 导航列表对象。 */
void motor_ui_style_navigation_list(lv_obj_t *list);

/** @brief 设置下拉框样式。 @param dropdown 下拉框对象。 */
void motor_ui_style_dropdown(lv_obj_t *dropdown);

/** @brief 设置文本输入框样式。 @param textarea 文本输入框对象。 */
void motor_ui_style_textarea(lv_obj_t *textarea);

/** @brief 设置软键盘样式。 @param keyboard 软键盘对象。 */
void motor_ui_style_keyboard(lv_obj_t *keyboard);

/** @brief 设置滑块的轨道、指示条和旋钮样式。 @param slider 滑块对象。 */
void motor_ui_style_slider(lv_obj_t *slider);

/** @brief 设置根屏幕样式。 @param screen 根屏幕对象。 */
void motor_ui_style_screen(lv_obj_t *screen);

/** @brief 设置页面视口样式。 @param viewport 页面视口对象。 */
void motor_ui_style_viewport(lv_obj_t *viewport);

/** @brief 设置单个页面容器样式。 @param page 页面容器对象。 */
void motor_ui_style_page(lv_obj_t *page);

#endif /* MOTOR_UI_STYLE_H */
