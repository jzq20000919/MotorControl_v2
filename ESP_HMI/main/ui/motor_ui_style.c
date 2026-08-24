#include "motor_ui_style.h"

#define MOTOR_UI_COLOR_BACKGROUND  0x08111FU
#define MOTOR_UI_COLOR_PANEL       0x111D2EU
#define MOTOR_UI_COLOR_PANEL_LIGHT 0x1B2A41U
#define MOTOR_UI_COLOR_TEXT        0xF4F7FBU
#define MOTOR_UI_COLOR_MUTED       0x8FA3BFU
#define MOTOR_UI_COLOR_BLUE        0x2D8CFFU
#define MOTOR_UI_COLOR_CYAN        0x20D6C7U
#define MOTOR_UI_COLOR_GREEN       0x32D583U
#define MOTOR_UI_COLOR_RED         0xFF304FU
#define MOTOR_UI_COLOR_YELLOW      0xFFB454U

/** @brief 各主题颜色角色对应的 24 位 RGB 数值。 */
static const uint32_t s_color_values[MOTOR_UI_STYLE_COLOR_COUNT] = {
    [MOTOR_UI_STYLE_COLOR_BACKGROUND] = MOTOR_UI_COLOR_BACKGROUND,
    [MOTOR_UI_STYLE_COLOR_PANEL] = MOTOR_UI_COLOR_PANEL,
    [MOTOR_UI_STYLE_COLOR_PANEL_LIGHT] = MOTOR_UI_COLOR_PANEL_LIGHT,
    [MOTOR_UI_STYLE_COLOR_TEXT] = MOTOR_UI_COLOR_TEXT,
    [MOTOR_UI_STYLE_COLOR_MUTED] = MOTOR_UI_COLOR_MUTED,
    [MOTOR_UI_STYLE_COLOR_BLUE] = MOTOR_UI_COLOR_BLUE,
    [MOTOR_UI_STYLE_COLOR_CYAN] = MOTOR_UI_COLOR_CYAN,
    [MOTOR_UI_STYLE_COLOR_GREEN] = MOTOR_UI_COLOR_GREEN,
    [MOTOR_UI_STYLE_COLOR_RED] = MOTOR_UI_COLOR_RED,
    [MOTOR_UI_STYLE_COLOR_YELLOW] = MOTOR_UI_COLOR_YELLOW,
};

/** @brief 获取指定主题颜色。 */
lv_color_t motor_ui_style_color(motor_ui_style_color_t color)
{
    if (color < 0 || color >= MOTOR_UI_STYLE_COLOR_COUNT) {
        color = MOTOR_UI_STYLE_COLOR_TEXT;
    }
    return lv_color_hex(s_color_values[color]);
}

/** @brief 设置标签的文字颜色和字体。 */
void motor_ui_style_label(lv_obj_t *label, motor_ui_style_color_t color, const lv_font_t *font)
{
    lv_obj_set_style_text_color(label, motor_ui_style_color(color), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
}

/** @brief 设置对象的文字颜色。 */
void motor_ui_style_set_text_color(lv_obj_t *object, motor_ui_style_color_t color)
{
    lv_obj_set_style_text_color(object, motor_ui_style_color(color), LV_PART_MAIN);
}

/** @brief 设置通用面板样式。 */
void motor_ui_style_panel(lv_obj_t *object, int32_t radius, int32_t padding)
{
    lv_obj_set_style_bg_color(object, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(object, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(object, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(object, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_radius(object, radius, LV_PART_MAIN);
    lv_obj_set_style_pad_all(object, padding, LV_PART_MAIN);
    lv_obj_remove_flag(object, LV_OBJ_FLAG_SCROLLABLE);
}

/** @brief 设置通用按钮样式。 */
void motor_ui_style_button(lv_obj_t *button, motor_ui_style_color_t color, int32_t radius)
{
    lv_obj_set_style_radius(button, radius, LV_PART_MAIN);
    lv_obj_set_style_bg_color(button, motor_ui_style_color(color), LV_PART_MAIN);
}

/** @brief 设置页面导航列表样式。 */
void motor_ui_style_navigation_list(lv_obj_t *list)
{
    lv_obj_set_style_bg_color(list, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(list, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(list, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_border_width(list, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(list, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(list, 7, LV_PART_MAIN);
}

/** @brief 设置下拉框样式。 */
void motor_ui_style_dropdown(lv_obj_t *dropdown)
{
    lv_obj_set_style_bg_color(dropdown, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_text_color(dropdown, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_TEXT), LV_PART_MAIN);
}

/** @brief 设置文本输入框样式。 */
void motor_ui_style_textarea(lv_obj_t *textarea)
{
    lv_obj_set_style_bg_color(textarea, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_text_color(textarea, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_border_color(textarea, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL_LIGHT), LV_PART_MAIN);
}

/** @brief 设置软键盘样式。 */
void motor_ui_style_keyboard(lv_obj_t *keyboard)
{
    lv_obj_set_style_bg_color(keyboard, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_BACKGROUND), LV_PART_MAIN);
}

/** @brief 设置滑块样式。 */
void motor_ui_style_slider(lv_obj_t *slider)
{
    lv_obj_set_style_bg_color(slider, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_PANEL_LIGHT), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_TEXT), LV_PART_KNOB);
    lv_obj_set_style_pad_all(slider, 8, LV_PART_KNOB);
}

/** @brief 设置根屏幕样式。 */
void motor_ui_style_screen(lv_obj_t *screen)
{
    lv_obj_set_style_bg_color(screen, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
}

/** @brief 设置页面视口样式。 */
void motor_ui_style_viewport(lv_obj_t *viewport)
{
    lv_obj_set_style_bg_color(viewport, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_width(viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(viewport, 0, LV_PART_MAIN);
}

/** @brief 设置单个页面容器样式。 */
void motor_ui_style_page(lv_obj_t *page)
{
    lv_obj_set_style_bg_color(page, motor_ui_style_color(MOTOR_UI_STYLE_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(page, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_width(page, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(page, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(page, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(page, 0, LV_PART_MAIN);
}
