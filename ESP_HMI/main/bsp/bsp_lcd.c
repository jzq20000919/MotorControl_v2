#include "bsp_lcd.h"

#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"

#define LCD_H_RES          320
#define LCD_V_RES          240
#define LCD_DRAW_BUF_LINES LCD_V_RES
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)

#define LCD_GPIO_CS        GPIO_NUM_1
#define LCD_GPIO_DC        GPIO_NUM_2
#define LCD_GPIO_RD        GPIO_NUM_41
#define LCD_GPIO_WR        GPIO_NUM_42
#define LCD_GPIO_D0        GPIO_NUM_40
#define LCD_GPIO_D1        GPIO_NUM_39
#define LCD_GPIO_D2        GPIO_NUM_38
#define LCD_GPIO_D3        GPIO_NUM_12
#define LCD_GPIO_D4        GPIO_NUM_11
#define LCD_GPIO_D5        GPIO_NUM_10
#define LCD_GPIO_D6        GPIO_NUM_9
#define LCD_GPIO_D7        GPIO_NUM_46
#define LCD_GPIO_RST       GPIO_NUM_NC
#define LCD_SWAP_XY        true
#define LCD_MIRROR_X       true
#define LCD_MIRROR_Y       false

static const char *TAG = "BSP_LCD";
static esp_lcd_i80_bus_handle_t s_lcd_bus;//定义总线句柄
static esp_lcd_panel_io_handle_t s_lcd_io;//定义面板IO句柄
static esp_lcd_panel_handle_t s_lcd_panel;//定义面板句柄
static lv_display_t *s_lvgl_display;//定义LVGL显示器对象指针

/**
 * @brief 在 8 位 I80 并口上创建并初始化 ST7789 面板。
 *
 * 本设计不读取 LCD 数据，因此 RD 始终保持高电平。RGB565 字节序和屏幕方向
 * 必须与 LVGL 显示配置及触摸坐标变换保持一致。
 */
static void bsp_lcd_panel_init(void)
{   
    //首先创建I80总线句柄，并检查是否成功。bsp_lcd_panel_init初始化了总线，规定了哪些引脚是I80总线，实际上配置了8个GPIO口，配置并创建I80句柄后，任何使用I80接口的设备在写BSP时都可以用这个句柄。配置方法为ESP官方写法。
    //构建gpio结构体，一次性配置RD引脚为输出模式，并设置上拉电阻
    const gpio_config_t rd_config = {
        .pin_bit_mask = 1ULL << LCD_GPIO_RD,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rd_config));//检查是否配置成功
    ESP_ERROR_CHECK(gpio_set_level(LCD_GPIO_RD, 1));//拉高RD引脚，保持高电平
    //构建总线句柄的配置
    const esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_GPIO_DC,
        .wr_gpio_num = LCD_GPIO_WR,
        .data_gpio_nums = {LCD_GPIO_D0, LCD_GPIO_D1, LCD_GPIO_D2, LCD_GPIO_D3, LCD_GPIO_D4, LCD_GPIO_D5, LCD_GPIO_D6, LCD_GPIO_D7},
        .bus_width = 8,
        .max_transfer_bytes = LCD_H_RES * LCD_DRAW_BUF_LINES * sizeof(uint16_t),
        .dma_burst_size = 64,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &s_lcd_bus));
    //然后构建面板IO配置结构体，设置CS引脚、像素时钟频率、事务队列深度、D/C线电平、命令和参数位宽等参数。这也是ESP官方写法，构建好后创建面板IO句柄，并检查是否成功。
    const esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 2,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .swap_color_bytes = true,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(s_lcd_bus, &io_config, &s_lcd_io));
    //ST7789面板配置结构体，设置颜色顺序为RGB，颜色深度为16位，复位引脚未使用。
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(s_lcd_io, &panel_config, &s_lcd_panel));//初始化mini ST7789面板，并检查是否成功
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_lcd_panel));//复位面板
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_lcd_panel));//初始化面板
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_lcd_panel, true));//反转颜色
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_lcd_panel, 0, 0));//设置面板间隙为0
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_lcd_panel, LCD_SWAP_XY));//交换XY轴
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_lcd_panel, LCD_MIRROR_X, LCD_MIRROR_Y));//镜像X轴，Y轴不镜像
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_lcd_panel, true));//打开显示
}

/**
 * @brief 初始化 esp_lvgl_port 并注册已经创建的 ST7789 面板。
 * @return 注册成功后返回 LVGL 显示器对象，失败时返回 NULL。
 */
static lv_display_t *bsp_lcd_lvgl_init(void)
{
    lvgl_port_cfg_t lvgl_config = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_config.task_priority = 8;
    lvgl_config.task_affinity = 1;
    lvgl_config.task_max_sleep_ms = 5;
    lvgl_config.timer_period_ms = 2;
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_config));//初始化LVGL端口，并检查是否成功

    /* 使用单个整屏 RGB565 DMA 缓冲区，避免页面切换产生残影。 */
    const lvgl_port_display_cfg_t display_config = 
    {
        .io_handle = s_lcd_io,
        .panel_handle = s_lcd_panel,
        .buffer_size = LCD_H_RES * LCD_DRAW_BUF_LINES,
        .double_buffer = false,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
        .swap_xy = LCD_SWAP_XY,
        .mirror_x = LCD_MIRROR_X,
        .mirror_y = LCD_MIRROR_Y,
        },
        .flags = {
            .buff_dma = true,
            .full_refresh = true,
            .swap_bytes = false,
            .sw_rotate = false,
        },
    };
    return lvgl_port_add_disp(&display_config);//注册显示器并返回LVGL显示器对象
}

/** @brief 初始化 LCD 硬件与 LVGL 显示端口。 */
//bsp_lcd_init返回了一个我们在bsp_lcd_lvgl_init函数中配置结构体产生的句柄，而在配置这个结构体的时候又用之前的bsp_lcd_panel_init中配置的总线与IO句柄。
lv_display_t *bsp_lcd_init(void)
{
    if (s_lvgl_display != NULL) {
        return s_lvgl_display;
    }
    ESP_LOGI(TAG, "Initializing I80 ST7789 display");
    bsp_lcd_panel_init();//初始化LCD屏幕硬件面板
    s_lvgl_display = bsp_lcd_lvgl_init();
    if (s_lvgl_display == NULL) {
        ESP_LOGE(TAG, "Failed to register LVGL display");
        abort();
    }
    ESP_LOGI(TAG, "LCD and LVGL display initialized");
    return s_lvgl_display;
}

