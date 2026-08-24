#include "bsp_xl9555.h"

#include <stdint.h>

#include "board_keys.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

#define BSP_I2C_PORT          I2C_NUM_0
#define BSP_I2C_SDA           GPIO_NUM_48
#define BSP_I2C_SCL           GPIO_NUM_45
#define BSP_I2C_FREQ_HZ       400000U

#define XL9555_I2C_ADDR       0x20U
#define XL9555_INPUT_PORT0    0x00U
#define XL9555_OUTPUT_PORT0   0x02U
#define XL9555_CONFIG_PORT0   0x06U

#define XL9555_LCD_BL_MASK    (1U << 7)
#define XL9555_TOUCH_RST_MASK (1U << 6)
#define XL9555_KEY0_MASK      (1U << 4)
#define XL9555_KEY1_MASK      (1U << 3)
#define BOARD_KEY0_GPIO       GPIO_NUM_0

static const char *TAG = "BSP_XL9555";
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_xl9555;

/**
 * @brief 在一次 I2C 事务中读取两个连续的 XL9555 寄存器。
 * @param start_register 第一个寄存器地址。
 * @param[out] values 接收两个寄存器值的缓冲区。
 * @return I2C 事务执行结果。
 */
static esp_err_t xl9555_read_pair(uint8_t start_register, uint8_t values[2])
{
    if (s_xl9555 == NULL || values == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_xl9555, &start_register, sizeof(start_register), values, 2U, 10);
}

/**
 * @brief 在一次 I2C 事务中写入两个连续的 XL9555 寄存器。
 * @param start_register 第一个寄存器地址。
 * @param values 对应起始寄存器及其后继寄存器的值。
 * @return I2C 事务执行结果。
 */
static esp_err_t xl9555_write_pair(uint8_t start_register, const uint8_t values[2])
{
    if (s_xl9555 == NULL || values == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t tx_buffer[3] = {start_register, values[0], values[1]};
    return i2c_master_transmit(s_xl9555, tx_buffer, sizeof(tx_buffer), 1000);
}

/** @brief 初始化板级共享 I2C 总线和 XL9555 I/O 扩展器。 */
esp_err_t bsp_xl9555_init(void)
{
    if (s_xl9555 != NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing shared I2C bus");
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BSP_I2C_PORT,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus), TAG, "i2c_new_master_bus failed");

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XL9555_I2C_ADDR,
        .scl_speed_hz = BSP_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_xl9555), TAG, "add XL9555 failed");
    ESP_RETURN_ON_ERROR(i2c_master_probe(s_i2c_bus, XL9555_I2C_ADDR, 1000), TAG, "XL9555 probe failed");

    /* 先关闭背光并释放触摸复位，避免修改引脚方向时闪屏。 */
    uint8_t output_values[2];
    ESP_RETURN_ON_ERROR(xl9555_read_pair(XL9555_OUTPUT_PORT0, output_values), TAG, "read output failed");
    output_values[0] &= (uint8_t)~XL9555_LCD_BL_MASK;
    output_values[0] |= XL9555_TOUCH_RST_MASK;
    ESP_RETURN_ON_ERROR(xl9555_write_pair(XL9555_OUTPUT_PORT0, output_values), TAG, "write output failed");

    /* P0.7 和 P0.6 设为输出，其他引脚方向保持不变。 */
    uint8_t config_values[2];
    ESP_RETURN_ON_ERROR(xl9555_read_pair(XL9555_CONFIG_PORT0, config_values), TAG, "read config failed");
    config_values[0] &= (uint8_t)~(XL9555_LCD_BL_MASK | XL9555_TOUCH_RST_MASK);
    ESP_RETURN_ON_ERROR(xl9555_write_pair(XL9555_CONFIG_PORT0, config_values), TAG, "write config failed");

    ESP_LOGI(TAG, "XL9555 initialized");
    return ESP_OK;
}

/** @brief 获取 BSP 创建的共享 I2C 主机总线。 */
i2c_master_bus_handle_t bsp_xl9555_get_i2c_bus(void)
{
    return s_i2c_bus;
}

/** @brief 通过 XL9555 P0.7 打开或关闭 LCD 背光。 */
esp_err_t bsp_xl9555_set_lcd_backlight(bool enabled)
{
    uint8_t output_values[2];
    ESP_RETURN_ON_ERROR(xl9555_read_pair(XL9555_OUTPUT_PORT0, output_values), TAG, "read backlight failed");
    if (enabled) {
        output_values[0] |= XL9555_LCD_BL_MASK;
    } else {
        output_values[0] &= (uint8_t)~XL9555_LCD_BL_MASK;
    }
    return xl9555_write_pair(XL9555_OUTPUT_PORT0, output_values);
}

/** @brief 通过 XL9555 P0.6 驱动低有效的触摸复位线。 */
esp_err_t bsp_xl9555_set_touch_reset(bool asserted)
{
    uint8_t output_values[2];
    ESP_RETURN_ON_ERROR(xl9555_read_pair(XL9555_OUTPUT_PORT0, output_values), TAG, "read touch reset failed");
    if (asserted) {
        output_values[0] &= (uint8_t)~XL9555_TOUCH_RST_MASK;
    } else {
        output_values[0] |= XL9555_TOUCH_RST_MASK;
    }
    return xl9555_write_pair(XL9555_OUTPUT_PORT0, output_values);
}

/** @brief 将 XL9555 上的两个按键及 ESP32 BOOT 按键配置为输入。 */
void board_keys_init(void)
{
    uint8_t config_values[2];
    ESP_ERROR_CHECK(xl9555_read_pair(XL9555_CONFIG_PORT0, config_values));
    config_values[0] |= XL9555_KEY0_MASK | XL9555_KEY1_MASK;
    ESP_ERROR_CHECK(xl9555_write_pair(XL9555_CONFIG_PORT0, config_values));

    const gpio_config_t boot_key_config = {
        .pin_bit_mask = 1ULL << BOARD_KEY0_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_key_config));
}

/** @brief 采样全部低有效物理按键并返回逻辑按键位掩码。 */
uint8_t board_keys_read(void)
{
    uint8_t input_values[2];
    uint8_t keys = 0U;
    if (xl9555_read_pair(XL9555_INPUT_PORT0, input_values) == ESP_OK) {
        if ((input_values[0] & XL9555_KEY0_MASK) == 0U) {
            keys |= BOARD_KEY_K1;
        }
        if ((input_values[0] & XL9555_KEY1_MASK) == 0U) {
            keys |= BOARD_KEY_K2;
        }
    }
    if (gpio_get_level(BOARD_KEY0_GPIO) == 0) {
        keys |= BOARD_KEY_K0;
    }
    return keys;
}
