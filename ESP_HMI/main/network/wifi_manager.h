#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** @brief 单次扫描结果中保存的最大接入点数量。 */
#define WIFI_MANAGER_MAX_APS 12U
/** @brief IEEE 802.11 SSID 的最大有效字符数。 */
#define WIFI_MANAGER_SSID_MAX_LEN 32U

/** @brief 一条 Wi-Fi 扫描结果。 */
typedef struct
{
    char ssid[WIFI_MANAGER_SSID_MAX_LEN + 1U]; /**< 接入点 SSID，以空字符结尾。 */
    int8_t rssi;                              /**< 接收信号强度，单位 dBm。 */
    bool secured;                             /**< true 表示接入点需要认证。 */
} wifi_manager_ap_t;

/** @brief Wi-Fi 管理器的线程安全状态快照。 */
typedef struct
{
    bool initialized;                         /**< Wi-Fi 驱动已初始化。 */
    bool scanning;                            /**< 正在异步扫描接入点。 */
    bool connecting;                          /**< 正在连接目标接入点。 */
    bool connected;                           /**< STA 已连接并取得网络配置。 */
    uint16_t ap_count;                        /**< aps 数组中的有效条目数。 */
    uint32_t revision;                        /**< 任意状态变化时递增的版本号。 */
    uint32_t scan_generation;                 /**< 每次扫描完成时递增的代次号。 */
    char ssid[WIFI_MANAGER_SSID_MAX_LEN + 1U]; /**< 当前或目标 SSID。 */
    char ip_address[16];                      /**< 点分十进制 IPv4 地址。 */
    char status[64];                          /**< 面向界面显示的状态文本。 */
    wifi_manager_ap_t aps[WIFI_MANAGER_MAX_APS]; /**< 最近一次扫描结果。 */
} wifi_manager_snapshot_t;

/** @brief 初始化 NVS、网络接口和 Wi-Fi STA 驱动。 @return 成功返回 ESP_OK。 */
esp_err_t wifi_manager_init(void);
/** @brief 启动非阻塞无线接入点扫描。 @return 扫描请求成功提交时返回 ESP_OK。 */
esp_err_t wifi_manager_scan_async(void);
/**
 * @brief 异步连接指定无线接入点。
 * @param ssid 目标接入点名称。
 * @param password 接入密码；开放网络可传空字符串。
 * @return 连接请求成功提交时返回 ESP_OK。
 */
esp_err_t wifi_manager_connect(
    const char *ssid,
    const char *password);
/** @brief 断开当前 STA 连接并取消重连任务。 @return 请求处理结果。 */
esp_err_t wifi_manager_disconnect(void);
/** @brief 复制最新线程安全 Wi-Fi 状态和扫描结果列表。 @param[out] snapshot 接收快照的对象。 */
void wifi_manager_get_snapshot(wifi_manager_snapshot_t *snapshot);

#endif /* WIFI_MANAGER_H */
