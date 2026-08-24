#include "wifi_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

static const char *TAG = "WIFI_MANAGER";

static SemaphoreHandle_t s_lock;
static wifi_manager_snapshot_t s_snapshot;
static esp_netif_t *s_station_netif;
static esp_event_handler_instance_t s_wifi_event_instance;
static esp_event_handler_instance_t s_ip_event_instance;
static esp_timer_handle_t s_reconnect_timer;
static bool s_ignore_next_disconnect;
static bool s_user_disconnect;
static uint32_t s_reconnect_attempt;

#define WIFI_MANAGER_RECONNECT_MAX_DELAY_MS 10000U

/** @brief 获取保护 Wi-Fi 快照和重连状态的互斥锁。 */
static void wifi_manager_lock(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

/** @brief 释放保护 Wi-Fi 快照和重连状态的互斥锁。 */
static void wifi_manager_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

/** @brief 在已持有 Wi-Fi 互斥锁时更新 UI 可见状态。 */
static void wifi_manager_set_status_locked(const char *status)
{
    strlcpy(s_snapshot.status, status, sizeof(s_snapshot.status));
    s_snapshot.revision++;
}

/** @brief 返回重连次数对应的、经过上限限制的指数退避延时。 */
static uint32_t wifi_manager_reconnect_delay_ms(uint32_t attempt)
{
    uint32_t delay_ms = 1000U << (attempt < 4U ? attempt : 4U);
    if (delay_ms > WIFI_MANAGER_RECONNECT_MAX_DELAY_MS) {
        delay_ms = WIFI_MANAGER_RECONNECT_MAX_DELAY_MS;
    }
    return delay_ms;
}

/**
 * @brief STA 断开后安排一次自动重连。
 * @note 调用者必须持有 s_lock；用户主动请求断开时不会自动重连。
 */
static void wifi_manager_schedule_reconnect_locked(uint16_t reason)
{
    if (s_reconnect_timer == NULL || s_user_disconnect) {
        s_snapshot.connecting = false;
        snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Wi-Fi offline (reason %u) - tap CONNECT", (unsigned int)reason);
        s_snapshot.revision++;
        return;
    }

    const uint32_t delay_ms =
        wifi_manager_reconnect_delay_ms(s_reconnect_attempt);
    if (s_reconnect_attempt < UINT32_MAX) {
        s_reconnect_attempt++;
    }
    s_snapshot.connecting = true;
    snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Wi-Fi lost (%u), retry %lu in %lus", (unsigned int)reason, (unsigned long)s_reconnect_attempt, (unsigned long)((delay_ms + 999U) / 1000U));
    s_snapshot.revision++;

    (void)esp_timer_stop(s_reconnect_timer);
    const esp_err_t timer_result =
        esp_timer_start_once(s_reconnect_timer, delay_ms * 1000ULL);
    if (timer_result != ESP_OK) {
        s_snapshot.connecting = false;
        snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Wi-Fi retry timer failed: %s", esp_err_to_name(timer_result));
        s_snapshot.revision++;
    }
}

/**
 * @brief 启动延迟重连请求的 ESP 定时器回调。
 * @param argument 未使用的定时器参数。
 */
static void wifi_manager_reconnect_timer_callback(void *argument)
{
    (void)argument;

    wifi_manager_lock();
    const bool should_reconnect =
        s_snapshot.initialized && !s_snapshot.connected &&
        !s_user_disconnect;
    if (should_reconnect) {
        snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Reconnecting Wi-Fi (attempt %lu)...", (unsigned long)s_reconnect_attempt);
        s_snapshot.revision++;
    }
    wifi_manager_unlock();

    if (!should_reconnect) {
        return;
    }

    const esp_err_t result = esp_wifi_connect();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi reconnect request failed: %s", esp_err_to_name(result));
        wifi_manager_lock();
        wifi_manager_schedule_reconnect_locked(0U);
        wifi_manager_unlock();
    }
}

/** @brief 将 ESP-IDF 错误格式化到共享 Wi-Fi 状态文本中。 */
static void wifi_manager_set_error(esp_err_t error, const char *operation)
{
    wifi_manager_lock();
    snprintf(s_snapshot.status, sizeof(s_snapshot.status), "%s: %s", operation, esp_err_to_name(error));
    s_snapshot.revision++;
    wifi_manager_unlock();
}

/** @brief qsort 比较函数：按 RSSI 从高到低排列扫描到的 AP。 */
static int wifi_manager_compare_ap(const void *left, const void *right)
{
    const wifi_ap_record_t *a = left;
    const wifi_ap_record_t *b = right;
    return (int)b->rssi - (int)a->rssi;
}

/**
 * @brief 获取、排序、去重并发布已完成的 AP 扫描结果。
 *
 * 受 UI 列表容量限制，只保留信号最强的 WIFI_MANAGER_MAX_APS 个可见网络。
 */
static void wifi_manager_store_scan_results(void)
{
    wifi_ap_record_t records[WIFI_MANAGER_MAX_APS];
    uint16_t record_count = WIFI_MANAGER_MAX_APS;
    uint16_t total_count = 0U;

    esp_err_t result = esp_wifi_scan_get_ap_num(&total_count);
    if (result == ESP_OK) {
        result = esp_wifi_scan_get_ap_records(&record_count, records);
    }
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_snapshot.scanning = false;
        snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Scan failed: %s", esp_err_to_name(result));
        s_snapshot.revision++;
        wifi_manager_unlock();
        return;
    }

    qsort(records, record_count, sizeof(records[0]), wifi_manager_compare_ap);

    wifi_manager_lock();
    memset(s_snapshot.aps, 0, sizeof(s_snapshot.aps));
    s_snapshot.ap_count = 0U;
    for (uint16_t i = 0U;
         i < record_count && s_snapshot.ap_count < WIFI_MANAGER_MAX_APS;
         i++) {
        if (records[i].ssid[0] == '\0') {
            continue;
        }

        bool duplicate = false;
        for (uint16_t j = 0U; j < s_snapshot.ap_count; j++) {
            if (strncmp(s_snapshot.aps[j].ssid, (const char *)records[i].ssid, WIFI_MANAGER_SSID_MAX_LEN) == 0) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        wifi_manager_ap_t *destination =
            &s_snapshot.aps[s_snapshot.ap_count++];
        strlcpy(destination->ssid, (const char *)records[i].ssid, sizeof(destination->ssid));
        destination->rssi = records[i].rssi;
        destination->secured = records[i].authmode != WIFI_AUTH_OPEN;
    }

    s_snapshot.scanning = false;
    s_snapshot.scan_generation++;
    if (s_snapshot.connected) {
        snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Connected - found %u networks", (unsigned int)s_snapshot.ap_count);
    } else {
        snprintf(s_snapshot.status, sizeof(s_snapshot.status), "Found %u networks", (unsigned int)s_snapshot.ap_count);
    }
    s_snapshot.revision++;
    wifi_manager_unlock();

    ESP_LOGI(TAG, "Wi-Fi scan complete: %u visible, %u retained", (unsigned int)total_count, (unsigned int)record_count);
}

/**
 * @brief 处理来自 ESP-IDF 的异步 Wi-Fi 和 IP 事件。
 *
 * 此回调负责更新快照并启动重连定时器，不直接操作 LVGL，因此 UI 可以安全地
 * 轮询生成的状态快照。
 */
static void wifi_manager_event_handler(
    void *argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)argument;

    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_SCAN_DONE:
            wifi_manager_store_scan_results();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            wifi_manager_lock();
            s_snapshot.connecting = true;
            wifi_manager_set_status_locked("Connected to AP - waiting for IP");
            wifi_manager_unlock();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            const wifi_event_sta_disconnected_t *event = event_data;
            const uint16_t reason =
                event != NULL ? (uint16_t)event->reason : 0U;
            wifi_manager_lock();
            s_snapshot.connected = false;
            s_snapshot.ip_address[0] = '\0';
            if (s_ignore_next_disconnect) {
                s_ignore_next_disconnect = false;
            } else if (s_user_disconnect) {
                s_snapshot.connecting = false;
                s_user_disconnect = false;
                s_reconnect_attempt = 0U;
                wifi_manager_set_status_locked("Disconnected");
            } else {
                wifi_manager_schedule_reconnect_locked(reason);
            }
            wifi_manager_unlock();
            ESP_LOGW(TAG, "Wi-Fi station disconnected, reason=%u", (unsigned int)reason);
            break;
        }

        default:
            break;
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        wifi_manager_lock();
        s_snapshot.connected = true;
        s_snapshot.connecting = false;
        s_reconnect_attempt = 0U;
        (void)esp_timer_stop(s_reconnect_timer);
        if (event != NULL) {
            esp_ip4addr_ntoa(&event->ip_info.ip, s_snapshot.ip_address, sizeof(s_snapshot.ip_address));
        }
        wifi_manager_set_status_locked("Connected");
        wifi_manager_unlock();
        ESP_LOGI(TAG, "Connected to %s with IP %s", s_snapshot.ssid, s_snapshot.ip_address);
    }
}

/**
 * @brief 初始化 NVS、网络事件循环和 Wi-Fi STA 模式。
 * @return 就绪时返回 ESP_OK，否则返回初始化过程中遇到的首个 ESP-IDF 错误码。
 */
esp_err_t wifi_manager_init(void)
{
    if (s_lock != NULL) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    strlcpy(s_snapshot.status, "Initializing Wi-Fi", sizeof(s_snapshot.status));

    const esp_timer_create_args_t reconnect_timer_configuration = {
        .callback = wifi_manager_reconnect_timer_callback,
        .name = "wifi_reconnect",
    };
    esp_err_t result = esp_timer_create(&reconnect_timer_configuration, &s_reconnect_timer);
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi retry timer failed");
        return result;
    }

    result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "NVS init failed");
        return result;
    }

    result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        wifi_manager_set_error(result, "Network init failed");
        return result;
    }

    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        wifi_manager_set_error(result, "Event loop failed");
        return result;
    }

    s_station_netif = esp_netif_create_default_wifi_sta();
    if (s_station_netif == NULL) {
        result = ESP_ERR_NO_MEM;
        wifi_manager_set_error(result, "STA interface failed");
        return result;
    }

    const wifi_init_config_t configuration = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&configuration);
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi init failed");
        return result;
    }

    result = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_manager_event_handler, NULL, &s_wifi_event_instance);
    if (result == ESP_OK) {
        result = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_manager_event_handler, NULL, &s_ip_event_instance);
    }
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi event setup failed");
        return result;
    }

    result = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    if (result == ESP_OK) {
        result = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (result == ESP_OK) {
        result = esp_wifi_start();
    }
    if (result == ESP_OK) {
        result = esp_wifi_set_ps(WIFI_PS_NONE);
    }
    if (result != ESP_OK) {
        wifi_manager_set_error(result, "Wi-Fi start failed");
        return result;
    }

    wifi_manager_lock();
    s_snapshot.initialized = true;
    wifi_manager_set_status_locked("Ready - tap SCAN");
    wifi_manager_unlock();
    ESP_LOGI(TAG, "Wi-Fi station initialized");
    return ESP_OK;
}

/**
 * @brief 启动非阻塞 Wi-Fi 扫描；结果通过 WIFI_EVENT_SCAN_DONE 返回。
 * @return 管理器未就绪或扫描已在进行时返回 ESP_ERR_INVALID_STATE。
 */
esp_err_t wifi_manager_scan_async(void)
{
    wifi_manager_lock();
    if (!s_snapshot.initialized) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (s_snapshot.scanning) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_snapshot.scanning = true;
    wifi_manager_set_status_locked("Scanning...");
    wifi_manager_unlock();

    const esp_err_t result = esp_wifi_scan_start(NULL, false);
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_snapshot.scanning = false;
        wifi_manager_unlock();
        wifi_manager_set_error(result, "Scan failed");
    }
    return result;
}

/**
 * @brief 配置并开始连接指定的无线 AP。
 * @param ssid 以空字符结尾且不超过 32 字节的 SSID。
 * @param password 以空字符结尾且不超过 63 字节的 WPA 密码。
 * @return 请求执行结果；返回 ESP_OK 仅表示连接请求已开始，并不表示连接完成。
 */
esp_err_t wifi_manager_connect(const char *ssid, const char *password)
{
    if (ssid == NULL || ssid[0] == '\0' || password == NULL ||
        strlen(ssid) > WIFI_MANAGER_SSID_MAX_LEN ||
        strlen(password) > 63U) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t configuration = {0};
    strlcpy((char *)configuration.sta.ssid, ssid, sizeof(configuration.sta.ssid));
    strlcpy((char *)configuration.sta.password, password, sizeof(configuration.sta.password));
    configuration.sta.threshold.authmode = WIFI_AUTH_OPEN;
    configuration.sta.pmf_cfg.capable = true;
    configuration.sta.pmf_cfg.required = false;

    wifi_manager_lock();
    if (!s_snapshot.initialized) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    (void)esp_timer_stop(s_reconnect_timer);
    s_reconnect_attempt = 0U;
    strlcpy(s_snapshot.ssid, ssid, sizeof(s_snapshot.ssid));
    s_snapshot.ip_address[0] = '\0';
    s_snapshot.connected = false;
    s_snapshot.connecting = true;
    s_user_disconnect = false;
    wifi_manager_set_status_locked("Connecting...");
    wifi_manager_unlock();

    wifi_manager_lock();
    s_ignore_next_disconnect = true;
    wifi_manager_unlock();
    const esp_err_t disconnect_result = esp_wifi_disconnect();
    if (disconnect_result != ESP_OK) {
        wifi_manager_lock();
        s_ignore_next_disconnect = false;
        wifi_manager_unlock();
    }

    esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &configuration);
    if (result == ESP_OK) {
        result = esp_wifi_connect();
    }
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_snapshot.connecting = false;
        wifi_manager_unlock();
        wifi_manager_set_error(result, "Connect failed");
    }
    return result;
}

/**
 * @brief 停止自动重连并断开 Wi-Fi STA。
 * @return 操作成功时返回 ESP_OK；工作站原本已断开时也返回 ESP_OK。
 */
esp_err_t wifi_manager_disconnect(void)
{
    wifi_manager_lock();
    if (!s_snapshot.initialized) {
        wifi_manager_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    (void)esp_timer_stop(s_reconnect_timer);
    s_reconnect_attempt = 0U;
    s_user_disconnect = true;
    s_ignore_next_disconnect = false;
    s_snapshot.connecting = false;
    wifi_manager_set_status_locked("Disconnecting...");
    wifi_manager_unlock();

    const esp_err_t result = esp_wifi_disconnect();
    if (result != ESP_OK) {
        wifi_manager_lock();
        s_user_disconnect = false;
        s_snapshot.connected = false;
        s_snapshot.ip_address[0] = '\0';
        wifi_manager_set_status_locked("Disconnected");
        wifi_manager_unlock();
    }
    return result == ESP_ERR_WIFI_NOT_CONNECT ? ESP_OK : result;
}

/**
 * @brief 在管理器互斥锁保护下复制当前 Wi-Fi 状态和可见 AP 列表。
 * @param[out] snapshot 接收状态的目标快照；传入 NULL 时忽略。
 */
void wifi_manager_get_snapshot(wifi_manager_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    wifi_manager_lock();
    *snapshot = s_snapshot;
    wifi_manager_unlock();
}
