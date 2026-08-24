#include "can_ESP.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MOTOR_CAN_TX_GPIO                  GPIO_NUM_5
#define MOTOR_CAN_RX_GPIO                  GPIO_NUM_6
#define MOTOR_CAN_CONTROL_QUEUE_LENGTH     12U
#define MOTOR_CAN_RX_QUEUE_LENGTH          24U
#define MOTOR_CAN_TX_QUEUE_DEPTH           4U
#define MOTOR_CAN_TX_TIMEOUT_MS            5
#define MOTOR_CAN_TX_TASK_PERIOD_MS        5U
#define CAN_ESP_HEARTBEAT_PERIOD_MS      100U
#define MOTOR_CAN_LINK_TIMEOUT_MS          250U

typedef struct
{
    MotorCan_Command_t command; /**< 等待 TX 任务发送的 CAN 电机命令码。 */
    int32_t value;              /**< 命令携带的有符号参数；具体单位由 command 决定。 */
} CAN_ESP_request_t;

/*
 * twai_frame_t 只保存指向负载的指针，因此离开 ISR 前需要把接收帧复制到
 * 这个自包含对象中。
 */
typedef struct
{
    uint32_t identifier;                    /**< 接收帧的 11 位或 29 位 CAN 标识符。 */
    uint8_t data_length;                    /**< 接收帧的有效负载长度，单位为字节。 */
    bool extended;                          /**< 为 true 时表示扩展帧，为 false 时表示标准帧。 */
    bool remote;                            /**< 为 true 时表示远程帧，为 false 时表示数据帧。 */
    uint8_t data[MOTOR_CAN_FRAME_SIZE];      /**< 从驱动帧复制出的 8 字节自包含负载。 */
} CAN_ESP_rx_frame_t;

static const char *TAG = "MOTOR_CAN";       /**< 本模块输出 ESP-IDF 日志时使用的标签。 */
static twai_node_handle_t s_twai_node;      /**< ESP-IDF 片上 TWAI 控制器节点句柄。 */
static QueueHandle_t s_control_queue;       /**< 保存 START、STOP、MODE 等离散控制请求的队列。 */
static QueueHandle_t s_rx_queue;            /**< ISR 向 RX 任务传递完整接收帧的队列。 */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED; /**< 保护快照和跨任务共享状态的自旋锁。 */
static CAN_ESP_snapshot_t s_snapshot;     /**< 最近一次 CAN 遥测及诊断状态的统一快照。 */
static int16_t s_pending_speed_rpm;         /**< 等待发送的最新速度目标，单位为 rpm。 */
static uint16_t s_pending_position_cdeg;    /**< 等待发送的最新单圈位置目标，单位为 0.01°。 */
static bool s_speed_dirty;                  /**< 为 true 时表示最新速度目标尚未成功发送。 */
static bool s_position_dirty;               /**< 为 true 时表示最新位置目标尚未成功发送。 */
static bool s_control_enabled;              /**< 为 true 时 CAN 通道拥有电机命令发送权。 */
static uint8_t s_tx_sequence;               /**< CAN 命令帧的递增序列号，用于诊断命令先后顺序。 */
static int64_t s_last_status_us;            /**< 最近收到 0x180 状态帧的时间戳，单位为 μs。 */
static bool s_recovery_requested;           /**< 为 true 时表示已经请求 TWAI 从 Bus-Off 恢复。 */
static bool s_transceiver_test_passed;      /**< 最近一次外部 CAN 收发器 GPIO 自检结果。 */
static uint32_t s_pending_error_flags;      /**< ISR 累积、等待任务读取的 TWAI 错误位集合。 */
static int64_t s_last_error_log_us;         /**< 最近输出 CAN 错误日志的时间戳，单位为 μs。 */
static uint8_t s_tx_data[MOTOR_CAN_FRAME_SIZE]; /**< 持久存在的 CAN 命令帧发送负载。 */
/** ESP-IDF TWAI 发送帧描述符；buffer 始终指向持久数组 s_tx_data。 */
static twai_frame_t s_tx_frame = {
    .header = {
        .id = MOTOR_CAN_ID_COMMAND,
        .dlc = MOTOR_CAN_FRAME_SIZE,
    },
    .buffer = s_tx_data,
    .buffer_len = sizeof(s_tx_data),
};
static bool s_tx_pending;                   /**< 为 true 时驱动仍可能持有 s_tx_frame/s_tx_data。 */
static TaskHandle_t s_rx_task;              /**< CAN 接收解析任务的 FreeRTOS 句柄。 */
static TaskHandle_t s_tx_task;              /**< CAN 命令发送与总线维护任务的 FreeRTOS 句柄。 */
static bool s_initialized;                  /**< 为 true 时 TWAI、队列和工作任务均已创建。 */

/**
 * @brief 对外部 CAN 收发器执行 GPIO 电平连通性自检。
 *
 * 在 TWAI 接管引脚前，分别驱动 TXD 为隐性和显性电平并观察 RXD。此结果仅
 * 用作诊断提示；本地测试失败不会阻止 CAN 启动，因为有效状态帧更能证明链路正常。
 * @return RXD 跟随预期本地电平变化时返回 true，否则返回 false。
 */
static bool CAN_ESP_transceiver_self_test(void)
{
    /** 将 CAN RXD 配置为带上拉输入，用于读取收发器反馈电平。 */
    const gpio_config_t rx_config = {
        .pin_bit_mask = 1ULL << MOTOR_CAN_RX_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    /** 将 CAN TXD 临时配置为普通输出，用于产生显性/隐性测试电平。 */
    const gpio_config_t tx_config = {
        .pin_bit_mask = 1ULL << MOTOR_CAN_TX_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&rx_config));
    ESP_ERROR_CHECK(gpio_config(&tx_config));

    gpio_set_level(MOTOR_CAN_TX_GPIO, 1);
    esp_rom_delay_us(10);
    /** TXD 输出隐性高电平时读取到的 RXD 电平。 */
    const int recessive_level = gpio_get_level(MOTOR_CAN_RX_GPIO);

    gpio_set_level(MOTOR_CAN_TX_GPIO, 0);
    esp_rom_delay_us(10);
    /** TXD 输出显性低电平时读取到的 RXD 电平。 */
    const int dominant_level = gpio_get_level(MOTOR_CAN_RX_GPIO);

    gpio_set_level(MOTOR_CAN_TX_GPIO, 1);
    esp_rom_delay_us(10);
    /** TXD 恢复高电平后读取到的 RXD 电平，用于确认链路能够释放。 */
    const int released_level = gpio_get_level(MOTOR_CAN_RX_GPIO);
    /** 三次 RXD 采样是否都符合收发器本地回读预期。 */
    const bool passed =
        (recessive_level == 1) &&
        (dominant_level == 0) &&
        (released_level == 1);

    ESP_LOGI(TAG, "Transceiver self-test RXD: idle=%d dominant=%d release=%d -> %s", recessive_level, dominant_level, released_level, passed ? "PASS" : "FAIL");
    if (!passed) {
        ESP_LOGW(TAG, "Local CAN path check failed; live TWAI diagnostics remain enabled. " "Verify Port1 GPIO5->TXD, GPIO6<-RXD, VCC/VIO/EN and S=LOW");
    }
    return passed;
}

/** @brief 在模块锁保护下递增共享发送错误计数。 */
static void CAN_ESP_record_tx_error(void)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot.transmit_errors++;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 编码并发送一帧 CAN 命令帧。
 *
 * ESP-IDF 6 将帧负载指针放入队列，因此复用持久缓冲区之前，本函数会等待前一帧
 * 和当前帧发送完成。只有 CAN 持有控制权时才允许发送会影响电机的命令。
 *
 * @param command 协议命令操作码。
 * @param value 按照 @p command 规定格式编码的命令值。
 * @return 发送完成时返回 ESP_OK，否则返回 TWAI 错误码。
 */
static esp_err_t CAN_ESP_transmit(MotorCan_Command_t command, int32_t value)
{
    if (command != MOTOR_CAN_CMD_NOP && command != MOTOR_CAN_CMD_PING) {
        /** 当前 CAN 通道是否获得发送电机控制命令的权限。 */
        bool enabled;
        portENTER_CRITICAL(&s_lock);
        enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);
        if (!enabled) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    /*
     * ESP-IDF 6 将帧指针入队，而不是复制完整对象。前一帧仍可能由驱动持有时，
     * 不得覆盖持久发送缓冲区。
     */
    if (s_tx_pending) {
        /** 等待上一持久帧发送完成的结果，成功后才可覆盖发送缓冲区。 */
        const esp_err_t pending_result =
            twai_node_transmit_wait_all_done(s_twai_node, MOTOR_CAN_TX_TIMEOUT_MS);
        if (pending_result != ESP_OK) {
            CAN_ESP_record_tx_error();
            return pending_result;
        }
        s_tx_pending = false;
    }

    memset(s_tx_data, 0, sizeof(s_tx_data));
    s_tx_data[0] = MOTOR_CAN_PROTOCOL_VERSION;
    s_tx_data[1] = ++s_tx_sequence;
    s_tx_data[2] = (uint8_t)command;

    switch (command) {
    case MOTOR_CAN_CMD_SET_MODE:
        s_tx_data[3] = (uint8_t)value;
        break;
    case MOTOR_CAN_CMD_SET_SPEED_RPM:
        MotorCan_WriteS16(&s_tx_data[3], (int16_t)value);
        break;
    case MOTOR_CAN_CMD_SET_POSITION_CDEG:
        MotorCan_WriteS32(&s_tx_data[3], value);
        break;
    default:
        break;
    }

    /** 当前命令帧排队及等待物理发送完成的结果。 */
    esp_err_t result = twai_node_transmit(s_twai_node, &s_tx_frame,MOTOR_CAN_TX_TIMEOUT_MS);
    if (result == ESP_OK) {
        s_tx_pending = true;
        result = twai_node_transmit_wait_all_done(s_twai_node, MOTOR_CAN_TX_TIMEOUT_MS);
        if (result == ESP_OK) {
            s_tx_pending = false;
        }
    }
    if (result != ESP_OK) {
        CAN_ESP_record_tx_error();
    } else {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.transmitted_frames++;
        portEXIT_CRITICAL(&s_lock);
    }
    return result;
}

/**
 * @brief 为 CAN TX 工作任务排队一个离散控制动作。
 *
 * 有界队列已满时丢弃最旧条目，使最新用户动作优先于过时的界面操作。
 */
static void CAN_ESP_queue_control(MotorCan_Command_t command, int32_t value)
{
    if (s_control_queue == NULL) {
        return;
    }

    /** 由命令码及参数组成、准备写入控制队列的请求。 */
    const CAN_ESP_request_t request = {
        .command = command,
        .value = value,
    };

    if (xQueueSend(s_control_queue, &request, 0) != pdTRUE) {
        /** 队列已满时移除的最旧请求，用于给最新用户操作腾出空间。 */
        CAN_ESP_request_t discarded;
        (void)xQueueReceive(s_control_queue, &discarded, 0);
        (void)xQueueSend(s_control_queue, &request, 0);
    }
}

/**
 * @brief 解码 CAN 状态帧 0x180 并刷新电机/链路状态。
 * @param frame 包含 8 字节负载的完整 CAN 接收帧。
 */
static void CAN_ESP_parse_status(const CAN_ESP_rx_frame_t *frame)
{
    if (frame->data[0] != MOTOR_CAN_PROTOCOL_VERSION) {
        return;
    }

    /** 状态帧 byte3 中由 STM32 编码的模式、运行和故障标志。 */
    const uint8_t flags = frame->data[3];
    /** 收到有效状态帧时的单调时钟时间戳，单位为 μs。 */
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);
    s_snapshot.mode =
        (flags & MOTOR_CAN_STATUS_POSITION_MODE) != 0U
            ? MOTOR_CAN_MODE_POSITION
            : MOTOR_CAN_MODE_SPEED;
    s_snapshot.motor_running =
        (flags & MOTOR_CAN_STATUS_MOTOR_RUNNING) != 0U;
    s_snapshot.motor_fault =
        (flags & MOTOR_CAN_STATUS_MOTOR_FAULT) != 0U;
    s_snapshot.command_rejected =
        (flags & MOTOR_CAN_STATUS_COMMAND_REJECTED) != 0U;
    s_snapshot.measured_speed_rpm =
        MotorCan_ReadS16(&frame->data[4]);
    s_snapshot.faults = MotorCan_ReadU16(&frame->data[6]);
    s_snapshot.received_frames++;
    /* 有效总线帧比可选的 GPIO 自检结果更能证明收发器链路正常。 */
    s_snapshot.transceiver_fault = false;
    s_transceiver_test_passed = true;
    s_last_status_us = now_us;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将 CAN 参考值/位置帧 0x181 解码到遥测快照中。
 * @param frame 完整的 CAN 接收帧。
 */
static void CAN_ESP_parse_references(const CAN_ESP_rx_frame_t *frame)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot.reference_speed_rpm =
        MotorCan_ReadS16(&frame->data[0]);
    s_snapshot.current_position_cdeg =
        MotorCan_ReadU16(&frame->data[2]);
    s_snapshot.target_position_cdeg =
        MotorCan_ReadU16(&frame->data[4]);
    s_snapshot.position_error_cdeg =
        MotorCan_ReadS16(&frame->data[6]);
    s_snapshot.received_frames++;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将 CAN 电气电流帧 0x182 解码到遥测快照中。
 * @param frame 完整的 CAN 接收帧。
 */
static void CAN_ESP_parse_electrical(const CAN_ESP_rx_frame_t *frame)
{
    portENTER_CRITICAL(&s_lock);
    s_snapshot.iq_ma = MotorCan_ReadS16(&frame->data[0]);
    s_snapshot.id_ma = MotorCan_ReadS16(&frame->data[2]);
    s_snapshot.iq_reference_ma = MotorCan_ReadS16(&frame->data[4]);
    s_snapshot.id_reference_ma = MotorCan_ReadS16(&frame->data[6]);
    s_snapshot.received_frames++;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief ISR 上下文的 TWAI 接收回调：将帧复制到 RTOS 队列。
 *
 * 此处禁止执行帧解析、日志记录或界面操作；中断返回后由 RX 任务完成所有
 * 非简单处理。
 */
static bool IRAM_ATTR CAN_ESP_rx_callback(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *event_data,
    void *user_context)
{
    (void)event_data;

    /** 自包含的接收帧副本，将在 ISR 结束前写入 RX 队列。 */
    CAN_ESP_rx_frame_t received = {0};
    /** 传给 TWAI ISR 接收 API 的临时帧描述符，其负载写入 received.data。 */
    twai_frame_t frame = {
        .buffer = received.data,
        .buffer_len = sizeof(received.data),
    };
    /** 指示入队操作是否唤醒了更高优先级任务。 */
    BaseType_t task_woken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        received.identifier = frame.header.id;
        received.data_length = (uint8_t)frame.header.dlc;
        received.extended = frame.header.ide;
        received.remote = frame.header.rtr;

        (void)xQueueSendFromISR((QueueHandle_t)user_context, &received, &task_woken);
    }

    return task_woken == pdTRUE;
}

/**
 * @brief ISR 上下文的 TWAI 错误回调：记录标志供后续任务处理。
 * @return 此回调不会唤醒更高优先级任务，因此始终返回 false。
 */
static bool IRAM_ATTR CAN_ESP_error_callback(
    twai_node_handle_t handle,
    const twai_error_event_data_t *event_data,
    void *user_context)
{
    (void)handle;
    (void)user_context;

    portENTER_CRITICAL_ISR(&s_lock);
    s_pending_error_flags |= event_data->err_flags.val;
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

/**
 * @brief 校验队列中 CAN 帧并按协议 ID 分发的接收任务。
 * @param argument 未使用的任务参数。
 * @note 该任务运行在 ISR 上下文之外，因此可以进入模块临界区。
 */
static void CAN_ESP_rx_task(void *argument)
{
    (void)argument;
    /** 从 RX 队列取出的完整 CAN 帧，在任务上下文中进行校验与解析。 */
    CAN_ESP_rx_frame_t frame;

    for (;;) {
        if (xQueueReceive(s_rx_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (frame.extended || frame.remote ||
            frame.data_length != MOTOR_CAN_FRAME_SIZE) {
            continue;
        }

        switch (frame.identifier) {
        case MOTOR_CAN_ID_STATUS:
            CAN_ESP_parse_status(&frame);
            break;
        case MOTOR_CAN_ID_REFERENCES:
            CAN_ESP_parse_references(&frame);
            break;
        case MOTOR_CAN_ID_ELECTRICAL:
            CAN_ESP_parse_electrical(&frame);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief 更新 CAN 诊断信息，并启动/跟踪 Bus-Off 恢复流程。
 * @return 仅当控制器能够安全发送正常报文时返回 true。
 */
static bool CAN_ESP_service_bus_state(void)
{
    /** TWAI 控制器当前状态、发送错误计数和接收错误计数。 */
    twai_node_status_t status;
    if (twai_node_get_info(s_twai_node, &status, NULL) != ESP_OK) {
        return false;
    }

    uint32_t error_flags;       /**< 本轮从 ISR 错误位集合中取出的待处理标志。 */
    bool transceiver_fault;     /**< 快照记录的外部 CAN 收发器自检故障状态。 */
    portENTER_CRITICAL(&s_lock);
    error_flags = s_pending_error_flags;
    s_pending_error_flags = 0U;
    transceiver_fault = s_snapshot.transceiver_fault;
    portEXIT_CRITICAL(&s_lock);
    /** 查询与限流错误日志所使用的当前单调时间，单位为 μs。 */
    const int64_t now_us = esp_timer_get_time();
    if ((error_flags != 0U) &&
        ((now_us - s_last_error_log_us) >= 1000000LL)) {
        /** 将原始错误位映射为 ACK、位、格式、填充和仲裁错误字段。 */
        const twai_error_flags_t decoded = {.val = error_flags};
        s_last_error_log_us = now_us;
        ESP_LOGW(TAG, "CAN error flags=0x%02lx ACK=%u BIT=%u FORM=%u STUFF=%u ARB=%u " "RXD=%d LOCAL=%s", (unsigned long)error_flags, (unsigned)decoded.ack_err, (unsigned)decoded.bit_err, (unsigned)decoded.form_err, (unsigned)decoded.stuff_err, (unsigned)decoded.arb_lost, gpio_get_level(MOTOR_CAN_RX_GPIO), transceiver_fault ? "WARN" : "PASS");
    }

    /** 控制器当前是否已因错误计数过高进入 Bus-Off 状态。 */
    const bool bus_off = status.state == TWAI_ERROR_BUS_OFF;
    portENTER_CRITICAL(&s_lock);
    s_snapshot.bus_off = bus_off;
    portEXIT_CRITICAL(&s_lock);

    if (bus_off) {
        if (!s_recovery_requested) {
            ESP_LOGW(TAG, "Bus-off (TEC=%u REC=%u RXD=%d LOCAL=%s); starting recovery", (unsigned)status.tx_error_count, (unsigned)status.rx_error_count, gpio_get_level(MOTOR_CAN_RX_GPIO), transceiver_fault ? "WARN" : "PASS");
            if (twai_node_recover(s_twai_node) == ESP_OK) {
                s_recovery_requested = true;
            }
        }
        return false;
    }

    if ((status.state == TWAI_ERROR_ACTIVE) && s_recovery_requested) {
        ESP_LOGI(TAG, "CAN bus recovered");
        s_recovery_requested = false;
    }
    return true;
}

/**
 * @brief 仅当没有更新目标覆盖时，重新标记发送失败的位置目标。
 * @param position_cdeg 发送失败的位置目标值，单位为 0.01°。
 */
static void CAN_ESP_restore_position_if_latest(uint16_t position_cdeg)
{
    portENTER_CRITICAL(&s_lock);
    if (s_pending_position_cdeg == position_cdeg) {
        s_position_dirty = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 仅当没有更新目标覆盖时，重新标记发送失败的速度目标。
 * @param speed_rpm 发送失败的速度目标值，单位为 rpm。
 */
static void CAN_ESP_restore_speed_if_latest(int16_t speed_rpm)
{
    portENTER_CRITICAL(&s_lock);
    if (s_pending_speed_rpm == speed_rpm) {
        s_speed_dirty = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 周期处理排队动作、最新目标和心跳的 CAN TX 工作任务。
 *
 * 总线关闭期间任务不会取出请求。短暂发送失败后，离散命令会重新放回队首；
 * 连续速度和位置目标仅在仍是最新请求值时才重试。
 * @param argument 未使用的任务参数。
 */
static void CAN_ESP_tx_task(void *argument)
{
    (void)argument;
    /** 周期任务的上一次唤醒节拍，用于 vTaskDelayUntil 保持固定周期。 */
    TickType_t last_wake = xTaskGetTickCount();
    /** 下一次允许发送 PING 心跳的时间戳，单位为 μs。 */
    int64_t next_heartbeat_us = 0;

    for (;;)
    {
        /*
         * 控制器处于总线关闭状态时，不得取出命令或调用驱动的发送等待接口。
         * 待发送帧保持有效，并在恢复后由驱动重试。
         */
        if (!CAN_ESP_service_bus_state()) 
        {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CAN_TX_TASK_PERIOD_MS));
            continue;
        }

        /** 本轮任务开始时读取的 CAN 控制权快照。 */
        bool control_enabled;
        portENTER_CRITICAL(&s_lock);
        control_enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);

        /** 从离散控制队列中取出的待发送命令。 */
        CAN_ESP_request_t request;
        while (control_enabled &&xQueueReceive(s_control_queue, &request, 0) == pdTRUE) 
        {
            if (CAN_ESP_transmit(request.command, request.value) != ESP_OK) {
                /*
                 * START、STOP、MODE 和 ACK 均为边沿触发的界面动作。短暂发送
                 * 超时时将命令保留在队首，避免静默丢失按键操作。
                 */
                (void)xQueueSendToFront(s_control_queue, &request, 0);
                break;
            }
        }

        bool send_speed;          /**< 本轮是否需要发送最新速度目标。 */
        bool send_position;       /**< 本轮是否需要发送最新位置目标。 */
        int16_t speed_rpm;        /**< 本轮复制出的速度目标，单位为 rpm。 */
        uint16_t position_cdeg;   /**< 本轮复制出的位置目标，单位为 0.01°。 */

        portENTER_CRITICAL(&s_lock);
        send_speed = control_enabled && s_speed_dirty;
        speed_rpm = s_pending_speed_rpm;
        s_speed_dirty = false;
        send_position = control_enabled && s_position_dirty;
        position_cdeg = s_pending_position_cdeg;
        s_position_dirty = false;
        portEXIT_CRITICAL(&s_lock);

        if (send_position &&
            CAN_ESP_transmit(MOTOR_CAN_CMD_SET_POSITION_CDEG, position_cdeg) != ESP_OK) {
            CAN_ESP_restore_position_if_latest(position_cdeg);
        }
        if (send_speed &&
            CAN_ESP_transmit(MOTOR_CAN_CMD_SET_SPEED_RPM, speed_rpm) != ESP_OK) {
            CAN_ESP_restore_speed_if_latest(speed_rpm);
        }

        /** 当前单调时间，用于判断是否到达下一心跳发送时刻。 */
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_heartbeat_us) {
            (void)CAN_ESP_transmit(MOTOR_CAN_CMD_PING, 0);
            next_heartbeat_us =
                now_us + (CAN_ESP_HEARTBEAT_PERIOD_MS * 1000LL);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CAN_TX_TASK_PERIOD_MS));
    }
}

/** @brief 删除已经创建的 CAN 专属 FreeRTOS 队列。 */
static void CAN_ESP_delete_queues(void)
{
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    if (s_control_queue != NULL) {
        vQueueDelete(s_control_queue);
        s_control_queue = NULL;
    }
}

/**
 * @brief 初始化 TWAI、过滤器、回调以及绑定核心的 CAN 工作任务。
 *
 * 硬件过滤器允许 0x180～0x183 反馈帧通过。两个工作任务均绑定到 Core 0，
 * 从而与 Core 1 上的界面工作隔离。
 * @return 传输通道就绪时返回 ESP_OK，否则返回内存分配或 TWAI 错误码。
 */
esp_err_t CAN_ESP_Init(void)//初始化CAN
{
    if (s_initialized) {
        return ESP_OK;
    }
    memset(&s_snapshot, 0, sizeof(s_snapshot));//清空
    s_control_enabled = false;
    s_snapshot.mode = MOTOR_CAN_MODE_SPEED;
    s_pending_speed_rpm = 0;
    s_pending_position_cdeg = 0;
    s_speed_dirty = false;
    s_position_dirty = false;
    s_tx_sequence = 0;
    s_last_status_us = 0;
    s_recovery_requested = false;
    s_pending_error_flags = 0U;
    s_last_error_log_us = 0;
    s_tx_pending = false;

    s_transceiver_test_passed = CAN_ESP_transceiver_self_test();
    s_snapshot.transceiver_fault = !s_transceiver_test_passed;

    s_control_queue =
        xQueueCreate(MOTOR_CAN_CONTROL_QUEUE_LENGTH, sizeof(CAN_ESP_request_t));
    s_rx_queue =
        xQueueCreate(MOTOR_CAN_RX_QUEUE_LENGTH, sizeof(CAN_ESP_rx_frame_t));
    if ((s_control_queue == NULL) || (s_rx_queue == NULL)) {
        CAN_ESP_delete_queues();
        return ESP_ERR_NO_MEM;
    }

    /** 片上 TWAI 节点的引脚、位率、重试次数及发送队列配置。 */
    const twai_onchip_node_config_t node_config = 
    {
        .io_cfg = 
        {
            .tx = MOTOR_CAN_TX_GPIO,
            .rx = MOTOR_CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {.bitrate = MOTOR_CAN_BITRATE,},
        .fail_retry_cnt = 3,
        .tx_queue_depth = MOTOR_CAN_TX_QUEUE_DEPTH,
    }; 

    /** 初始化各 TWAI 资源时复用的 ESP-IDF 返回状态。 */
    esp_err_t result =twai_new_node_onchip(&node_config, &s_twai_node);
    if (result != ESP_OK) {
        CAN_ESP_delete_queues();
        return result;
    }

    /*
     * 匹配 0x180～0x183。应用实际使用 0x180～0x182，第四个值用于保证该掩码
     * 能由经典 CAN 控制器表示。
     */
    /** 仅接收 STM32 反馈 ID 0x180～0x183 的硬件掩码过滤器。 */
    const twai_mask_filter_config_t filter_config = {
        .id = MOTOR_CAN_ID_STATUS,
        .mask = 0x7FCU,//这个掩码可以让0x180～0x183通过
        .is_ext = false,
        .no_classic = false,
        .no_fd = true,
    };
    result = twai_node_config_mask_filter(s_twai_node, 0, &filter_config);
    if (result != ESP_OK) {
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        CAN_ESP_delete_queues();
        return result;
    }

    /** TWAI 接收完成和错误事件对应的 ISR 回调表。 */
    const twai_event_callbacks_t callbacks = {
        .on_rx_done = CAN_ESP_rx_callback,
        .on_error = CAN_ESP_error_callback,
    };
    result = twai_node_register_event_callbacks(s_twai_node, &callbacks, s_rx_queue);
    if (result != ESP_OK) {
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        CAN_ESP_delete_queues();
        return result;
    }

    result = twai_node_enable(s_twai_node);
    if (result != ESP_OK) {
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        CAN_ESP_delete_queues();
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    /** TWAI 接管引脚后采集的 RXD 空闲电平，正常隐性状态应为高。 */
    const int rxd_idle_level = gpio_get_level(MOTOR_CAN_RX_GPIO);
    ESP_LOGI(TAG, "Port1 RXD idle level=%d (expected 1); TXD is peripheral output", rxd_idle_level);
    if (rxd_idle_level == 0) {
        ESP_LOGE(TAG, "RXD is stuck low: check transceiver VCC/VIO, S pin, CAN short and wiring");
    }

    if (xTaskCreatePinnedToCore(CAN_ESP_rx_task, "CAN_ESP_rx", 3072, NULL, 12, &s_rx_task, 0) != pdPASS ||
        xTaskCreatePinnedToCore(CAN_ESP_tx_task, "CAN_ESP_tx", 3072, NULL, 11, &s_tx_task, 0) != pdPASS) {
        if (s_rx_task != NULL) {
            vTaskDelete(s_rx_task);
            s_rx_task = NULL;
        }
        if (s_tx_task != NULL) {
            vTaskDelete(s_tx_task);
            s_tx_task = NULL;
        }
        (void)twai_node_disable(s_twai_node);
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
        CAN_ESP_delete_queues();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "TWAI ready: TX=GPIO%d RX=GPIO%d bitrate=%u", MOTOR_CAN_TX_GPIO, MOTOR_CAN_RX_GPIO, MOTOR_CAN_BITRATE);
    s_initialized = true;
    return ESP_OK;
}

/**
 * @brief 在销毁队列和 TWAI 节点前停止 CAN 工作任务。
 * @note 此释放顺序可防止任务或回调访问已经释放的资源。
 */
void CAN_ESP_Deinit(void)
{
    if (!s_initialized) {
        return;
    }

    /* 删除队列或 TWAI 节点之前先停止工作任务。 */
    if (s_rx_task != NULL) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_tx_task != NULL) {
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }
    if (s_twai_node != NULL) {
        (void)twai_node_disable(s_twai_node);
        (void)twai_node_delete(s_twai_node);
        s_twai_node = NULL;
    }
    CAN_ESP_delete_queues();
    gpio_reset_pin(MOTOR_CAN_TX_GPIO);
    gpio_reset_pin(MOTOR_CAN_RX_GPIO);
    s_initialized = false;
    s_transceiver_test_passed = false;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

/** @brief 返回 CAN 传输通道是否已经完成初始化。 */
bool CAN_ESP_IsInitialized(void)
{
    return s_initialized;
}

/**
 * @brief 复制最新 CAN 遥测，并根据时间戳计算链路在线状态。
 * @param[out] snapshot 接收状态的目标快照；传入 NULL 时忽略。
 */
void CAN_ESP_GetSnapshot(CAN_ESP_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    *snapshot = s_snapshot;
    /** 在临界区内复制的最后状态帧时间，离开锁后据此计算链路状态。 */
    const int64_t last_status_us = s_last_status_us;
    portEXIT_CRITICAL(&s_lock);

    snapshot->link_active =
        (last_status_us > 0) &&
        ((esp_timer_get_time() - last_status_us) <=
         (MOTOR_CAN_LINK_TIMEOUT_MS * 1000LL));
}

/**
 * @brief 授予或撤销 CAN 命令控制权。
 * @param enabled 选择 CAN 作为电机控制通道时传入 true，否则传入 false。
 */
void CAN_ESP_SetControlEnabled(bool enabled)
{
    portENTER_CRITICAL(&s_lock);
    s_control_enabled = enabled;
    if (!enabled) {
        s_speed_dirty = false;
        s_position_dirty = false;
    }
    portEXIT_CRITICAL(&s_lock);
    if (!enabled && s_control_queue != NULL) {
        xQueueReset(s_control_queue);
    }
}

/** @brief 排队一个 CAN 模式切换命令。 */
void CAN_ESP_SetMode(MotorCan_Mode_t mode)
{
    CAN_ESP_queue_control(MOTOR_CAN_CMD_SET_MODE, mode);
}

/** @brief 替换最新 CAN 速度目标，单位 rpm。 */
void CAN_ESP_SetSpeedRPM(int16_t speed_rpm)
{
    portENTER_CRITICAL(&s_lock);
    s_pending_speed_rpm = speed_rpm;
    s_speed_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将角度环绕到 0..35999 后替换最新 CAN 位置目标。
 * @param position_cdeg 请求角度，单位为 0.01°。
 */
void CAN_ESP_SetPositionCdeg(uint16_t position_cdeg)
{
    portENTER_CRITICAL(&s_lock);
    s_pending_position_cdeg = position_cdeg % 36000U;
    s_position_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

/** @brief 排队一个 CAN 电机启动命令。 */
void CAN_ESP_Start(void)
{
    CAN_ESP_queue_control(MOTOR_CAN_CMD_START, 0);
}

/** @brief 排队一个 CAN 电机停止命令。 */
void CAN_ESP_Stop(void)
{
    CAN_ESP_queue_control(MOTOR_CAN_CMD_STOP, 0);
}

/** @brief 排队一个 CAN 故障确认命令。 */
void CAN_ESP_AcknowledgeFault(void)
{
    CAN_ESP_queue_control(MOTOR_CAN_CMD_ACK_FAULT, 0);
}

/** @brief 排队一个 CAN 位置清零命令。 */
void CAN_ESP_ZeroPosition(void)
{
    CAN_ESP_queue_control(MOTOR_CAN_CMD_ZERO_POSITION, 0);
}
