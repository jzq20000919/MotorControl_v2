#include "usart_ESP.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MOTOR_UART_PORT                 UART_NUM_1
/* PORT2 专用于 UART 链路；PORT1（GPIO5/6）保留给 CAN 使用。 */
#define MOTOR_UART_TX_GPIO              GPIO_NUM_18
#define MOTOR_UART_RX_GPIO              GPIO_NUM_8
#define MOTOR_UART_RX_BUFFER_SIZE       (1024U)
#define MOTOR_UART_TX_BUFFER_SIZE       (1024U)
#define MOTOR_UART_CONTROL_QUEUE_SIZE   (8U)
#define MOTOR_UART_LINK_TIMEOUT_MS      (300U)
#define MOTOR_UART_TX_TASK_PERIOD_MS    (2U)
#define MOTOR_UART_PING_PERIOD_MS       (100U)

typedef struct
{
    MotorUart_Command_t command;
    int32_t value;
} USART_ESP_request_t;//串口命令结构体

typedef struct
{
    uint8_t buffer[MOTOR_UART_MAX_FRAME_SIZE];
    uint8_t length;
} USART_ESP_parser_t;

static const char *TAG = "MOTOR_UART";
static QueueHandle_t s_control_queue;//队列变量
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static USART_ESP_snapshot_t s_snapshot;//反馈快照信息，包括所有显示的状态和诊断信息
static USART_ESP_parser_t s_parser;
static uint8_t s_command_sequence;
static uint32_t s_last_telemetry_ms;
static int16_t s_latest_speed_rpm;
static uint16_t s_latest_position_cdeg;
static bool s_speed_dirty;
static bool s_position_dirty;
static bool s_control_enabled;
static uint32_t s_requested_baud_rate;
static bool s_reconnect_requested;
static bool s_force_ping;
static TaskHandle_t s_rx_task;
static TaskHandle_t s_tx_task;
static bool s_initialized;

/** @brief 返回单调递增的 ESP 定时器毫秒值。 */
static uint32_t USART_ESP_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief 计算协议使用的 CRC-16/Modbus 校验和。
 * @param data 参与校验的数据首地址。
 * @param length 参与校验的字节数。
 * @return 使用初值 0xFFFF 和多项式 0xA001 计算得到的 CRC。
 */
static uint16_t USART_ESP_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/**
 * @brief 在 UART RX 任务上下文中应用待执行的波特率变更。
 *
 * RX 任务独占解析器，因此在此处刷新输入和清除解析状态不会与字节解析竞争。
 * 更改硬件分频器期间，TX 任务通过 snapshot.reconnecting 暂停正常命令发送。
 */
static void USART_ESP_apply_reconnect(void)
{
    uint32_t baud_rate;

    portENTER_CRITICAL(&s_lock);
    if (!s_reconnect_requested) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    baud_rate = s_requested_baud_rate;
    s_reconnect_requested = false;
    portEXIT_CRITICAL(&s_lock);

    /*
     * RX 任务独占解析器，因此在此处复位解析器不会与字节解析竞争。
     * 更改 UART 分频器和缓冲区期间，snapshot.reconnecting 会暂停 TX 任务。
     */
    (void)uart_wait_tx_done(MOTOR_UART_PORT, pdMS_TO_TICKS(20));
    const esp_err_t result =
        uart_set_baudrate(MOTOR_UART_PORT, baud_rate);
    if (result == ESP_OK) {
        (void)uart_flush_input(MOTOR_UART_PORT);
        memset(&s_parser, 0, sizeof(s_parser));
        xQueueReset(s_control_queue);

        portENTER_CRITICAL(&s_lock);
        s_snapshot.link_active = false;
        s_snapshot.reconnecting = false;
        s_snapshot.baud_rate = baud_rate;
        s_snapshot.reconnect_count++;
        s_last_telemetry_ms = 0U;
        s_speed_dirty = false;
        s_position_dirty = false;
        s_force_ping = true;
        portEXIT_CRITICAL(&s_lock);

        ESP_LOGI(TAG, "UART reconnected at %lu baud", (unsigned long)baud_rate);
    } else {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.link_active = false;
        s_snapshot.reconnecting = false;
        s_snapshot.reconnect_errors++;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGE(TAG, "UART reconnect at %lu baud failed: %s", (unsigned long)baud_rate, esp_err_to_name(result));
    }
}

/**
 * @brief 编码并发送一帧完整的 UART 协议命令帧。
 *
 * 非诊断命令要求当前传输通道持有控制权。帧计数器在模块临界区内更新，
 * 实际物理发送由 UART 驱动自身的发送缓冲区异步完成。
 *
 * @param command motor_uart_protocol.h 中定义的命令操作码。
 * @param value 以小端序编码到负载中的有符号命令值。
 * @return UART 驱动接受完整帧时返回 ESP_OK，否则返回驱动错误码。
 */
static esp_err_t USART_ESP_transmit(
    MotorUart_Command_t command,
    int32_t value)
{
    if (command != MOTOR_UART_CMD_NOP &&
        command != MOTOR_UART_CMD_PING) {
        bool enabled;
        portENTER_CRITICAL(&s_lock);
        enabled = s_control_enabled;
        portEXIT_CRITICAL(&s_lock);
        if (!enabled) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    uint8_t frame[
        MOTOR_UART_FRAME_OVERHEAD +
        MOTOR_UART_COMMAND_PAYLOAD_SIZE] = {0};
    const uint8_t sequence = s_command_sequence++;
    uint16_t crc;
    int written;

    frame[0] = MOTOR_UART_SOF0;
    frame[1] = MOTOR_UART_SOF1;
    frame[2] = MOTOR_UART_PROTOCOL_VERSION;
    frame[3] = MOTOR_UART_FRAME_COMMAND;
    frame[4] = sequence;
    frame[5] = MOTOR_UART_COMMAND_PAYLOAD_SIZE;
    frame[6] = (uint8_t)command;
    MotorUart_WriteS32(&frame[7], value);//从第7字节开始填入4字节数值
    crc = USART_ESP_crc16(&frame[2], 4U + MOTOR_UART_COMMAND_PAYLOAD_SIZE);
    MotorUart_WriteU16(&frame[6U + MOTOR_UART_COMMAND_PAYLOAD_SIZE], crc);

    written = uart_write_bytes(MOTOR_UART_PORT, frame, sizeof(frame));
    if (written != (int)sizeof(frame)) {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.transmit_errors++;
        portEXIT_CRITICAL(&s_lock);
        return ESP_FAIL;
    }
    portENTER_CRITICAL(&s_lock);
    s_snapshot.transmitted_frames++;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

/**
 * @brief 将离散控制命令放入有长度限制的 TX 命令队列。
 *
 * 队列已满时丢弃最旧命令。对于 HMI 产生的目标请求，“最新请求优先”可避免
 * 新选择的模式被过时用户操作延迟。
 *
 * @param command 需要入队的命令操作码。
 * @param value 与命令关联的参数值。
 */
static void USART_ESP_queue_control(MotorUart_Command_t command,int32_t value)
{
    const USART_ESP_request_t request = 
    {
        .command = command,
        .value = value,
    };
    if (xQueueSend(s_control_queue, &request, 0) != pdTRUE)//如果满了则丢弃最旧命令
     {
        USART_ESP_request_t discarded;//定义一个USART_ESP_request_t类型的变量作为最旧命令
        (void)xQueueReceive(s_control_queue, &discarded, 0);//从队列取出一条命令放到最旧命令的地址。空出一个队列格子
        (void)xQueueSend(s_control_queue, &request, 0);//将最新的命令塞入队列
    }
}

/**
 * @brief 将已校验的遥测负载解码到共享 UART 状态快照。
 * @param payload UART 遥测帧头之后的负载数据。
 * @param length 负载长度；仅与协议规定遥测长度完全一致时才进行解析。
 */
static void USART_ESP_parse_telemetry(
    const uint8_t *payload,
    uint8_t length)
{
    if (length != MOTOR_UART_TELEMETRY_PAYLOAD_SIZE) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_snapshot.motor_running =
        (payload[0] & MOTOR_UART_STATUS_MOTOR_RUNNING) != 0U;
    s_snapshot.motor_fault =
        (payload[0] & MOTOR_UART_STATUS_MOTOR_FAULT) != 0U;
    s_snapshot.command_rejected =
        (payload[0] & MOTOR_UART_STATUS_COMMAND_REJECTED) != 0U;
    s_snapshot.link_active =
        (payload[0] & MOTOR_UART_STATUS_LINK_ACTIVE) != 0U;
    s_snapshot.mode =
        payload[1] == MOTOR_UART_MODE_POSITION
            ? MOTOR_UART_MODE_POSITION
            : MOTOR_UART_MODE_SPEED;
    s_snapshot.faults = MotorUart_ReadU16(&payload[2]);//取出电机故障位集合，16位无符号整数
    s_snapshot.measured_speed_rpm =MotorUart_ReadS16(&payload[4]);//从payload[4]和payload[5]中读取有符号16位整数，表示实测机械转速，单位rpm
    s_snapshot.reference_speed_rpm =MotorUart_ReadS16(&payload[6]);
    s_snapshot.current_position_cdeg =MotorUart_ReadU16(&payload[8]);
    s_snapshot.target_position_cdeg =MotorUart_ReadU16(&payload[10]);
    s_snapshot.position_error_cdeg =MotorUart_ReadS16(&payload[12]);
    s_snapshot.iq_ma = MotorUart_ReadS16(&payload[14]);
    s_snapshot.id_ma = MotorUart_ReadS16(&payload[16]);
    s_snapshot.iq_reference_ma =MotorUart_ReadS16(&payload[18]);
    s_snapshot.uq_mv = MotorUart_ReadS16(&payload[20]);
    s_snapshot.ud_mv = MotorUart_ReadS16(&payload[22]);
    s_snapshot.received_frames++;
    s_last_telemetry_ms = USART_ESP_now_ms();
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 将一个接收字节送入流式 UART 帧解析器。
 *
 * UART 读取操作不保证按帧对齐，因此该状态机负责查找双字节起始标记、校验
 * 长度和 CRC，并且只分发有效遥测。解析器状态由 RX 任务独占。
 *
 * @param byte 新接收到的串口字节。
 */
static void USART_ESP_parse_byte(uint8_t byte)
{
    uint8_t payload_length;
    uint16_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (s_parser.length == 0U) {
        if (byte == MOTOR_UART_SOF0) 
        {
            s_parser.buffer[s_parser.length++] = byte;//是0XA5则保留
        }
        return;//不是则返回
    }

    if (s_parser.length == 1U) {
        if (byte == MOTOR_UART_SOF1) 
        {
            s_parser.buffer[s_parser.length++] = byte;//是0X5A则保留
        } 
        else 
        {
            s_parser.length = byte == MOTOR_UART_SOF0 ? 1U : 0U;
        }
        return;
    }

    if (s_parser.length >= MOTOR_UART_MAX_FRAME_SIZE) {
        s_parser.length = 0U;
        return;
    }
    s_parser.buffer[s_parser.length++] = byte;

    if (s_parser.length < 6U) {
        return;
    }

    payload_length = s_parser.buffer[5];
    if (payload_length > MOTOR_UART_MAX_PAYLOAD) {
        s_parser.length = 0U;
        return;
    }

    expected_length = (uint16_t)(6U + payload_length + 2U);//完整帧长度
    if (s_parser.length < expected_length) {
        return;
    }
    //收集齐完整一帧后
    received_crc = MotorUart_ReadU16(&s_parser.buffer[6U + payload_length]);//从一帧中读取CRC
    calculated_crc = USART_ESP_crc16(&s_parser.buffer[2], (uint16_t)(4U + payload_length));//根据数据计算出CRC

    if (received_crc != calculated_crc) {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.crc_errors++;
        portEXIT_CRITICAL(&s_lock);
    } else if (
        s_parser.buffer[2] == MOTOR_UART_PROTOCOL_VERSION &&
        s_parser.buffer[3] == MOTOR_UART_FRAME_TELEMETRY) {
        USART_ESP_parse_telemetry(&s_parser.buffer[6], payload_length);//协议版本检查
    } else {
        portENTER_CRITICAL(&s_lock);
        s_snapshot.protocol_errors++;
        portEXIT_CRITICAL(&s_lock);
    }
    s_parser.length = 0U;
}

/**
 * @brief 接收 UART 字节、解析遥测并检测超时的 FreeRTOS 任务。
 * @param argument 未使用的任务参数。
 * @note 此任务独占 s_parser，并负责执行待处理的重连请求。
 */
static void USART_ESP_rx_task(void *argument)
{
    (void)argument;
    uint8_t received[96];//定义接收数组，96字节
    while (true) 
    {
        USART_ESP_apply_reconnect();
        const int count = uart_read_bytes(MOTOR_UART_PORT, received, sizeof(received), pdMS_TO_TICKS(10));//每次从接收缓冲区接收最多96字节数据
        if (count > 0)  
        {
            portENTER_CRITICAL(&s_lock);
            s_snapshot.received_bytes += (uint32_t)count;
            portEXIT_CRITICAL(&s_lock);
        }
        for (int i = 0; i < count; i++)
        {
            USART_ESP_parse_byte(received[i]);
        }
        portENTER_CRITICAL(&s_lock);
        if (s_snapshot.link_active &&(USART_ESP_now_ms() - s_last_telemetry_ms >
             MOTOR_UART_LINK_TIMEOUT_MS)) 
        {
            s_snapshot.link_active = false;
        }
        portEXIT_CRITICAL(&s_lock);
    }
}

/**
 * @brief 周期发送排队命令、最新目标和 Ping 的 FreeRTOS 任务。
 *
 * 每个周期先发送离散请求，再分别最多发送一个最新位置和速度目标。固定的
 * vTaskDelayUntil 调度可避免忙循环，并使链路命令发送节奏保持可预测。
 *
 * @param argument 未使用的任务参数。
 */
static void USART_ESP_tx_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();//记录任务周期时间基准
    uint32_t next_ping_ms = USART_ESP_now_ms();//下一次

    while (true) {
        USART_ESP_request_t request;
        uint8_t budget = 4U;
        bool reconnecting;
        bool control_enabled;
        portENTER_CRITICAL(&s_lock);
        reconnecting = s_snapshot.reconnecting;
        control_enabled = s_control_enabled;//允许UART控制
        portEXIT_CRITICAL(&s_lock);
        if (reconnecting) 
        {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_UART_TX_TASK_PERIOD_MS));
            continue;
        }//如果UART正在重连则延迟2ms并跳过本次循环

        while (control_enabled && budget-- > 0U &&
               xQueueReceive(s_control_queue, &request, 0) == pdTRUE) {
            (void)USART_ESP_transmit(request.command, request.value);
        }

        bool send_position;
        bool send_speed;
        uint16_t position;
        int16_t speed;
        portENTER_CRITICAL(&s_lock);
        send_position = control_enabled && s_position_dirty;
        send_speed = control_enabled && s_speed_dirty;
        position = s_latest_position_cdeg;
        speed = s_latest_speed_rpm;
        s_position_dirty = false;
        s_speed_dirty = false;
        portEXIT_CRITICAL(&s_lock);

        if (send_position) {
            (void)USART_ESP_transmit(MOTOR_UART_CMD_SET_POSITION_CDEG, position);
        }
        if (send_speed) {
            (void)USART_ESP_transmit(MOTOR_UART_CMD_SET_SPEED_RPM, speed);
        }

        const uint32_t now = USART_ESP_now_ms();
        bool force_ping;
        portENTER_CRITICAL(&s_lock);
        force_ping = s_force_ping;
        s_force_ping = false;
        portEXIT_CRITICAL(&s_lock);
        if (force_ping || (int32_t)(now - next_ping_ms) >= 0) {
            next_ping_ms = now + MOTOR_UART_PING_PERIOD_MS;
            (void)USART_ESP_transmit(MOTOR_UART_CMD_PING, 0);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_UART_TX_TASK_PERIOD_MS));
    }
}

/**
 * @brief 安装 UART1 驱动并启动协议 RX/TX 任务。
 * @return 驱动、队列及两个任务均就绪时返回 ESP_OK；所需 FreeRTOS 对象
 *         创建失败时返回 ESP_ERR_NO_MEM。
 */
esp_err_t USART_ESP_Init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    //配置结构体
    const uart_config_t config = {
        .baud_rate = MOTOR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_control_enabled = false;
    s_snapshot.mode = MOTOR_UART_MODE_SPEED;
    s_snapshot.baud_rate = MOTOR_UART_BAUD_RATE;
    s_control_queue = xQueueCreate(MOTOR_UART_CONTROL_QUEUE_SIZE, sizeof(USART_ESP_request_t));//创建一个rtos队列，队列大小为8，队列中每个元素的大小为USART_ESP_request_t结构体的大小
    if (s_control_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(uart_driver_install( MOTOR_UART_PORT, MOTOR_UART_RX_BUFFER_SIZE, MOTOR_UART_TX_BUFFER_SIZE, 0, NULL, 0), TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(MOTOR_UART_PORT, &config), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin( MOTOR_UART_PORT, MOTOR_UART_TX_GPIO, MOTOR_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin failed");
    ESP_RETURN_ON_ERROR(uart_flush_input(MOTOR_UART_PORT), TAG, "uart_flush_input failed");

    if (xTaskCreate(USART_ESP_rx_task, "USART_ESP_rx", 3072, NULL, 8, &s_rx_task) != pdPASS 
        ||xTaskCreate(USART_ESP_tx_task, "USART_ESP_tx", 3072, NULL, 8, &s_tx_task) != pdPASS) 
    {
        if (s_rx_task != NULL) 
        {
            vTaskDelete(s_rx_task);
            s_rx_task = NULL;
        }
        if (s_tx_task != NULL) {
            vTaskDelete(s_tx_task);
            s_tx_task = NULL;
        }
        (void)uart_driver_delete(MOTOR_UART_PORT);
        vQueueDelete(s_control_queue);
        s_control_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;

    ESP_LOGI(TAG, "USART link ready: TX GPIO%d, RX GPIO%d, %lu baud", MOTOR_UART_TX_GPIO, MOTOR_UART_RX_GPIO, (unsigned long)MOTOR_UART_BAUD_RATE);
    return ESP_OK;
}

/**
 * @brief 停止 UART 任务并释放 UART 独占资源。
 * @note 仅应在上层撤销 UART 控制权后调用。
 */
void USART_ESP_Deinit(void)
{
    if (!s_initialized) {
        return;
    }

    if (s_rx_task != NULL) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_tx_task != NULL) {
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }
    (void)uart_driver_delete(MOTOR_UART_PORT);
    if (s_control_queue != NULL) {
        vQueueDelete(s_control_queue);
        s_control_queue = NULL;
    }
    gpio_reset_pin(MOTOR_UART_TX_GPIO);
    gpio_reset_pin(MOTOR_UART_RX_GPIO);
    s_initialized = false;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

/** @brief 返回 UART 驱动、队列和工作任务是否已启动。 */
bool USART_ESP_IsInitialized(void)
{
    return s_initialized;
}

/**
 * @brief 标记一个由 RX 任务执行的 UART 波特率更新请求。
 * @param baud_rate 新的非零 UART 波特率。
 * @return 波特率为零时返回 ESP_ERR_INVALID_ARG；成功记录请求时返回 ESP_OK。
 */
esp_err_t USART_ESP_RequestReconnect(uint32_t baud_rate)
{
    if (baud_rate == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    s_requested_baud_rate = baud_rate;
    s_reconnect_requested = true;
    s_snapshot.link_active = false;
    s_snapshot.reconnecting = true;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

/**
 * @brief 为 UI/网络读者复制最新遥测和诊断信息。
 * @param[out] snapshot 接收状态的目标对象；传入 NULL 时忽略。
 */
void USART_ESP_GetSnapshot(USART_ESP_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *snapshot = s_snapshot;
    portEXIT_CRITICAL(&s_lock);
}

/**
 * @brief 授予或撤销 UART 链路发送电机命令的控制权。
 *
 * 撤销控制权时清除尚未发送的连续目标和控制请求，防止 CAN 成为活动通道后，
 * 旧的 UART 选择仍继续影响电机。
 *
 * @param enabled UART 获得控制权时传入 true，撤销时传入 false。
 */
void USART_ESP_SetControlEnabled(bool enabled)
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

/** @brief 为 UART TX 排队一个离散模式切换命令。 */
void USART_ESP_SetMode(MotorUart_Mode_t mode)
{
    USART_ESP_queue_control(MOTOR_UART_CMD_SET_MODE, mode);
}

/** @brief 替换待发送的 UART 速度目标，单位 rpm。 */
void USART_ESP_SetSpeedRPM(int16_t speed_rpm)
{
    portENTER_CRITICAL(&s_lock);
    s_latest_speed_rpm = speed_rpm;
    s_speed_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

/** @brief 替换待发送的 UART 位置目标，单位 0.01°。 */
void USART_ESP_SetPositionCdeg(uint16_t position_cdeg)
{
    portENTER_CRITICAL(&s_lock);
    s_latest_position_cdeg = position_cdeg;
    s_position_dirty = true;
    portEXIT_CRITICAL(&s_lock);
}

/** @brief 排队一个 UART 电机启动命令。 */
void USART_ESP_Start(void)
{
    USART_ESP_queue_control(MOTOR_UART_CMD_START, 0);
}

/** @brief 排队一个 UART 电机停止命令。 */
void USART_ESP_Stop(void)
{
    USART_ESP_queue_control(MOTOR_UART_CMD_STOP, 0);
}

/** @brief 排队一个 UART 故障确认命令。 */
void USART_ESP_AcknowledgeFault(void)
{
    USART_ESP_queue_control(MOTOR_UART_CMD_ACK_FAULT, 0);
}

/** @brief 排队一个 UART 位置清零命令。 */
void USART_ESP_ZeroPosition(void)
{
    USART_ESP_queue_control(MOTOR_UART_CMD_ZERO_POSITION, 0);
}
