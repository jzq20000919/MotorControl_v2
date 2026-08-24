#include "comm_mgr_ESP.h"

#include <string.h>

#include "can_ESP.h"
#include "usart_ESP.h"

static CommMgr_ESP_transport_t s_transport;

/**
 * @brief 初始化传输通道仲裁层。
 *
 * 本函数不打开 UART 或 CAN 硬件，仅清除当前控制通道。因此在成功连接前
 * 调用控制命令不会对电机产生影响。
 */
void CommMgr_ESP_Init(void)
{
    s_transport = COMM_MGR_ESP_NONE;
}

/**
 * @brief 选择 UART 作为唯一的电机控制通道，并请求重新连接。
 *
 * 首次使用时初始化 UART。波特率请求成功后，启用 UART 命令输出并关闭 CAN
 * 命令输出；若请求失败，则保留原来的活动通道。
 *
 * @param baud_rate 两端必须保持一致的 UART 波特率。
 * @return 成功返回 ESP_OK，否则返回 UART 层的 ESP-IDF 错误码。
 */
esp_err_t CommMgr_ESP_SelectUSART(uint32_t baud_rate)
{
    const CommMgr_ESP_transport_t previous = s_transport;
    if (!USART_ESP_IsInitialized()) {
        const esp_err_t result = USART_ESP_Init();
        if (result != ESP_OK) {
            return result;
        }
    }
    const esp_err_t result = USART_ESP_RequestReconnect(baud_rate);
    s_transport = result == ESP_OK ? COMM_MGR_ESP_USART : previous;
    if (result == ESP_OK) {
        USART_ESP_SetControlEnabled(true);
        CAN_ESP_SetControlEnabled(false);
    }
    return result;
}

/**
 * @brief 选择 CAN 作为唯一的电机控制通道。
 *
 * CAN 硬件采用按需初始化。初始化成功后启用 CAN 命令输出并关闭 UART 命令
 * 输出，防止两个接口同时下发冲突的电机目标值。
 *
 * @return 成功返回 ESP_OK，否则返回 CAN 初始化错误码。
 */
esp_err_t CommMgr_ESP_SelectCAN(void)
{
    const CommMgr_ESP_transport_t previous = s_transport;
    if (!CAN_ESP_IsInitialized()) {
        const esp_err_t result = CAN_ESP_Init();
        if (result != ESP_OK) {
            s_transport = previous;
            return result;
        }
    }
    s_transport = COMM_MGR_ESP_CAN;
    CAN_ESP_SetControlEnabled(true);
    USART_ESP_SetControlEnabled(false);
    return ESP_OK;
}

/**
 * @brief 关闭并释放 UART 电机控制通道。
 *
 * 如果 UART 是当前活动通道，统一链路切换为 COMM_MGR_ESP_NONE；已选择的 CAN
 * 通道不会被误操作。
 */
void CommMgr_ESP_DisconnectUSART(void)
{
    USART_ESP_SetControlEnabled(false);
    if (USART_ESP_IsInitialized()) {
        USART_ESP_Deinit();
    }
    if (s_transport == COMM_MGR_ESP_USART) {
        s_transport = COMM_MGR_ESP_NONE;
    }
}

/**
 * @brief 关闭并释放 CAN 电机控制通道。
 *
 * 如果 CAN 是当前活动通道，统一链路切换为 COMM_MGR_ESP_NONE；已选择的 UART
 * 通道不会被误操作。
 */
void CommMgr_ESP_DisconnectCAN(void)
{
    CAN_ESP_SetControlEnabled(false);
    if (CAN_ESP_IsInitialized()) {
        CAN_ESP_Deinit();
    }
    if (s_transport == COMM_MGR_ESP_CAN) {
        s_transport = COMM_MGR_ESP_NONE;
    }
}

/**
 * @brief 构造供 UI 与网络层使用的、与传输方式无关的状态快照。
 *
 * 函数将最新 UART 或 CAN 遥测复制到统一的 CommMgr_ESP_State。它不会等待
 * 新报文，因此适合 UI 周期刷新；协议中不存在的字段保持为零，例如 UART v1
 * 中没有 Id 电流参考值。
 *
 * @param[out] snapshot 目标快照；传入 NULL 时直接忽略。
 */
void CommMgr_ESP_GetState(CommMgr_ESP_State *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->transport = s_transport;
    snapshot->uart_connected = USART_ESP_IsInitialized();
    snapshot->can_connected = CAN_ESP_IsInitialized();

    USART_ESP_snapshot_t uart;
    memset(&uart, 0, sizeof(uart));
    if (snapshot->uart_connected) {
        USART_ESP_GetSnapshot(&uart);
        snapshot->uart_link_active = uart.link_active;
    }

    CAN_ESP_snapshot_t can;
    memset(&can, 0, sizeof(can));
    if (snapshot->can_connected) {
        CAN_ESP_GetSnapshot(&can);
        snapshot->can_link_active = can.link_active;
    }

    if (s_transport == COMM_MGR_ESP_USART && snapshot->uart_connected) {
        snapshot->link_active = uart.link_active;
        snapshot->reconnecting = uart.reconnecting;
        snapshot->motor_running = uart.motor_running;
        snapshot->motor_fault = uart.motor_fault;
        snapshot->command_rejected = uart.command_rejected;
        snapshot->mode = (CommMgr_ESP_mode_t)uart.mode;
        snapshot->faults = uart.faults;
        snapshot->measured_speed_rpm = uart.measured_speed_rpm;
        snapshot->reference_speed_rpm = uart.reference_speed_rpm;
        snapshot->current_position_cdeg = uart.current_position_cdeg;
        snapshot->target_position_cdeg = uart.target_position_cdeg;
        snapshot->position_error_cdeg = uart.position_error_cdeg;
        snapshot->iq_ma = uart.iq_ma;
        snapshot->id_ma = uart.id_ma;
        snapshot->iq_reference_ma = uart.iq_reference_ma;
        /* UART v1 遥测协议不包含 Id 参考值字段。 */
        snapshot->id_reference_ma = 0;
        snapshot->uq_mv = uart.uq_mv;
        snapshot->ud_mv = uart.ud_mv;
        snapshot->received_frames = uart.received_frames;
        snapshot->transmitted_frames = uart.transmitted_frames;
        snapshot->transmit_errors = uart.transmit_errors;
        snapshot->baud_rate = uart.baud_rate;
    } else if (s_transport == COMM_MGR_ESP_CAN && snapshot->can_connected) {
        snapshot->link_active = can.link_active;
        snapshot->bus_off = can.bus_off;
        snapshot->transceiver_fault = can.transceiver_fault;
        snapshot->motor_running = can.motor_running;
        snapshot->motor_fault = can.motor_fault;
        snapshot->command_rejected = can.command_rejected;
        snapshot->mode = (CommMgr_ESP_mode_t)can.mode;
        snapshot->faults = can.faults;
        snapshot->measured_speed_rpm = can.measured_speed_rpm;
        snapshot->reference_speed_rpm = can.reference_speed_rpm;
        snapshot->current_position_cdeg = can.current_position_cdeg;
        snapshot->target_position_cdeg = can.target_position_cdeg;
        snapshot->position_error_cdeg = can.position_error_cdeg;
        snapshot->iq_ma = can.iq_ma;
        snapshot->id_ma = can.id_ma;
        snapshot->iq_reference_ma = can.iq_reference_ma;
        snapshot->id_reference_ma = can.id_reference_ma;
        snapshot->received_frames = can.received_frames;
        snapshot->transmitted_frames = can.transmitted_frames;
        snapshot->transmit_errors = can.transmit_errors;
    }
}

/**
 * @brief 在当前选择的通道上排队发送控制模式请求。
 * @param mode 请求的统一电机模式。
 * @note 没有通道拥有控制权时，该调用会被忽略。
 */
void CommMgr_ESP_SetMode(CommMgr_ESP_mode_t mode)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_SetMode((MotorUart_Mode_t)mode);
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_SetMode((MotorCan_Mode_t)mode);
}

/**
 * @brief 在当前选择的通道上排队发送速度参考值。
 * @param speed_rpm 请求的机械转速，单位 rpm。
 */
void CommMgr_ESP_SetSpeedRPM(int16_t speed_rpm)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_SetSpeedRPM(speed_rpm);
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_SetSpeedRPM(speed_rpm);
}

/**
 * @brief 在当前选择的通道上排队发送位置参考值。
 * @param position_cdeg 请求的位置，单位 0.01°。
 */
void CommMgr_ESP_SetPositionCdeg(uint16_t position_cdeg)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_SetPositionCdeg(position_cdeg);
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_SetPositionCdeg(position_cdeg);
}

/** @brief 在当前选择的通道上排队发送电机启动命令。 */
void CommMgr_ESP_Start(void)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_Start();
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_Start();
}

/** @brief 在当前选择的通道上排队发送电机停止命令。 */
void CommMgr_ESP_Stop(void)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_Stop();
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_Stop();
}

/** @brief 排队发送当前电机故障确认/复位请求。 */
void CommMgr_ESP_AcknowledgeFault(void)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_AcknowledgeFault();
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_AcknowledgeFault();
}

/** @brief 排队发送将当前机械位置设为零点的请求。 */
void CommMgr_ESP_ZeroPosition(void)
{
    if (s_transport == COMM_MGR_ESP_USART) USART_ESP_ZeroPosition();
    if (s_transport == COMM_MGR_ESP_CAN) CAN_ESP_ZeroPosition();
}
