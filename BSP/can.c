#include <stdint.h>
#include "can.h"
#include "can_data.h"
#include "lwrb.h"

#define CAN_TX_FIFO_QUEUE_ELEMENT_COUNT 3U

static lwrb_t can_recv_ring;
static char can_recv_msg_buf[CAN_MSG_BUFF_SIZE];
static volatile uint32_t can_rx_isr_count = 0U;
static volatile uint32_t can_rx_ring_drop_count = 0U;
static volatile uint32_t can_tx_fail_count = 0U;
static volatile uint8_t can_last_rx_dlc = 0U;
static volatile uint8_t can_last_rx_fd = 0U;
static volatile uint8_t can_last_rx_brs = 0U;
static volatile uint8_t can_last_reject_reason = CAN_DIAG_REJECT_NONE;
volatile uint32_t can_debug_txfqs_raw = 0U;
volatile uint32_t can_debug_txbrp_raw = 0U;
volatile uint32_t can_debug_psr_raw = 0U;
volatile uint32_t can_debug_ecr_raw = 0U;
volatile uint32_t can_debug_cccr_raw = 0U;
volatile uint32_t can_debug_ir_raw = 0U;
volatile uint32_t can_debug_tx_fifo_free_level = 0U;
volatile int can_debug_last_send_return = 0;

static uint32_t can_tx_pending_count_from_txbrp(uint32_t txbrp)
{
    uint32_t pending = txbrp & FDCAN_TXBRP_TRP;
    uint32_t count = 0U;

    for (uint32_t bit = 1U; bit <= FDCAN_TXBRP_TRP; bit <<= 1U) {
        if ((pending & bit) != 0U) {
            count++;
        }
    }
    return count;
}

static uint32_t can_tx_effective_free_level(void)
{
    uint32_t txfqs = hfdcan1.Instance->TXFQS;
    uint32_t free_level = txfqs & FDCAN_TXFQS_TFFL;

    if (free_level != 0U || (txfqs & FDCAN_TXFQS_TFQF) != 0U) {
        return free_level;
    }

    uint32_t pending_count =
        can_tx_pending_count_from_txbrp(hfdcan1.Instance->TXBRP);
    return pending_count >= CAN_TX_FIFO_QUEUE_ELEMENT_COUNT ?
        0U : CAN_TX_FIFO_QUEUE_ELEMENT_COUNT - pending_count;
}

static void can_debug_capture(int last_send_return)
{
    can_debug_txfqs_raw = hfdcan1.Instance->TXFQS;
    can_debug_txbrp_raw = hfdcan1.Instance->TXBRP;
    can_debug_psr_raw = hfdcan1.Instance->PSR;
    can_debug_ecr_raw = hfdcan1.Instance->ECR;
    can_debug_cccr_raw = hfdcan1.Instance->CCCR;
    can_debug_ir_raw = hfdcan1.Instance->IR;
    can_debug_tx_fifo_free_level = can_tx_effective_free_level();
    can_debug_last_send_return = last_send_return;
}

static HAL_StatusTypeDef can_activate_notifications(void)
{
    return HAL_FDCAN_ActivateNotification(&hfdcan1,
                                          FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                          FDCAN_IT_LIST_PROTOCOL_ERROR |
                                          FDCAN_IT_DATA_PROTOCOL_ERROR |
                                          FDCAN_IT_BUS_OFF |
                                          FDCAN_IT_ERROR_PASSIVE,
                                          0);
}

static uint32_t can_dlc_from_len(uint32_t len)
{
    switch (len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return UINT32_MAX;
    }
}

int can_init(void)
{
    lwrb_init(&can_recv_ring, can_recv_msg_buf, CAN_MSG_BUFF_SIZE);
    can_rx_isr_count = 0U;
    can_rx_ring_drop_count = 0U;
    can_tx_fail_count = 0U;
    can_last_rx_dlc = 0U;
    can_last_rx_fd = 0U;
    can_last_rx_brs = 0U;
    can_last_reject_reason = CAN_DIAG_REJECT_NONE;
    can_debug_capture(0);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        can_debug_capture(-1);
        return -1;
    }

    if (can_activate_notifications() != HAL_OK) {
        can_debug_capture(-2);
        return -2;
    }
    can_debug_capture(0);

    return 0;
}

int can_data_len_get(uint32_t frame_len)
{
    switch (frame_len) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return -1;
    }
}

static uint8_t can_u8_saturate(uint32_t value)
{
    return value > UINT8_MAX ? UINT8_MAX : (uint8_t)value;
}

static int can_data_frame_send_reserved(uint32_t id, const uint8_t *data,
                                        uint32_t len,
                                        uint32_t reserved_free_level,
                                        uint32_t fd_format,
                                        uint32_t bit_rate_switch)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint32_t dlc = can_dlc_from_len(len);

    if (len > CAN_DATA_LEN || dlc == UINT32_MAX) {
        can_debug_capture(-1);
        return -1;
    }
    if (data == NULL && len > 0U) {
        can_debug_capture(-2);
        return -2;
    }
    if (fd_format == FDCAN_CLASSIC_CAN && len > 8U) {
        can_debug_capture(-1);
        return -1;
    }

    tx_header.Identifier = id & 0x7FFu;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = bit_rate_switch;
    tx_header.FDFormat = fd_format;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (can_tx_effective_free_level() <= reserved_free_level) {
        if (can_recover_bus_off() <= 0 ||
            can_tx_effective_free_level() <= reserved_free_level) {
            can_tx_fail_count++;
            can_debug_capture(-3);
            return -3;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) != HAL_OK) {
        can_tx_fail_count++;
        can_debug_capture(-4);
        return -4;
    }

    can_debug_capture((int)len);
    return (int)len;
}

int can_fd_data_frame_send(uint32_t id, const uint8_t *data, uint32_t len)
{
    return can_data_frame_send_reserved(id, data, len, 0U,
                                        FDCAN_FD_CAN, FDCAN_BRS_ON);
}

int can_fd_data_frame_send_low_priority(uint32_t id, const uint8_t *data,
                                        uint32_t len)
{
    return can_data_frame_send_reserved(id, data, len, 1U,
                                        FDCAN_FD_CAN, FDCAN_BRS_ON);
}

int can_classic_data_frame_send(uint32_t id, const uint8_t *data, uint32_t len)
{
    return can_data_frame_send_reserved(id, data, len, 0U,
                                        FDCAN_CLASSIC_CAN, FDCAN_BRS_OFF);
}

int can_recover_bus_off(void)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) != HAL_OK ||
        protocol.BusOff == 0U) {
        return 0;
    }

    (void)HAL_FDCAN_AbortTxRequest(
        &hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
    if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
        can_tx_fail_count++;
        return -1;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        can_tx_fail_count++;
        return -2;
    }
    if (can_activate_notifications() != HAL_OK) {
        can_tx_fail_count++;
        return -3;
    }
    return 1;
}

void can_diag_record_reject(uint8_t reason)
{
    can_last_reject_reason = reason;
}

void can_diag_get(can_diag_status_t *status)
{
    if (status == NULL) {
        return;
    }

    status->rx_isr_count = can_rx_isr_count;
    status->rx_ring_drop_count = can_rx_ring_drop_count;
    status->tx_fail_count = can_tx_fail_count;
    status->last_rx_dlc = can_last_rx_dlc;
    status->last_rx_fd = can_last_rx_fd;
    status->last_rx_brs = can_last_rx_brs;
    status->last_reject_reason = can_last_reject_reason;
    status->bus_off = 0U;
    status->error_passive = 0U;
    status->error_warning = 0U;
    status->tx_error_count = 0U;
    status->rx_error_count = 0U;

    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef counters = {0};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) == HAL_OK) {
        status->bus_off = protocol.BusOff != 0U ? 1U : 0U;
        status->error_passive = protocol.ErrorPassive != 0U ? 1U : 0U;
        status->error_warning = protocol.Warning != 0U ? 1U : 0U;
    }
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &counters) == HAL_OK) {
        status->tx_error_count = can_u8_saturate(counters.TxErrorCnt);
        status->rx_error_count = can_u8_saturate(counters.RxErrorCnt);
    }
}

static void can_recv_push(const can_msg_t *msg)
{
    const uint32_t msg_size = sizeof(*msg);

    if (lwrb_get_free(&can_recv_ring) < msg_size) {
        /*
         * The ISR owns writes and the main loop owns reads. Do not advance the
         * read pointer here: doing so would introduce a second reader and race
         * with can_recv(). Commands are rare, so dropping a new frame on
         * overflow is the predictable failure mode.
         */
        can_rx_ring_drop_count++;
        can_diag_record_reject(CAN_DIAG_REJECT_RX_OVERFLOW);
        return;
    }
    (void)lwrb_write(&can_recv_ring, msg, msg_size);
}

int can_recv(can_msg_t *msg, uint32_t max_count)
{
    if (msg == NULL || max_count == 0U) {
        return 0;
    }

    uint32_t bytes = lwrb_read(&can_recv_ring,
                               (char *)msg,
                               max_count * sizeof(can_msg_t));
    return (int)(bytes / sizeof(can_msg_t));
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static can_msg_t can_recv_msg;

    if (hfdcan->Instance != FDCAN1 ||
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &can_recv_msg.RxHeader,
                                   can_recv_msg.data) != HAL_OK) {
            break;
        }
        can_rx_isr_count++;
        int dlc_bytes = can_data_len_get(can_recv_msg.RxHeader.DataLength);
        can_last_rx_dlc = dlc_bytes < 0 ? 0U : (uint8_t)dlc_bytes;
        can_last_rx_fd =
            can_recv_msg.RxHeader.FDFormat == FDCAN_FD_CAN ? 1U : 0U;
        can_last_rx_brs =
            can_recv_msg.RxHeader.BitRateSwitch == FDCAN_BRS_ON ? 1U : 0U;
        can_recv_push(&can_recv_msg);
    }
}
