#include <stdint.h>
#include "can.h"
#include "lwrb.h"

static lwrb_t can_recv_ring;
static char can_recv_msg_buf[CAN_MSG_BUFF_SIZE];

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
        default: return UINT32_MAX;
    }
}

int can_init(void)
{
    lwrb_init(&can_recv_ring, can_recv_msg_buf, CAN_MSG_BUFF_SIZE);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return -1;
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return -2;
    }

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

int can_classic_data_frame_send(uint32_t id, const uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint32_t dlc = can_dlc_from_len(len);

    if (len > CAN_DATA_LEN || dlc == UINT32_MAX) {
        return -1;
    }
    if (data == NULL && len > 0U) {
        return -2;
    }

    tx_header.Identifier = id & 0x7FFu;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        return -3;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) != HAL_OK) {
        return -4;
    }

    return (int)len;
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
        can_recv_push(&can_recv_msg);
    }
}
