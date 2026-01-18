
#include "stdint.h"
#include <stdint.h>
#include "can.h"
#include "lwrb.h"

static lwrb_t can_recv_ring;
static char can_recv_msg_buf[CAN_MSG_BUFF_SIZE];

int can_init(void)
{
    // 初始化接收消息缓冲区
    lwrb_init(&can_recv_ring, can_recv_msg_buf, CAN_MSG_BUFF_SIZE);
    // 4. 启动FDCAN并激活接收中断
    HAL_FDCAN_Start(&hfdcan1);
    // 激活FIFO0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return 0;
}


int can_data_len_get(uint32_t frame_len)
{
    if(frame_len <= 8 && frame_len >= FDCAN_DLC_BYTES_0) {
        return frame_len;
    }
    else {
        switch (frame_len) {
            case FDCAN_DLC_BYTES_12: return 12; break;
            case FDCAN_DLC_BYTES_16: return 16; break;
            case FDCAN_DLC_BYTES_20: return 20; break;
            case FDCAN_DLC_BYTES_24: return 24; break;
            case FDCAN_DLC_BYTES_32: return 32; break;
            case FDCAN_DLC_BYTES_48: return 48; break;
            case FDCAN_DLC_BYTES_64: return 64; break;
            default: return -1;  // 长度错误
        }
    }
}


int can_classic_data_frame_send(uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    HAL_StatusTypeDef ret;
    uint32_t send_len = len;
    uint8_t  current_len = 0;

    if (data == NULL) {
        return -2;
    }

    TxHeader.Identifier = id % 0x7FFu; // 标准ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
    // 关键：以下三个标志必须正确设置，以符合经典CAN格式
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE; // 对于正常节点，ESI通常处于PASSIVE
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 关键：关闭比特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; // 关键：帧格式为经典CAN
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不产生发送事件
    TxHeader.MessageMarker = 0; // 用户自定义标识，可用于回调函数区分消息

    do {
        current_len = send_len >= 8 ? 8 : send_len;
        TxHeader.DataLength = current_len;
        ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
        if (ret != HAL_OK) {
            return -3;  // 发送失败
        }
        data += current_len;
        send_len -= current_len;
    } while (send_len > 0);
    
    return (int)len;
}

int can_recv(can_msg_t *msg, uint32_t count)
{
    return lwrb_read(&can_recv_ring, (char *)msg, count * sizeof(can_msg_t));
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{  
    static can_msg_t can_recv_msg;
    // 检查中断类型
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
      // 从FIFO0读取消息
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, 
        &can_recv_msg.RxHeader, can_recv_msg.data) == HAL_OK){
        // 处理接收到的消息
        lwrb_write(&can_recv_ring, (char *)&can_recv_msg, sizeof(can_recv_msg));
      }
      else {
        // 处理接收错误
        return;
      }
    }
    
    // 检查FIFO满中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != RESET) {

    }
    
    // 检查消息丢失中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != RESET) {

    }
}