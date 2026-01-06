#include "fdcan.h"
#include "stdint.h"
#include <stdint.h>

int can_init(void)
{
    // 4. 启动FDCAN并激活接收中断
    HAL_FDCAN_Start(&hfdcan1);
    // 激活FIFO0新消息中断
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    return 0;
}

int can_send(uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    HAL_StatusTypeDef ret;
    
    uint32_t send_len = len;
    uint8_t  current_len = 0;

    TxHeader.Identifier = id; // 标准ID
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
        TxHeader.DataLength = current_len; // 关键：数据长度设为8字节
        ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
        if (ret != HAL_OK) {
            // 处理发送失败
        }
        data += current_len;
        send_len -= current_len;
    } while (send_len > 0);
    
    return (int)len;
}

int can_recv(uint32_t id, uint8_t *data, uint32_t len)
{
    return -1;
}