#include "fdcan.h"
#include "stdint.h"
#include <stdint.h>

int can_init(void)
{
    // 4. 启动FDCAN并激活接收中断
    HAL_FDCAN_Start(&hfdcan1);
    // 激活FIFO0新消息中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
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
        switch (len) {
            case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
            case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
            case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
            case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
            case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
            case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
            case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
            case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
            default: return 0;  // 长度错误
        }
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[64];  // 最大64字节
  
  // 检查中断类型
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    // 从FIFO0读取消息
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      // 处理接收到的消息
    //   ProcessCANMessage(&RxHeader, RxData);
    }
  }
  
  // 检查FIFO满中断
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != RESET)
  {
    // // FIFO0已满，需要尽快处理数据
    // printf("FDCAN FIFO0 Full!\n");
    
    // // 可以一次性读取所有消息
    // while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    // {
    //   if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    //   {
    //     ProcessCANMessage(&RxHeader, RxData);
    //   }
    // }
  }
  
  // 检查消息丢失中断
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != RESET)
  {
    // printf("FDCAN FIFO0 Message Lost!\n");
    // 可以记录错误计数或采取恢复措施
  }
}