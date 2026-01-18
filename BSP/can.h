#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>
#include "fdcan.h"

typedef struct can_msg can_msg_t;

#define CAN_DATA_LEN         (8)     /** @brief CAN数据长度 class can = 8 ，fd can = 64 */
#define CAN_MSG_BUFF_SIZE    (sizeof(can_msg_t) * 64) /** @brief CAN接受消息环形缓冲区长度 */

typedef struct can_msg {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t data[CAN_DATA_LEN];
} can_msg_t;


int can_init(void);
int can_data_len_get(uint32_t frame_len);
int can_classic_data_frame_send(uint32_t id, uint8_t *data, uint32_t len);
int can_recv(can_msg_t *msg, uint32_t count);

#endif /* __CAN_H__ */
