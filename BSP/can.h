#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>
#include "fdcan.h"

typedef struct can_msg can_msg_t;

#define CAN_FD_MAX_DATA_LEN  (64U)
#define CAN_DATA_LEN         CAN_FD_MAX_DATA_LEN
#define CAN_MSG_BUFF_COUNT   (64U)
#define CAN_MSG_BUFF_SIZE    ((sizeof(can_msg_t) * CAN_MSG_BUFF_COUNT) + 1U)

typedef struct can_msg {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t data[CAN_DATA_LEN];
} can_msg_t;

int can_init(void);
int can_data_len_get(uint32_t frame_len);
int can_fd_data_frame_send(uint32_t id, const uint8_t *data, uint32_t len);
int can_fd_data_frame_send_low_priority(uint32_t id, const uint8_t *data,
                                        uint32_t len);
int can_recv(can_msg_t *msg, uint32_t max_count);

#endif /* __CAN_H__ */
