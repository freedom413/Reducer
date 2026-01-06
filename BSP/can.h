#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>


int can_init(void);
int can_send(uint32_t id, uint8_t *data, uint32_t len);
int can_recv(uint32_t id, uint8_t *data, uint32_t len);

#endif /* __CAN_H__ */
