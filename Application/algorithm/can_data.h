#ifndef __CAN_DATA_H__
#define __CAN_DATA_H__

#include <stdint.h>

typedef struct can_data {
    uint16_t ch_and_id;
    int16_t displacement;
    int16_t moment;
    int16_t stress;
} can_data_t;

void can_data_conv( uint16_t ch, 
                    uint16_t disp,
                    uint16_t moment,
                    uint16_t stress,
                    can_data_t *can_data);
#endif // __CAN_DATA_H__
