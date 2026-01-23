
#include <stdint.h>
#include <sys/types.h>
#include "can_data.h"
void can_data_conv( uint16_t ch, 
                    uint16_t disp,
                    uint16_t moment,
                    uint16_t stress,
                    can_data_t *can_data)
{
    can_data->ch_and_id = ch; 
    can_data->displacement = disp;
    can_data->moment = moment;
    can_data->stress = stress;
}
