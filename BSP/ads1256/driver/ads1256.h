#ifndef __ADS1256_H__
#define __ADS1256_H__
#include "stdbool.h"
#include "stdint.h"

#define  FCLK  7680000UL  // 7.68MHz crystal oscillator frequency

#define  ADS1256_SCOPE

typedef enum ads1256_pin_op ads1256_pin_op_t;
typedef enum ads1256_pin ads1256_pin_t;

/* parameter p_data: data buffer pointer
   parameter nbytes: number of bytes to read/write
   return val < 0 means error
   return val >=0 means successed number of bytes read/write*/
typedef int (*pfn_ads1256_io_t)(uint8_t *p_data, uint8_t nbytes);

/* parameter pin: pin type
   parameter op:  pin operation
   return val < 0 means error
   return 0 means pin low level 
   return 1 means pin high level
   return 2 means pin not support*/
typedef int (*pfn_ads1256_pin_t)(ads1256_pin_t pin ,ads1256_pin_op_t op);

/* parameter us: delay time in us
   return val < 0 means error
   return val >=0 means successed*/
typedef int (*pfn_ads1256_delay_us_t)(uint32_t us);

typedef enum ads1256_pin {
    ADS1256_Pin_CS = 0,
    ADS1256_Pin_DRDY,
    ADS1256_Pin_RST,
    ADS1256_Pin_SYNC,
} ads1256_pin_t;

typedef enum ads1256_pin_op{
    ADS1256_PIN_OP_LOW = 0,
    ADS1256_PIN_OP_HIGH,
    ADS1256_PIN_OP_READ,
} ads1256_pin_op_t;

typedef enum ads1256_ain{
    ADS1256_AIN0 = 0,
    ADS1256_AIN1,
    ADS1256_AIN2,
    ADS1256_AIN3,
    ADS1256_AIN4,
    ADS1256_AIN5,
    ADS1256_AIN6,
    ADS1256_AIN7,
    ADS1256_AINCOM,
} ads1256_ain_t;

typedef enum ads1256_gpa{
    ADS1256_GPA_1 = 0,
    ADS1256_GPA_2,
    ADS1256_GPA_4,
    ADS1256_GPA_8,
    ADS1256_GPA_16,
    ADS1256_GPA_32,
    ADS1256_GPA_64,
} ads1256_gpa_t;

/*
 11110000 = 30,000SPS (default)
 11100000 = 15,000SPS
 11010000 = 7,500SPS
 11000000 = 3,750SPS
 10110000 = 2,000SPS
 10100001 = 1,000SPS
 10010010 = 500SPS
 10000010 = 100SPS
 01110010 = 60SPS
 01100011 = 50SPS
 01010011 = 30SPS
 01000011 = 25SPS
 00110011 = 15SPS
 00100011 = 10SPS
 00010011 = 5SPS
 00000011 = 2.5SPS 
 */
typedef enum ads1256_sps{
    ADS1256_SPS_30000 = 0xF0,
    ADS1256_SPS_15000 = 0xE0,
    ADS1256_SPS_7500 = 0xD0,
    ADS1256_SPS_3750 = 0xC0,
    ADS1256_SPS_2000 = 0xB0,
    ADS1256_SPS_1000 = 0xA1,
    ADS1256_SPS_500 = 0x92,
    ADS1256_SPS_100 = 0x82,
    ADS1256_SPS_60 = 0x72,
    ADS1256_SPS_50 = 0x63,
    ADS1256_SPS_30 = 0x53,
    ADS1256_SPS_25 = 0x43,
    ADS1256_SPS_15 = 0x33,
    ADS1256_SPS_10 = 0x23,
    ADS1256_SPS_5 = 0x13,
    ADS1256_SPS_2_5 = 0x03,
} ads1256_sps_t;

/*
#define ADS1256_CMD_SELFCAL     0xF0
#define ADS1256_CMD_SELFOCAL    0xF1
#define ADS1256_CMD_SELFGCAL    0xF2
#define ADS1256_CMD_SYSOCAL     0xF3
#define ADS1256_CMD_SYSGCAL     0xF4
*/

typedef enum ads1256_calibration{
    ADS1256_CAL_SELF = 0xf0,
    ADS1256_CAL_SELF_OFFSET = 0xf1,
    ADS1256_CAL_SELF_GAIN = 0xf2,
    ADS1256_CAL_SYSTEM_OFFSET = 0xf3,
    ADS1256_CAL_SYSTEM_GAIN = 0xf4,
} ads1256_calibration_t;


typedef struct ads1256{
    pfn_ads1256_io_t  read;
    pfn_ads1256_io_t  write;
    pfn_ads1256_pin_t pin_op; 
    pfn_ads1256_delay_us_t delay_us;
    /* private data */
    bool is_init;
}ADS1256_t;




/* first call ads1256_init function to initialize the ads1256 */

/* api use For the usage method of the API, please refer to the official data manual. */

/* return value: < 0 means error, negative means success */
int ads1256_sync(ADS1256_t *ads1256);
int ads1256_wakeup(ADS1256_t *ads1256);
int ads1256_reset(ADS1256_t *ads1256);
int ads1256_is_data_ready(ADS1256_t *ads1256);
int ads1256_is_data_ready_wait(ADS1256_t *ads1256, uint32_t try_count);
int ads1256_read_data(ADS1256_t *ads1256, int32_t *p_data);
int ads1256_continue_read_start(ADS1256_t *ads1256);
int ads1256_continue_read_stop(ADS1256_t *ads1256);
int ads1256_calibration(ADS1256_t *ads1256, ads1256_calibration_t cal);
int ads1256_into_standby(ADS1256_t *ads1256);
int ads1256_low_order_enable(ADS1256_t *ads1256, bool enable);
int ads1256_auto_calibration_enable(ADS1256_t *ads1256, bool enable);
int ads1256_buff_enable(ADS1256_t *ads1256, bool enable);
int ads1256_set_ain_pin(ADS1256_t *ads1256, ads1256_ain_t ainp, ads1256_ain_t ainn);
int ads1256_set_gpa(ADS1256_t *ads1256, ads1256_gpa_t gpa);
int ads1256_get_gpa(ADS1256_t *ads1256, ads1256_gpa_t *p_gpa);
int ads1256_set_sps(ADS1256_t *ads1256, ads1256_sps_t sps);
int ads1256_get_sps(ADS1256_t *ads1256, ads1256_sps_t *p_sps);
int ads1256_init(ADS1256_t                 *ads1256, 
                 pfn_ads1256_io_t           read,
                 pfn_ads1256_io_t           write, 
                 pfn_ads1256_pin_t          pin_op,
                 pfn_ads1256_delay_us_t     delay_us);



#endif /* __ADS1256_H__ */
