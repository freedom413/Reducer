#ifndef __ADS1256_H__
#define __ADS1256_H__
#include <stdbool.h>
#include <stdint.h>

#define  FCLK  7680000UL  // 7.68MHz crystal oscillator frequency

#define  ADS1256_DEFAULT_VREF  2.5f

#define ADS1256_DIFF_0_1  ((uint8_t)0x01)
#define ADS1256_DIFF_2_3  ((uint8_t)0x23)
#define ADS1256_DIFF_4_5  ((uint8_t)0x45)
#define ADS1256_DIFF_6_7  ((uint8_t)0x67)

#define ADS1256_SING_0    ((uint8_t)0x0F)
#define ADS1256_SING_1    ((uint8_t)0x1F)
#define ADS1256_SING_2    ((uint8_t)0x2F)
#define ADS1256_SING_3    ((uint8_t)0x3F)
#define ADS1256_SING_4    ((uint8_t)0x4F)
#define ADS1256_SING_5    ((uint8_t)0x5F)
#define ADS1256_SING_6    ((uint8_t)0x6F)
#define ADS1256_SING_7    ((uint8_t)0x7F)

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
    ADS1256_Pin_CS_A = 0,
    ADS1256_Pin_CS_B,
    ADS1256_Pin_DRDY_A,
    ADS1256_Pin_DRDY_B,
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

typedef enum ads1256_pga {
    ADS1256_PGA_1 = 0,
    ADS1256_PGA_2,
    ADS1256_PGA_4,
    ADS1256_PGA_8,
    ADS1256_PGA_16,
    ADS1256_PGA_32,
    ADS1256_PGA_64,
} ads1256_pga_t;

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

typedef enum ads1256_byte_order {
    ADS1256_BYTE_ORDER_MSB = 0,
    ADS1256_BYTE_ORDER_LSB = 1,
} ads1256_byte_order_t;

typedef enum ads1256_clkout {
    ADS1256_CLKOUT_OFF = 0,
    ADS1256_CLKOUT_FCLK = 1,
    ADS1256_CLKOUT_FCLK_DIV2 = 2,
    ADS1256_CLKOUT_FCLK_DIV4 = 3,
} ads1256_clkout_t;

typedef enum ads1256_sensor_detect_current {
    ADS1256_SDCS_OFF = 0,
    ADS1256_SDCS_0_5_UA = 1,
    ADS1256_SDCS_2_UA = 2,
    ADS1256_SDCS_10_UA = 3,
} ads1256_sdcs_t;

typedef struct ads1256{
    pfn_ads1256_io_t  read;
    pfn_ads1256_io_t  write;
    pfn_ads1256_pin_t pin_op; 
    pfn_ads1256_delay_us_t delay_us;
    ads1256_pin_t cs_pin;
    ads1256_pin_t drdy_pin;
    /* private data */
    bool is_init;
    bool acquisition_running;
    uint8_t cycle;
    ads1256_pga_t pga;
    float vref;
    float conversion_parameter;
}ADS1256_t;




/* first call ads1256_init function to initialize the ads1256 */

/* api use For the usage method of the API, please refer to the official data manual. */

/* return value: < 0 means error, 0 or positive means success */
int ads1256_sync(ADS1256_t *ads1256);
int ads1256_wakeup(ADS1256_t *ads1256);
int ads1256_reset(ADS1256_t *ads1256);
int ads1256_is_data_ready(ADS1256_t *ads1256);
int ads1256_read_data(ADS1256_t *ads1256, int32_t *p_data);
/* Use only after DRDY is already low; this reads the latched previous result. */
int ads1256_read_data_nowait(ADS1256_t *ads1256, int32_t *p_data);
int ads1256_read_single(ADS1256_t *ads1256, int32_t *p_data);
int ads1256_read_single_voltage(ADS1256_t *ads1256, float *voltage);
int ads1256_read_continuous(ADS1256_t *ads1256, int32_t *p_data);
int ads1256_continue_read_start(ADS1256_t *ads1256);
int ads1256_continue_read_stop(ADS1256_t *ads1256);
int ads1256_calibration(ADS1256_t *ads1256, ads1256_calibration_t cal);
int ads1256_into_standby(ADS1256_t *ads1256);
int ads1256_set_byte_order(ADS1256_t *ads1256, ads1256_byte_order_t order);
int ads1256_get_byte_order(ADS1256_t *ads1256, ads1256_byte_order_t *order);
int ads1256_set_buffer(ADS1256_t *ads1256, bool enable);
int ads1256_get_buffer(ADS1256_t *ads1256, bool *enable);
int ads1256_set_auto_calibration(ADS1256_t *ads1256, bool enable);
int ads1256_get_auto_calibration(ADS1256_t *ads1256, bool *enable);
int ads1256_set_clkout(ADS1256_t *ads1256, ads1256_clkout_t clkout);
int ads1256_set_sensor_detect_current(ADS1256_t *ads1256, ads1256_sdcs_t sdcs);
int ads1256_set_ain_pin(ADS1256_t *ads1256, ads1256_ain_t ainp, ads1256_ain_t ainn);
int ads1256_get_ain_pin(ADS1256_t *ads1256, ads1256_ain_t *ainp, ads1256_ain_t *ainn);
int ads1256_set_pga(ADS1256_t *ads1256, ads1256_pga_t pga);
int ads1256_get_pga(ADS1256_t *ads1256, ads1256_pga_t *p_pga);
int ads1256_set_sps(ADS1256_t *ads1256, ads1256_sps_t sps);
int ads1256_get_sps(ADS1256_t *ads1256, ads1256_sps_t *p_sps);
int ads1256_read_reg_raw(ADS1256_t *ads1256, uint8_t reg, uint8_t *value);
int ads1256_write_reg_raw(ADS1256_t *ads1256, uint8_t reg, uint8_t value);
int ads1256_set_gpio_dir(ADS1256_t *ads1256, bool dir0_input, bool dir1_input, bool dir2_input, bool dir3_input);
int ads1256_write_gpio(ADS1256_t *ads1256, bool dio0, bool dio1, bool dio2, bool dio3);
int ads1256_read_gpio(ADS1256_t *ads1256, uint8_t gpio_pin, bool *value);
int ads1256_start_sync_conv(ADS1256_t *ads1256);
int ads1256_cycle_single(ADS1256_t *ads1256, int32_t *p_data, ads1256_ain_t *channel);
int ads1256_cycle_differential(ADS1256_t *ads1256, int32_t *p_data, ads1256_ain_t *ainp, ads1256_ain_t *ainn);
int ads1256_set_vref(ADS1256_t *ads1256, float vref);
float ads1256_convert_to_voltage(ADS1256_t *ads1256, int32_t raw_data);
int ads1256_initialize_default(ADS1256_t *ads1256, float vref);
int ads1256_init(ADS1256_t                 *ads1256, 
                 pfn_ads1256_io_t           read,
                 pfn_ads1256_io_t           write, 
                 ads1256_pin_t              cs_pin,
                 ads1256_pin_t              drdy_pin,
                 pfn_ads1256_pin_t          pin_op,
                 pfn_ads1256_delay_us_t     delay_us);

#endif /* __ADS1256_H__ */
