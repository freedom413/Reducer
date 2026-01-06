#include "ads1256.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/_intsup.h>

#define  T6_US    ((50U * 1000000U / FCLK) + 1)  // t6 = 50 / 7,680,000 ≈ 6.51 µs
#define  T16_US   ((4U * 1000000U / FCLK) + 1)  // t6 = 50 / 7,680,000 ≈ 6.51 µs


#define  DRDY_WAIT_COUNT  (3)
/**
ADDRESS REGISTER RESET
VALUE BIT 7 BIT 6 BIT 5 BIT 4 BIT 3 BIT 2 BIT 1 BIT 0
00h STATUS x1H ID3 ID2 ID1 ID0 ORDER ACAL BUFEN DRDY
01h MUX 01H PSEL3 PSEL2 PSEL1 PSEL0 NSEL3 NSEL2 NSEL1 NSEL0
02h ADCON 20H 0 CLK1 CLK0 SDCS1 SDCS0 PGA2 PGA1 PGA0
03h DRATE F0H DR7 DR6 DR5 DR4 DR3 DR2 DR1 DR0
04h IO E0H DIR3 DIR2 DIR1 DIR0 DIO3 DIO2 DIO1 DIO0
05h OFC0 xxH OFC07 OFC06 OFC05 OFC04 OFC03 OFC02 OFC01 OFC00
06h OFC1 xxH OFC15 OFC14 OFC13 OFC12 OFC11 OFC10 OFC09 OFC08
07h OFC2 xxH OFC23 OFC22 OFC21 OFC20 OFC19 OFC18 OFC17 OFC16
08h FSC0 xxH FSC07 FSC06 FSC05 FSC04 FSC03 FSC02 FSC01 FSC00
09h FSC1 xxH FSC15 FSC14 FSC13 FSC12 FSC11 FSC10 FSC09 FSC08
0Ah FSC2 xxH FSC23 FSC22 FSC21 FSC20 FSC19 FSC18 FSC17 FSC16
 */

#define ADS1256_REG_STATUS    0x00
#define ADS1256_REG_MUX       0x01
#define ADS1256_REG_ADCON     0x02
#define ADS1256_REG_DRATE     0x03
#define ADS1256_REG_IO        0x04
#define ADS1256_REG_OFC0      0x05
#define ADS1256_REG_OFC1      0x06
#define ADS1256_REG_OFC2      0x07
#define ADS1256_REG_FSC0      0x08
#define ADS1256_REG_FSC1      0x09
#define ADS1256_REG_FSC2      0x0A

#define ADS1256_REG_STATUS_ID_POS   4U
#define ADS1256_REG_STATUS_ID_MASK  0xf0
#define ADS1256_REG_STATUS_ORDER_POS 3U
#define ADS1256_REG_STATUS_ORDER_MASK 0x08
#define ADS1256_REG_STATUS_ACAL_POS 2U
#define ADS1256_REG_STATUS_ACAL_MASK 0x04
#define ADS1256_REG_STATUS_BUFEN_POS 1U
#define ADS1256_REG_STATUS_BUFEN_MASK 0x02
#define ADS1256_REG_STATUS_DRDY_POS 0U
#define ADS1256_REG_STATUS_DRDY_MASK 0x01

#define ADS1256_REG_MUX_PSEL_POS    4U
#define ADS1256_REG_MUX_PSEL_MASK   0xf0
#define ADS1256_REG_MUX_NSEL_POS    0U
#define ADS1256_REG_MUX_NSEL_MASK   0x0f

#define ADS1256_REG_ADCON_CLK_POS    3U
#define ADS1256_REG_ADCON_CLK_MASK   0xC0
#define ADS1256_REG_ADCON_SDCS_POS   1U
#define ADS1256_REG_ADCON_SDCS_MASK  0x30
#define ADS1256_REG_ADCON_PGA_POS    0U
#define ADS1256_REG_ADCON_PGA_MASK   0x07

#define ADS1256_REG_DRATE_DR_POS     0U
#define ADS1256_REG_DRATE_DR_MASK    0xFF

#define ADS1256_REG_IO_DIR_POS    4U
#define ADS1256_REG_IO_DIR_MASK   0xf0
#define ADS1256_REG_IO_DIO_POS    0U
#define ADS1256_REG_IO_DIO_MASK   0x0f

#define ADS1256_REG_OFC0_OFC0_POS    0U
#define ADS1256_REG_OFC0_OFC0_MASK   0xFF
#define ADS1256_REG_OFC1_OFC1_POS    8U
#define ADS1256_REG_OFC1_OFC1_MASK   0xFF
#define ADS1256_REG_OFC2_OFC2_POS    16U
#define ADS1256_REG_OFC2_OFC2_MASK   0xFF

#define ADS1256_REG_FSC0_FSC0_POS    0U
#define ADS1256_REG_FSC0_FSC0_MASK   0xFF
#define ADS1256_REG_FSC1_FSC1_POS    8U
#define ADS1256_REG_FSC1_FSC1_MASK   0xFF
#define ADS1256_REG_FSC2_FSC2_POS    16U
#define ADS1256_REG_FSC2_FSC2_MASK   0xFF


/*
COMMAND DESCRIPTION 1ST COMMAND BYTE 2ND COMMAND BYTE
WAKEUP Completes SYNC and Exits Standby Mode 0000  0000 (00h)
RDATA Read Data 0000  0001 (01h)
RDATAC Read Data Continuously 0000   0011 (03h)
SDATAC Stop Read Data Continuously 0000   1111 (0Fh)
RREG Read from REG rrr 0001 rrrr (1xh) 0000 nnnn
WREG Write to REG rrr 0101 rrrr (5xh) 0000 nnnn
SELFCAL Offset and Gain Self-Calibration 1111    0000 (F0h)
SELFOCAL Offset Self-Calibration 1111    0001 (F1h)
SELFGCAL Gain Self-Calibration 1111   0010 (F2h)
SYSOCAL System Offset Calibration 1111   0011 (F3h)
SYSGCAL System Gain Calibration 1111   0100 (F4h)
SYNC Synchronize the A/D Conversion 1111   1100 (FCh)
STANDBY Begin Standby Mode 1111   1101 (FDh)
RESET Reset to Power-Up Values 1111   1110 (FEh)
WAKEUP Completes SYNC and Exits Standby Mode 1111   1111 (FFh)
*/

#define ADS1256_CMD_WAKEUP      0x00
#define ADS1256_CMD_RDATA       0x01
#define ADS1256_CMD_RDATAC      0x03
#define ADS1256_CMD_SDATAC      0x0F
#define ADS1256_CMD_RREG        0x10
#define ADS1256_CMD_WREG        0x50
#define ADS1256_CMD_SELFCAL     0xF0
#define ADS1256_CMD_SELFOCAL    0xF1
#define ADS1256_CMD_SELFGCAL    0xF2
#define ADS1256_CMD_SYSOCAL     0xF3
#define ADS1256_CMD_SYSGCAL     0xF4
#define ADS1256_CMD_SYNC        0xFC
#define ADS1256_CMD_STANDBY     0xFD
#define ADS1256_CMD_RESET       0xFE
// #define ADS1256_CMD_WAKEUP      0xFF


static int __ads1256_write_reg(ADS1256_t *ads1256, uint8_t start_reg, uint8_t *p_data, uint8_t nbytes)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t cmd = ADS1256_CMD_WREG | start_reg;
    uint8_t count = 0x0f & (nbytes - 1);
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&cmd, 1);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&count, 1);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(p_data, nbytes);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_HIGH);
    if (ret < 0) {
        return ret;
    }

    return ret;
}

static int __ads1256_read_reg(ADS1256_t *ads1256, uint8_t start_reg, uint8_t *p_data, uint8_t nbytes)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t cmd = ADS1256_CMD_RREG | start_reg;
    uint8_t count = 0x0f & (nbytes - 1);
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&cmd, 1);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&count, 1);
    if (ret < 0) {
        return ret;
    }
    // Dummy byte for t6 delay
    ret = ads1256->delay_us(T6_US);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->read(p_data, nbytes);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_HIGH);
    if (ret < 0) {
        return ret;
    }

    return ret;
}

static int __ads1256_set_reg_bit(ADS1256_t *ads1256, uint8_t reg, uint8_t bit_mask)
{
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, reg, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    reg_val |= bit_mask;
    ret = __ads1256_write_reg(ads1256, reg, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    return ret;
}

static int __ads1256_clear_reg_bit(ADS1256_t *ads1256, uint8_t reg, uint8_t bit_mask)
{
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, reg, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    reg_val &= ~bit_mask;
    ret = __ads1256_write_reg(ads1256, reg, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    return ret;
}

static int __ads1256_write_cmd(ADS1256_t *ads1256, uint8_t cmd)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&cmd, 1);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_HIGH);
    if (ret < 0) {
        return ret;
    }

    return ret;
}

/****************************ads1256 api************************************/

int ads1256_sync(ADS1256_t *ads1256)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    if (ads1256->pin_op != NULL) {
        ret = ads1256->pin_op(ADS1256_Pin_SYNC, ADS1256_PIN_OP_LOW);
        if (ret == 2) {
            goto nopin;
        }
        if (ret < 0) {
            return ret;
        }
        ret = ads1256->delay_us(T16_US);
        if (ret < 0) {
            return ret;
        }
        ret = ads1256->pin_op(ADS1256_Pin_SYNC, ADS1256_PIN_OP_HIGH);
        if (ret < 0) {
            return ret;
        }
    } else {
nopin:
        ret = __ads1256_write_cmd(ads1256, ADS1256_CMD_SYNC);
        if (ret < 0) {
            return ret;
        }
    }
    return ret;
}

int ads1256_wakeup(ADS1256_t *ads1256)
{
    if (!ads1256->is_init) {
        return -1;
    }

    return __ads1256_write_cmd(ads1256, ADS1256_CMD_WAKEUP);

}

int ads1256_reset(ADS1256_t *ads1256)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    if (ads1256->pin_op != NULL) {
        ret = ads1256->pin_op(ADS1256_Pin_RST, ADS1256_PIN_OP_LOW);
        if (ret == 2) {
            goto nopin;
        }
        if (ret < 0) {
            return ret;
        }
        ret = ads1256->delay_us(T16_US);
        if (ret < 0) {
            return ret;
        }
        ret = ads1256->pin_op(ADS1256_Pin_RST, ADS1256_PIN_OP_HIGH);
        if (ret < 0) {
            return ret;
        }
    } else {
nopin:
        ret = __ads1256_write_cmd(ads1256, ADS1256_CMD_RESET);
        if (ret < 0) {
            return ret;
        }
    }
    return ret;
}

int ads1256_is_data_ready(ADS1256_t *ads1256)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t reg = 0;
    if (ads1256->pin_op != NULL) {
        ret = ads1256->pin_op(ADS1256_Pin_DRDY, ADS1256_PIN_OP_READ);
        if (ret == 2) {
            goto nopin;
        }
        if (ret < 0) {
            return ret;
        }
        return (((ads1256_pin_op_t)ret) == ADS1256_PIN_OP_LOW) ? 1 : 0;
    } else {
nopin:
        ret = __ads1256_read_reg(ads1256, ADS1256_REG_STATUS, &reg, 1);
        if (ret < 0) {
            return ret;
        }
        return (reg & ADS1256_REG_STATUS_DRDY_MASK) ? 1 : 0;
    }
}

int ads1256_read_data(ADS1256_t *ads1256, int32_t *p_data)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t cmd = ADS1256_CMD_RDATA;
    uint8_t buf[3] = {0};
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&cmd, 1);
    if (ret < 0) {
        return ret;
    }
    // Dummy byte for t6 delay
    ret = ads1256->delay_us(T6_US);
    if (ret < 0) {
        return ret;
    }
    // Read 3 bytes from ADS1256 ADC 24-bit data
    ret = ads1256->read(buf, 3);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->pin_op(ADS1256_Pin_CS, ADS1256_PIN_OP_HIGH);
    if (ret < 0) {
        return ret;
    }
    // Convert 3 bytes to 24-bit data
    uint32_t val = (((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) | ((uint32_t)buf[2]);
    if (val & 0x800000) {
        val |= 0xFF000000;
    }else {
        val &= 0x00FFFFFF;
    }
   *p_data = (int32_t)val;
    return ret;
}


int ads1256_continue_read_start(ADS1256_t *ads1256)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    int i = 0;
    while (true) {
        ret = ads1256_is_data_ready(ads1256);
        if (ret < 0) {
            return ret;
        }
        if (ret == 1) {
            break;
        }
        if (++i >= DRDY_WAIT_COUNT) {
            return -2;
        }
        ads1256->delay_us(T6_US);
    }
    ret =  __ads1256_write_cmd(ads1256, ADS1256_CMD_RDATAC);
    if (ret < 0) {
        return ret;
    }
    // Dummy byte for t6 delay
    ret = ads1256->delay_us(T6_US);
    if (ret < 0) {
        return ret;
    }
    
    return ret;
}

int ads1256_continue_read_stop(ADS1256_t *ads1256)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    int i = 0;
    while (true) {
        ret = ads1256_is_data_ready(ads1256);
        if (ret < 0) {
            return ret;
        }
        if (ret == 1) {
            break;
        }
        if (++i >= DRDY_WAIT_COUNT) {
            return -2;
        }
        ads1256->delay_us(T6_US);
    }
    ret =  __ads1256_write_cmd(ads1256, ADS1256_CMD_SDATAC);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->delay_us(T6_US);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

int ads1256_calibration(ADS1256_t *ads1256, ads1256_calibration_t cal)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t cmd = (uint8_t)cal;
    ret = __ads1256_write_cmd(ads1256, cmd);
    if (ret < 0) {
        return ret;
    }
    
    while (ads1256_is_data_ready(ads1256) != 1) {
        ads1256->delay_us(T6_US);
    }

    return ret;
}

int ads1256_low_order_enable(ADS1256_t *ads1256, bool enable)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    if (enable) {
        ret = __ads1256_set_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ORDER_MASK);
    } else {
        ret = __ads1256_clear_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ORDER_MASK);
    }
    return ret;
}

int ads1256_auto_calibration_enable(ADS1256_t *ads1256, bool enable)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    if (enable) {
        ret = __ads1256_set_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ACAL_MASK);
    } else {
        ret = __ads1256_clear_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ACAL_MASK);
    }
    return ret;
}

int ads1256_buff_enable(ADS1256_t *ads1256, bool enable)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    if (enable) {
        ret = __ads1256_set_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_BUFEN_MASK);
    } else {
        ret = __ads1256_clear_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_BUFEN_MASK);
    }
    return ret;
}


int ads1256_set_ain_pin(ADS1256_t *ads1256, ads1256_ain_t ainp, ads1256_ain_t ainn)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    reg_val |= ((((uint8_t)ainp) << ADS1256_REG_MUX_PSEL_POS) & ADS1256_REG_MUX_PSEL_MASK);
    reg_val |= ((((uint8_t)ainn) << ADS1256_REG_MUX_NSEL_POS) & ADS1256_REG_MUX_NSEL_MASK);
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_MUX, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    return ret;
}


int ads1256_set_gpa(ADS1256_t *ads1256, ads1256_gpa_t gpa)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    reg_val &= ~ADS1256_REG_ADCON_PGA_MASK;
    reg_val |= ((((uint8_t)gpa) << ADS1256_REG_ADCON_PGA_POS) & ADS1256_REG_ADCON_PGA_MASK);
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    return ret;
}

int ads1256_get_gpa(ADS1256_t *ads1256, ads1256_gpa_t *p_gpa)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
   *p_gpa = (ads1256_gpa_t)((reg_val & ADS1256_REG_ADCON_PGA_MASK) >> ADS1256_REG_ADCON_PGA_POS);
    return ret;
}

int ads1256_set_sps(ADS1256_t *ads1256, ads1256_sps_t sps)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = (uint8_t)sps;
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_DRATE, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    return ret;
}


int ads1256_get_sps(ADS1256_t *ads1256, ads1256_sps_t *p_sps)
{
    if (!ads1256->is_init) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, ADS1256_REG_DRATE, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
   *p_sps = (ads1256_sps_t)((reg_val & ADS1256_REG_DRATE_DR_MASK) >> ADS1256_REG_DRATE_DR_POS);
    return ret;
}

int ads1256_init(ADS1256_t                 *ads1256, 
                 pfn_ads1256_io_t           read,
                 pfn_ads1256_io_t           write, 
                 pfn_ads1256_pin_t          pin_op,
                 pfn_ads1256_delay_us_t     delay_us)
{   
    if (ads1256 == NULL || read == NULL || write == NULL || pin_op == NULL || delay_us == NULL) {
        return -1;
    }
    ads1256->read = read;
    ads1256->write = write;
    ads1256->pin_op = pin_op;
    ads1256->delay_us = delay_us;
    ads1256->is_init = true;
    return 0;
}

