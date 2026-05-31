#include "ads1256.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>



/* from datasheet */

#define  T11_4    ((4U * 1000000U / FCLK) + 1) 
#define  T11_24   ((24U * 1000000U / FCLK) + 1) 
#define  T6       ((50U * 1000000U / FCLK) + 1)  
#define  T16      ((4U * 1000000U / FCLK) + 1) 


#define  DRDY_WAIT_COUNT     (3000U)
#define  DRDY_WAIT_DELAY_US  (1000U)
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

#define ADS1256_REG_ADCON_CLK_POS    5U
#define ADS1256_REG_ADCON_CLK_MASK   0x60
#define ADS1256_REG_ADCON_SDCS_POS   3U
#define ADS1256_REG_ADCON_SDCS_MASK  0x18
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

#define ADS1256_REG_LAST             ADS1256_REG_FSC2
#define ADS1256_REG_COUNT            (ADS1256_REG_LAST + 1U)


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

#define ADS1256_FULL_SCALE_CODE 8388608.0f

static int __ads1256_write_cmd(ADS1256_t *ads1256, uint8_t cmd);

static const uint8_t ads1256_single_mux[8] = {
    ADS1256_SING_0,
    ADS1256_SING_1,
    ADS1256_SING_2,
    ADS1256_SING_3,
    ADS1256_SING_4,
    ADS1256_SING_5,
    ADS1256_SING_6,
    ADS1256_SING_7,
};

static const uint8_t ads1256_diff_mux[4] = {
    ADS1256_DIFF_0_1,
    ADS1256_DIFF_2_3,
    ADS1256_DIFF_4_5,
    ADS1256_DIFF_6_7,
};

static const ads1256_ain_t ads1256_diff_p[4] = {
    ADS1256_AIN0,
    ADS1256_AIN2,
    ADS1256_AIN4,
    ADS1256_AIN6,
};

static const ads1256_ain_t ads1256_diff_n[4] = {
    ADS1256_AIN1,
    ADS1256_AIN3,
    ADS1256_AIN5,
    ADS1256_AIN7,
};

static bool __ads1256_is_valid(ADS1256_t *ads1256)
{
    return (ads1256 != NULL) && ads1256->is_init;
}

static bool __ads1256_valid_ain(ads1256_ain_t ain)
{
    return ain <= ADS1256_AINCOM;
}

static bool __ads1256_valid_pga(ads1256_pga_t pga)
{
    return pga <= ADS1256_PGA_64;
}

static uint8_t __ads1256_make_mux(ads1256_ain_t ainp, ads1256_ain_t ainn)
{
    return (uint8_t)((((uint8_t)ainp) << ADS1256_REG_MUX_PSEL_POS) |
                     (((uint8_t)ainn) << ADS1256_REG_MUX_NSEL_POS));
}

static int32_t __ads1256_sign_extend_24(uint8_t msb, uint8_t mid, uint8_t lsb)
{
    uint32_t val = (((uint32_t)msb) << 16) | (((uint32_t)mid) << 8) | ((uint32_t)lsb);

    if ((val & 0x800000UL) != 0U) {
        val |= 0xFF000000UL;
    } else {
        val &= 0x00FFFFFFUL;
    }

    return (int32_t)val;
}

static void __ads1256_update_conversion_parameter(ADS1256_t *ads1256)
{
    uint32_t gain = 1UL << (uint8_t)ads1256->pga;
    ads1256->conversion_parameter = ((2.0f * ads1256->vref) / ADS1256_FULL_SCALE_CODE) / (float)gain;
}

static int __ads1256_read_data_bytes(ADS1256_t *ads1256, int32_t *p_data)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL) {
        return -1;
    }

    int ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }

    uint8_t buf[3] = {0};
    ret = ads1256->read(buf, sizeof(buf));
    if (ret < 0) {
        (void)ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
        return ret;
    }

    ret = ads1256->delay_us(T11_4);
    if (ret < 0) {
        (void)ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
        return ret;
    }

    ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
    if (ret < 0) {
        return ret;
    }

    *p_data = __ads1256_sign_extend_24(buf[0], buf[1], buf[2]);
    return 0;
}

static int __ads1256_stop_continuous_if_needed(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    if (!ads1256->acquisition_running) {
        return 0;
    }

    int ret = __ads1256_write_cmd(ads1256, ADS1256_CMD_SDATAC);
    if (ret < 0) {
        return ret;
    }

    ads1256->acquisition_running = false;
    return 0;
}

static int __ads1256_write_reg(ADS1256_t *ads1256, uint8_t start_reg, uint8_t *p_data, uint8_t nbytes)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL ||
        nbytes == 0U || nbytes > ADS1256_REG_COUNT || start_reg > ADS1256_REG_LAST) {
        return -1;
    }
    int ret = 0;
    uint8_t wreg_cmd [2] = {0};
    wreg_cmd[0] = ADS1256_CMD_WREG | (start_reg & 0x0FU);
    wreg_cmd[1] = 0x0f & (nbytes - 1);

    ret = __ads1256_stop_continuous_if_needed(ads1256);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(wreg_cmd, sizeof(wreg_cmd));
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->write(p_data, nbytes);
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->delay_us(T11_4);

done:
    {
        int cs_ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
        if (ret < 0) {
            return ret;
        }
        return (cs_ret < 0) ? cs_ret : ret;
    }
}

static int __ads1256_read_reg(ADS1256_t *ads1256, uint8_t start_reg, uint8_t *p_data, uint8_t nbytes)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL ||
        nbytes == 0U || nbytes > ADS1256_REG_COUNT || start_reg > ADS1256_REG_LAST) {
        return -1;
    }
    int ret = 0;
    uint8_t rreg_cmd [2] = {0};
    rreg_cmd[0] = ADS1256_CMD_RREG | (start_reg & 0x0FU);
    rreg_cmd[1] = 0x0f & (nbytes - 1);

    ret = __ads1256_stop_continuous_if_needed(ads1256);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(rreg_cmd, sizeof(rreg_cmd));
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->delay_us(T6);
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->read(p_data, nbytes);
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->delay_us(T11_4);

done:
    {
        int cs_ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
        if (ret < 0) {
            return ret;
        }
        return (cs_ret < 0) ? cs_ret : ret;
    }
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

static uint32_t __ads1256_cmd_hold_us(uint8_t cmd)
{
    switch (cmd) {
        case ADS1256_CMD_RDATAC:
        case ADS1256_CMD_SYNC:
            return T11_24;
        default:
            return T11_4;
    }
}

static int __ads1256_write_cmd(ADS1256_t *ads1256, uint8_t cmd)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret = 0;
    ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&cmd, 1);
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->delay_us(__ads1256_cmd_hold_us(cmd));

done:
    {
        int cs_ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
        if (ret < 0) {
            return ret;
        }
        return (cs_ret < 0) ? cs_ret : ret;
    }
}

/****************************ads1256 api************************************/

int ads1256_is_data_ready(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret = 0;
    uint8_t reg = 0;
    if (ads1256->pin_op != NULL) {
        ret = ads1256->pin_op(ads1256->drdy_pin, ADS1256_PIN_OP_READ);
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
        return (reg & ADS1256_REG_STATUS_DRDY_MASK) ? 0 : 1;
    }
}

static int ads1256_is_data_ready_wait(ADS1256_t *ads1256)
{
    for (uint32_t i = 0; i < DRDY_WAIT_COUNT; i++) {
        int ready = ads1256_is_data_ready(ads1256);
        if (ready > 0) {
            return 0;
        }
        if (ready < 0) {
            return ready;
        }
        if (ads1256->delay_us(DRDY_WAIT_DELAY_US) < 0) {
            return -1;
        }
    }

    return -2;
}


int ads1256_wakeup(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    return __ads1256_write_cmd(ads1256, ADS1256_CMD_WAKEUP);

}


int ads1256_sync(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
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
        ret = ads1256->delay_us(T16);
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

int ads1256_reset(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
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
        ret = ads1256->delay_us(T16);
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
    ret = ads1256_is_data_ready_wait(ads1256);
    if (ret < 0) {
        return ret;
    }
    ads1256->acquisition_running = false;
    return ret;
}

static int __ads1256_read_data_command(ADS1256_t *ads1256, int32_t *p_data, bool wait_ready)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL) {
        return -1;
    }
    int ret = 0;
    uint8_t cmd = ADS1256_CMD_RDATA;

    ret = __ads1256_stop_continuous_if_needed(ads1256);
    if (ret < 0) {
        return ret;
    }

    if (wait_ready) {
        ret = ads1256_is_data_ready_wait(ads1256);
        if (ret < 0) {
            return ret;
        }
    }

    ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_LOW);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256->write(&cmd, 1);
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->delay_us(T6);
    if (ret < 0) {
        goto done;
    }

    uint8_t buf[3] = {0};
    ret = ads1256->read(buf, sizeof(buf));
    if (ret < 0) {
        goto done;
    }
    ret = ads1256->delay_us(T11_4);
    if (ret < 0) {
        goto done;
    }

    *p_data = __ads1256_sign_extend_24(buf[0], buf[1], buf[2]);
    ret = 0;

done:
    {
        int cs_ret = ads1256->pin_op(ads1256->cs_pin, ADS1256_PIN_OP_HIGH);
        if (ret < 0) {
            return ret;
        }
        return (cs_ret < 0) ? cs_ret : ret;
    }
}

int ads1256_read_data(ADS1256_t *ads1256, int32_t *p_data)
{
    return __ads1256_read_data_command(ads1256, p_data, true);
}

int ads1256_read_data_nowait(ADS1256_t *ads1256, int32_t *p_data)
{
    return __ads1256_read_data_command(ads1256, p_data, false);
}

int ads1256_read_single(ADS1256_t *ads1256, int32_t *p_data)
{
    return ads1256_read_data(ads1256, p_data);
}

int ads1256_read_single_voltage(ADS1256_t *ads1256, float *voltage)
{
    if (!__ads1256_is_valid(ads1256) || voltage == NULL) {
        return -1;
    }

    int32_t raw_data = 0;
    int ret = ads1256_read_single(ads1256, &raw_data);
    if (ret < 0) {
        return ret;
    }

    *voltage = ads1256_convert_to_voltage(ads1256, raw_data);
    return 0;
}


int ads1256_continue_read_start(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret = 0;
    ret = ads1256_is_data_ready_wait(ads1256);
    if (ret < 0) {
        return ret;
    }
    ret =  __ads1256_write_cmd(ads1256, ADS1256_CMD_RDATAC);
    if (ret < 0) {
        return ret;
    }
    // Dummy byte for t6 delay
    ret = ads1256->delay_us(T6);
    if (ret < 0) {
        return ret;
    }
    ads1256->acquisition_running = true;
    return ret;
}

int ads1256_read_continuous(ADS1256_t *ads1256, int32_t *p_data)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL) {
        return -1;
    }

    int ret = 0;
    if (!ads1256->acquisition_running) {
        ret = ads1256_continue_read_start(ads1256);
        if (ret < 0) {
            return ret;
        }
    } else {
        ret = ads1256_is_data_ready_wait(ads1256);
        if (ret < 0) {
            return ret;
        }
    }

    return __ads1256_read_data_bytes(ads1256, p_data);
}

int ads1256_continue_read_stop(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret = 0;
    ret = ads1256_is_data_ready_wait(ads1256);
    if (ret < 0) {
        return ret;
    }

    ret =  __ads1256_write_cmd(ads1256, ADS1256_CMD_SDATAC);
    if (ret < 0) {
        return ret;
    }
    ads1256->acquisition_running = false;
    return 0;
}

int ads1256_calibration(ADS1256_t *ads1256, ads1256_calibration_t cal)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret = 0;
    uint8_t cmd = (uint8_t)cal;
    ret = __ads1256_write_cmd(ads1256, cmd);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_is_data_ready_wait(ads1256);
    if (ret < 0) {
        return ret;
    }

    return ret;
}


int ads1256_into_standby(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret = 0;
    ret = __ads1256_write_cmd(ads1256, ADS1256_CMD_STANDBY);
    if (ret < 0) {
        return ret;
    }
    
    ret = ads1256_is_data_ready_wait(ads1256);
    if (ret < 0) {
        return ret;
    }
    ads1256->acquisition_running = false;
    return ret;
}

int ads1256_set_byte_order(ADS1256_t *ads1256, ads1256_byte_order_t order)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    if (order == ADS1256_BYTE_ORDER_LSB) {
        return __ads1256_set_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ORDER_MASK);
    }

    return __ads1256_clear_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ORDER_MASK);
}

int ads1256_get_byte_order(ADS1256_t *ads1256, ads1256_byte_order_t *order)
{
    if (!__ads1256_is_valid(ads1256) || order == NULL) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_STATUS, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    *order = ((reg_val & ADS1256_REG_STATUS_ORDER_MASK) != 0U) ?
             ADS1256_BYTE_ORDER_LSB : ADS1256_BYTE_ORDER_MSB;
    return 0;
}

int ads1256_set_buffer(ADS1256_t *ads1256, bool enable)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    if (enable) {
        return __ads1256_set_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_BUFEN_MASK);
    }

    return __ads1256_clear_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_BUFEN_MASK);
}

int ads1256_get_buffer(ADS1256_t *ads1256, bool *enable)
{
    if (!__ads1256_is_valid(ads1256) || enable == NULL) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_STATUS, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    *enable = (reg_val & ADS1256_REG_STATUS_BUFEN_MASK) != 0U;
    return 0;
}

int ads1256_set_auto_calibration(ADS1256_t *ads1256, bool enable)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    if (enable) {
        return __ads1256_set_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ACAL_MASK);
    }

    return __ads1256_clear_reg_bit(ads1256, ADS1256_REG_STATUS, ADS1256_REG_STATUS_ACAL_MASK);
}

int ads1256_get_auto_calibration(ADS1256_t *ads1256, bool *enable)
{
    if (!__ads1256_is_valid(ads1256) || enable == NULL) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_STATUS, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    *enable = (reg_val & ADS1256_REG_STATUS_ACAL_MASK) != 0U;
    return 0;
}

int ads1256_set_clkout(ADS1256_t *ads1256, ads1256_clkout_t clkout)
{
    if (!__ads1256_is_valid(ads1256) || clkout > ADS1256_CLKOUT_FCLK_DIV4) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    reg_val &= (uint8_t)~ADS1256_REG_ADCON_CLK_MASK;
    reg_val |= (uint8_t)((((uint8_t)clkout) << ADS1256_REG_ADCON_CLK_POS) & ADS1256_REG_ADCON_CLK_MASK);
    return __ads1256_write_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
}

int ads1256_set_sensor_detect_current(ADS1256_t *ads1256, ads1256_sdcs_t sdcs)
{
    if (!__ads1256_is_valid(ads1256) || sdcs > ADS1256_SDCS_10_UA) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    reg_val &= (uint8_t)~ADS1256_REG_ADCON_SDCS_MASK;
    reg_val |= (uint8_t)((((uint8_t)sdcs) << ADS1256_REG_ADCON_SDCS_POS) & ADS1256_REG_ADCON_SDCS_MASK);
    return __ads1256_write_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
}


int ads1256_set_ain_pin(ADS1256_t *ads1256, ads1256_ain_t ainp, ads1256_ain_t ainn)
{
    if (!__ads1256_is_valid(ads1256) || !__ads1256_valid_ain(ainp) || !__ads1256_valid_ain(ainn)) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    reg_val = __ads1256_make_mux(ainp, ainn);
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_MUX, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    return ret;
}

int ads1256_get_ain_pin(ADS1256_t *ads1256, ads1256_ain_t *ainp, ads1256_ain_t *ainn)
{
    if (!__ads1256_is_valid(ads1256) || ainp == NULL || ainn == NULL) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, ADS1256_REG_MUX, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    *ainp = (ads1256_ain_t)((reg_val & ADS1256_REG_MUX_PSEL_MASK) >> ADS1256_REG_MUX_PSEL_POS);
    *ainn = (ads1256_ain_t)((reg_val & ADS1256_REG_MUX_NSEL_MASK) >> ADS1256_REG_MUX_NSEL_POS);
    return ret;
}


int ads1256_set_pga(ADS1256_t *ads1256, ads1256_pga_t pga)
{
    if (!__ads1256_is_valid(ads1256) || !__ads1256_valid_pga(pga)) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    reg_val &= ~ADS1256_REG_ADCON_PGA_MASK;
    reg_val |= ((((uint8_t)pga) << ADS1256_REG_ADCON_PGA_POS) & ADS1256_REG_ADCON_PGA_MASK);
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    ads1256->pga = pga;
    __ads1256_update_conversion_parameter(ads1256);
    return ret;
}

int ads1256_get_pga(ADS1256_t *ads1256, ads1256_pga_t *p_pga)
{
    if (!__ads1256_is_valid(ads1256) || p_pga == NULL) {
        return -1;
    }
    int ret = 0;
    uint8_t reg_val = 0;
    ret = __ads1256_read_reg(ads1256, ADS1256_REG_ADCON, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }
    *p_pga = (ads1256_pga_t)((reg_val & ADS1256_REG_ADCON_PGA_MASK) >> ADS1256_REG_ADCON_PGA_POS);
    ads1256->pga = *p_pga;
    __ads1256_update_conversion_parameter(ads1256);
    return ret;
}

int ads1256_set_sps(ADS1256_t *ads1256, ads1256_sps_t sps)
{
    if (!__ads1256_is_valid(ads1256)) {
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
    if (!__ads1256_is_valid(ads1256) || p_sps == NULL) {
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

int ads1256_read_reg_raw(ADS1256_t *ads1256, uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return -1;
    }

    return __ads1256_read_reg(ads1256, reg, value, 1);
}

int ads1256_write_reg_raw(ADS1256_t *ads1256, uint8_t reg, uint8_t value)
{
    int ret = __ads1256_write_reg(ads1256, reg, &value, 1);
    if (ret < 0) {
        return ret;
    }

    if (reg == ADS1256_REG_ADCON) {
        ads1256->pga = (ads1256_pga_t)(value & ADS1256_REG_ADCON_PGA_MASK);
        __ads1256_update_conversion_parameter(ads1256);
    }

    return ret;
}

int ads1256_set_gpio_dir(ADS1256_t *ads1256, bool dir0_input, bool dir1_input, bool dir2_input, bool dir3_input)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_IO, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    reg_val &= (uint8_t)~ADS1256_REG_IO_DIR_MASK;
    reg_val |= (uint8_t)(((dir0_input ? 1U : 0U) << 4) |
                         ((dir1_input ? 1U : 0U) << 5) |
                         ((dir2_input ? 1U : 0U) << 6) |
                         ((dir3_input ? 1U : 0U) << 7));

    return __ads1256_write_reg(ads1256, ADS1256_REG_IO, &reg_val, 1);
}

int ads1256_write_gpio(ADS1256_t *ads1256, bool dio0, bool dio1, bool dio2, bool dio3)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_IO, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    reg_val &= (uint8_t)~ADS1256_REG_IO_DIO_MASK;
    reg_val |= (uint8_t)((dio0 ? 1U : 0U) |
                         ((dio1 ? 1U : 0U) << 1) |
                         ((dio2 ? 1U : 0U) << 2) |
                         ((dio3 ? 1U : 0U) << 3));

    return __ads1256_write_reg(ads1256, ADS1256_REG_IO, &reg_val, 1);
}

int ads1256_read_gpio(ADS1256_t *ads1256, uint8_t gpio_pin, bool *value)
{
    if (!__ads1256_is_valid(ads1256) || gpio_pin > 3U || value == NULL) {
        return -1;
    }

    uint8_t reg_val = 0;
    int ret = __ads1256_read_reg(ads1256, ADS1256_REG_IO, &reg_val, 1);
    if (ret < 0) {
        return ret;
    }

    *value = ((reg_val >> gpio_pin) & 0x01U) != 0U;
    return 0;
}

int ads1256_init(ADS1256_t                 *ads1256, 
                 pfn_ads1256_io_t           read,
                 pfn_ads1256_io_t           write, 
                 ads1256_pin_t              cs_pin,
                 ads1256_pin_t              drdy_pin,
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
    ads1256->cs_pin = cs_pin;
    ads1256->drdy_pin = drdy_pin;
    ads1256->acquisition_running = false;
    ads1256->cycle = 0;
    ads1256->pga = ADS1256_PGA_1;
    ads1256->vref = ADS1256_DEFAULT_VREF;
    __ads1256_update_conversion_parameter(ads1256);
    ads1256->is_init = true;
    return 0;
}

int ads1256_start_sync_conv(ADS1256_t *ads1256)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }
    int ret;
    ret = ads1256_sync(ads1256);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256_wakeup(ads1256);
    if (ret < 0) {
        return ret;
    }
    return ret;
}

int ads1256_cycle_single(ADS1256_t *ads1256, int32_t *p_data, ads1256_ain_t *channel)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL) {
        return -1;
    }

    uint8_t index = ads1256->cycle % 8U;
    int ret = ads1256_write_reg_raw(ads1256, ADS1256_REG_MUX, ads1256_single_mux[index]);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_start_sync_conv(ads1256);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_read_single(ads1256, p_data);
    if (ret < 0) {
        return ret;
    }

    if (channel != NULL) {
        *channel = (ads1256_ain_t)index;
    }

    ads1256->cycle = (uint8_t)((index + 1U) % 8U);
    return 0;
}

int ads1256_cycle_differential(ADS1256_t *ads1256, int32_t *p_data, ads1256_ain_t *ainp, ads1256_ain_t *ainn)
{
    if (!__ads1256_is_valid(ads1256) || p_data == NULL) {
        return -1;
    }

    uint8_t index = ads1256->cycle % 4U;
    int ret = ads1256_write_reg_raw(ads1256, ADS1256_REG_MUX, ads1256_diff_mux[index]);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_start_sync_conv(ads1256);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_read_single(ads1256, p_data);
    if (ret < 0) {
        return ret;
    }

    if (ainp != NULL) {
        *ainp = ads1256_diff_p[index];
    }
    if (ainn != NULL) {
        *ainn = ads1256_diff_n[index];
    }

    ads1256->cycle = (uint8_t)((index + 1U) % 4U);
    return 0;
}

int ads1256_set_vref(ADS1256_t *ads1256, float vref)
{
    if (!__ads1256_is_valid(ads1256) || vref <= 0.0f) {
        return -1;
    }

    ads1256->vref = vref;
    __ads1256_update_conversion_parameter(ads1256);
    return 0;
}

float ads1256_convert_to_voltage(ADS1256_t *ads1256, int32_t raw_data)
{
    if (!__ads1256_is_valid(ads1256)) {
        return 0.0f;
    }

    return ads1256->conversion_parameter * (float)raw_data;
}

int ads1256_initialize_default(ADS1256_t *ads1256, float vref)
{
    if (!__ads1256_is_valid(ads1256)) {
        return -1;
    }

    int ret = ads1256_set_vref(ads1256, vref);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_reset(ads1256);
    if (ret < 0) {
        return ret;
    }

    uint8_t status = 0x36U;
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_STATUS, &status, 1);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_write_reg_raw(ads1256, ADS1256_REG_MUX, ADS1256_DIFF_0_1);
    if (ret < 0) {
        return ret;
    }

    uint8_t adcon = 0x00U;
    ret = __ads1256_write_reg(ads1256, ADS1256_REG_ADCON, &adcon, 1);
    if (ret < 0) {
        return ret;
    }
    ads1256->pga = ADS1256_PGA_1;
    __ads1256_update_conversion_parameter(ads1256);

    ret = ads1256_set_sps(ads1256, ADS1256_SPS_100);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_calibration(ads1256, ADS1256_CAL_SELF);
    if (ret < 0) {
        return ret;
    }

    ads1256->cycle = 0;
    ads1256->acquisition_running = false;
    return 0;
}
