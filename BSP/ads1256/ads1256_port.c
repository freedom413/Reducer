#include "ads1256.h"
#include "ads1256_config.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>
#include "delay.h"

#define ADS1256_SPI_TIMEOUT_MS  10U
static int ads1256_write(uint8_t *p_data, uint8_t nbytes)
{
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi1, p_data, nbytes, ADS1256_SPI_TIMEOUT_MS);
    return (ret == HAL_OK) ? (int)nbytes : -1;
}

static int ads1256_read(uint8_t *p_data, uint8_t nbytes)
{
    uint8_t dummy[16];
    if (nbytes > sizeof(dummy)) {
        return -1;
    }
    for (uint8_t i = 0; i < nbytes; i++) {
        dummy[i] = 0xFFU;
    }

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, dummy, p_data,
                                                    nbytes,
                                                    ADS1256_SPI_TIMEOUT_MS);

    return (ret == HAL_OK) ? (int)nbytes : -1;
}

static int ads1256_delay_us(uint32_t us)
{
    delay_us(us);
    return 0;
}

static void ads1256_all_cs_high(void)
{
    HAL_GPIO_WritePin(ADC1_CS_GPIO_Port, ADC1_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ADC2_CS_GPIO_Port, ADC2_CS_Pin, GPIO_PIN_SET);
}

static int ads1256_pin_op(ads1256_pin_t pin, ads1256_pin_op_t op)
{
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio_port;

    switch (pin) {
        case ADS1256_Pin_CS_A:
            gpio_pin = ADC1_CS_Pin;
            gpio_port = ADC1_CS_GPIO_Port;
            break;
        case ADS1256_Pin_CS_B:
            gpio_pin = ADC2_CS_Pin;
            gpio_port = ADC2_CS_GPIO_Port;
            break;
        case ADS1256_Pin_DRDY_A:
            gpio_pin = ADC1_DRDY_Pin;
            gpio_port = ADC1_DRDY_GPIO_Port;
            break;
        case ADS1256_Pin_DRDY_B:
            gpio_pin = ADC2_DRDY_Pin;
            gpio_port = ADC2_DRDY_GPIO_Port;
            break;
        case ADS1256_Pin_RST:
        case ADS1256_Pin_SYNC:
            /*
             * RESET and SYNC are shared between the two ADS1256 devices on this
             * board. Report them as unsupported so the generic driver uses
             * per-chip SPI commands under CS instead of disturbing the other ADC.
             */
            return 2;
        default:
            return 2;
    }

    switch (op) {
        case ADS1256_PIN_OP_HIGH:
            HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
            break;
        case ADS1256_PIN_OP_LOW:
            HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
            break;
        case ADS1256_PIN_OP_READ:
            return (int)HAL_GPIO_ReadPin(gpio_port, gpio_pin);
        default:
            return -1;
    }

    return 0;
}

ADS1256_t ads1256_a;
ADS1256_t ads1256_b;

static int ads1256_config_one(ADS1256_t *ads1256)
{
    ads1256_pga_t pga;
    ads1256_sps_t sps;
    int ret;

    ret = ads1256_reset(ads1256);
    if (ret < 0) {
        return ret;
    }

    ret = ads1256_set_pga(ads1256, ADS1256_PGA_16);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256_get_pga(ads1256, &pga);
    if (ret < 0) {
        return ret;
    }

    if (pga != ADS1256_PGA_16) {
        return -11;
    }

    ret = ads1256_set_sps(ads1256, ADS1256_SPS_100);
    if (ret < 0) {
        return ret;
    }
    ret = ads1256_get_sps(ads1256, &sps);
    if (ret < 0) {
        return ret;
    }
    if (sps != ADS1256_SPS_100) {
        return -12;
    }

    return ads1256_calibration(ads1256, ADS1256_CAL_SELF);
}

int adc_ads1256_init(void)
{
    int ret;

    ads1256_init(&ads1256_a,
                 ads1256_read,
                 ads1256_write,
                 ADS1256_Pin_CS_A,
                 ADS1256_Pin_DRDY_A,
                 ads1256_pin_op,
                 ads1256_delay_us);

    ads1256_init(&ads1256_b,
                 ads1256_read,
                 ads1256_write,
                 ADS1256_Pin_CS_B,
                 ADS1256_Pin_DRDY_B,
                 ads1256_pin_op,
                 ads1256_delay_us);

    ads1256_all_cs_high();

#if ADS1256_ENABLE_A
    ret = ads1256_config_one(&ads1256_a);
    if (ret < 0) {
        return ret;
    }
#endif

#if ADS1256_ENABLE_B
    ret = ads1256_config_one(&ads1256_b);
    if (ret < 0) {
        return ret;
    }
#endif

    return 0;
}
