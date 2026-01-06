#include "ads1256.h"
#include "cmsis_gcc.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>
#include "delay.h"

static int ads1256_write(uint8_t *p_data, uint8_t nbytes)
{
    HAL_StatusTypeDef ret;
    ret = HAL_SPI_Transmit(&hspi1, p_data, nbytes, HAL_MAX_DELAY);
    if(ret != HAL_OK) {
        return -1;
    } else {
        return nbytes;
    }
}

static int ads1256_read(uint8_t *p_data, uint8_t nbytes)
{
    HAL_StatusTypeDef ret;
    ret = HAL_SPI_Receive(&hspi1, p_data, nbytes, HAL_MAX_DELAY);
    if(ret != HAL_OK) {
        return -1;
    } else {
        return nbytes;
    }
}

static int ads1256_delay_us(uint32_t us)
{
    delay_us(us);
    return 0;
}

static int ads1256_a_pin_op(ads1256_pin_t pin ,ads1256_pin_op_t op)
{
    uint16_t      gpio_pin;
    GPIO_TypeDef *gpio_port;
    switch (pin) {
        case ADS1256_Pin_CS:
            gpio_pin = ADC1_CS_Pin;
            gpio_port = ADC1_CS_GPIO_Port;
            break;
        case ADS1256_Pin_DRDY:
            gpio_pin = ADC1_DRDY_Pin;
            gpio_port = ADC1_DRDY_GPIO_Port;
            break;
        case ADS1256_Pin_RST:
            gpio_pin = ADC_RESET_Pin;
            gpio_port = ADC_RESET_GPIO_Port;
            break;
        // case ADS1256_Pin_SYNC:
        //     gpio_pin = ADC_SYNC_Pin;
        //     gpio_port = ADC_SYNC_GPIO_Port;
        //     break;
        default:
            // mcu未控制该引脚
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
            break;
        default:
            return -1;
    }
    return 0;
}

static int ads1256_b_pin_op(ads1256_pin_t pin ,ads1256_pin_op_t op)
{
    uint16_t      gpio_pin;
    GPIO_TypeDef *gpio_port;
    switch (pin) {
        case ADS1256_Pin_CS:
            gpio_pin = ADC2_CS_Pin;
            gpio_port = ADC2_CS_GPIO_Port;
            break;
        case ADS1256_Pin_DRDY:
            gpio_pin = ADC2_DRDY_Pin;
            gpio_port = ADC2_DRDY_GPIO_Port;
            break;
        case ADS1256_Pin_RST:
            gpio_pin = ADC_RESET_Pin;
            gpio_port = ADC_RESET_GPIO_Port;
            break;
        // case ADS1256_Pin_SYNC:
        //     gpio_pin = ADC_SYNC_Pin;
        //     gpio_port = ADC_SYNC_GPIO_Port;
        //     break;
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
            break;
        default:
            return -1;
    }
    return 0;
}

ADS1256_t ads1256_a;
ADS1256_t ads1256_b;

int adc_ads1256_init(void)
{
    int ret;
    ret = ads1256_init( &ads1256_a, 
                            ads1256_write, 
                           ads1256_read, 
                          ads1256_a_pin_op, 
                        ads1256_delay_us);

    if(ret != 0) {
        return ret;
    }

    ret = ads1256_init( &ads1256_b, 
                            ads1256_write, 
                           ads1256_read, 
                          ads1256_b_pin_op, 
                        ads1256_delay_us);
    if (ret != 0) {
        return ret;
    }
    
    // 复位
    ads1256_reset(&ads1256_a);
    ads1256_reset(&ads1256_b);
    // 配置放大增益
    ads1256_set_gpa(&ads1256_a, ADS1256_GPA_64);
    ads1256_set_gpa(&ads1256_b, ADS1256_GPA_64);
    // 配置采样速率
    ads1256_set_sps(&ads1256_a, ADS1256_SPS_3750);
    ads1256_set_sps(&ads1256_b, ADS1256_SPS_3750);
    // 自校准
    ads1256_calibration(&ads1256_a, ADS1256_CAL_SELF);
    ads1256_calibration(&ads1256_b, ADS1256_CAL_SELF);
    
    return 0;
}
