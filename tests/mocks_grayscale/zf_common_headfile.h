#ifndef TEST_ZF_COMMON_HEADFILE_H
#define TEST_ZF_COMMON_HEADFILE_H

#include <stdio.h>

#include "zf_common_typedef.h"

typedef struct
{
    uint32 placeholder;
} ADC12_Regs;

extern ADC12_Regs mock_adc0;

#define ADC0                        (&mock_adc0)

#define B10                         (10)
#define B11                         (11)
#define B14                         (14)
#define B25                         (25)
#define ADC0_CH4_B25                (0x402019)

#define GPI                         (0)
#define GPO                         (1)
#define GPIO_LOW                    (0)
#define GPIO_HIGH                   (1)
#define GPI_ANAOG_IN                (2)
#define GPO_PUSH_PULL               (3)
#define ADC_12BIT                   (0)

void gpio_init(int pin, int direction, int level, int mode);
void gpio_set_level(int pin, int level);
void adc_init(int pin, int resolution);
uint16 adc_convert(int pin);
uint16 adc_mean_filter_convert(int pin, uint8 count);
void system_delay_us(uint32 time);
void system_delay_ms(uint32 time);
void DL_ADC12_setSampleTime0(ADC12_Regs *adc12, uint16 adcclks);
void ips200_show_string(uint16 x, uint16 y, const char dat[]);
void ips200_show_uint(uint16 x, uint16 y, uint32 dat, uint8 num);

#include "Grayscale_Sensor.h"

#endif
