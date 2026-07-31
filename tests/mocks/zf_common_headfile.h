#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define B8                         (8)
#define B9                         (9)
#define B12                        (12)
#define B13                        (13)
#define PWM_TIM_A0_CH0_B8          (0)
#define PWM_TIM_A0_CH2_B12         (1)
#define GPO                        (0)
#define GPO_PUSH_PULL              (0)
#define GPIO_LOW                   (0)
#define GPIO_HIGH                  (1)

extern volatile float angle_Z;
extern float Motor1_Speed;
extern float Motor2_Speed;

void gpio_init(int pin, int mode, int level, int config);
void gpio_set_level(int pin, int level);
void pwm_init(int channel, uint32 frequency, uint32 duty);
void pwm_set_duty(int channel, uint32 duty);
void system_delay_ms(uint32 time);
void MEASURE_MOTORS_SPEED(void);

#include "motor.h"

#endif
