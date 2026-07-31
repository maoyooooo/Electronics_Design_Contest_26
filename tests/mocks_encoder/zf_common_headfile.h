#ifndef TEST_ENCODER_ZF_COMMON_HEADFILE_H
#define TEST_ENCODER_ZF_COMMON_HEADFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define TIM_G8                          (8)
#define TIMG8_ENCODER1_CH1_A26          (26)
#define TIMG8_ENCODER1_CH2_A27          (27)
#define B17                             (17)
#define B18                             (18)
#define EXTI_TRIGGER_BOTH               (0)

typedef void (*encoder_test_exti_callback_t)(uint32 event, void *ptr);

void encoder_quad_init(int timer, int a_pin, int b_pin);
int16 encoder_get_count(int timer);
void encoder_clear_count(int timer);
void exti_init(int pin, int trigger, encoder_test_exti_callback_t callback,
               void *ptr);
uint8 gpio_get_level(int pin);
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 primask);

#include "encoder.h"

#endif
