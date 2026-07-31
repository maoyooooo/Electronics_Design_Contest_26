#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "encoder.h"

static int16 left_hardware_count;
static uint8 gpio_levels[32];
static encoder_test_exti_callback_t right_a_callback;

static void fail(const char *message)
{
    fprintf(stderr, "FAIL: %s\n", message);
    exit(EXIT_FAILURE);
}

static void expect_close(float actual, float expected, const char *message)
{
    if (fabsf(actual - expected) > 0.001f)
    {
        fail(message);
    }
}

void encoder_quad_init(int timer, int a_pin, int b_pin)
{
    (void)timer;
    (void)a_pin;
    (void)b_pin;
}

int16 encoder_get_count(int timer)
{
    (void)timer;
    return left_hardware_count;
}

void encoder_clear_count(int timer)
{
    (void)timer;
    left_hardware_count = 0;
}

void exti_init(int pin, int trigger, encoder_test_exti_callback_t callback,
               void *ptr)
{
    (void)trigger;
    (void)ptr;

    if (pin == ENCODER_R_A_PIN)
    {
        right_a_callback = callback;
    }
}

uint8 gpio_get_level(int pin)
{
    return gpio_levels[pin];
}

uint32 interrupt_global_disable(void)
{
    return 0U;
}

void interrupt_global_enable(uint32 primask)
{
    (void)primask;
}

static void set_right_encoder_state(uint8 state)
{
    gpio_levels[ENCODER_R_A_PIN] = (uint8)(state >> 1);
    gpio_levels[ENCODER_R_B_PIN] = (uint8)(state & 1U);

    if (right_a_callback != NULL)
    {
        right_a_callback(0U, NULL);
    }
}

static void add_right_forward_counts(uint16 cycles)
{
    uint16 index;

    for (index = 0U; index < cycles; ++index)
    {
        set_right_encoder_state(2U);
        set_right_encoder_state(3U);
        set_right_encoder_state(1U);
        set_right_encoder_state(0U);
    }
}

static void add_right_reverse_counts(uint16 cycles)
{
    uint16 index;

    for (index = 0U; index < cycles; ++index)
    {
        set_right_encoder_state(1U);
        set_right_encoder_state(3U);
        set_right_encoder_state(2U);
        set_right_encoder_state(0U);
    }
}

static void test_one_wheel_revolution_matches_its_circumference(void)
{
    encoder_distance_t distance;

    left_hardware_count = (int16)-ENCODER_COUNTS_PER_WHEEL_REV;
    add_right_forward_counts((uint16)(ENCODER_COUNTS_PER_WHEEL_REV / 4.0f));
    MEASURE_MOTORS_SPEED();
    encoder_get_distance(&distance);

    expect_close(distance.left_mm, ENCODER_WHEEL_CIRCUMFERENCE_MM,
                 "left wheel one revolution must equal its circumference");
    expect_close(distance.right_mm, ENCODER_WHEEL_CIRCUMFERENCE_MM,
                 "right wheel one revolution must equal its circumference");
}

static void test_circumference_uses_the_existing_wheel_diameter(void)
{
    expect_close(ENCODER_WHEEL_CIRCUMFERENCE_MM, PI * RR,
                 "distance circumference must use the existing wheel diameter");
}

static void test_reverse_distance_is_negative(void)
{
    encoder_distance_t distance;

    encoder_reset_distance();
    left_hardware_count = (int16)ENCODER_COUNTS_PER_WHEEL_REV;
    add_right_reverse_counts((uint16)(ENCODER_COUNTS_PER_WHEEL_REV / 4.0f));
    MEASURE_MOTORS_SPEED();
    encoder_get_distance(&distance);

    expect_close(distance.left_mm, -ENCODER_WHEEL_CIRCUMFERENCE_MM,
                 "left reverse distance must be negative");
    expect_close(distance.right_mm, -ENCODER_WHEEL_CIRCUMFERENCE_MM,
                 "right reverse distance must be negative");
}

static void test_reset_keeps_future_distance_measurements(void)
{
    encoder_distance_t distance;

    encoder_reset_distance();
    encoder_get_distance(&distance);
    expect_close(distance.left_mm, 0.0f, "left reset must produce zero");
    expect_close(distance.right_mm, 0.0f, "right reset must produce zero");

    left_hardware_count = (int16)-ENCODER_COUNTS_PER_WHEEL_REV;
    add_right_forward_counts((uint16)(ENCODER_COUNTS_PER_WHEEL_REV / 4.0f));
    MEASURE_MOTORS_SPEED();
    encoder_get_distance(&distance);

    expect_close(distance.left_mm, ENCODER_WHEEL_CIRCUMFERENCE_MM,
                 "left distance must accumulate after reset");
    expect_close(distance.right_mm, ENCODER_WHEEL_CIRCUMFERENCE_MM,
                 "right distance must accumulate after reset");
}

static void test_distance_reader_rejects_null_output(void)
{
    if (encoder_get_distance(NULL))
    {
        fail("distance reader must reject a null output pointer");
    }
}

int main(void)
{
    encoder_init();
    test_circumference_uses_the_existing_wheel_diameter();
    test_one_wheel_revolution_matches_its_circumference();
    test_reverse_distance_is_negative();
    test_reset_keeps_future_distance_measurements();
    test_distance_reader_rejects_null_output();

    puts("encoder distance tests passed");
    return EXIT_SUCCESS;
}
