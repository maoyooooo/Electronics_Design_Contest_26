#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor.h"

#ifndef TEST_PLANT_YAW_SIGN
#define TEST_PLANT_YAW_SIGN (1.0f)
#endif

volatile float angle_Z = 0.0f;
float Motor1_Speed = 0.0f;
float Motor2_Speed = 0.0f;

static int gpio_levels[32];
static uint32 pwm_duties[2];
static uint32 delay_calls;
static uint32 nonzero_pwm_writes;
static uint32 invalidate_angle_after_delays;
static bool freeze_angle;
static bool use_inertia;
static bool saw_zero_after_motion;
static bool resumed_after_coast;
static float angular_velocity;
static uint32 zero_commands_after_resume;
static uint32 measure_speed_calls;
static float plant_left_speed;
static float plant_right_speed;

static void fail(const char *message)
{
    fprintf(stderr, "FAIL: %s\n", message);
    exit(EXIT_FAILURE);
}

void gpio_init(int pin, int mode, int level, int config)
{
    (void)mode;
    (void)config;
    gpio_levels[pin] = level;
}

void gpio_set_level(int pin, int level)
{
    gpio_levels[pin] = level;
}

void pwm_init(int channel, uint32 frequency, uint32 duty)
{
    (void)frequency;
    pwm_duties[channel] = duty;
}

void pwm_set_duty(int channel, uint32 duty)
{
    pwm_duties[channel] = duty;
    if (duty != 0U)
    {
        ++nonzero_pwm_writes;
    }

    if ((channel == MOTOR2_PWM) && resumed_after_coast)
    {
        if ((pwm_duties[MOTOR1_PWM] == 0U) &&
            (pwm_duties[MOTOR2_PWM] == 0U))
        {
            ++zero_commands_after_resume;
        }
        else
        {
            zero_commands_after_resume = 0U;
        }
    }
}

void MEASURE_MOTORS_SPEED(void)
{
    const float pwm_to_speed = 0.04f;
    float left_pwm = (float)pwm_duties[MOTOR1_PWM];
    float right_pwm = (float)pwm_duties[MOTOR2_PWM];

    if (gpio_levels[MOTOR1_DIR] == GPIO_LOW)
    {
        left_pwm = -left_pwm;
    }
    if (gpio_levels[MOTOR2_DIR] == GPIO_LOW)
    {
        right_pwm = -right_pwm;
    }

    plant_left_speed += (left_pwm * pwm_to_speed - plant_left_speed) * 0.5f;
    plant_right_speed += (right_pwm * pwm_to_speed - plant_right_speed) * 0.5f;
    Motor1_Speed = plant_left_speed;
    Motor2_Speed = plant_right_speed;
    ++measure_speed_calls;
}

void system_delay_ms(uint32 time)
{
    const float degrees_per_duty_ms = 0.00008f;
    float direction = 0.0f;
    uint32 combined_duty;

    ++delay_calls;
    Motor_UpdateRpmFromEncoder();

    if ((invalidate_angle_after_delays != 0U) &&
        (delay_calls >= invalidate_angle_after_delays))
    {
        angle_Z = NAN;
        return;
    }

    if (freeze_angle)
    {
        return;
    }

    if ((gpio_levels[MOTOR1_DIR] == GPIO_HIGH) &&
        (gpio_levels[MOTOR2_DIR] == GPIO_LOW))
    {
        direction = 1.0f;
    }
    else if ((gpio_levels[MOTOR1_DIR] == GPIO_LOW) &&
             (gpio_levels[MOTOR2_DIR] == GPIO_HIGH))
    {
        direction = -1.0f;
    }

    direction *= TEST_PLANT_YAW_SIGN;
    combined_duty = pwm_duties[MOTOR1_PWM] + pwm_duties[MOTOR2_PWM];

    if (use_inertia)
    {
        if ((combined_duty == 0U) && (nonzero_pwm_writes != 0U) &&
            !saw_zero_after_motion)
        {
            saw_zero_after_motion = true;
            angular_velocity = (angular_velocity < 0.0f) ? -3.0f : 3.0f;
        }
        else if ((combined_duty != 0U) && saw_zero_after_motion)
        {
            resumed_after_coast = true;
        }

        angular_velocity = angular_velocity * 0.92f +
                           direction * (float)combined_duty * 0.00004f;
        angle_Z += angular_velocity;
        return;
    }

    angle_Z += direction * (float)combined_duty *
               degrees_per_duty_ms * (float)time;
}

static void reset_fixture(float initial_angle)
{
    angle_Z = initial_angle;
    delay_calls = 0;
    nonzero_pwm_writes = 0;
    invalidate_angle_after_delays = 0U;
    freeze_angle = false;
    use_inertia = false;
    saw_zero_after_motion = false;
    resumed_after_coast = false;
    angular_velocity = 0.0f;
    zero_commands_after_resume = 0U;
    measure_speed_calls = 0U;
    plant_left_speed = 0.0f;
    plant_right_speed = 0.0f;
    current_L_speed = 0.0f;
    current_R_speed = 0.0f;
    gpio_levels[MOTOR1_DIR] = GPIO_HIGH;
    gpio_levels[MOTOR2_DIR] = GPIO_HIGH;
    pwm_duties[MOTOR1_PWM] = 0;
    pwm_duties[MOTOR2_PWM] = 0;
    Set_PID_Param(&pid_Turn, 0.0f, 0.0f, 0.0f);
    Set_PID_Param(&pid_Angle, 3.1f, 0.0f, 0.0f);
    Set_PID_Param(&pid_Motor1_Speed, 17.0f, 0.0f, 0.0f);
    Set_PID_Param(&pid_Motor2_Speed, 17.0f, 0.0f, 0.0f);
}

static void expect_stopped(void)
{
    if ((pwm_duties[MOTOR1_PWM] != 0U) ||
        (pwm_duties[MOTOR2_PWM] != 0U))
    {
        fail("motors must be stopped when Turn_Control returns");
    }
}

static void test_speed_sampling_is_separate_from_speed_control(void)
{
    reset_fixture(0.0f);

    Motor_SpeedLoop(100.0f, -100.0f);
    if (measure_speed_calls != 0U)
    {
        fail("Motor_SpeedLoop must not read and clear the encoders");
    }

    Motor_UpdateRpmFromEncoder();
    if (measure_speed_calls != 1U)
    {
        fail("TIMG6 speed sampling entry point did not update the encoders");
    }
}

static void test_positive_relative_turn(void)
{
    reset_fixture(35.0f);

    if (!Turn_Control(90.0f))
    {
        fail("positive relative turn unexpectedly timed out");
    }
    if (fabsf(angle_Z - 125.0f) > TURN_ANGLE_TOLERANCE_DEG)
    {
        fail("positive turn did not use the current angle as its origin");
    }
    if (delay_calls < 2U)
    {
        fail("Turn_Control returned without completing the turn");
    }
    expect_stopped();
}

static void test_negative_relative_turn(void)
{
    reset_fixture(-20.0f);

    if (!Turn_Control(-90.0f))
    {
        fail("negative relative turn unexpectedly timed out");
    }
    if (fabsf(angle_Z - (-110.0f)) > TURN_ANGLE_TOLERANCE_DEG)
    {
        fail("negative relative turn ended at the wrong angle");
    }
    expect_stopped();
}

static void test_large_relative_turn_keeps_requested_direction(void)
{
    reset_fixture(10.0f);

    if (!Turn_Control(270.0f))
    {
        fail("270 degree relative turn unexpectedly timed out");
    }
    if (fabsf(angle_Z - 280.0f) > TURN_ANGLE_TOLERANCE_DEG)
    {
        fail("270 degree turn was wrapped to the opposite direction");
    }
    expect_stopped();
}

static void test_turn_crosses_wrapped_yaw_boundary(void)
{
    reset_fixture(170.0f);

    if (!Turn_Control(30.0f))
    {
        fail("turn across 180 degrees unexpectedly timed out");
    }
    if (fabsf(angle_Z - 200.0f) > TURN_ANGLE_TOLERANCE_DEG)
    {
        fail("continuous angle was incorrectly wrapped at 180 degrees");
    }
    expect_stopped();
}

static void test_small_relative_turn_does_not_drive_motors(void)
{
    reset_fixture(42.0f);

    if (!Turn_Control(TURN_ANGLE_TOLERANCE_DEG * 0.5f))
    {
        fail("turn already inside the tolerance unexpectedly timed out");
    }
    if (nonzero_pwm_writes != 0U)
    {
        fail("turn inside the tolerance must not drive the motors");
    }
    expect_stopped();
}

static void test_sensor_stall_times_out_and_stops(void)
{
    reset_fixture(0.0f);
    freeze_angle = true;

    if (Turn_Control(90.0f))
    {
        fail("stalled angle source must report a timeout");
    }
    if (delay_calls != (TURN_TIMEOUT_MS / TURN_CONTROL_PERIOD_MS))
    {
        fail("timeout duration does not match the configured limit");
    }
    expect_stopped();
}

static void test_invalid_relative_angles_fail_immediately(void)
{
    const float invalid_angles[] = {
        NAN,
        INFINITY,
        TURN_MAX_RELATIVE_ANGLE_DEG + 1.0f
    };
    uint32 index;

    for (index = 0U;
         index < (uint32)(sizeof(invalid_angles) / sizeof(invalid_angles[0]));
         ++index)
    {
        reset_fixture(0.0f);
        if (Turn_Control(invalid_angles[index]))
        {
            fail("invalid relative angle must not report success");
        }
        if ((delay_calls != 0U) || (nonzero_pwm_writes != 0U))
        {
            fail("invalid relative angle must fail without driving");
        }
        expect_stopped();
    }
}

static void test_angle_source_failure_during_turn_stops(void)
{
    reset_fixture(0.0f);
    invalidate_angle_after_delays = 3U;

    if (Turn_Control(90.0f))
    {
        fail("angle source failure during a turn must report failure");
    }
    if (delay_calls != invalidate_angle_after_delays)
    {
        fail("runtime angle failure was not detected on the next control step");
    }
    expect_stopped();
}

static void test_inertia_can_reset_the_settle_window(void)
{
    reset_fixture(0.0f);
    use_inertia = true;

    if (!Turn_Control(45.0f))
    {
        fail("inertial plant unexpectedly timed out");
    }
    if (!resumed_after_coast)
    {
        fail("controller did not resume after inertia carried it out of tolerance");
    }
    if (zero_commands_after_resume != (TURN_SETTLE_SAMPLE_COUNT + 1U))
    {
        fail("settle window was not restarted after control resumed");
    }
    if (fabsf(angle_Z - 45.0f) > TURN_ANGLE_TOLERANCE_DEG)
    {
        fail("inertial plant did not settle inside the angle tolerance");
    }
    expect_stopped();
}

static void test_invalid_angle_source_fails_immediately(void)
{
    reset_fixture(NAN);

    if (Turn_Control(90.0f))
    {
        fail("invalid angle source must not report success");
    }
    if (delay_calls != 0U)
    {
        fail("invalid angle source must fail immediately");
    }
    if (nonzero_pwm_writes != 0U)
    {
        fail("invalid angle source must not drive the motors");
    }
    expect_stopped();
}

int main(void)
{
    Motor_Init();
    test_speed_sampling_is_separate_from_speed_control();
    test_positive_relative_turn();
    test_negative_relative_turn();
    test_large_relative_turn_keeps_requested_direction();
    test_turn_crosses_wrapped_yaw_boundary();
    test_small_relative_turn_does_not_drive_motors();
    test_sensor_stall_times_out_and_stops();
    test_invalid_relative_angles_fail_immediately();
    test_angle_source_failure_during_turn_stops();
    test_inertia_can_reset_the_settle_window();
    test_invalid_angle_source_fails_immediately();
    puts("PASS: Turn_Control host tests");
    return EXIT_SUCCESS;
}
