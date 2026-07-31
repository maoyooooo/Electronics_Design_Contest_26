#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor.h"

volatile float angle_Z;
float Motor1_Speed;
float Motor2_Speed;

static uint32 pwm_duties[2];

void gpio_init(int pin, int mode, int level, int config)
{
    (void)pin;
    (void)mode;
    (void)level;
    (void)config;
}

void gpio_set_level(int pin, int level)
{
    (void)pin;
    (void)level;
}

void pwm_init(int channel, uint32 frequency, uint32 duty)
{
    (void)frequency;
    pwm_duties[channel] = duty;
}

void pwm_set_duty(int channel, uint32 duty)
{
    pwm_duties[channel] = duty;
}

void system_delay_ms(uint32 time)
{
    (void)time;
}

void MEASURE_MOTORS_SPEED(void)
{
}

grayscale_status_t Grayscale_Read(grayscale_frame_t *frame)
{
    *frame = (grayscale_frame_t){0};
    return GRAYSCALE_STATUS_OK;
}

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

static void reset_fixture(void)
{
    Motor1_Speed = 0.0f;
    Motor2_Speed = 0.0f;
    current_L_speed = 0.0f;
    current_R_speed = 0.0f;
    pwm_duties[MOTOR1_PWM] = 0U;
    pwm_duties[MOTOR2_PWM] = 0U;
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_DEFAULT);
    Motor_SpeedPid_ResetActiveProfile();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION2);
    Motor_SpeedPid_ResetActiveProfile();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION4);
    Motor_SpeedPid_ResetActiveProfile();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION5);
    Motor_SpeedPid_ResetActiveProfile();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION6);
    Motor_SpeedPid_ResetActiveProfile();
}

static void test_profiles_keep_speed_loop_state_isolated(void)
{
    reset_fixture();

    if (!Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION2))
    {
        fail("question 2 PID profile was rejected");
    }
    Motor_SpeedLoop(120.0f, 130.0f);
    expect_close(pid_Question2_Motor1_Speed.Target, 120.0f,
                 "question 2 left PID did not receive its target");
    expect_close(pid_Question2_Motor2_Speed.Target, 130.0f,
                 "question 2 right PID did not receive its target");
    expect_close(pid_Motor1_Speed.Target, 0.0f,
                 "default left PID was changed by question 2");

    if (!Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION4))
    {
        fail("question 4 PID profile was rejected");
    }
    Motor_SpeedLoop(210.0f, 220.0f);
    expect_close(pid_Question4_Motor1_Speed.Target, 210.0f,
                 "question 4 left PID did not receive its target");
    expect_close(pid_Question4_Motor2_Speed.Target, 220.0f,
                 "question 4 right PID did not receive its target");
    expect_close(pid_Question2_Motor1_Speed.Target, 120.0f,
                 "question 4 overwrote question 2 left PID state");
}

static void test_stop_resets_only_the_selected_profile(void)
{
    reset_fixture();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION2);
    Motor_SpeedLoop(100.0f, 100.0f);
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION5);
    Motor_SpeedLoop(150.0f, 150.0f);
    Motor_Stop();

    expect_close(pid_Question5_Motor1_Speed.Target, 0.0f,
                 "stop did not reset the active question 5 left PID");
    expect_close(pid_Question5_Motor2_Speed.Target, 0.0f,
                 "stop did not reset the active question 5 right PID");
    expect_close(pid_Question2_Motor1_Speed.Target, 100.0f,
                 "stop cleared an inactive question 2 PID");
}

static void test_question5_and_question6_profiles_route_to_own_pid_pairs(void)
{
    reset_fixture();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION5);
    Motor_SpeedLoop(150.0f, 160.0f);
    expect_close(pid_Question5_Motor1_Speed.Target, 150.0f,
                 "question 5 left PID did not receive its target");
    expect_close(pid_Question5_Motor2_Speed.Target, 160.0f,
                 "question 5 right PID did not receive its target");

    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION6);
    Motor_SpeedLoop(170.0f, 180.0f);
    expect_close(pid_Question6_Motor1_Speed.Target, 170.0f,
                 "question 6 left PID did not receive its target");
    expect_close(pid_Question6_Motor2_Speed.Target, 180.0f,
                 "question 6 right PID did not receive its target");
    expect_close(pid_Question5_Motor1_Speed.Target, 150.0f,
                 "question 6 overwrote question 5 left PID state");
}

static void test_invalid_profile_does_not_change_active_profile(void)
{
    reset_fixture();
    Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION6);

    if (Motor_SpeedPid_SelectProfile((motor_speed_pid_profile_t)99))
    {
        fail("invalid PID profile was accepted");
    }

    Motor_SpeedLoop(240.0f, 250.0f);
    expect_close(pid_Question6_Motor1_Speed.Target, 240.0f,
                 "invalid selection changed the active PID profile");
    expect_close(pid_Question6_Motor2_Speed.Target, 250.0f,
                 "invalid selection changed the active right PID profile");
}

int main(void)
{
    Motor_Init();
    test_profiles_keep_speed_loop_state_isolated();
    test_stop_resets_only_the_selected_profile();
    test_question5_and_question6_profiles_route_to_own_pid_pairs();
    test_invalid_profile_does_not_change_active_profile();
    puts("PASS: speed PID profile host tests");
    return EXIT_SUCCESS;
}
