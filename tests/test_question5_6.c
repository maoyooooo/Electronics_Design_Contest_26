#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Question5.h"
#include "Question6.h"

static encoder_distance_t encoder_distance;
static uint32 encoder_reset_calls;
static uint32 line_follow_reset_calls;
static uint32 line_follow_calls;
static uint32 motor_stop_calls;
static uint32 speed_pid_reset_calls;
static float configured_line_speed;
static motor_speed_pid_profile_t selected_speed_pid_profile;
static uint32 display_float_calls;
static float displayed_float[5];
static uint8 displayed_pointnum[5];
float Motor1_Speed;
float Motor2_Speed;

bool encoder_get_distance(encoder_distance_t *distance)
{
    *distance = encoder_distance;
    return true;
}

void encoder_reset_distance(void)
{
    ++encoder_reset_calls;
}

void Line_Follow_ResetState(void)
{
    ++line_follow_reset_calls;
}

bool Line_Follow_SetBaseSpeed(float base_speed_mm_s)
{
    configured_line_speed = base_speed_mm_s;
    return true;
}

void Line_Follow_Control(void)
{
    ++line_follow_calls;
}

void Motor_Stop(void)
{
    ++motor_stop_calls;
}

bool Motor_SpeedPid_SelectProfile(motor_speed_pid_profile_t profile)
{
    selected_speed_pid_profile = profile;
    return true;
}

void Motor_SpeedPid_ResetActiveProfile(void)
{
    ++speed_pid_reset_calls;
}

void ips200_show_string(uint16 x, uint16 y, const char dat[])
{
    (void)x;
    (void)y;
    (void)dat;
}

void ips200_show_float(uint16 x, uint16 y, double dat, uint8 num, uint8 pointnum)
{
    (void)x;
    (void)y;
    (void)num;
    (void)pointnum;

    if (display_float_calls < 5U)
    {
        displayed_float[display_float_calls] = (float)dat;
        displayed_pointnum[display_float_calls] = pointnum;
    }
    ++display_float_calls;
}

static void fail(const char *message)
{
    fprintf(stderr, "FAIL: %s\n", message);
    exit(EXIT_FAILURE);
}

static void reset_fixture(void)
{
    encoder_distance = (encoder_distance_t){0};
    encoder_reset_calls = 0U;
    line_follow_reset_calls = 0U;
    line_follow_calls = 0U;
    motor_stop_calls = 0U;
    speed_pid_reset_calls = 0U;
    configured_line_speed = 0.0f;
    selected_speed_pid_profile = MOTOR_SPEED_PID_PROFILE_DEFAULT;
    display_float_calls = 0U;
    displayed_float[0] = 0.0f;
    displayed_float[1] = 0.0f;
    displayed_float[2] = 0.0f;
    displayed_float[3] = 0.0f;
    displayed_float[4] = 0.0f;
    displayed_pointnum[0] = 0U;
    displayed_pointnum[1] = 0U;
    displayed_pointnum[2] = 0U;
    displayed_pointnum[3] = 0U;
    displayed_pointnum[4] = 0U;
    Motor1_Speed = 0.0f;
    Motor2_Speed = 0.0f;
}

static void test_question5_runs_one_lap_at_250_mm_s(void)
{
    reset_fixture();
    Question5_Init();
    if (encoder_reset_calls != 1U || line_follow_reset_calls != 1U ||
        selected_speed_pid_profile != MOTOR_SPEED_PID_PROFILE_QUESTION5 ||
        speed_pid_reset_calls != 1U ||
        fabsf(configured_line_speed - QUESTION5_TARGET_SPEED_MM_S) > 0.001f ||
        Question5_IsStopped())
    {
        fail("question 5 initialization did not configure 250 mm/s lap following");
    }

    encoder_distance.left_mm = QUESTION5_LAP_DISTANCE_MM - 1.0f;
    encoder_distance.right_mm = QUESTION5_LAP_DISTANCE_MM - 1.0f;
    Question5_Update();
    if (line_follow_calls != 1U || Question5_IsStopped())
    {
        fail("question 5 did not line-follow below the lap distance");
    }

    encoder_distance.left_mm = QUESTION5_LAP_DISTANCE_MM;
    encoder_distance.right_mm = QUESTION5_LAP_DISTANCE_MM;
    Question5_Update();
    if (!Question5_IsStopped() || motor_stop_calls != 1U)
    {
        fail("question 5 did not stop at the lap distance");
    }

    Motor1_Speed = 247.2f;
    Motor2_Speed = 246.5f;
    Question5_Display();
    if (display_float_calls != 5U || displayed_pointnum[2] != 1U ||
        fabsf(displayed_float[3] - Motor1_Speed) > 0.001f ||
        fabsf(displayed_float[4] - Motor2_Speed) > 0.001f ||
        displayed_pointnum[3] != 1U || displayed_pointnum[4] != 1U)
    {
        fail("question 5 display did not show real wheel speeds");
    }
}

static void test_question6_runs_one_lap_at_250_mm_s(void)
{
    uint8 index;

    reset_fixture();
    Question6_Init();
    if (encoder_reset_calls != 1U || line_follow_reset_calls != 1U ||
        selected_speed_pid_profile != MOTOR_SPEED_PID_PROFILE_QUESTION6 ||
        speed_pid_reset_calls != 1U ||
        fabsf(configured_line_speed - QUESTION6_TARGET_SPEED_MM_S) > 0.001f ||
        Question6_IsStopped())
    {
        fail("question 6 initialization did not configure 250 mm/s lap following");
    }

    for (index = 0U; index < 6U; ++index)
    {
        Question6_TimerTick10ms();
    }
    encoder_distance.left_mm = QUESTION6_LAP_DISTANCE_MM;
    encoder_distance.right_mm = QUESTION6_LAP_DISTANCE_MM;
    Question6_Update();
    for (index = 0U; index < 4U; ++index)
    {
        Question6_TimerTick10ms();
    }
    if (!Question6_IsStopped() || motor_stop_calls != 1U ||
        Question6_GetElapsedMs() != 60U)
    {
        fail("question 6 did not stop once and freeze elapsed time");
    }

    Motor1_Speed = 248.3f;
    Motor2_Speed = 245.1f;
    Question6_Display();
    if (display_float_calls != 5U ||
        fabsf(displayed_float[0] - QUESTION6_LAP_DISTANCE_MM) > 0.001f ||
        fabsf(displayed_float[1] - 0.06f) > 0.001f ||
        fabsf(displayed_float[2] - QUESTION6_TARGET_SPEED_MM_S) > 0.001f ||
        displayed_pointnum[2] != 1U ||
        fabsf(displayed_float[3] - Motor1_Speed) > 0.001f ||
        fabsf(displayed_float[4] - Motor2_Speed) > 0.001f ||
        displayed_pointnum[3] != 1U || displayed_pointnum[4] != 1U)
    {
        fail("question 6 display did not show real wheel speeds");
    }
}

int main(void)
{
    test_question5_runs_one_lap_at_250_mm_s();
    test_question6_runs_one_lap_at_250_mm_s();
    puts("PASS: Question5/Question6 host tests");
    return EXIT_SUCCESS;
}
