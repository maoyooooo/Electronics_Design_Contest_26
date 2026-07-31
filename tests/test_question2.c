#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Question2.h"

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

static void test_question2_tracks_distance_and_elapsed_time(void)
{
    uint8 index;

    reset_fixture();
    Question2_Init();
    if (encoder_reset_calls != 1U ||
        line_follow_reset_calls != 1U ||
        selected_speed_pid_profile != MOTOR_SPEED_PID_PROFILE_QUESTION2 ||
        speed_pid_reset_calls != 1U ||
        fabsf(configured_line_speed - QUESTION2_TARGET_SPEED_MM_S) > 0.001f ||
        Question2_IsStopped() ||
        Question2_GetElapsedMs() != 0U)
    {
        fail("question 2 initialization did not reset its run state");
    }

    encoder_distance.left_mm = 120.0f;
    encoder_distance.right_mm = 120.0f;
    Question2_Update();
    if (line_follow_calls != 1U || motor_stop_calls != 0U ||
        fabsf(Question2_GetDistanceMm() - 120.0f) > 0.001f)
    {
        fail("question 2 did not line-follow below the lap distance");
    }

    for (index = 0U; index < 10U; ++index)
    {
        Question2_TimerTick10ms();
    }
    if (Question2_GetElapsedMs() != 100U)
    {
        fail("question 2 elapsed time did not use 10 ms timer ticks");
    }
}

static void test_question2_latches_stop_and_freezes_time(void)
{
    uint8 index;

    reset_fixture();
    Question2_Init();
    for (index = 0U; index < 5U; ++index)
    {
        Question2_TimerTick10ms();
    }

    encoder_distance.left_mm = QUESTION2_LAP_DISTANCE_MM;
    encoder_distance.right_mm = QUESTION2_LAP_DISTANCE_MM;
    Question2_Update();
    if (!Question2_IsStopped() || motor_stop_calls != 1U || line_follow_calls != 0U)
    {
        fail("question 2 did not latch a stop at the lap distance");
    }

    for (index = 0U; index < 5U; ++index)
    {
        Question2_TimerTick10ms();
    }
    Question2_Update();
    if (Question2_GetElapsedMs() != 50U || motor_stop_calls != 2U ||
        line_follow_calls != 0U)
    {
        fail("question 2 stop did not freeze time and block line following");
    }
}

static void test_question2_displays_distance_and_seconds(void)
{
    uint8 index;

    reset_fixture();
    Question2_Init();
    encoder_distance.left_mm = 120.0f;
    encoder_distance.right_mm = 120.0f;
    Question2_Update();
    for (index = 0U; index < 10U; ++index)
    {
        Question2_TimerTick10ms();
    }

    Motor1_Speed = 321.4f;
    Motor2_Speed = 318.7f;

    Question2_Display();
    if (display_float_calls != 5U ||
        fabsf(displayed_float[0] - 120.0f) > 0.001f ||
        fabsf(displayed_float[1] - 0.1f) > 0.001f ||
        fabsf(displayed_float[2] - QUESTION2_TARGET_SPEED_MM_S) > 0.001f ||
        displayed_pointnum[2] != 1U ||
        fabsf(displayed_float[3] - Motor1_Speed) > 0.001f ||
        fabsf(displayed_float[4] - Motor2_Speed) > 0.001f ||
        displayed_pointnum[3] != 1U || displayed_pointnum[4] != 1U)
    {
        fail("question 2 display did not show real wheel speeds");
    }
}

int main(void)
{
    test_question2_tracks_distance_and_elapsed_time();
    test_question2_latches_stop_and_freezes_time();
    test_question2_displays_distance_and_seconds();
    puts("PASS: Question2 host tests");
    return EXIT_SUCCESS;
}
