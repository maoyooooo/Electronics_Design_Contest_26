#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Question4.h"

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

static void test_question4_initializes_speed_distance_and_time(void)
{
    uint8 index;

    reset_fixture();
    Question4_Init();
    if (encoder_reset_calls != 1U ||
        line_follow_reset_calls != 1U ||
        selected_speed_pid_profile != MOTOR_SPEED_PID_PROFILE_QUESTION4 ||
        speed_pid_reset_calls != 1U ||
        fabsf(configured_line_speed - QUESTION4_TARGET_SPEED_MM_S) > 0.001f ||
        Question4_IsStopped() ||
        Question4_GetElapsedMs() != 0U)
    {
        fail("question 4 initialization did not configure run state");
    }

    encoder_distance.left_mm = 300.0f;
    encoder_distance.right_mm = 300.0f;
    Question4_Update();
    if (line_follow_calls != 1U || motor_stop_calls != 0U ||
        fabsf(Question4_GetDistanceMm() - 300.0f) > 0.001f)
    {
        fail("question 4 did not line-follow below 1500 mm");
    }

    for (index = 0U; index < 10U; ++index)
    {
        Question4_TimerTick10ms();
    }
    if (Question4_GetElapsedMs() != 100U)
    {
        fail("question 4 elapsed time did not use the 10 ms timer tick");
    }
}

static void test_question4_latches_stop_at_target_distance(void)
{
    uint8 index;

    reset_fixture();
    Question4_Init();
    for (index = 0U; index < 7U; ++index)
    {
        Question4_TimerTick10ms();
    }

    encoder_distance.left_mm = QUESTION4_TARGET_DISTANCE_MM;
    encoder_distance.right_mm = QUESTION4_TARGET_DISTANCE_MM;
    Question4_Update();
    if (!Question4_IsStopped() || motor_stop_calls != 1U || line_follow_calls != 0U)
    {
        fail("question 4 did not lock stop at 1500 mm");
    }

    for (index = 0U; index < 4U; ++index)
    {
        Question4_TimerTick10ms();
    }
    Question4_Update();
    if (Question4_GetElapsedMs() != 70U || motor_stop_calls != 2U ||
        line_follow_calls != 0U)
    {
        fail("question 4 stop did not freeze time and block line following");
    }
}

static void test_question4_displays_distance_seconds_and_speed(void)
{
    uint8 index;

    reset_fixture();
    Question4_Init();
    encoder_distance.left_mm = 300.0f;
    encoder_distance.right_mm = 300.0f;
    Question4_Update();
    for (index = 0U; index < 10U; ++index)
    {
        Question4_TimerTick10ms();
    }

    Motor1_Speed = 322.8f;
    Motor2_Speed = 319.6f;

    Question4_Display();
    if (display_float_calls != 5U ||
        fabsf(displayed_float[0] - 300.0f) > 0.001f ||
        fabsf(displayed_float[1] - 0.1f) > 0.001f ||
        fabsf(displayed_float[2] - QUESTION4_TARGET_SPEED_MM_S) > 0.001f ||
        displayed_pointnum[2] != 1U ||
        fabsf(displayed_float[3] - Motor1_Speed) > 0.001f ||
        fabsf(displayed_float[4] - Motor2_Speed) > 0.001f ||
        displayed_pointnum[3] != 1U || displayed_pointnum[4] != 1U)
    {
        fail("question 4 display did not show real wheel speeds");
    }
}

int main(void)
{
    test_question4_initializes_speed_distance_and_time();
    test_question4_latches_stop_at_target_distance();
    test_question4_displays_distance_seconds_and_speed();
    puts("PASS: Question4 host tests");
    return EXIT_SUCCESS;
}
