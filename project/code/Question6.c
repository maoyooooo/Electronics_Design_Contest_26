#include "zf_common_headfile.h"
#include "Question6.h"

typedef struct
{
    volatile uint32 elapsed_ms;
    uint32 last_display_ms;
    float distance_mm;
    volatile bool stopped;
    bool display_pending;
} question6_state_t;

static question6_state_t question6_state;

void Question6_Init(void)
{
    encoder_reset_distance();
    Line_Follow_ResetState();
    (void)Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION6);
    Motor_SpeedPid_ResetActiveProfile();
    question6_state.elapsed_ms = 0U;
    question6_state.last_display_ms = 0U;
    question6_state.distance_mm = 0.0f;
    question6_state.stopped = !Line_Follow_SetBaseSpeed(QUESTION6_TARGET_SPEED_MM_S);
    question6_state.display_pending = true;

    if (question6_state.stopped)
    {
        Motor_Stop();
    }
}

void Question6_Update(void)
{
    encoder_distance_t distance;

    if (question6_state.stopped)
    {
        Motor_Stop();
        return;
    }

    if (!encoder_get_distance(&distance))
    {
        question6_state.stopped = true;
        question6_state.display_pending = true;
        Motor_Stop();
        return;
    }

    question6_state.distance_mm = (distance.left_mm + distance.right_mm) * 0.5f;

    if (question6_state.distance_mm >= QUESTION6_LAP_DISTANCE_MM)
    {
        question6_state.stopped = true;
        question6_state.display_pending = true;
        Motor_Stop();
        return;
    }

    Line_Follow_Control();
}

void Question6_TimerTick10ms(void)
{
    if (!question6_state.stopped)
    {
        question6_state.elapsed_ms += QUESTION6_TIME_TICK_MS;
    }
}

void Question6_Display(void)
{
    uint32 elapsed_ms = question6_state.elapsed_ms;

    if (!question6_state.display_pending &&
        (elapsed_ms - question6_state.last_display_ms) < QUESTION6_DISPLAY_PERIOD_MS)
    {
        return;
    }

    ips200_show_string(10U, 10U, "D:");
    ips200_show_float(42U, 10U, question6_state.distance_mm, 5U, 1U);
    ips200_show_string(10U, 36U, "T:");
    ips200_show_float(42U, 36U, (double)elapsed_ms / 1000.0, 3U, 2U);
    ips200_show_string(10U, 62U, "V:");
    ips200_show_float(42U, 62U, QUESTION6_TARGET_SPEED_MM_S, 3U, 1U);
    ips200_show_string(10U, 88U, "VL:");
    ips200_show_float(42U, 88U, Motor1_Speed, 3U, 1U);
    ips200_show_string(10U, 114U, "VR:");
    ips200_show_float(42U, 114U, Motor2_Speed, 3U, 1U);
    ips200_show_string(10U, 140U, question6_state.stopped ? "STOP" : "RUN ");

    question6_state.last_display_ms = elapsed_ms;
    question6_state.display_pending = false;
}

float Question6_GetDistanceMm(void)
{
    return question6_state.distance_mm;
}

uint32 Question6_GetElapsedMs(void)
{
    return question6_state.elapsed_ms;
}

bool Question6_IsStopped(void)
{
    return question6_state.stopped;
}
