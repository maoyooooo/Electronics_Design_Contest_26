#include "zf_common_headfile.h"
#include "Question2.h"

typedef struct
{
    volatile uint32 elapsed_ms;
    uint32 last_display_ms;
    float distance_mm;
    volatile bool stopped;
    bool display_pending;
} question2_state_t;

static question2_state_t question2_state;

void Question2_Init(void)
{
    encoder_reset_distance();
    Line_Follow_ResetState();
    (void)Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION2);
    Motor_SpeedPid_ResetActiveProfile();
    question2_state.elapsed_ms = 0U;
    question2_state.last_display_ms = 0U;
    question2_state.distance_mm = 0.0f;
    question2_state.stopped = !Line_Follow_SetBaseSpeed(QUESTION2_TARGET_SPEED_MM_S);
    question2_state.display_pending = true;

    if (question2_state.stopped)
    {
        Motor_Stop();
    }
}

void Question2_Update(void)
{
    encoder_distance_t distance;

    if (question2_state.stopped)
    {
        Motor_Stop();
        return;
    }

    if (!encoder_get_distance(&distance))
    {
        question2_state.stopped = true;
        question2_state.display_pending = true;
        Motor_Stop();
        return;
    }

    question2_state.distance_mm = (distance.left_mm + distance.right_mm) * 0.5f;

    if (!question2_state.stopped &&
        question2_state.distance_mm >= QUESTION2_LAP_DISTANCE_MM)
    {
        question2_state.stopped = true;
        question2_state.display_pending = true;
    }

    if (question2_state.stopped)
    {
        Motor_Stop();
    }
    else
    {
        Line_Follow_Control();
    }
}

void Question2_TimerTick10ms(void)
{
    if (!question2_state.stopped)
    {
        question2_state.elapsed_ms += QUESTION2_TIME_TICK_MS;
    }
}

void Question2_Display(void)
{
    uint32 elapsed_ms = question2_state.elapsed_ms;

    if (!question2_state.display_pending &&
        (elapsed_ms - question2_state.last_display_ms) < QUESTION2_DISPLAY_PERIOD_MS)
    {
        return;
    }

    ips200_show_string(10U, 10U, "D:");
    ips200_show_float(42U, 10U, question2_state.distance_mm, 5U, 1U);
    ips200_show_string(10U, 36U, "T:");
    ips200_show_float(42U, 36U, (double)elapsed_ms / 1000.0, 3U, 2U);
    ips200_show_string(10U, 62U, "V:");
    ips200_show_float(42U, 62U, QUESTION2_TARGET_SPEED_MM_S, 3U, 1U);
    ips200_show_string(10U, 88U, "VL:");
    ips200_show_float(42U, 88U, Motor1_Speed, 3U, 1U);
    ips200_show_string(10U, 114U, "VR:");
    ips200_show_float(42U, 114U, Motor2_Speed, 3U, 1U);
    ips200_show_string(10U, 140U, question2_state.stopped ? "STOP" : "RUN ");

    question2_state.last_display_ms = elapsed_ms;
    question2_state.display_pending = false;
}

float Question2_GetDistanceMm(void)
{
    return question2_state.distance_mm;
}

uint32 Question2_GetElapsedMs(void)
{
    return question2_state.elapsed_ms;
}

bool Question2_IsStopped(void)
{
    return question2_state.stopped;
}
