#include "zf_common_headfile.h"
#include "Question4.h"

typedef struct
{
    volatile uint32 elapsed_ms;
    uint32 last_display_ms;
    float distance_mm;
    volatile bool stopped;
    bool display_pending;
} question4_state_t;

static question4_state_t question4_state;

void Question4_Init(void)
{
    encoder_reset_distance();
    Line_Follow_ResetState();
    (void)Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION4);
    Motor_SpeedPid_ResetActiveProfile();
    question4_state.elapsed_ms = 0U;
    question4_state.last_display_ms = 0U;
    question4_state.distance_mm = 0.0f;
    question4_state.stopped = !Line_Follow_SetBaseSpeed(QUESTION4_TARGET_SPEED_MM_S);
    question4_state.display_pending = true;

    if (question4_state.stopped)
    {
        Motor_Stop();
    }
}

void Question4_Update(void)
{
    encoder_distance_t distance;

    if (question4_state.stopped)
    {
        Motor_Stop();
        return;
    }

    if (!encoder_get_distance(&distance))
    {
        question4_state.stopped = true;
        question4_state.display_pending = true;
        Motor_Stop();
        return;
    }

    question4_state.distance_mm = (distance.left_mm + distance.right_mm) * 0.5f;

    if (question4_state.distance_mm >= QUESTION4_TARGET_DISTANCE_MM)
    {
        question4_state.stopped = true;
        question4_state.display_pending = true;
        Motor_Stop();
        return;
    }

    Line_Follow_Control();
}

void Question4_TimerTick10ms(void)
{
    if (!question4_state.stopped)
    {
        question4_state.elapsed_ms += QUESTION4_TIME_TICK_MS;
    }
}

void Question4_Display(void)
{
    uint32 elapsed_ms = question4_state.elapsed_ms;

    if (!question4_state.display_pending &&
        (elapsed_ms - question4_state.last_display_ms) < QUESTION4_DISPLAY_PERIOD_MS)
    {
        return;
    }

    ips200_show_string(10U, 10U, "D:");
    ips200_show_float(42U, 10U, question4_state.distance_mm, 4U, 1U);
    ips200_show_string(10U, 36U, "T:");
    ips200_show_float(42U, 36U, (double)elapsed_ms / 1000.0, 3U, 2U);
    ips200_show_string(10U, 62U, "V:");
    ips200_show_float(42U, 62U, QUESTION4_TARGET_SPEED_MM_S, 3U, 1U);
    ips200_show_string(10U, 88U, "VL:");
    ips200_show_float(42U, 88U, Motor1_Speed, 3U, 1U);
    ips200_show_string(10U, 114U, "VR:");
    ips200_show_float(42U, 114U, Motor2_Speed, 3U, 1U);
    ips200_show_string(10U, 140U, question4_state.stopped ? "STOP" : "RUN ");

    question4_state.last_display_ms = elapsed_ms;
    question4_state.display_pending = false;
}

float Question4_GetDistanceMm(void)
{
    return question4_state.distance_mm;
}

uint32 Question4_GetElapsedMs(void)
{
    return question4_state.elapsed_ms;
}

bool Question4_IsStopped(void)
{
    return question4_state.stopped;
}
