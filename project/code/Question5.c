#include "zf_common_headfile.h"
#include "Question5.h"

typedef struct
{
    volatile uint32 elapsed_ms;
    uint32 last_display_ms;
    float distance_mm;
    volatile bool stopped;
    volatile bool time_frozen;
    bool slowing_down;
    bool display_pending;
} question5_state_t;

static question5_state_t question5_state;

void Question5_Init(void)
{
    encoder_reset_distance();
    Line_Follow_ResetState();
    (void)Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_QUESTION5);
    Motor_SpeedPid_ResetActiveProfile();
    question5_state.elapsed_ms = 0U;
    question5_state.last_display_ms = 0U;
    question5_state.distance_mm = 0.0f;
    question5_state.time_frozen = false;
    question5_state.stopped = !Line_Follow_StartRamped(QUESTION5_TARGET_SPEED_MM_S,
                                                         LINE_SOFT_START_ACCEL_MM_S2,
                                                         LINE_SOFT_STOP_DECEL_MM_S2);
    question5_state.slowing_down = false;
    question5_state.display_pending = true;

    if (question5_state.stopped)
    {
        Motor_Stop();
    }
}

void Question5_Update(void)
{
    encoder_distance_t distance;
    float stop_distance_mm;

    if (question5_state.stopped)
    {
        Motor_Stop();
        return;
    }

    if (!encoder_get_distance(&distance))
    {
        question5_state.stopped = true;
        question5_state.display_pending = true;
        Motor_Stop();
        return;
    }

    question5_state.distance_mm = (distance.left_mm + distance.right_mm) * 0.5f;

    if (!question5_state.time_frozen &&
        question5_state.distance_mm >= QUESTION5_LAP_DISTANCE_MM)
    {
        question5_state.time_frozen = true;
        question5_state.display_pending = true;
    }

    if (!question5_state.slowing_down)
    {
        stop_distance_mm = Line_Follow_GetSmoothStopDistanceMm() +
                           QUESTION5_SOFT_STOP_MARGIN_MM;
        if (question5_state.distance_mm >=
            QUESTION5_LAP_DISTANCE_MM - stop_distance_mm)
        {
            question5_state.slowing_down = Line_Follow_RequestSmoothStop();
            if (!question5_state.slowing_down)
            {
                question5_state.stopped = true;
                question5_state.display_pending = true;
                Motor_Stop();
                return;
            }
        }
    }

    Line_Follow_Control();

    if (question5_state.slowing_down && Line_Follow_IsSmoothStopComplete())
    {
        question5_state.stopped = true;
        question5_state.display_pending = true;
        Motor_Stop();
    }
}

void Question5_TimerTick10ms(void)
{
    if (!question5_state.stopped && !question5_state.time_frozen)
    {
        question5_state.elapsed_ms += QUESTION5_TIME_TICK_MS;
    }
}

void Question5_Display(void)
{
    uint32 elapsed_ms = question5_state.elapsed_ms;

    if (!question5_state.display_pending &&
        (elapsed_ms - question5_state.last_display_ms) < QUESTION5_DISPLAY_PERIOD_MS)
    {
        return;
    }

    ips200_show_string(10U, 10U, "D:");
    ips200_show_float(42U, 10U, question5_state.distance_mm, 5U, 1U);
    ips200_show_string(10U, 36U, "T:");
    ips200_show_float(42U, 36U, (double)elapsed_ms / 1000.0, 3U, 2U);
    ips200_show_string(10U, 62U, "V:");
    ips200_show_float(42U, 62U, QUESTION5_TARGET_SPEED_MM_S, 3U, 1U);
    ips200_show_string(10U, 88U, "VL:");
    ips200_show_float(42U, 88U, Motor1_Speed, 3U, 1U);
    ips200_show_string(10U, 114U, "VR:");
    ips200_show_float(42U, 114U, Motor2_Speed, 3U, 1U);
    ips200_show_string(10U, 140U, question5_state.stopped ? "STOP" :
                                      (question5_state.slowing_down ? "SLOW" : "RUN "));

    question5_state.last_display_ms = elapsed_ms;
    question5_state.display_pending = false;
}

float Question5_GetDistanceMm(void)
{
    return question5_state.distance_mm;
}

uint32 Question5_GetElapsedMs(void)
{
    return question5_state.elapsed_ms;
}

bool Question5_IsStopped(void)
{
    return question5_state.stopped;
}
