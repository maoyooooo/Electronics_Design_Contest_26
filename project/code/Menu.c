#include "zf_common_headfile.h"
#include "Menu.h"
#include "Question2.h"
#include "Question4.h"
#include "Question5.h"
#include "Question6.h"
#include "Task_Notify.h"

typedef enum
{
    MENU_QUESTION_2 = 2U,
    MENU_QUESTION_3 = 3U,
    MENU_QUESTION_4 = 4U,
    MENU_QUESTION_5 = 5U,
    MENU_QUESTION_6 = 6U,
} menu_question_t;

#define MENU_OPTION_COUNT                  (5U)
#define MENU_QUESTION6_CM_MIN               (-15)
#define MENU_QUESTION6_CM_MAX               (15)
#define MENU_QUESTION6_MM_MIN               (-9)
#define MENU_QUESTION6_MM_MAX               (9)

typedef enum
{
    MENU_MODE_SELECT_QUESTION,
    MENU_MODE_SELECT_QUESTION6_CM,
    MENU_MODE_SELECT_QUESTION6_MM,
    MENU_MODE_WAIT_STABILIZATION,
    MENU_MODE_RUNNING,
} menu_mode_t;

typedef struct
{
    uint8 selected_index;
    menu_mode_t mode;
    int8 question6_centimeters;
    int8 question6_millimeters;
    menu_question_t pending_question;
    volatile uint32 stabilization_elapsed_ms;
    uint32 stabilization_last_display_ms;
    bool display_pending;
} menu_state_t;

static const menu_question_t menu_questions[MENU_OPTION_COUNT] =
{
    MENU_QUESTION_2,
    MENU_QUESTION_3,
    MENU_QUESTION_4,
    MENU_QUESTION_5,
    MENU_QUESTION_6,
};

static menu_state_t menu_state;

static menu_question_t Menu_GetSelected(void)
{
    return menu_questions[menu_state.selected_index];
}

static void Menu_StartStabilizationDelay(menu_question_t question)
{
    Motor_Stop();
    menu_state.pending_question = question;
    menu_state.stabilization_elapsed_ms = 0U;
    menu_state.stabilization_last_display_ms = 0U;
    menu_state.mode = MENU_MODE_WAIT_STABILIZATION;
    menu_state.display_pending = true;
}

static void Menu_StartDelayedQuestion(void)
{
    ips200_clear();

    switch (menu_state.pending_question)
    {
        case MENU_QUESTION_4:
            Question4_Init();
            break;

        case MENU_QUESTION_5:
            Question5_Init();
            break;

        case MENU_QUESTION_6:
            Question6_Init();
            break;

        default:
            Motor_Stop();
            menu_state.mode = MENU_MODE_SELECT_QUESTION;
            menu_state.display_pending = true;
            return;
    }

    menu_state.mode = MENU_MODE_RUNNING;
}

static void Menu_StartQuestion6Setup(void)
{
    Motor_Stop();
    Task_Notify_SendQuestion((uint8)MENU_QUESTION_6);
    menu_state.question6_centimeters = 0;
    menu_state.question6_millimeters = 0;
    menu_state.mode = MENU_MODE_SELECT_QUESTION6_CM;
    menu_state.display_pending = true;
}

static void Menu_StartSelected(void)
{
    ips200_clear();

    switch (Menu_GetSelected())
    {
        case MENU_QUESTION_2:
            Question2_Init();
            break;

        case MENU_QUESTION_3:
            Motor_Stop();
            ips200_show_string(10U, 10U, "QUESTION 3");
            ips200_show_string(10U, 36U, "IDLE MODE");
            break;

        case MENU_QUESTION_4:
        case MENU_QUESTION_5:
            Task_Notify_SendQuestion((uint8)Menu_GetSelected());
            Menu_StartStabilizationDelay(Menu_GetSelected());
            return;

        case MENU_QUESTION_6:
            Menu_StartQuestion6Setup();
            return;

        default:
            Motor_Stop();
            return;
    }

    Task_Notify_SendQuestion((uint8)Menu_GetSelected());
    menu_state.mode = MENU_MODE_RUNNING;
}

static void Menu_UpdateSelection(key_state_enum key1_state, key_state_enum key2_state,
                                 key_state_enum key3_state)
{
    if (key1_state == KEY_SHORT_PRESS)
    {
        if (menu_state.selected_index == 0U)
        {
            menu_state.selected_index = MENU_OPTION_COUNT - 1U;
        }
        else
        {
            --menu_state.selected_index;
        }
        menu_state.display_pending = true;
    }
    else if (key2_state == KEY_SHORT_PRESS)
    {
        menu_state.selected_index = (menu_state.selected_index + 1U) % MENU_OPTION_COUNT;
        menu_state.display_pending = true;
    }
    else if (key3_state == KEY_SHORT_PRESS)
    {
        Menu_StartSelected();
    }
}

static bool Menu_AdjustQuestion6Value(int8 *value,
                                      int8 minimum,
                                      int8 maximum,
                                      key_state_enum key1_state,
                                      key_state_enum key2_state)
{
    if (key1_state == KEY_SHORT_PRESS && *value < maximum)
    {
        ++(*value);
        return true;
    }

    if (key2_state == KEY_SHORT_PRESS && *value > minimum)
    {
        --(*value);
        return true;
    }

    return false;
}

static void Menu_UpdateQuestion6Setup(key_state_enum key1_state,
                                      key_state_enum key2_state,
                                      key_state_enum key3_state)
{
    int16 distance_mm;

    if (menu_state.mode == MENU_MODE_SELECT_QUESTION6_CM)
    {
        if (Menu_AdjustQuestion6Value(&menu_state.question6_centimeters,
                                      MENU_QUESTION6_CM_MIN,
                                      MENU_QUESTION6_CM_MAX,
                                      key1_state,
                                      key2_state))
        {
            menu_state.display_pending = true;
        }

        if (key3_state == KEY_SHORT_PRESS)
        {
            menu_state.mode = MENU_MODE_SELECT_QUESTION6_MM;
            menu_state.display_pending = true;
        }
        return;
    }

    if (Menu_AdjustQuestion6Value(&menu_state.question6_millimeters,
                                  MENU_QUESTION6_MM_MIN,
                                  MENU_QUESTION6_MM_MAX,
                                  key1_state,
                                  key2_state))
    {
        menu_state.display_pending = true;
    }

    if (key3_state == KEY_SHORT_PRESS)
    {
        distance_mm = (int16)((int16)menu_state.question6_centimeters * 10 +
                              (int16)menu_state.question6_millimeters);
        Task_Notify_SendQuestion6DistanceMm(distance_mm);
        Menu_StartStabilizationDelay(MENU_QUESTION_6);
    }
}

static void Menu_UpdateTask(void)
{
    switch (Menu_GetSelected())
    {
        case MENU_QUESTION_2:
            Question2_Update();
            break;

        case MENU_QUESTION_3:
            break;

        case MENU_QUESTION_4:
            Question4_Update();
            break;

        case MENU_QUESTION_5:
            Question5_Update();
            break;

        case MENU_QUESTION_6:
            Question6_Update();
            break;

        default:
            Motor_Stop();
            break;
    }
}

void Menu_Init(void)
{
    key_init(MENU_KEY_SCAN_PERIOD_MS);
    key_clear_all_state();
    Motor_Stop();
    menu_state.selected_index = 0U;
    menu_state.mode = MENU_MODE_SELECT_QUESTION;
    menu_state.question6_centimeters = 0;
    menu_state.question6_millimeters = 0;
    menu_state.pending_question = MENU_QUESTION_2;
    menu_state.stabilization_elapsed_ms = 0U;
    menu_state.stabilization_last_display_ms = 0U;
    menu_state.display_pending = true;
}

void Menu_Update(void)
{
    key_state_enum key1_state;
    key_state_enum key2_state;
    key_state_enum key3_state;
    key_state_enum key4_state;

    key_scanner();
    key1_state = key_get_state(KEY_1);
    key2_state = key_get_state(KEY_2);
    key3_state = key_get_state(KEY_3);
    key4_state = key_get_state(KEY_4);

    if (key1_state != KEY_RELEASE)
    {
        key_clear_state(KEY_1);
    }
    if (key2_state != KEY_RELEASE)
    {
        key_clear_state(KEY_2);
    }
    if (key3_state != KEY_RELEASE)
    {
        key_clear_state(KEY_3);
    }
    if (key4_state != KEY_RELEASE)
    {
        key_clear_state(KEY_4);
    }

    if (key4_state == KEY_SHORT_PRESS &&
        menu_state.mode != MENU_MODE_SELECT_QUESTION)
    {
        if (menu_state.mode == MENU_MODE_RUNNING)
        {
            Motor_Stop();
        }
        menu_state.mode = MENU_MODE_SELECT_QUESTION;
        menu_state.display_pending = true;
        return;
    }

    if (menu_state.mode == MENU_MODE_RUNNING)
    {
        Menu_UpdateTask();
        return;
    }

    if (menu_state.mode == MENU_MODE_WAIT_STABILIZATION)
    {
        if (menu_state.stabilization_elapsed_ms >= MENU_STABILIZATION_DELAY_MS)
        {
            Menu_StartDelayedQuestion();
        }
        return;
    }

    if (menu_state.mode == MENU_MODE_SELECT_QUESTION)
    {
        Menu_UpdateSelection(key1_state, key2_state, key3_state);
    }
    else
    {
        Menu_UpdateQuestion6Setup(key1_state, key2_state, key3_state);
    }
}

void Menu_TimerTick10ms(void)
{
    if (menu_state.mode == MENU_MODE_WAIT_STABILIZATION)
    {
        if (menu_state.stabilization_elapsed_ms < MENU_STABILIZATION_DELAY_MS)
        {
            menu_state.stabilization_elapsed_ms += MENU_KEY_SCAN_PERIOD_MS;
        }
        return;
    }

    if (menu_state.mode != MENU_MODE_RUNNING)
    {
        return;
    }

    switch (Menu_GetSelected())
    {
        case MENU_QUESTION_2:
            Question2_TimerTick10ms();
            break;

        case MENU_QUESTION_3:
            break;

        case MENU_QUESTION_4:
            Question4_TimerTick10ms();
            break;

        case MENU_QUESTION_5:
            Question5_TimerTick10ms();
            break;

        case MENU_QUESTION_6:
            Question6_TimerTick10ms();
            break;

        default:
            Motor_Stop();
            break;
    }
}

void Menu_Display(void)
{
    if (menu_state.mode == MENU_MODE_RUNNING)
    {
        switch (Menu_GetSelected())
        {
            case MENU_QUESTION_2:
                Question2_Display();
                break;

            case MENU_QUESTION_3:
                break;

            case MENU_QUESTION_4:
                Question4_Display();
                break;

            case MENU_QUESTION_5:
                Question5_Display();
                break;

            case MENU_QUESTION_6:
                Question6_Display();
                break;

            default:
                Motor_Stop();
                break;
        }
        return;
    }

    if (menu_state.mode == MENU_MODE_WAIT_STABILIZATION)
    {
        uint32 elapsed_ms = menu_state.stabilization_elapsed_ms;
        uint32 remaining_ms = elapsed_ms >= MENU_STABILIZATION_DELAY_MS
                            ? 0U
                            : MENU_STABILIZATION_DELAY_MS - elapsed_ms;

        if (!menu_state.display_pending &&
            (elapsed_ms - menu_state.stabilization_last_display_ms) <
                MENU_STABILIZATION_DISPLAY_PERIOD_MS)
        {
            return;
        }

        ips200_clear();
        ips200_show_string(10U, 10U, "BALL STABILIZE");
        ips200_show_string(10U, 50U, "WAIT:");
        ips200_show_float(74U, 50U, (double)remaining_ms / 1000.0, 1U, 1U);
        ips200_show_string(118U, 50U, "s");
        ips200_show_string(10U, 100U, "MOTOR STOPPED");
        ips200_show_string(10U, 176U, "K4 CANCEL");
        menu_state.stabilization_last_display_ms = elapsed_ms;
        menu_state.display_pending = false;
        return;
    }

    if (menu_state.mode == MENU_MODE_SELECT_QUESTION6_CM ||
        menu_state.mode == MENU_MODE_SELECT_QUESTION6_MM)
    {
        if (!menu_state.display_pending)
        {
            return;
        }

        ips200_clear();
        ips200_show_string(10U, 10U, "QUESTION 6 SETUP");
        if (menu_state.mode == MENU_MODE_SELECT_QUESTION6_CM)
        {
            ips200_show_string(10U, 42U, "SET OFFSET CM");
            ips200_show_int(90U, 68U,
                            (int32)menu_state.question6_centimeters,
                            2U);
            ips200_show_string(126U, 68U, "cm");
            ips200_show_string(10U, 112U, "K1 +1  K2 -1");
            ips200_show_string(10U, 150U, "K3 NEXT");
        }
        else
        {
            ips200_show_string(10U, 42U, "SET OFFSET MM");
            ips200_show_int(90U, 68U,
                            (int32)menu_state.question6_millimeters,
                            1U);
            ips200_show_string(126U, 68U, "mm");
            ips200_show_string(10U, 102U, "TOTAL:");
            ips200_show_int(74U, 102U,
                            (int32)menu_state.question6_centimeters * 10 +
                            (int32)menu_state.question6_millimeters,
                            3U);
            ips200_show_string(126U, 102U, "mm");
            ips200_show_string(10U, 136U, "K1 +1  K2 -1");
            ips200_show_string(10U, 146U, "K3 START");
        }
        ips200_show_string(10U, 176U, "K4 CANCEL");
        menu_state.display_pending = false;
        return;
    }

    if (!menu_state.display_pending)
    {
        return;
    }

    ips200_clear();
    ips200_show_string(10U, 10U, "SELECT QUESTION");
    ips200_show_string(2U, 36U, menu_state.selected_index == 0U ? ">" : " ");
    ips200_show_string(18U, 36U, "Q2 LAP 330");
    ips200_show_string(2U, 58U, menu_state.selected_index == 1U ? ">" : " ");
    ips200_show_string(18U, 58U, "Q3 IDLE UART");
    ips200_show_string(2U, 80U, menu_state.selected_index == 2U ? ">" : " ");
    ips200_show_string(18U, 80U, "Q4 1500MM 330");
    ips200_show_string(2U, 102U, menu_state.selected_index == 3U ? ">" : " ");
    ips200_show_string(18U, 102U, "Q5 LAP 250");
    ips200_show_string(2U, 124U, menu_state.selected_index == 4U ? ">" : " ");
    ips200_show_string(18U, 124U, "Q6 LAP 250");
    ips200_show_string(10U, 154U, "K1 UP K2 DN K3 OK");
    ips200_show_string(10U, 176U, "K4 EXIT RUN");
    menu_state.display_pending = false;
}

uint8 Menu_GetSelectedQuestion(void)
{
    return (uint8)Menu_GetSelected();
}

bool Menu_IsTaskRunning(void)
{
    return menu_state.mode == MENU_MODE_RUNNING;
}
