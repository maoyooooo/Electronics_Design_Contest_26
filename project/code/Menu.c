#include "zf_common_headfile.h"
#include "Menu.h"
#include "Question2.h"
#include "Question4.h"
#include "Question5.h"
#include "Question6.h"

typedef enum
{
    MENU_QUESTION_2 = 2U,
    MENU_QUESTION_4 = 4U,
    MENU_QUESTION_5 = 5U,
    MENU_QUESTION_6 = 6U,
} menu_question_t;

#define MENU_OPTION_COUNT                  (4U)

typedef struct
{
    uint8 selected_index;
    bool task_running;
    bool display_pending;
} menu_state_t;

static const menu_question_t menu_questions[MENU_OPTION_COUNT] =
{
    MENU_QUESTION_2,
    MENU_QUESTION_4,
    MENU_QUESTION_5,
    MENU_QUESTION_6,
};

static menu_state_t menu_state;

static menu_question_t Menu_GetSelected(void)
{
    return menu_questions[menu_state.selected_index];
}

static void Menu_StartSelected(void)
{
    ips200_clear();

    switch (Menu_GetSelected())
    {
        case MENU_QUESTION_2:
            Question2_Init();
            break;

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
            return;
    }

    menu_state.task_running = true;
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

static void Menu_UpdateTask(void)
{
    switch (Menu_GetSelected())
    {
        case MENU_QUESTION_2:
            Question2_Update();
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
    menu_state.task_running = false;
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

    if (menu_state.task_running)
    {
        if (key4_state == KEY_SHORT_PRESS)
        {
            Motor_Stop();
            menu_state.task_running = false;
            menu_state.display_pending = true;
            return;
        }

        Menu_UpdateTask();
        return;
    }

    Menu_UpdateSelection(key1_state, key2_state, key3_state);
}

void Menu_TimerTick10ms(void)
{
    if (!menu_state.task_running)
    {
        return;
    }

    switch (Menu_GetSelected())
    {
        case MENU_QUESTION_2:
            Question2_TimerTick10ms();
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
    if (menu_state.task_running)
    {
        switch (Menu_GetSelected())
        {
            case MENU_QUESTION_2:
                Question2_Display();
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

    if (!menu_state.display_pending)
    {
        return;
    }

    ips200_clear();
    ips200_show_string(10U, 10U, "SELECT QUESTION");
    ips200_show_string(2U, 36U, menu_state.selected_index == 0U ? ">" : " ");
    ips200_show_string(18U, 36U, "Q2 LAP 330");
    ips200_show_string(2U, 58U, menu_state.selected_index == 1U ? ">" : " ");
    ips200_show_string(18U, 58U, "Q4 1500MM 330");
    ips200_show_string(2U, 80U, menu_state.selected_index == 2U ? ">" : " ");
    ips200_show_string(18U, 80U, "Q5 LAP 250");
    ips200_show_string(2U, 102U, menu_state.selected_index == 3U ? ">" : " ");
    ips200_show_string(18U, 102U, "Q6 LAP 250");
    ips200_show_string(10U, 132U, "K1 UP K2 DN K3 OK");
    ips200_show_string(10U, 154U, "K4 EXIT RUN");
    menu_state.display_pending = false;
}

uint8 Menu_GetSelectedQuestion(void)
{
    return (uint8)Menu_GetSelected();
}

bool Menu_IsTaskRunning(void)
{
    return menu_state.task_running;
}
