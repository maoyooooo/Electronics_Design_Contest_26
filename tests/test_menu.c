#include <stdio.h>
#include <stdlib.h>

#include "Menu.h"

static key_state_enum key_states[KEY_NUMBER];
static uint32 key_init_period;
static uint32 key_scanner_calls;
static uint32 motor_stop_calls;
static uint32 question_init_calls[4];
static uint32 question_update_calls[4];
static uint32 question_tick_calls[4];
static uint32 question_display_calls[4];

void key_init(uint32 period)
{
    key_init_period = period;
}

void key_scanner(void)
{
    ++key_scanner_calls;
}

key_state_enum key_get_state(key_index_enum key_n)
{
    return key_states[key_n];
}

void key_clear_state(key_index_enum key_n)
{
    key_states[key_n] = KEY_RELEASE;
}

void key_clear_all_state(void)
{
    uint8 index;

    for (index = 0U; index < KEY_NUMBER; ++index)
    {
        key_states[index] = KEY_RELEASE;
    }
}

void Motor_Stop(void)
{
    ++motor_stop_calls;
}

void Question2_Init(void) { ++question_init_calls[0]; }
void Question2_Update(void) { ++question_update_calls[0]; }
void Question2_TimerTick10ms(void) { ++question_tick_calls[0]; }
void Question2_Display(void) { ++question_display_calls[0]; }
void Question4_Init(void) { ++question_init_calls[1]; }
void Question4_Update(void) { ++question_update_calls[1]; }
void Question4_TimerTick10ms(void) { ++question_tick_calls[1]; }
void Question4_Display(void) { ++question_display_calls[1]; }
void Question5_Init(void) { ++question_init_calls[2]; }
void Question5_Update(void) { ++question_update_calls[2]; }
void Question5_TimerTick10ms(void) { ++question_tick_calls[2]; }
void Question5_Display(void) { ++question_display_calls[2]; }
void Question6_Init(void) { ++question_init_calls[3]; }
void Question6_Update(void) { ++question_update_calls[3]; }
void Question6_TimerTick10ms(void) { ++question_tick_calls[3]; }
void Question6_Display(void) { ++question_display_calls[3]; }

void ips200_clear(void) { }
void ips200_show_string(uint16 x, uint16 y, const char dat[])
{
    (void)x;
    (void)y;
    (void)dat;
}

static void fail(const char *message)
{
    fprintf(stderr, "FAIL: %s\n", message);
    exit(EXIT_FAILURE);
}

static void reset_fixture(void)
{
    uint8 index;

    key_init_period = 0U;
    key_scanner_calls = 0U;
    motor_stop_calls = 0U;
    for (index = 0U; index < KEY_NUMBER; ++index)
    {
        key_states[index] = KEY_RELEASE;
    }
    for (index = 0U; index < 4U; ++index)
    {
        question_init_calls[index] = 0U;
        question_update_calls[index] = 0U;
        question_tick_calls[index] = 0U;
        question_display_calls[index] = 0U;
    }
}

static void test_menu_selects_questions_with_key1_and_key2(void)
{
    reset_fixture();
    Menu_Init();
    if (key_init_period != MENU_KEY_SCAN_PERIOD_MS || motor_stop_calls != 1U ||
        Menu_GetSelectedQuestion() != 2U || Menu_IsTaskRunning())
    {
        fail("menu initialization did not select question 2 safely");
    }

    key_states[KEY_2] = KEY_SHORT_PRESS;
    Menu_Update();
    if (Menu_GetSelectedQuestion() != 4U || key_scanner_calls != 1U)
    {
        fail("KEY2 did not move selection down to question 4");
    }

    key_states[KEY_1] = KEY_SHORT_PRESS;
    Menu_Update();
    if (Menu_GetSelectedQuestion() != 2U)
    {
        fail("KEY1 did not move selection up to question 2");
    }

    key_states[KEY_1] = KEY_SHORT_PRESS;
    Menu_Update();
    if (Menu_GetSelectedQuestion() != 6U)
    {
        fail("KEY1 did not wrap selection from question 2 to question 6");
    }
}

static void test_menu_starts_selected_question_and_routes_callbacks(void)
{
    reset_fixture();
    Menu_Init();
    key_states[KEY_2] = KEY_SHORT_PRESS;
    Menu_Update();
    key_states[KEY_3] = KEY_SHORT_PRESS;
    Menu_Update();
    if (!Menu_IsTaskRunning() || question_init_calls[1] != 1U ||
        question_init_calls[0] != 0U || question_init_calls[2] != 0U ||
        question_init_calls[3] != 0U)
    {
        fail("KEY3 did not start only the selected question");
    }

    Menu_Update();
    Menu_TimerTick10ms();
    Menu_Display();
    if (question_update_calls[1] != 1U || question_tick_calls[1] != 1U ||
        question_display_calls[1] != 1U)
    {
        fail("menu did not route runtime callbacks to question 4");
    }
}

static void test_key4_stops_active_task_and_returns_to_menu(void)
{
    reset_fixture();
    Menu_Init();
    key_states[KEY_3] = KEY_SHORT_PRESS;
    Menu_Update();
    key_states[KEY_4] = KEY_SHORT_PRESS;
    Menu_Update();
    Menu_TimerTick10ms();
    if (Menu_IsTaskRunning() || motor_stop_calls != 2U ||
        question_tick_calls[0] != 0U)
    {
        fail("KEY4 did not stop the active task and return to the menu");
    }
}

int main(void)
{
    test_menu_selects_questions_with_key1_and_key2();
    test_menu_starts_selected_question_and_routes_callbacks();
    test_key4_stops_active_task_and_returns_to_menu();
    puts("PASS: Menu host tests");
    return EXIT_SUCCESS;
}
