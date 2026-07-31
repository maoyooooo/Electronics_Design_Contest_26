#ifndef MENU_H
#define MENU_H

#include "zf_common_headfile.h"

#define MENU_KEY_SCAN_PERIOD_MS            (10U)

void Menu_Init(void);
void Menu_Update(void);
void Menu_TimerTick10ms(void);
void Menu_Display(void);
uint8 Menu_GetSelectedQuestion(void);
bool Menu_IsTaskRunning(void);

#endif
