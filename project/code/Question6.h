#ifndef QUESTION6_H
#define QUESTION6_H

#include "Question2.h"

#define QUESTION6_LAP_DISTANCE_MM          (QUESTION2_LAP_DISTANCE_MM)
#define QUESTION6_TARGET_SPEED_MM_S        (250.0f)
#define QUESTION6_TIME_TICK_MS             (10U)
#define QUESTION6_DISPLAY_PERIOD_MS        (100U)

void Question6_Init(void);
void Question6_Update(void);
void Question6_TimerTick10ms(void);
void Question6_Display(void);
float Question6_GetDistanceMm(void);
uint32 Question6_GetElapsedMs(void);
bool Question6_IsStopped(void);

#endif
