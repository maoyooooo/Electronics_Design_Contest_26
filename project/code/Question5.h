#ifndef QUESTION5_H
#define QUESTION5_H

#include "Question2.h"

#define QUESTION5_LAP_DISTANCE_MM          (QUESTION2_LAP_DISTANCE_MM)
#define QUESTION5_TARGET_SPEED_MM_S        (250.0f)
#define QUESTION5_TIME_TICK_MS             (10U)
#define QUESTION5_DISPLAY_PERIOD_MS        (100U)

void Question5_Init(void);
void Question5_Update(void);
void Question5_TimerTick10ms(void);
void Question5_Display(void);
float Question5_GetDistanceMm(void);
uint32 Question5_GetElapsedMs(void);
bool Question5_IsStopped(void);

#endif
