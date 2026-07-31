#ifndef QUESTION2_H
#define QUESTION2_H

#include "zf_common_headfile.h"

#define QUESTION2_LAP_DISTANCE_MM          (6000.0f)
#define QUESTION2_TARGET_SPEED_MM_S         (400.0f)
#define QUESTION2_TIME_TICK_MS             (10U)
#define QUESTION2_DISPLAY_PERIOD_MS        (100U)

void Question2_Init(void);
void Question2_Update(void);
void Question2_TimerTick10ms(void);
void Question2_Display(void);
float Question2_GetDistanceMm(void);
uint32 Question2_GetElapsedMs(void);
bool Question2_IsStopped(void);

#endif
