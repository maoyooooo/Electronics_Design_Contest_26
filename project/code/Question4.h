#ifndef QUESTION4_H
#define QUESTION4_H

#include "zf_common_headfile.h"

#define QUESTION4_TARGET_DISTANCE_MM       (1800.0f)
#define QUESTION4_TARGET_SPEED_MM_S        (280.0f)
#define QUESTION4_TIME_TICK_MS             (10U)
#define QUESTION4_DISPLAY_PERIOD_MS        (100U)

void Question4_Init(void);
void Question4_Update(void);
void Question4_TimerTick10ms(void);
void Question4_Display(void);
float Question4_GetDistanceMm(void);
uint32 Question4_GetElapsedMs(void);
bool Question4_IsStopped(void);

#endif
