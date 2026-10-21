#ifndef TASK_NOTIFY_H
#define TASK_NOTIFY_H

#include "zf_common_headfile.h"

void Task_Notify_Init(void);
void Task_Notify_SendQuestion(uint8 question_number);
void Task_Notify_SendQuestion6DistanceMm(int16 distance_mm);

#endif
