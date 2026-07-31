#ifndef QUESTION5_6_TEST_ZF_COMMON_HEADFILE_H
#define QUESTION5_6_TEST_ZF_COMMON_HEADFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

typedef struct
{
    float left_mm;
    float right_mm;
} encoder_distance_t;

typedef enum
{
    MOTOR_SPEED_PID_PROFILE_DEFAULT,
    MOTOR_SPEED_PID_PROFILE_QUESTION2,
    MOTOR_SPEED_PID_PROFILE_QUESTION4,
    MOTOR_SPEED_PID_PROFILE_QUESTION5,
    MOTOR_SPEED_PID_PROFILE_QUESTION6,
} motor_speed_pid_profile_t;

bool encoder_get_distance(encoder_distance_t *distance);
void encoder_reset_distance(void);
void Line_Follow_ResetState(void);
bool Line_Follow_SetBaseSpeed(float base_speed_mm_s);
void Line_Follow_Control(void);
void Motor_Stop(void);
bool Motor_SpeedPid_SelectProfile(motor_speed_pid_profile_t profile);
void Motor_SpeedPid_ResetActiveProfile(void);
extern float Motor1_Speed;
extern float Motor2_Speed;
void ips200_show_string(uint16 x, uint16 y, const char dat[]);
void ips200_show_float(uint16 x, uint16 y, double dat, uint8 num, uint8 pointnum);

#endif
