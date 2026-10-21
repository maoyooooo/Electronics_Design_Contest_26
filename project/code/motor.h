#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"
#include "Grayscale_Sensor.h"

/*-------------------------------------------------------------------------------------------*/
/*---------------------------------------PID函数---------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/*PID结构体部分*/
/*PID三个权重*/
/*目标值、获取值、三个误差*/
/*比例、积分、微分输出、总输出*/
/*输出限幅以及积分限幅*/	
typedef struct pid_t
{
  float Kp;					//P参数
  float Ki;					//I参数
  float Kd;					//D参数
		
  float Target;				//目标值
  float Measure;			//当前获取值
  float Error[3];			//三次误差
		
  float KpOut;				//比例输出
  float KiOut;				//积分输出
  float KdOut;				//微分输出
  float PID_Out;			//PID总输出
		
  uint32_t PID_Limit_MAX; 	//最大输出限幅
  uint32_t Ki_Limit_MAX;	//最大积分输出限幅
}pid_t;

typedef enum
{
    MOTOR_SPEED_PID_PROFILE_DEFAULT,
    MOTOR_SPEED_PID_PROFILE_QUESTION2,
    MOTOR_SPEED_PID_PROFILE_QUESTION4,
    MOTOR_SPEED_PID_PROFILE_QUESTION5,
    MOTOR_SPEED_PID_PROFILE_QUESTION6,
} motor_speed_pid_profile_t;

float PID_Calculate(pid_t *pid, float Target, float Measure);
float PID_Calculate_Angle(pid_t *pid, float Measure, float Target);
void Set_PID_Param(pid_t *pid, float P, float I,float D);
/*-------------------------------------------------------------------------------------------*/
/*------------------------------------电机操作函数-------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

#define MOTOR1_DIR               (B9 )
#define MOTOR1_PWM               (PWM_TIM_A0_CH0_B8 )

#define MOTOR2_DIR               (B13 )
#define MOTOR2_PWM               (PWM_TIM_A0_CH2_B12 )

// 电机占空比上限（防越界，必须 <= 逐飞库 PWM_DUTY_MAX=10000）
// 台架首次测试建议先设小一点（如 5000=50%）更安全，稳定后再调大到 10000
#define MOTOR_DUTY_MAX           (10000)

// 相对角转向参数（需要根据底盘惯量和电机静摩擦做实车微调）
#define TURN_CONTROL_PERIOD_MS       (10U)
#define TURN_TIMEOUT_MS              (8000U)
#define TURN_ANGLE_TOLERANCE_DEG     (2.0f)
#define TURN_SETTLE_SAMPLE_COUNT     (5U)
#define TURN_MIN_SPEED_MM_S          (80.0f)
#define TURN_MAX_SPEED_MM_S          (300.0f)
#define TURN_MAX_RELATIVE_ANGLE_DEG  (3600.0f)

// 循迹控制参数
#define LINE_BASE_SPEED_MM_S            (250.0f)        // 基准速度，单位 mm/s
#define LINE_MAX_CORRECTION_MM_S        (180.0f)        // 最大修正速度，单位 mm/s
#define LINE_MIN_SPEED_MM_S             (130.0f)        // 最小速度，单位 mm/s
#define LINE_MIN_WHEEL_SPEED_MM_S       (0.0f)          // 最小轮子速度，单位 mm/s
#define LINE_SPEED_CURVE_GAIN_MM_S      (18.0f)         // 速度曲线增益，单位 mm/s
#define LINE_SPEED_DECEL_STEP_MM_S      (25.0f)         // 最大减速步长，单位 mm/s
#define LINE_SPEED_ACCEL_STEP_MM_S      (5.0f)          // 最大加速步长，单位 mm/s
#define LINE_SOFT_START_ACCEL_MM_S2     (200.0f)
#define LINE_SOFT_STOP_DECEL_MM_S2      (200.0f)
#define LINE_SOFT_STOP_SPEED_EPSILON_MM_S (10.0f)
#define LINE_SOFT_STOP_TURN_FADE_SPEED_MM_S (80.0f)
#define LINE_TURN_BOOST_START_DEVIATION (1.5f)          // 开始转向增益的偏差阈值，单位 mm
#define LINE_TURN_BOOST_FULL_DEVIATION  (2.5f)          // 完全转向增益的偏差阈值，单位 mm
#define LINE_TURN_BOOST_MAX             (1.5f)         // 最大转向增益倍率，单位倍
#define LINE_RECOVERY_ARM_DEVIATION     (2.0f)          // 触发循迹恢复的偏差阈值，单位 mm
#define LINE_RECOVERY_EDGE_MEMORY_CYCLES (3U)           // 触发循迹恢复的边缘记忆周期数
#define LINE_RECOVERY_MAX_LOST_CYCLES   (15U)       // 触发循迹恢复的最大丢线周期数
#define LINE_RECOVERY_BASE_SPEED_MM_S   (150.0f)         // 循迹恢复的基准速度，单位 mm/s
#define LINE_RECOVERY_CORRECTION_MM_S   (100.0f)     // 循迹恢复的修正速度，单位 mm/s

#define LINE_FOLLOW_CONTROL_PERIOD_MS          (10U)

// 若 Set_Motor_PWM(+,-) 使 angle_Z 增大则保持 +1；反之改为 -1。
#ifndef TURN_YAW_DIRECTION_SIGN
#define TURN_YAW_DIRECTION_SIGN      (1.0f)
#endif




extern uint8_t MOTOR1_ENABLE_FLAG;
extern uint8_t MOTOR2_ENABLE_FLAG;
//PID参数
extern pid_t pid_Motor1_Speed;
extern pid_t pid_Motor2_Speed;
extern pid_t pid_Question2_Motor1_Speed;
extern pid_t pid_Question2_Motor2_Speed;
extern pid_t pid_Question4_Motor1_Speed;
extern pid_t pid_Question4_Motor2_Speed;
extern pid_t pid_Question5_Motor1_Speed;
extern pid_t pid_Question5_Motor2_Speed;
extern pid_t pid_Question6_Motor1_Speed;
extern pid_t pid_Question6_Motor2_Speed;
extern pid_t pid_Turn;
extern pid_t pid_Distance;
extern pid_t pid_Gyro;
extern pid_t pid_Angle;

extern uint8_t Turn_PID_Flag;
extern uint8_t Distance_PID_Flag;
extern uint8_t Gyro_PID_Flag;
extern uint8_t Angle_PID_Flag;

extern volatile float current_L_speed;
extern volatile float current_R_speed;

// ---函数原型声明---
void Motor_Init(void);
void Motor_Control_Loop(float target_L, float target_R);
void Motor_UpdateRpmFromEncoder(void);
void Motor_SpeedLoop(float target_L_speed, float target_R_speed);
void Motor_Stop(void);
bool Motor_SpeedPid_SelectProfile(motor_speed_pid_profile_t profile);
void Motor_SpeedPid_ResetActiveProfile(void);
motor_speed_pid_profile_t Motor_SpeedPid_GetActiveProfile(void);
void Line_Follow_ResetState(void);
bool Line_Follow_SetBaseSpeed(float base_speed_mm_s);
bool Line_Follow_StartRamped(float target_speed_mm_s,
                             float start_accel_mm_s2,
                             float stop_decel_mm_s2);
bool Line_Follow_RequestSmoothStop(void);
float Line_Follow_GetSmoothStopDistanceMm(void);
bool Line_Follow_IsSmoothStopComplete(void);
void Line_Follow_Control(void);
// 仅从主循环/普通函数调用；IMU 角度更新中断必须保持运行。
// relative_angle > 0 表示 angle_Z 正方向，返回 true 表示到位，false 表示输入异常或超时。
bool Turn_Control(float relative_angle);

#endif
