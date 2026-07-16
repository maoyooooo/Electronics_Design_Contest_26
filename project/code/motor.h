#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

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
float PID_Calculate(pid_t *pid,float Measure,  float Target);
float PID_Calculate_Angle(pid_t *pid,float Measure,  float Target);
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




extern uint8_t MOTOR1_ENABLE_FLAG;
extern uint8_t MOTOR2_ENABLE_FLAG;
//PID参数
extern pid_t pid_Motor1_Speed;
extern pid_t pid_Motor2_Speed;
extern pid_t pid_Turn;
extern pid_t pid_Distance;
extern pid_t pid_Gyro;
extern pid_t pid_Angle;

extern uint8_t Turn_PID_Flag;
extern uint8_t Distance_PID_Flag;
extern uint8_t Gyro_PID_Flag;
extern uint8_t Angle_PID_Flag;

extern float current_L_speed;
extern float current_R_speed;

// ---函数原型声明---
void Motor_Init(void);
void Motor_Control_Loop(float target_L, float target_R);
void Motor_SpeedLoop(float target_L_speed, float target_R_speed);

#endif
