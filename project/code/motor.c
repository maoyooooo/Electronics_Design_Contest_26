#include "zf_common_headfile.h"
#include <math.h>

volatile float current_L_speed = 0.0f;
volatile float current_R_speed = 0.0f;
#define speed_FILTER_ALPHA 0.3f

typedef struct
{
    float base_speed_mm_s;
    float requested_base_speed_mm_s;
    float last_edge_deviation;
    uint8 edge_age_cycles;
    uint8 lost_line_cycles;
    bool recovery_armed;
} line_follow_state_t;

static line_follow_state_t line_follow_state =
{
    .base_speed_mm_s = LINE_BASE_SPEED_MM_S,
    .requested_base_speed_mm_s = LINE_BASE_SPEED_MM_S
};

/**
 * @brief 电机硬件初始化
 */
void Motor_Init(void)
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    pwm_init(MOTOR2_PWM, 17000, 0);
    pwm_init(MOTOR1_PWM, 17000, 0);

    (void)Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_DEFAULT);
    Motor_SpeedPid_ResetActiveProfile();
    Line_Follow_ResetState();
}

/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------PID计算部分部分---------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
//速度环参数修改
pid_t pid_Motor1_Speed ={	\
	.Kp = 20,				\
	.Ki = 0.2,				\
	.Kd = 0,				\
	.Target = 0,			\
	.Measure = 0,			\
	{0,0,0},				\
	.KpOut = 0,				\
	.KiOut = 0,				\
	.KdOut = 0,				\
	.PID_Out = 0,			\
	.PID_Limit_MAX = 9999,	\
	.Ki_Limit_MAX = 2500,	\
};
pid_t pid_Motor2_Speed ={	\
	.Kp = 20,				\
	.Ki = 0.2,				\
	.Kd = 0,				\
	.Target = 0,			\
	.Measure = 0,			\
	{0,0,0},				\
	.KpOut = 0,				\
	.KiOut = 0,				\
	.KdOut = 0,				\
	.PID_Out = 0,			\
	.PID_Limit_MAX = 9999,	\
	.Ki_Limit_MAX = 2500,	\
};
//转向环参数修改			60cm/s	6	140
pid_t pid_Turn ={			\
	.Kp = 45,				\
	.Ki = 0,				\
	.Kd = 0,				\
	.Target = 0,			\
	.Measure = 0,			\
	{0,0,0},				\
	.KpOut = 0,				\
	.KiOut = 0,				\
	.KdOut = 0,				\
	.PID_Out = 0,			\
	.PID_Limit_MAX = 9999,	\
	.Ki_Limit_MAX = 9999,	\
};
//距离环参数修改
pid_t pid_Distance ={		\
	.Kp = 10,				\
	.Ki = 0,				\
	.Kd = 5,				\
	.Target = 0,			\
	.Measure = 0,			\
	{0,0,0},				\
	.KpOut = 0,				\
	.KiOut = 0,				\
	.KdOut = 0,				\
	.PID_Out = 0,			\
	.PID_Limit_MAX = 9999,	\
	.Ki_Limit_MAX = 9999,	\
};
//角速度环参数修改
pid_t pid_Gyro ={			\
	.Kp = 0.08,				\
	.Ki = 0.015,				\
	.Kd = 0.015,				\
	.Target = 0,			\
	.Measure = 0,			\
	{0,0,0},				\
	.KpOut = 0,				\
	.KiOut = 0,				\
	.KdOut = 0,				\
	.PID_Out = 0,			\
	.PID_Limit_MAX = 9999,	\
	.Ki_Limit_MAX = 9999,	\
};
//转向环参数修改
pid_t pid_Angle ={			\
	.Kp = 3.1,				\
	.Ki = 0,				\
	.Kd = 18,				\
	.Target = 0,			\
	.Measure = 0,			\
	{0,0,0},				\
	.KpOut = 0,				\
	.KiOut = 0,				\
	.KdOut = 0,				\
	.PID_Out = 0,			\
	.PID_Limit_MAX = 1000,	\
	.Ki_Limit_MAX = 1000,	\
};


//PWM限幅函数 *a传入要限幅的参数  ABS_MAX限幅大小
// 各题目速度环 PID 参数统一在此处管理。
pid_t pid_Question2_Motor1_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question2_Motor2_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question4_Motor1_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question4_Motor2_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question5_Motor1_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question5_Motor2_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question6_Motor1_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};
pid_t pid_Question6_Motor2_Speed ={
	.Kp = 20,
	.Ki = 0.2,
	.Kd = 0,
	.Target = 0,
	.Measure = 0,
	{0,0,0},
	.KpOut = 0,
	.KiOut = 0,
	.KdOut = 0,
	.PID_Out = 0,
	.PID_Limit_MAX = 9999,
	.Ki_Limit_MAX = 2500,
};

static motor_speed_pid_profile_t active_speed_pid_profile =
    MOTOR_SPEED_PID_PROFILE_DEFAULT;
static pid_t *active_left_speed_pid = &pid_Motor1_Speed;
static pid_t *active_right_speed_pid = &pid_Motor2_Speed;

void PID_Limit(float *a, float ABS_MAX){
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}
/*************************************************************************************************
*	函 数 名:	PID_Calculate
*
*	函数功能:	PID计算公式
*
*   参    数：  PID结构体，目标值，当前测量值
*************************************************************************************************/
float PID_Calculate(pid_t *pid, float Target, float Measure)
{
	pid->Target = Target;//赋值目标值
	pid->Measure = Measure;//赋值测量值
	pid->Error[0] = Target - Measure;//计算当前误差 

    pid->KpOut = pid->Kp * (pid->Error[0]);//比例计算，Kp*当前误差
    pid->KiOut += pid->Ki * pid->Error[0];//积分计算，Ki*误差累加值
	PID_Limit(&(pid->KiOut), pid->Ki_Limit_MAX);//积分限幅
    pid->KdOut = pid->Kd * (pid->Error[0] - pid->Error[1]);//微分计算，Kd*(当前误差-上一次误差)

    pid->PID_Out = pid->KpOut + pid->KiOut + pid->KdOut;//PID计算
    PID_Limit(&(pid->PID_Out), pid->PID_Limit_MAX);//PID输出限幅

	pid->Error[2] = pid->Error[1];
	pid->Error[1] = pid->Error[0];

    return pid->PID_Out;
}



float PID_Calculate_Angle(pid_t *pid,float Measure,  float Target)
{
	pid->Target = Target;//赋值目标值
	pid->Measure = Measure;//赋值测量值
	pid->Error[0] = Target - Measure;//计算当前误差 
	if(pid->Error[0] > 180)pid->Error[0] -= 360;
	if(pid->Error[0] < -180)pid->Error[0] += 360;
    pid->KpOut = pid->Kp * (pid->Error[0]);//比例计算，Kp*当前误差
    pid->KiOut += pid->Ki * pid->Error[0];//积分计算，Ki*误差累加值
	PID_Limit(&(pid->KiOut), pid->Ki_Limit_MAX);//积分限幅
    pid->KdOut = pid->Kd * (pid->Error[0] - pid->Error[1]);//微分计算，Kd*(当前误差-上一次误差)

    pid->PID_Out = pid->KpOut + pid->KiOut + pid->KdOut;//PID计算
    PID_Limit(&(pid->PID_Out), pid->PID_Limit_MAX);//PID输出限幅

	pid->Error[2] = pid->Error[1];
	pid->Error[1] = pid->Error[0];

    return pid->PID_Out;
}


void Set_PID_Param(pid_t *pid, float P, float I,float D)
{
	pid->Kp = P;pid->Ki = I;pid->Kd = D;
	pid->KpOut = 0;pid->KiOut = 0;pid->KdOut = 0;pid->PID_Out = 0;//输出清零
}
//PWM限幅函数 *a传入要限幅的参数  ABS_MAX限幅大小
void PWM_Limit(int *a, int ABS_MAX){
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}



// out_L / out_R 为 PID 输出，直接作为占空比使用（量级 0~PWM_DUTY_MAX），符号决定方向
void Set_Motor_PWM(float out_L, float out_R)
{
    uint32 duty_l, duty_r;

    // 左电机
    if (out_L >= 0) { gpio_set_level(MOTOR1_DIR, GPIO_HIGH); duty_l = (uint32)( out_L); }
    else            { gpio_set_level(MOTOR1_DIR, GPIO_LOW ); duty_l = (uint32)(-out_L); }
    if (duty_l > MOTOR_DUTY_MAX) duty_l = MOTOR_DUTY_MAX;       // 钳位，防止越界触发 pwm 断言
    pwm_set_duty(MOTOR1_PWM, duty_l);

    // 右电机
    if (out_R >= 0) { gpio_set_level(MOTOR2_DIR, GPIO_HIGH); duty_r = (uint32)( out_R); }
    else            { gpio_set_level(MOTOR2_DIR, GPIO_LOW ); duty_r = (uint32)(-out_R); }
    if (duty_r > MOTOR_DUTY_MAX) duty_r = MOTOR_DUTY_MAX;
    pwm_set_duty(MOTOR2_PWM, duty_r);
}

void Motor_UpdateRpmFromEncoder(void)
{
    MEASURE_MOTORS_SPEED();

    current_L_speed = current_L_speed * (1.0f - speed_FILTER_ALPHA) + Motor1_Speed* speed_FILTER_ALPHA;
    current_R_speed = current_R_speed * (1.0f - speed_FILTER_ALPHA) + Motor2_Speed * speed_FILTER_ALPHA;
}

void Motor_SpeedLoop(float target_L_speed, float target_R_speed)
{
    float out_l = PID_Calculate(active_left_speed_pid, target_L_speed, current_L_speed);
    float out_r = PID_Calculate(active_right_speed_pid, target_R_speed, current_R_speed);
    Set_Motor_PWM(out_l, out_r);
}

static void PID_ResetState(pid_t *pid, float initial_error)
{
	pid->Target = 0.0f;
	pid->Measure = 0.0f;
	pid->Error[0] = initial_error;
	pid->Error[1] = initial_error;
	pid->Error[2] = initial_error;
	pid->KpOut = 0.0f;
	pid->KiOut = 0.0f;
	pid->KdOut = 0.0f;
    pid->PID_Out = 0.0f;
}

bool Motor_SpeedPid_SelectProfile(motor_speed_pid_profile_t profile)
{
    switch (profile)
    {
        case MOTOR_SPEED_PID_PROFILE_DEFAULT:
            active_left_speed_pid = &pid_Motor1_Speed;
            active_right_speed_pid = &pid_Motor2_Speed;
            break;

        case MOTOR_SPEED_PID_PROFILE_QUESTION2:
            active_left_speed_pid = &pid_Question2_Motor1_Speed;
            active_right_speed_pid = &pid_Question2_Motor2_Speed;
            break;

        case MOTOR_SPEED_PID_PROFILE_QUESTION4:
            active_left_speed_pid = &pid_Question4_Motor1_Speed;
            active_right_speed_pid = &pid_Question4_Motor2_Speed;
            break;

        case MOTOR_SPEED_PID_PROFILE_QUESTION5:
            active_left_speed_pid = &pid_Question5_Motor1_Speed;
            active_right_speed_pid = &pid_Question5_Motor2_Speed;
            break;

        case MOTOR_SPEED_PID_PROFILE_QUESTION6:
            active_left_speed_pid = &pid_Question6_Motor1_Speed;
            active_right_speed_pid = &pid_Question6_Motor2_Speed;
            break;

        default:
            return false;
    }

    active_speed_pid_profile = profile;
    return true;
}

void Motor_SpeedPid_ResetActiveProfile(void)
{
    PID_ResetState(active_left_speed_pid, 0.0f);
    PID_ResetState(active_right_speed_pid, 0.0f);
}

motor_speed_pid_profile_t Motor_SpeedPid_GetActiveProfile(void)
{
    return active_speed_pid_profile;
}

void Motor_Stop(void)
{
    Set_Motor_PWM(0.0f, 0.0f);
    Motor_SpeedPid_ResetActiveProfile();
}

static void Line_Follow_ResetRuntimeState(void)
{
    line_follow_state.base_speed_mm_s =
        line_follow_state.requested_base_speed_mm_s;
    line_follow_state.last_edge_deviation = 0.0f;
    line_follow_state.edge_age_cycles = 0U;
    line_follow_state.lost_line_cycles = 0U;
    line_follow_state.recovery_armed = false;
}

void Line_Follow_ResetState(void)
{
    line_follow_state.requested_base_speed_mm_s = LINE_BASE_SPEED_MM_S;
    Line_Follow_ResetRuntimeState();
}

bool Line_Follow_SetBaseSpeed(float base_speed_mm_s)
{
    if (!isfinite(base_speed_mm_s) || base_speed_mm_s <= 0.0f)
    {
        return false;
    }

    line_follow_state.requested_base_speed_mm_s = base_speed_mm_s;
    line_follow_state.base_speed_mm_s = base_speed_mm_s;
    return true;
}

static float Line_Follow_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float Line_Follow_UpdateBaseSpeed(float deviation)
{
    float absolute_deviation = fabsf(deviation);
    float minimum_speed = line_follow_state.requested_base_speed_mm_s <
                          LINE_MIN_SPEED_MM_S
                        ? line_follow_state.requested_base_speed_mm_s
                        : LINE_MIN_SPEED_MM_S;
    float target_speed = line_follow_state.requested_base_speed_mm_s -
                          LINE_SPEED_CURVE_GAIN_MM_S * absolute_deviation *
                          absolute_deviation;
    float speed_difference;

    target_speed = Line_Follow_Clamp(target_speed,
                                     minimum_speed,
                                     line_follow_state.requested_base_speed_mm_s);
    speed_difference = target_speed - line_follow_state.base_speed_mm_s;

    if (speed_difference < -LINE_SPEED_DECEL_STEP_MM_S)
    {
        line_follow_state.base_speed_mm_s -= LINE_SPEED_DECEL_STEP_MM_S;
    }
    else if (speed_difference > LINE_SPEED_ACCEL_STEP_MM_S)
    {
        line_follow_state.base_speed_mm_s += LINE_SPEED_ACCEL_STEP_MM_S;
    }
    else
    {
        line_follow_state.base_speed_mm_s = target_speed;
    }

    return line_follow_state.base_speed_mm_s;
}

static float Line_Follow_TurnBoost(float deviation)
{
    float absolute_deviation = fabsf(deviation);
    float ratio;

    if (absolute_deviation <= LINE_TURN_BOOST_START_DEVIATION)
    {
        return 1.0f;
    }
    if (absolute_deviation >= LINE_TURN_BOOST_FULL_DEVIATION)
    {
        return LINE_TURN_BOOST_MAX;
    }

    ratio = (absolute_deviation - LINE_TURN_BOOST_START_DEVIATION) /
            (LINE_TURN_BOOST_FULL_DEVIATION -
             LINE_TURN_BOOST_START_DEVIATION);
    return 1.0f + (LINE_TURN_BOOST_MAX - 1.0f) * ratio;
}

static void Line_Follow_Drive(float base_speed, float correction)
{
    float correction_limit = base_speed - LINE_MIN_WHEEL_SPEED_MM_S;

    correction = Line_Follow_Clamp(correction,
                                   -correction_limit,
                                   correction_limit);
    Motor_SpeedLoop(base_speed - correction, base_speed + correction);
}

static void Line_Follow_Stop(void)
{
    Motor_Stop();
    PID_ResetState(&pid_Turn, 0.0f);
    Line_Follow_ResetRuntimeState();
}

static void Line_Follow_RecordEdge(float deviation)
{
    if (fabsf(deviation) >= LINE_RECOVERY_ARM_DEVIATION)
    {
        line_follow_state.last_edge_deviation = deviation;
        line_follow_state.edge_age_cycles = 0U;
        line_follow_state.recovery_armed = true;
    }
    else if (line_follow_state.recovery_armed)
    {
        if (line_follow_state.edge_age_cycles < UINT8_MAX)
        {
            ++line_follow_state.edge_age_cycles;
        }
        if (line_follow_state.edge_age_cycles >
            LINE_RECOVERY_EDGE_MEMORY_CYCLES)
        {
            line_follow_state.recovery_armed = false;
        }
    }
}

static bool Line_Follow_HandleLineLoss(grayscale_status_t status)
{
    float correction;
    float recovery_base_speed;
    float recovery_correction;

    if (status != GRAYSCALE_STATUS_LINE_LOST ||
        !line_follow_state.recovery_armed ||
        line_follow_state.edge_age_cycles >
            LINE_RECOVERY_EDGE_MEMORY_CYCLES ||
        line_follow_state.lost_line_cycles >=
            LINE_RECOVERY_MAX_LOST_CYCLES)
    {
        Line_Follow_Stop();
        return false;
    }

    if (line_follow_state.lost_line_cycles == 0U)
    {
        PID_ResetState(&pid_Turn, 0.0f);
    }
    ++line_follow_state.lost_line_cycles;
    recovery_base_speed =
        line_follow_state.requested_base_speed_mm_s < LINE_RECOVERY_BASE_SPEED_MM_S
      ? line_follow_state.requested_base_speed_mm_s
      : LINE_RECOVERY_BASE_SPEED_MM_S;
    recovery_correction =
        recovery_base_speed < LINE_RECOVERY_CORRECTION_MM_S
      ? recovery_base_speed
      : LINE_RECOVERY_CORRECTION_MM_S;
    correction = (line_follow_state.last_edge_deviation > 0.0f)
               ? -recovery_correction
               : recovery_correction;
    Line_Follow_Drive(recovery_base_speed, correction);
    return true;
}

static float Turn_LimitSpeed(float target_speed, float error)
{
	PID_Limit(&target_speed, TURN_MAX_SPEED_MM_S);

	if ((error > TURN_ANGLE_TOLERANCE_DEG) &&
		(target_speed > 0.0f) && (target_speed < TURN_MIN_SPEED_MM_S))
	{
		return TURN_MIN_SPEED_MM_S;
	}

	if ((error < -TURN_ANGLE_TOLERANCE_DEG) &&
		(target_speed < 0.0f) && (target_speed > -TURN_MIN_SPEED_MM_S))
	{
		return -TURN_MIN_SPEED_MM_S;
	}

	return target_speed;
}

bool Turn_Control(float relative_angle)
{
	uint32 elapsed_ms = 0U;
	uint32 settled_samples = 0U;
	bool target_reached = false;
	float current_angle;
	float target_angle;
	float error;
	motor_speed_pid_profile_t previous_speed_pid_profile;

	current_angle = angle_Z;
	if (!isfinite(relative_angle) ||
		(fabsf(relative_angle) > TURN_MAX_RELATIVE_ANGLE_DEG) ||
		!isfinite(current_angle))
	{
		Set_Motor_PWM(0.0f, 0.0f);
		return false;
	}

	target_angle = current_angle + relative_angle;
	error = target_angle - current_angle;

	previous_speed_pid_profile = Motor_SpeedPid_GetActiveProfile();
	(void)Motor_SpeedPid_SelectProfile(MOTOR_SPEED_PID_PROFILE_DEFAULT);
	Motor_SpeedPid_ResetActiveProfile();
	PID_ResetState(&pid_Angle, error);

	while (elapsed_ms < TURN_TIMEOUT_MS)
	{
		float target_turn_speed;

		current_angle = angle_Z;
		if (!isfinite(current_angle))
		{
			break;
		}
		error = target_angle - current_angle;

		if (fabsf(error) <= TURN_ANGLE_TOLERANCE_DEG)
		{
			Set_Motor_PWM(0.0f, 0.0f);
			++settled_samples;
			if (settled_samples >= TURN_SETTLE_SAMPLE_COUNT)
			{
				target_reached = true;
				break;
			}
		}
		else
		{
			float motor_turn_speed;

			settled_samples = 0U;
			target_turn_speed = PID_Calculate(&pid_Angle, target_angle, current_angle);
			target_turn_speed = Turn_LimitSpeed(target_turn_speed, error);
			motor_turn_speed = target_turn_speed * TURN_YAW_DIRECTION_SIGN;
			Motor_SpeedLoop(motor_turn_speed, -motor_turn_speed);
		}

		system_delay_ms(TURN_CONTROL_PERIOD_MS);
		elapsed_ms += TURN_CONTROL_PERIOD_MS;
	}

	Set_Motor_PWM(0.0f, 0.0f);
	PID_ResetState(&pid_Angle, 0.0f);
	Motor_SpeedPid_ResetActiveProfile();
	(void)Motor_SpeedPid_SelectProfile(previous_speed_pid_profile);

	return target_reached;
}

void Line_Follow_Control(void)
{
	grayscale_frame_t frame = {0};
	float base_speed;
	float correction;
	grayscale_status_t status;
	status = Grayscale_Read(&frame);

	if (status != GRAYSCALE_STATUS_OK || !frame.line_detected)
	{
		(void)Line_Follow_HandleLineLoss(status);
		return;
	}

	if (line_follow_state.lost_line_cycles != 0U)
	{
		PID_ResetState(&pid_Turn, 0.0f);
		line_follow_state.last_edge_deviation = 0.0f;
		line_follow_state.edge_age_cycles = 0U;
		line_follow_state.recovery_armed = false;
	}
	line_follow_state.lost_line_cycles = 0U;
	Line_Follow_RecordEdge(frame.deviation);
	base_speed = Line_Follow_UpdateBaseSpeed(frame.deviation);

/*
 * Target = 0:期望线在中间
 * Measure = frame.deviation
 * correction = Target - Measure = 0 - frame.deviation = -frame.deviation
 * 线在右侧时 deviation > 0, correction < 0
 */
	correction = PID_Calculate(&pid_Turn, 0.0f, frame.deviation);
	/* pid_Turn currently uses Ki = Kd = 0; enable them only after retuning. */
	correction *= Line_Follow_TurnBoost(frame.deviation);
	correction = Line_Follow_Clamp(correction,
	                               -LINE_MAX_CORRECTION_MM_S,
	                               LINE_MAX_CORRECTION_MM_S);
	Line_Follow_Drive(base_speed, correction);
}
