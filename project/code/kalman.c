#include "zf_common_headfile.h"
#include "kalman.h"
/*
使用示例
1.调用gyroOffset_init()函数进行陀螺仪校准，确保GyroOffset_init为true。
2.在主循环中调用ICM_getEulerianAngles()函数获取欧拉角数据。
3.通过eulerAngle结构体访问pitch、roll和yaw角度。
注意事项
- 确保在调用ICM_getEulerianAngles()之前已经正确初始化IMU传感器并进行陀螺仪校准。
- 根据实际需求调整滤波参数alpha、Kp和Ki以获得更好的性能。
- 该代码假设IMU数据已经通过imu660ra_get_acc()和imu 660ra_get_gyro()函数获取，并存储在相应的变量中
*/

#define delta_T     0.005f  

float I_ex, I_ey, I_ez;

quater_param_t Q_info = {1, 0, 0};
volatile euler_param_t eulerAngle;

volatile icm_param_t icm_data;
gyro_param_t GyroOffset;

 volatile bool GyroOffset_init = 0;

float param_Kp = 0.0000;
float param_Ki = 0.0000;

volatile float angle_Z = 0,angle_R = 0,angle_P = 0;

float fast_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


void gyroOffset_init(void)  //陀螺仪校准
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;

    system_delay_ms(500);
     
    for (uint16_t i = 0; i < 500; ++i) 
    {
    imu660rb_get_gyro();
    GyroOffset.Xdata += imu660rb_gyro_x;
    GyroOffset.Ydata += imu660rb_gyro_y;
    GyroOffset.Zdata += imu660rb_gyro_z;
    system_delay_ms(10);
    }


    GyroOffset.Xdata /= 500;
    GyroOffset.Ydata /= 500;
    GyroOffset.Zdata /= 500;
    eulerAngle.Dirchange=0;
    
    GyroOffset_init = 1;
}


#define alpha           0.4f

void ICM_getValues()    //获取IMU数据并进行滤波
{

    icm_data.acc_x = (((float) imu660rb_acc_x) * alpha) * 8 / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = (((float) imu660rb_acc_y) * alpha) * 8 / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = (((float) imu660rb_acc_z) * alpha) * 8 / 4096 + icm_data.acc_z * (1 - alpha);



    if(fabs(icm_data.gyro_x) < 0.002f)  icm_data.gyro_x = 0;
    if(fabs(icm_data.gyro_y) < 0.002f)  icm_data.gyro_y = 0;
    if(fabs(icm_data.gyro_z) < 0.002f)  icm_data.gyro_z = 0;

    icm_data.gyro_x = ((float) imu660rb_gyro_x - GyroOffset.Xdata) * PI / 180 / 16.4f;
    icm_data.gyro_y = ((float) imu660rb_gyro_y - GyroOffset.Ydata) * PI / 180 / 16.4f;
    icm_data.gyro_z = ((float) imu660rb_gyro_z - GyroOffset.Zdata) * PI / 180 / 14.4f;  
}


/*
*@brief AHRS算法实现
*@param gx, gy, gz 陀螺仪测量的角速度（单位：弧度/秒）
*@param ax, ay, az 加速度计测量的加速度（单位：g）
*@return 无返回值，函数内部更新全局变量 Q_info 和 eulerAngle
*@note 该函数实现了一个基于互补滤波的 AHRS 算法，结合了陀螺仪和加速度计的数据来估计设备的姿态（欧拉角）。算法中包含了比例-积分控制来补偿陀螺仪的漂移，并且对四元数进行了归一化处理以保持数值稳定。
*/
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)     //AHRS算法实现
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;   
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0.17;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;


    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;



/*
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
*/
    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);

    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;



    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}


void ICM_getEulerianAngles(void)    //计算欧拉角
{
    static uint8 change_f=0;
    ICM_getValues();
    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;


    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / PI; // pitch
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI; // roll
    eulerAngle.yaw =  atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180/ PI; // yaw
//
    if (eulerAngle.yaw >= 180) { eulerAngle.yaw -= 360; } else if (eulerAngle.yaw <= -180) { eulerAngle.yaw += 360; }

    //偏航角航向处理
    if((eulerAngle.yaw-eulerAngle.last_yaw) < -350) eulerAngle.Dirchange++;
    else if ((eulerAngle.yaw-eulerAngle.last_yaw) > 350) eulerAngle.Dirchange--;

    angle_Z=360*eulerAngle.Dirchange+eulerAngle.yaw;//180*eulerAngle.Dirchange+eulerAngle.yaw 
    eulerAngle.last_yaw=eulerAngle.yaw;

}


