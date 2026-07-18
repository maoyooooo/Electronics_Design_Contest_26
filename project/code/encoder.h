#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "zf_common_headfile.h"

/*====================================================================================================================
 *  磁编码器驱动（混合方案：一路硬件 QEI + 一路外部中断软件正交）
 *  ------------------------------------------------------------------------------------------------------------------
 *  背景：
 *    MSPM0G3507 仅有 TIMG8 支持硬件正交(QEI)，只能给一个编码器用；第二个编码器只能靠外部中断
 *    在软件里做正交解码。两路都做成“连续计数 + 4 倍频”，左右对称，共用同一套速度/路程换算系数。
 *
 *  引脚 / 资源分配（均在允许使用的引脚内，且避开电机 PWM 的 TIM_A0）：
 *      左轮 Motor1 -> TIMG8 硬件 QEI ：A相=B21，B相=B22   （方向由硬件自动判别，零 CPU 开销）
 *      右轮 Motor2 -> 外部中断软件正交：A相=B17，B相=B18   （双边沿中断 + 查表 4 倍频解码）
 *
 *  为什么不用“方向模式(encoder_dir_init)”：
 *      方向模式在采样时刻读一次 B 相电平定正负，但磁编码器 B 相是持续翻转的正交方波，
 *      采样时刻电平随机，导致符号乱跳（实测已验证）。正确做法是在 A/B 边沿处读取（此时电平稳定），
 *      即硬件 QEI 或本文件的软件正交解码。
 *==================================================================================================================*/

/*--------------------------------- 引脚与资源配置（可修改） -------------------------------------------------------*/
// 左轮：硬件 QEI（注意 A/B 引脚枚举必须都属于 TIMG8）
#define ENCODER_L_TIMER          (TIM_G8)                        // 左轮使用的定时器（QEI 仅 TIMG8 可用）
#define ENCODER_L_QEI_A_PIN      (TIMG8_ENCODER1_CH1_A26)        // 左轮 A 相
#define ENCODER_L_QEI_B_PIN      (TIMG8_ENCODER1_CH2_A27)        // 左轮 B 相

// 右轮：外部中断软件正交（普通可中断 GPIO 即可）
#define ENCODER_R_A_PIN          (B17)                           // 右轮 A 相
#define ENCODER_R_B_PIN          (B18)                           // 右轮 B 相

// 方向修正：左右电机镜像安装，前进时一正一负。若某轮方向与实际相反，把对应值改成 -1。
#define ENCODER_L_DIR_SIGN       (-1)
#define ENCODER_R_DIR_SIGN       (1)

/*--------------------------------- 电机 / 车轮物理参数 -----------------------------------------------------------*/
#define ENCODE_LINE     13          // 编码器线数（单圈脉冲数）
#define JIANSUBI        28          // 减速比
#define BEIPIN          4           // 倍频：QEI 与软件正交都做 4 倍频，故为 4（左右一致）
#define SAMPLE_TIME     0.01f       // 采样周期，单位 s（须与调用 MEASURE_MOTORS_SPEED 的周期一致，默认 10ms）

// 每个采样周期内“一圈”对应的计数值，用于把计数换算成转速
#define CC              ((float)(ENCODE_LINE * JIANSUBI * BEIPIN) * SAMPLE_TIME)

#define PI              3.1415f
#define RR              65.0f       // 车轮直径，单位 mm（速度输出单位 mm/s，路程单位 mm）

/*--------------------------------- 分段路程跟踪结构体 -------------------------------------------------------------*/
typedef struct
{
    float segment_start_distance;   // 分段起点时的总路程
    float segment_distance;         // 当前分段已走距离
} DistanceTracker;

extern volatile DistanceTracker dist_tracker;

/*--------------------------------- 对外变量 ----------------------------------------------------------------------*/
extern int32_t Motor1_Encoder_Value;    // 左轮最近一次采样的原始计数（已含方向符号）
extern int32_t Motor2_Encoder_Value;    // 右轮最近一次采样的原始计数（已含方向符号）
extern float   Motor1_Speed;            // 左轮速度 mm/s
extern float   Motor2_Speed;            // 右轮速度 mm/s
extern float   Motor1_Lucheng;          // 左轮累计路程 mm
extern float   Motor2_Lucheng;          // 右轮累计路程 mm
extern float   Measure_Distance;        // 左右平均累计路程 mm

/*--------------------------------- 对外函数 ----------------------------------------------------------------------*/
// 编码器初始化：main 中外设初始化阶段调用一次
void  encoder_init          (void);

// 直接读取原始计数（读取后自动清零、已叠加方向符号）。可单独使用，与下面的速度函数二选一即可。
int16 encoder_read_left     (void);
int16 encoder_read_right    (void);

// 分别计算左/右轮速度（内部读取计数、清零、换算，结果写入 Motor1_Speed / Motor2_Speed）
void  Motor1_Get_Speed      (void);
void  Motor2_Get_Speed      (void);

// 采集全部电机速度 + 累计路程（建议放在周期为 SAMPLE_TIME 的定时中断/任务中调用）
void  MEASURE_MOTORS_SPEED  (void);

// 重置分段路程起点（状态机切换阶段时调用）
void  Reset_Segment_Distance(void);

#endif
