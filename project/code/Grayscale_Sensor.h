#ifndef GRAYSCALE_SENSOR_H
#define GRAYSCALE_SENSOR_H

#include "zf_common_headfile.h"

/*
 * 接线：
 *   灰度模块 S0  -> B10，复用器地址最低位
 *   灰度模块 S1  -> B11，复用器地址中间位
 *   灰度模块 S2  -> B14，复用器地址最高位
 *   灰度模块 OUT -> B25 / ADC0_CH4，八路灰度共用的模拟输出
 *   灰度模块 ERR -> 悬空，本驱动不使用该引脚
 */
#define GRAYSCALE_S0_PIN                  (B10)
#define GRAYSCALE_S1_PIN                  (B11)
#define GRAYSCALE_S2_PIN                  (B14)
#define GRAYSCALE_OUT_PIN                 (ADC0_CH4_B25)

#define GRAYSCALE_ADC_RESOLUTION          (ADC_12BIT)  /* ADC 分辨率为 12 位。 */
#define GRAYSCALE_ADC_MAX_VALUE           (4095U)      /* 12 位 ADC 最大值。 */
#define GRAYSCALE_ADC_SAMPLE_TIME_CYCLES  (64U)        /* 高阻复用模拟输出的 ADC 采样保持时长。 */
#define GRAYSCALE_ADC_SAMPLE_COUNT        (8U)         /* 每路有效采样 8 次并取平均。 */
#define GRAYSCALE_MUX_SETTLE_US           (5U)         /* 切换通道后等待模拟信号稳定。 */
#define GRAYSCALE_MIN_CALIBRATION_SPAN    (32U)        /* 线与背景至少相差 32 个 ADC 计数。 */

#define GRAYSCALE_CHANNEL_COUNT           (8U)         
#define GRAYSCALE_STRENGTH_MAX            (1000U)      /* 完全处于目标线上的归一化强度。 */
#define GRAYSCALE_CENTER_POSITION         (3500U)      /* 八路阵列的几何中心位置。 */
#define GRAYSCALE_LINE_DETECT_THRESHOLD   (100U)       /* 总强度低于该值时认为丢线。 */
#define GRAYSCALE_CALIBRATION_FRAMES      (20U)

/* 感为例程对三个地址位均取反，即写入 !(channel & bit)。 */
#define GRAYSCALE_MUX_ADDRESS_INVERTED    (1U)

/* 与感为例程 Direction=1 一致；若实车左右方向相反，将其改为 0。 */
#define GRAYSCALE_REVERSE_CHANNEL_ORDER   (1U)

/* 灰度函数返回状态。调用者应区分“正常检测”和“丢线”。 */
typedef enum
{
    GRAYSCALE_STATUS_OK = 0,                /* 成功读取并检测到目标线。 */
    GRAYSCALE_STATUS_LINE_LOST,             /* 读取成功，但总线强度不足。 */
    GRAYSCALE_STATUS_INVALID_ARGUMENT,      /* 传入了空指针。 */
    GRAYSCALE_STATUS_INVALID_CALIBRATION    /* 标定越界或线/背景差值太小。 */
} grayscale_status_t;

/*
 * 每一路都单独标定，抵消 LED、光敏器件和安装高度造成的通道差异。
 * line_level       ：该路传感器放在目标线上时测得的 ADC 值。
 * background_level ：该路传感器放在赛道背景上时测得的 ADC 值。
 *
 * line_level 可以小于或大于 background_level，所以黑线、亮线都支持。
 */
typedef struct
{
    uint16 line_level[GRAYSCALE_CHANNEL_COUNT];
    uint16 background_level[GRAYSCALE_CHANNEL_COUNT];
} grayscale_calibration_t;

/* 调用 Grayscale_Read() 后得到的一帧完整灰度数据。 */
typedef struct
{
    uint16 raw[GRAYSCALE_CHANNEL_COUNT];      /* 八路原始 ADC 值，默认按厂家 Direction=1 排列。 */
    uint16 strength[GRAYSCALE_CHANNEL_COUNT]; /* 0..1000，越大表示越接近目标线。 */
    uint16 center_position;                   /* 0..7000，阵列中心为 3500。 */
    float deviation;                          /* -3.5..+3.5；确认安装方向后约定左负、右正。 */
    uint32 total_strength;                    /* 八路 strength 之和，用于判断丢线。 */
    bool line_detected;                       /* true 表示当前质心和偏差有效。 */
} grayscale_frame_t;

/* 初始化三个通道选择 GPIO 和 OUT 对应的 ADC，只需在启动时调用一次。 */
void Grayscale_Init(void);

/*
 * 依次扫描八个复用通道，只返回原始 ADC 数据，不做标定和误差计算。
 * raw[] 下标与模块 MUX 地址号一致。MUX 地址是否对应实车从左到右，
 * 需要逐路遮挡确认；如果方向相反，修改 GRAYSCALE_REVERSE_CHANNEL_ORDER。
 */
grayscale_status_t Grayscale_ReadRaw(
    uint16 raw[GRAYSCALE_CHANNEL_COUNT]);

/*
 * 纯数据处理函数：原始 ADC -> 每路归一化强度 -> 加权质心 -> 循迹偏差。
 * 该函数不访问 GPIO 和 ADC，便于使用已有数据单独调试误差算法。
 */
grayscale_status_t Grayscale_ProcessFrame(
    const uint16 raw[GRAYSCALE_CHANNEL_COUNT],
    const grayscale_calibration_t *calibration,
    grayscale_frame_t *frame);

/*
 * 完整读取接口：内部先调用 Grayscale_ReadRaw()，再使用当前标定计算 frame。
 * 主循环通常只需要周期性调用该函数，并读取 frame->deviation。
 */
grayscale_status_t Grayscale_Read(grayscale_frame_t *frame);

/*
 * 保存八路“目标线电平”和“背景电平”。两个数组都必须包含 8 个值。
 * 建议实车分别测量每一路，不要直接长期使用驱动中的 0/4095 默认值。
 */
grayscale_status_t Grayscale_SetCalibration(
    const uint16 line_level[GRAYSCALE_CHANNEL_COUNT],
    const uint16 background_level[GRAYSCALE_CHANNEL_COUNT]);

void Grayscale_PrintRawAverage(const char *name);

#endif
