#include "zf_common_headfile.h"

/*====================================================================================================================
 *  磁编码器驱动实现（混合方案）
 *    左轮 Motor1：TIMG8 硬件 QEI            —— encoder_quad_init / encoder_get_count / encoder_clear_count
 *    右轮 Motor2：外部中断 + 软件正交解码   —— exti_init 双边沿触发，查表 4 倍频
 *
 *  使用流程：
 *    1. 上电初始化阶段调用一次 encoder_init();
 *    2. 在周期为 SAMPLE_TIME 的定时中断（如 PIT）或任务中调用 MEASURE_MOTORS_SPEED();
 *       之后即可直接读取 Motor1_Speed / Motor2_Speed（mm/s）与 Measure_Distance（mm）。
 *    3. 若只想拿原始计数，可直接用 encoder_read_left() / encoder_read_right()。
 *==================================================================================================================*/

/*--------------------------------- 全局变量 ----------------------------------------------------------------------*/
int32_t Motor1_Encoder_Value = 0;       // 左轮原始计数（已含方向符号）
int32_t Motor2_Encoder_Value = 0;       // 右轮原始计数（已含方向符号）

float Motor1_Speed = 0;                 // 左轮速度 mm/s
float Motor2_Speed = 0;                 // 右轮速度 mm/s

float Motor1_Lucheng = 0;               // 左轮累计路程 mm
float Motor2_Lucheng = 0;               // 右轮累计路程 mm
float Measure_Distance = 0;             // 左右平均累计路程 mm

volatile DistanceTracker dist_tracker = {0};

/*--------------------------------- 右轮软件正交解码（外部中断内累加） ---------------------------------------------*/
// 正交状态转移查表：index = (上一次状态 << 2) | (本次状态)，状态 = (A<<1)|B
// 合法的相邻状态跳变对应 +1/-1，非法跳变（漏边沿/抖动）记 0，天然抗抖动。
static const int8_t encoder_quad_table[16] =
{
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static volatile int32_t encoder_r_pulse = 0;    // 右轮脉冲累加值（ISR 内累加）
static uint8_t          encoder_r_state = 0;    // 右轮上一次的 (A<<1)|B 状态

// 右轮 A/B 相中断回调（A、B 两个引脚共用此回调，双边沿都会触发）
static volatile int32_t encoder_l_distance_count = 0;
static volatile int32_t encoder_r_distance_count = 0;

static void encoder_r_exti_handler(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;

    uint8_t a = (uint8_t)gpio_get_level(ENCODER_R_A_PIN);
    uint8_t b = (uint8_t)gpio_get_level(ENCODER_R_B_PIN);
    uint8_t state = (uint8_t)((a << 1) | b);

    encoder_r_pulse += encoder_quad_table[(encoder_r_state << 2) | state];
    encoder_r_state  = state;
}

/*--------------------------------------------------------------------------------------------------------------------
 *  编码器初始化
 *------------------------------------------------------------------------------------------------------------------*/
void encoder_init(void)
{
    // 左轮：硬件 QEI（方向由硬件自动判别，4 倍频）
    encoder_quad_init(ENCODER_L_TIMER, ENCODER_L_QEI_A_PIN, ENCODER_L_QEI_B_PIN);
    encoder_clear_count(ENCODER_L_TIMER);

    // 右轮：外部中断软件正交（exti_init 内部已把引脚配为上拉输入，可直接读电平）
    exti_init(ENCODER_R_A_PIN, EXTI_TRIGGER_BOTH, encoder_r_exti_handler, NULL);
    exti_init(ENCODER_R_B_PIN, EXTI_TRIGGER_BOTH, encoder_r_exti_handler, NULL);

    // 记录右轮初始状态，清零累加值
    encoder_r_state = (uint8_t)((gpio_get_level(ENCODER_R_A_PIN) << 1) | gpio_get_level(ENCODER_R_B_PIN));
    encoder_r_pulse = 0;
    encoder_l_distance_count = 0;
    encoder_r_distance_count = 0;
}

/*--------------------------------------------------------------------------------------------------------------------
 *  读取原始计数（读取后自动清零，返回值已叠加方向符号修正）
 *------------------------------------------------------------------------------------------------------------------*/
int16 encoder_read_left(void)
{
    int16 count = (int16)(encoder_get_count(ENCODER_L_TIMER) * ENCODER_L_DIR_SIGN);
    encoder_clear_count(ENCODER_L_TIMER);
    return count;
}

int16 encoder_read_right(void)
{
    int32_t count;

    // 与中断共享变量，需临界区内原子地“读取 + 清零”，避免竞态丢计数
    uint32 primask = interrupt_global_disable();
    count = encoder_r_pulse;
    encoder_r_pulse = 0;
    interrupt_global_enable(primask);

    return (int16)(count * ENCODER_R_DIR_SIGN);
}

/*--------------------------------------------------------------------------------------------------------------------
 *  左轮速度计算：读取计数 -> 清零 -> 换算为 mm/s
 *------------------------------------------------------------------------------------------------------------------*/
void Motor1_Get_Speed(void)
{
    Motor1_Encoder_Value = encoder_read_left();
    Motor1_Speed = (float)Motor1_Encoder_Value / CC * PI * RR;
}

/*--------------------------------------------------------------------------------------------------------------------
 *  右轮速度计算
 *------------------------------------------------------------------------------------------------------------------*/
void Motor2_Get_Speed(void)
{
    Motor2_Encoder_Value = encoder_read_right();
    Motor2_Speed = (float)Motor2_Encoder_Value / CC * PI * RR;
}

/*--------------------------------------------------------------------------------------------------------------------
 *  采集所有电机速度并累计路程（每 SAMPLE_TIME 调用一次）
 *------------------------------------------------------------------------------------------------------------------*/
void MEASURE_MOTORS_SPEED(void)
{
    Motor1_Get_Speed();
    Motor2_Get_Speed();

    encoder_l_distance_count += Motor1_Encoder_Value;
    encoder_r_distance_count += Motor2_Encoder_Value;

    Motor1_Lucheng += Motor1_Speed * SAMPLE_TIME;   // 路程累计
    Motor2_Lucheng += Motor2_Speed * SAMPLE_TIME;   // 路程累计
    Measure_Distance = Motor1_Lucheng / 2.0f + Motor2_Lucheng / 2.0f;

    // 计算当前分段路程
    dist_tracker.segment_distance = Measure_Distance - dist_tracker.segment_start_distance;

    // 防止长时间运行后浮点数累积过大
}

/*--------------------------------------------------------------------------------------------------------------------
 *  重置分段路程起点（状态机切换阶段时调用）
 *------------------------------------------------------------------------------------------------------------------*/
bool encoder_get_distance(encoder_distance_t *distance)
{
    int32_t left_count;
    int32_t right_count;
    uint32 primask;

    if (distance == NULL)
    {
        return false;
    }

    primask = interrupt_global_disable();
    left_count = encoder_l_distance_count;
    right_count = encoder_r_distance_count;
    interrupt_global_enable(primask);

    distance->left_mm = (float)left_count * ENCODER_MM_PER_COUNT;
    distance->right_mm = (float)right_count * ENCODER_MM_PER_COUNT;
    return true;
}

void encoder_reset_distance(void)
{
    uint32 primask = interrupt_global_disable();

    encoder_l_distance_count = 0;
    encoder_r_distance_count = 0;

    interrupt_global_enable(primask);
}

void Reset_Segment_Distance(void)
{
    dist_tracker.segment_start_distance = Measure_Distance;
    dist_tracker.segment_distance = 0;
}
