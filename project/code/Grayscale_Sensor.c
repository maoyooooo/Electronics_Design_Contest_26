#include <stddef.h>

#include "zf_common_headfile.h"

/*
 * 当前使用的八路标定值。
 * 默认 line=0、background=4095，假设“目标线 ADC 低、背景 ADC 高”，
 * 只适合初步上电检查，不是极性无关的有效标定。实车必须调用
 * Grayscale_SetCalibration() 写入每一路实际的线面值和背景值。
 */
static grayscale_calibration_t grayscale_calibration =
{
    {136U, 140U, 142U, 143U, 144U, 145U, 144U, 145U},
    {403U, 398U, 274U, 588U, 496U, 452U, 427U, 208U}
};

static void Grayscale_SelectChannel(uint8 channel)
{
    uint8 mux_address = channel;

    /*
     * S2、S1、S0 组成一个三位二进制地址，可选择 0..7 共八路输入：
     * 地址 0 = 000，地址 1 = 001，...，地址 7 = 111。
     *
     * 用户提供的感为例程写的是 !(channel & bit)，所以发送到模块前
     * 要把三位全部取反。与 0x07 异或等价于只翻转最低三位：
     * channel 0 -> mux_address 7，channel 1 -> 6，...，channel 7 -> 0。
     */
#if GRAYSCALE_MUX_ADDRESS_INVERTED
    mux_address ^= 0x07U;
#endif

    /* 分别取 mux_address 的 bit0、bit1、bit2，送到 S0、S1、S2。 */
    gpio_set_level(GRAYSCALE_S0_PIN, mux_address & 0x01U);
    gpio_set_level(GRAYSCALE_S1_PIN, (mux_address >> 1U) & 0x01U);
    gpio_set_level(GRAYSCALE_S2_PIN, (mux_address >> 2U) & 0x01U);
}

static bool Grayscale_CalibrationIsValid(
    const grayscale_calibration_t *calibration)
{
    uint8 channel;

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        uint16 line_level = calibration->line_level[channel];
        uint16 background_level = calibration->background_level[channel];
        uint16 span;

        /* 12 位 ADC 不可能产生大于 4095 的有效标定值。 */
        if (line_level > GRAYSCALE_ADC_MAX_VALUE ||
            background_level > GRAYSCALE_ADC_MAX_VALUE)
        {
            return false;
        }

        /*
         * 这里只关心线面与背景的差值大小，不限制两者谁大谁小。
         * 差值太小时，几个 ADC 计数的噪声就会被放大成很大的误差。
         */
        span = line_level > background_level
             ? (uint16)(line_level - background_level)
             : (uint16)(background_level - line_level);
        if (span < GRAYSCALE_MIN_CALIBRATION_SPAN)
        {
            return false;
        }
    }

    return true;
}

static uint16 Grayscale_NormalizeChannel(uint16 raw,
                                         uint16 line_level,
                                         uint16 background_level)
{
    /*
     * 归一化公式：
     *
     *   strength = (raw - background) * 1000
     *              --------------------------------
     *                    (line - background)
     *
     * raw 等于背景时 strength=0，raw 等于目标线时 strength=1000。
     * 当黑线 ADC 更低时，分子和分母同时为负；当亮线 ADC 更高时，
     * 两者同时为正，所以同一公式可以兼容两种传感器极性。
     */
    int32 span = (int32)line_level - (int32)background_level;
    int32 value = (int32)raw - (int32)background_level;
    int32 strength;

    /* ADC 比背景更远离目标线时，强度钳位为 0，避免出现负数。 */
    if ((span > 0 && value <= 0) || (span < 0 && value >= 0))
    {
        return 0U;
    }

    /* ADC 超过目标线标定点时，强度钳位为 1000，避免超量程。 */
    if ((span > 0 && value >= span) || (span < 0 && value <= span))
    {
        return GRAYSCALE_STRENGTH_MAX;
    }

    strength = value * (int32)GRAYSCALE_STRENGTH_MAX / span;
    return (uint16)strength;
}

void Grayscale_Init(void)
{
    /* S0/S1/S2 是数字输出，OUT 由逐飞库初始化为 ADC 模拟输入。 */
    gpio_init(GRAYSCALE_S0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_S1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(GRAYSCALE_S2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    adc_init(GRAYSCALE_OUT_PIN, GRAYSCALE_ADC_RESOLUTION);
    DL_ADC12_setSampleTime0(ADC0, GRAYSCALE_ADC_SAMPLE_TIME_CYCLES - 1U);
}

grayscale_status_t Grayscale_ReadRaw(
    uint16 raw[GRAYSCALE_CHANNEL_COUNT])
{
    uint8 channel;

    if (raw == NULL)
    {
        return GRAYSCALE_STATUS_INVALID_ARGUMENT;
    }

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        uint8 output_index = channel;

        /*
         * 感为例程的 Direction=1 使用 result[7-i] 保存数据。
         * 结合前面的地址取反后，output_index 最终等于模块的 MUX 地址号。
         * 这只能保证 raw[] 下标与 MUX 地址一致；地址号是否对应实车的
         * 左到右位置，还要根据模块朝向逐路遮挡确认。
         */
#if GRAYSCALE_REVERSE_CHANNEL_ORDER
        output_index = (uint8)(GRAYSCALE_CHANNEL_COUNT - 1U - channel);
#endif

        Grayscale_SelectChannel(channel);              /* 第 1 步：切换复用通道。 */
        system_delay_us(GRAYSCALE_MUX_SETTLE_US);      /* 第 2 步：等待模拟电压稳定。 */

        /*
         * 第 3 步：丢弃切换后的第一次 ADC 结果。第一次转换可能还残留
         * 上一个通道采样保持电容中的电压，直接使用会造成相邻通道串扰。
         */
        (void)adc_convert(GRAYSCALE_OUT_PIN);

        /* 第 4 步：连续采样 8 次取平均，降低电机和环境光造成的抖动。 */
        raw[output_index] = adc_mean_filter_convert(
            GRAYSCALE_OUT_PIN, GRAYSCALE_ADC_SAMPLE_COUNT);
    }

    return GRAYSCALE_STATUS_OK;
}

grayscale_status_t Grayscale_ProcessFrame(
    const uint16 raw[GRAYSCALE_CHANNEL_COUNT],
    const grayscale_calibration_t *calibration,
    grayscale_frame_t *frame)
{
    grayscale_frame_t next_frame = {0};
    uint32 weighted_position = 0U;
    uint8 channel;

    if (raw == NULL || calibration == NULL || frame == NULL)
    {
        return GRAYSCALE_STATUS_INVALID_ARGUMENT;
    }

    if (!Grayscale_CalibrationIsValid(calibration))
    {
        return GRAYSCALE_STATUS_INVALID_CALIBRATION;
    }

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        /* 先把该路原始 ADC 按它自己的线面/背景标定归一化到 0..1000。 */
        uint16 strength = Grayscale_NormalizeChannel(
            raw[channel],
            calibration->line_level[channel],
            calibration->background_level[channel]);

        next_frame.raw[channel] = raw[channel];
        next_frame.strength[channel] = strength;
        next_frame.total_strength += strength;

        /*
         * 给八路传感器分配位置 0、1000、2000、...、7000。
         * strength 越大的通道，对最终线位置的影响越大。
         * weighted_position 暂存“强度 * 位置”的总和。
         */
        weighted_position += (uint32)strength * (uint32)channel * 1000U;
    }

    /*
     * 所有通道的强度总和太小时，画面基本只有背景或噪声。
     * 此时不能用很小的总强度计算质心，否则偏差会被噪声主导。
     */
    if (next_frame.total_strength < GRAYSCALE_LINE_DETECT_THRESHOLD)
    {
        next_frame.center_position = GRAYSCALE_CENTER_POSITION;
        next_frame.deviation = 0.0f;
        next_frame.line_detected = false;
        *frame = next_frame;
        return GRAYSCALE_STATUS_LINE_LOST;
    }

    /*
     * 加权质心公式：
     *
     *   center_position = sum(strength[i] * position[i])
     *                     --------------------------------
     *                              sum(strength[i])
     *
     * 例如只有第 0 路看到线，位置接近 0；只有第 7 路看到线，位置
     * 接近 7000；第 3、4 路强度相同时，质心正好位于 3500。
     * 分子加 total_strength/2 是整数除法的四舍五入处理。
     */
    next_frame.center_position = (uint16)(
        (weighted_position + next_frame.total_strength / 2U) /
        next_frame.total_strength);
    /*
     * 把 0..7000 的绝对质心转换为相对阵列中心的循迹误差：
     *
     *   deviation = (center_position - 3500) / 1000
     *
     * 对应八路权重为 -3.5、-2.5、-1.5、-0.5、
     *                  +0.5、+1.5、+2.5、+3.5。
     * 在已经确认 raw[0] 对应实车左侧、raw[7] 对应右侧的前提下，
     * 线在左侧时误差为负，居中为 0，线在右侧时误差为正。
     */
    next_frame.deviation =
        ((float)next_frame.center_position -
         (float)GRAYSCALE_CENTER_POSITION) / 1000.0f;
    next_frame.line_detected = true;
    *frame = next_frame;

    return GRAYSCALE_STATUS_OK;
}

grayscale_status_t Grayscale_Read(grayscale_frame_t *frame)
{
    uint16 raw[GRAYSCALE_CHANNEL_COUNT];
    grayscale_status_t status;

    if (frame == NULL)
    {
        return GRAYSCALE_STATUS_INVALID_ARGUMENT;
    }

    /* 一次完整调用包含“硬件扫描”和“标定后误差计算”两个阶段。 */
    status = Grayscale_ReadRaw(raw);
    if (status != GRAYSCALE_STATUS_OK)
    {
        return status;
    }

    return Grayscale_ProcessFrame(raw, &grayscale_calibration, frame);
}

grayscale_status_t Grayscale_SetCalibration(
    const uint16 line_level[GRAYSCALE_CHANNEL_COUNT],
    const uint16 background_level[GRAYSCALE_CHANNEL_COUNT])
{
    grayscale_calibration_t next_calibration;
    uint8 channel;

    if (line_level == NULL || background_level == NULL)
    {
        return GRAYSCALE_STATUS_INVALID_ARGUMENT;
    }

    /* 先复制到临时结构，全部校验通过后再替换当前标定。 */
    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        next_calibration.line_level[channel] = line_level[channel];
        next_calibration.background_level[channel] =
            background_level[channel];
    }

    if (!Grayscale_CalibrationIsValid(&next_calibration))
    {
        return GRAYSCALE_STATUS_INVALID_CALIBRATION;
    }

    grayscale_calibration = next_calibration;
    return GRAYSCALE_STATUS_OK;
}

void Grayscale_PrintRawAverage(const char *name)
{
    uint16 raw[GRAYSCALE_CHANNEL_COUNT];
    uint32 sum[GRAYSCALE_CHANNEL_COUNT] = {0U};
    uint8 channel;
    uint8 frame;

    for (frame = 0U; frame < GRAYSCALE_CALIBRATION_FRAMES; ++frame)
    {
        if (Grayscale_ReadRaw(raw) != GRAYSCALE_STATUS_OK)
        {
            printf("Grayscale raw read failed.\r\n");
            return;
        }

        for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
        {
            sum[channel] += raw[channel];
        }

        system_delay_ms(5);
    }

    printf("%s = {", name);

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        printf("%u%s",
               (unsigned int)(sum[channel] /
                              GRAYSCALE_CALIBRATION_FRAMES),
               (channel == GRAYSCALE_CHANNEL_COUNT - 1U) ? "};\r\n" : ", ");
    }
}
