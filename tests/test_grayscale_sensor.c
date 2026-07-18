#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Grayscale_Sensor.h"

#define EXPECTED_ADC_SAMPLE_TIME_REGISTER_VALUE  (63U)

typedef enum
{
    EVENT_GPIO_INIT,
    EVENT_GPIO_WRITE,
    EVENT_ADC_INIT,
    EVENT_DELAY,
    EVENT_ADC_DISCARD,
    EVENT_ADC_MEAN
} test_event_type_t;

typedef struct
{
    test_event_type_t type;
    int value;
} test_event_t;

static test_event_t events[2048];
static size_t event_count;
static int gpio_levels[64];
ADC12_Regs mock_adc0;
static ADC12_Regs *sample_time_adc;
static uint16 sample_time_value;
static size_t sample_time_call_count;

typedef struct
{
    uint16 x;
    uint16 y;
    char text[16];
} display_string_call_t;

typedef struct
{
    uint16 x;
    uint16 y;
    uint32 value;
    uint8 digits;
} display_uint_call_t;

static display_string_call_t display_strings[16];
static size_t display_string_count;
static display_uint_call_t display_uints[16];
static size_t display_uint_count;

static void fail(const char *message)
{
    fprintf(stderr, "FAIL: %s\n", message);
    exit(EXIT_FAILURE);
}

static void expect_true(bool condition, const char *message)
{
    if (!condition)
    {
        fail(message);
    }
}

static void expect_float(float actual, float expected, float tolerance,
                         const char *message)
{
    if (fabsf(actual - expected) > tolerance)
    {
        fail(message);
    }
}

static void record_event(test_event_type_t type, int value)
{
    if (event_count >= (sizeof(events) / sizeof(events[0])))
    {
        fail("event buffer overflow");
    }
    events[event_count].type = type;
    events[event_count].value = value;
    ++event_count;
}

static int selected_mux_address(void)
{
    return gpio_levels[GRAYSCALE_S0_PIN] |
           (gpio_levels[GRAYSCALE_S1_PIN] << 1) |
           (gpio_levels[GRAYSCALE_S2_PIN] << 2);
}

void gpio_init(int pin, int direction, int level, int mode)
{
    (void)direction;
    (void)mode;
    gpio_levels[pin] = level;
    record_event(EVENT_GPIO_INIT, pin);
}

void gpio_set_level(int pin, int level)
{
    gpio_levels[pin] = level;
    record_event(EVENT_GPIO_WRITE, pin * 10 + level);
}

void adc_init(int pin, int resolution)
{
    expect_true(pin == GRAYSCALE_OUT_PIN, "wrong ADC input pin");
    expect_true(resolution == GRAYSCALE_ADC_RESOLUTION,
                "wrong ADC resolution");
    record_event(EVENT_ADC_INIT, pin);
}

uint16 adc_convert(int pin)
{
    expect_true(pin == GRAYSCALE_OUT_PIN, "discard used wrong ADC pin");
    record_event(EVENT_ADC_DISCARD, selected_mux_address());
    return 0U;
}

uint16 adc_mean_filter_convert(int pin, uint8 count)
{
    int mux_address = selected_mux_address();

    expect_true(pin == GRAYSCALE_OUT_PIN, "mean filter used wrong ADC pin");
    expect_true(count == GRAYSCALE_ADC_SAMPLE_COUNT,
                "wrong ADC averaging count");
    record_event(EVENT_ADC_MEAN, mux_address);
    return (uint16)(1000 + mux_address * 100);
}

void system_delay_us(uint32 time)
{
    expect_true(time == GRAYSCALE_MUX_SETTLE_US,
                "wrong mux settling delay");
    record_event(EVENT_DELAY, (int)time);
}

void system_delay_ms(uint32 time)
{
    (void)time;
}

void DL_ADC12_setSampleTime0(ADC12_Regs *adc12, uint16 adcclks)
{
    sample_time_adc = adc12;
    sample_time_value = adcclks;
    ++sample_time_call_count;
}

void ips200_show_string(uint16 x, uint16 y, const char dat[])
{
    if (display_string_count >=
        (sizeof(display_strings) / sizeof(display_strings[0])))
    {
        fail("display string buffer overflow");
    }

    display_strings[display_string_count].x = x;
    display_strings[display_string_count].y = y;
    (void)snprintf(display_strings[display_string_count].text,
                   sizeof(display_strings[display_string_count].text),
                   "%s", dat);
    ++display_string_count;
}

void ips200_show_uint(uint16 x, uint16 y, uint32 dat, uint8 num)
{
    if (display_uint_count >=
        (sizeof(display_uints) / sizeof(display_uints[0])))
    {
        fail("display integer buffer overflow");
    }

    display_uints[display_uint_count].x = x;
    display_uints[display_uint_count].y = y;
    display_uints[display_uint_count].value = dat;
    display_uints[display_uint_count].digits = num;
    ++display_uint_count;
}

static grayscale_calibration_t make_calibration(uint16 line, uint16 background)
{
    grayscale_calibration_t calibration;
    uint8 channel;

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        calibration.line_level[channel] = line;
        calibration.background_level[channel] = background;
    }
    return calibration;
}

static void test_initialization_uses_selected_pins(void)
{
    event_count = 0U;
    memset(gpio_levels, 0, sizeof(gpio_levels));

    Grayscale_Init();

    expect_true(event_count == 4U, "init should configure exactly four pins");
    expect_true(events[0].type == EVENT_GPIO_INIT &&
                events[0].value == GRAYSCALE_S0_PIN,
                "S0 was not initialized first");
    expect_true(events[1].type == EVENT_GPIO_INIT &&
                events[1].value == GRAYSCALE_S1_PIN,
                "S1 was not initialized second");
    expect_true(events[2].type == EVENT_GPIO_INIT &&
                events[2].value == GRAYSCALE_S2_PIN,
                "S2 was not initialized third");
    expect_true(events[3].type == EVENT_ADC_INIT,
                "ADC was not initialized after mux GPIOs");
}

static void test_initialization_extends_adc0_sample_time(void)
{
    sample_time_adc = NULL;
    sample_time_value = 0U;
    sample_time_call_count = 0U;

    Grayscale_Init();

    expect_true(sample_time_call_count == 1U,
                "grayscale initialization did not configure ADC0 sample time");
    expect_true(sample_time_adc == ADC0,
                "grayscale initialization configured the wrong ADC instance");
    expect_true(sample_time_value ==
                EXPECTED_ADC_SAMPLE_TIME_REGISTER_VALUE,
                "grayscale initialization used the wrong ADC sample time");
}

static void test_raw_scan_reads_all_mux_channels(void)
{
    uint16 raw[GRAYSCALE_CHANNEL_COUNT];
    uint8 channel;
    size_t scan_start;

    event_count = 0U;
    Grayscale_Init();
    scan_start = event_count;

    expect_true(Grayscale_ReadRaw(raw) == GRAYSCALE_STATUS_OK,
                "raw scan unexpectedly failed");
    expect_true(GRAYSCALE_MUX_ADDRESS_INVERTED == 1U,
                "default mux mapping no longer matches the Ganv example");
    expect_true(GRAYSCALE_REVERSE_CHANNEL_ORDER == 1U,
                "default output order no longer matches Direction=1");

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        size_t base = scan_start + (size_t)channel * 6U;
        uint8 mux_address =
            (uint8)(GRAYSCALE_CHANNEL_COUNT - 1U - channel);

        expect_true(events[base + 0U].type == EVENT_GPIO_WRITE &&
                    events[base + 0U].value ==
                    (int)(GRAYSCALE_S0_PIN * 10 +
                          (mux_address & 0x01U)),
                    "scan wrote the wrong S0 level");
        expect_true(events[base + 1U].type == EVENT_GPIO_WRITE &&
                    events[base + 1U].value ==
                    (int)(GRAYSCALE_S1_PIN * 10 +
                          ((mux_address >> 1U) & 0x01U)),
                    "scan wrote the wrong S1 level");
        expect_true(events[base + 2U].type == EVENT_GPIO_WRITE &&
                    events[base + 2U].value ==
                    (int)(GRAYSCALE_S2_PIN * 10 +
                          ((mux_address >> 2U) & 0x01U)),
                    "scan wrote the wrong S2 level");
        expect_true(events[base + 3U].type == EVENT_DELAY,
                    "scan did not wait after mux switching");
        expect_true(events[base + 4U].type == EVENT_ADC_DISCARD &&
                    events[base + 4U].value == mux_address,
                    "scan did not discard the first selected-channel sample");
        expect_true(events[base + 5U].type == EVENT_ADC_MEAN &&
                    events[base + 5U].value == mux_address,
                    "scan averaged the wrong selected channel");
        expect_true(raw[mux_address] ==
                    (uint16)(1000U + mux_address * 100U),
                    "raw channel order is inconsistent with configuration");
    }
}

static void test_dark_line_normalization_and_extremes(void)
{
    const uint16 raw[GRAYSCALE_CHANNEL_COUNT] =
        {0U, 1000U, 1000U, 1000U, 1000U, 1000U, 1000U, 1000U};
    grayscale_calibration_t calibration = make_calibration(0U, 1000U);
    grayscale_frame_t frame;

    expect_true(Grayscale_ProcessFrame(raw, &calibration, &frame) ==
                GRAYSCALE_STATUS_OK,
                "valid dark-line frame was rejected");
    expect_true(frame.strength[0] == GRAYSCALE_STRENGTH_MAX,
                "line calibration point did not normalize to full strength");
    expect_true(frame.strength[1] == 0U,
                "background calibration point did not normalize to zero");
    expect_true(frame.center_position == 0U,
                "leftmost line center is incorrect");
    expect_float(frame.deviation, -3.5f, 0.001f,
                 "leftmost line deviation is incorrect");
    expect_true(frame.line_detected, "strong line was reported lost");
}

static void test_bright_line_polarity_and_clamping(void)
{
    const uint16 raw[GRAYSCALE_CHANNEL_COUNT] =
        {0U, 0U, 0U, 0U, 0U, 0U, 500U, 1200U};
    grayscale_calibration_t calibration = make_calibration(1000U, 0U);
    grayscale_frame_t frame;

    expect_true(Grayscale_ProcessFrame(raw, &calibration, &frame) ==
                GRAYSCALE_STATUS_OK,
                "valid bright-line frame was rejected");
    expect_true(frame.strength[6] == 500U,
                "midpoint did not normalize to half strength");
    expect_true(frame.strength[7] == GRAYSCALE_STRENGTH_MAX,
                "value beyond line calibration was not clamped");
    expect_true(frame.deviation > 2.5f,
                "right-side bright line did not produce positive deviation");
}

static void test_symmetric_center_and_line_loss(void)
{
    const uint16 centered[GRAYSCALE_CHANNEL_COUNT] =
        {1000U, 1000U, 1000U, 0U, 0U, 1000U, 1000U, 1000U};
    const uint16 background[GRAYSCALE_CHANNEL_COUNT] =
        {1000U, 1000U, 1000U, 1000U, 1000U, 1000U, 1000U, 1000U};
    grayscale_calibration_t calibration = make_calibration(0U, 1000U);
    grayscale_frame_t frame;

    expect_true(Grayscale_ProcessFrame(centered, &calibration, &frame) ==
                GRAYSCALE_STATUS_OK,
                "centered line was rejected");
    expect_true(frame.center_position == GRAYSCALE_CENTER_POSITION,
                "symmetric line center is incorrect");
    expect_float(frame.deviation, 0.0f, 0.001f,
                 "symmetric line deviation is not zero");

    expect_true(Grayscale_ProcessFrame(background, &calibration, &frame) ==
                GRAYSCALE_STATUS_LINE_LOST,
                "background-only frame was not reported as line lost");
    expect_true(!frame.line_detected, "lost-line frame remained valid");
    expect_true(frame.deviation == 0.0f,
                "lost-line frame should expose a neutral deviation");
}

static void test_equal_strong_channels_remain_centered(void)
{
    const uint16 raw[GRAYSCALE_CHANNEL_COUNT] =
        {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    grayscale_calibration_t calibration = make_calibration(0U, 1000U);
    grayscale_frame_t frame;

    expect_true(Grayscale_ProcessFrame(raw, &calibration, &frame) ==
                GRAYSCALE_STATUS_OK,
                "equal strong channels were incorrectly reported lost");
    expect_true(frame.total_strength ==
                GRAYSCALE_CHANNEL_COUNT * GRAYSCALE_STRENGTH_MAX,
                "equal strong channel total is incorrect");
    expect_true(frame.center_position == GRAYSCALE_CENTER_POSITION,
                "equal strong channels were not centered");
    expect_float(frame.deviation, 0.0f, 0.001f,
                 "equal strong channels produced a deviation");
}

static void test_combined_read_uses_configured_calibration(void)
{
    uint16 line[GRAYSCALE_CHANNEL_COUNT];
    uint16 background[GRAYSCALE_CHANNEL_COUNT];
    grayscale_frame_t frame;
    uint8 channel;

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        line[channel] = 2000U;
        background[channel] = 1000U;
    }

    expect_true(Grayscale_SetCalibration(line, background) ==
                GRAYSCALE_STATUS_OK,
                "valid calibration was rejected");
    expect_true(Grayscale_Read(&frame) == GRAYSCALE_STATUS_OK,
                "combined scan and processing failed");

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        uint16 expected_raw = (uint16)(1000U + channel * 100U);

        expect_true(frame.raw[channel] == expected_raw,
                    "combined read returned the wrong raw channel");
        expect_true(frame.strength[channel] ==
                    (uint16)(channel * 100U),
                    "combined read used the wrong calibration");
    }
}

static void test_invalid_inputs_and_calibration_are_rejected(void)
{
    uint16 raw[GRAYSCALE_CHANNEL_COUNT] = {0U};
    uint16 valid_line[GRAYSCALE_CHANNEL_COUNT];
    uint16 valid_background[GRAYSCALE_CHANNEL_COUNT];
    grayscale_calibration_t calibration = make_calibration(500U, 500U);
    grayscale_frame_t frame;
    uint8 channel;

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        valid_line[channel] = 0U;
        valid_background[channel] = 1000U;
    }

    expect_true(Grayscale_ReadRaw(NULL) ==
                GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL raw scan output was accepted");
    expect_true(Grayscale_Read(NULL) == GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL processed frame was accepted");
    expect_true(Grayscale_ProcessFrame(NULL, &calibration, &frame) ==
                GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL raw frame was accepted");
    expect_true(Grayscale_ProcessFrame(raw, NULL, &frame) ==
                GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL calibration was accepted");
    expect_true(Grayscale_ProcessFrame(raw, &calibration, NULL) ==
                GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL output frame was accepted");
    expect_true(Grayscale_ProcessFrame(raw, &calibration, &frame) ==
                GRAYSCALE_STATUS_INVALID_CALIBRATION,
                "zero-span calibration was accepted");
    expect_true(Grayscale_SetCalibration(NULL, valid_background) ==
                GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL line calibration was accepted");
    expect_true(Grayscale_SetCalibration(valid_line, NULL) ==
                GRAYSCALE_STATUS_INVALID_ARGUMENT,
                "NULL background calibration was accepted");
    expect_true(Grayscale_SetCalibration(calibration.line_level,
                                         calibration.background_level) ==
                GRAYSCALE_STATUS_INVALID_CALIBRATION,
                "zero-span stored calibration was accepted");

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        valid_line[channel] = 1000U;
        valid_background[channel] =
            (uint16)(1000U + GRAYSCALE_MIN_CALIBRATION_SPAN - 1U);
    }
    expect_true(Grayscale_SetCalibration(valid_line, valid_background) ==
                GRAYSCALE_STATUS_INVALID_CALIBRATION,
                "sub-threshold calibration span was accepted");

    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; ++channel)
    {
        valid_background[channel] =
            (uint16)(1000U + GRAYSCALE_MIN_CALIBRATION_SPAN);
    }
    expect_true(Grayscale_SetCalibration(valid_line, valid_background) ==
                GRAYSCALE_STATUS_OK,
                "minimum valid calibration span was rejected");

    valid_line[3] = (uint16)(GRAYSCALE_ADC_MAX_VALUE + 1U);
    expect_true(Grayscale_SetCalibration(valid_line, valid_background) ==
                GRAYSCALE_STATUS_INVALID_CALIBRATION,
                "out-of-range 12-bit calibration was accepted");
}

static void test_raw_average_does_not_render_on_ips200(void)
{
    display_string_count = 0U;
    display_uint_count = 0U;

    Grayscale_PrintRawAverage("RAW");

    expect_true(display_string_count == 0U && display_uint_count == 0U,
                "raw average diagnostic still rendered on IPS200");
}

int main(void)
{
    test_initialization_uses_selected_pins();
    test_initialization_extends_adc0_sample_time();
    test_raw_scan_reads_all_mux_channels();
    test_dark_line_normalization_and_extremes();
    test_bright_line_polarity_and_clamping();
    test_symmetric_center_and_line_loss();
    test_equal_strong_channels_remain_centered();
    test_combined_read_uses_configured_calibration();
    test_invalid_inputs_and_calibration_are_rejected();
    test_raw_average_does_not_render_on_ips200();
    puts("PASS: grayscale sensor host tests");
    return EXIT_SUCCESS;
}
