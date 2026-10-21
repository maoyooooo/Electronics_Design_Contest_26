#include "Task_Notify.h"

#define TASK_NOTIFY_UART_INDEX              (UART_2)
#define TASK_NOTIFY_UART_BAUDRATE           (115200U)
#define TASK_NOTIFY_UART_TX_PIN             (UART2_TX_B15)
#define TASK_NOTIFY_UART_RX_PIN             (UART2_RX_B16)

#define TASK_NOTIFY_FRAME_HEADER_0          (0xA5U)
#define TASK_NOTIFY_FRAME_HEADER_1          (0x5AU)
#define TASK_NOTIFY_PROTOCOL_VERSION        (0x01U)
#define TASK_NOTIFY_MESSAGE_ENTER_QUESTION  (0x01U)
#define TASK_NOTIFY_MESSAGE_QUESTION6_DISTANCE (0x02U)
#define TASK_NOTIFY_ENTER_FRAME_LENGTH      (6U)
#define TASK_NOTIFY_DISTANCE_FRAME_LENGTH   (8U)
#define TASK_NOTIFY_ENTER_CRC_DATA_LENGTH   (TASK_NOTIFY_ENTER_FRAME_LENGTH - 1U)
#define TASK_NOTIFY_DISTANCE_CRC_DATA_LENGTH (TASK_NOTIFY_DISTANCE_FRAME_LENGTH - 1U)
#define TASK_NOTIFY_REPEAT_COUNT            (3U)

static bool Task_Notify_IsValidQuestion(uint8 question_number)
{
    return (question_number == 2U) || (question_number == 3U) ||
           (question_number == 4U) ||
           (question_number == 5U) || (question_number == 6U);
}

static uint8 Task_Notify_CalculateCrc8(const uint8 *data, uint8 length)
{
    uint8 crc = 0U;
    uint8 byte_index;
    uint8 bit_index;

    for (byte_index = 0U; byte_index < length; ++byte_index)
    {
        crc ^= data[byte_index];
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            crc = ((crc & 0x80U) != 0U) ? (uint8)((crc << 1U) ^ 0x07U) :
                                            (uint8)(crc << 1U);
        }
    }

    return crc;
}

void Task_Notify_Init(void)
{
    uart_init(TASK_NOTIFY_UART_INDEX,
              TASK_NOTIFY_UART_BAUDRATE,
              TASK_NOTIFY_UART_TX_PIN,
              TASK_NOTIFY_UART_RX_PIN);
}

void Task_Notify_SendQuestion(uint8 question_number)
{
    uint8 frame[TASK_NOTIFY_ENTER_FRAME_LENGTH] =
    {
        TASK_NOTIFY_FRAME_HEADER_0,
        TASK_NOTIFY_FRAME_HEADER_1,
        TASK_NOTIFY_PROTOCOL_VERSION,
        TASK_NOTIFY_MESSAGE_ENTER_QUESTION,
        question_number,
        0U,
    };
    uint8 repeat_index;

    if (!Task_Notify_IsValidQuestion(question_number))
    {
        return;
    }

    frame[TASK_NOTIFY_ENTER_FRAME_LENGTH - 1U] =
        Task_Notify_CalculateCrc8(frame, TASK_NOTIFY_ENTER_CRC_DATA_LENGTH);

    for (repeat_index = 0U; repeat_index < TASK_NOTIFY_REPEAT_COUNT; ++repeat_index)
    {
        uart_write_buffer(TASK_NOTIFY_UART_INDEX, frame, TASK_NOTIFY_ENTER_FRAME_LENGTH);
    }
}

void Task_Notify_SendQuestion6DistanceMm(int16 distance_mm)
{
    uint16 encoded_distance = (uint16)distance_mm;
    uint8 frame[TASK_NOTIFY_DISTANCE_FRAME_LENGTH] =
    {
        TASK_NOTIFY_FRAME_HEADER_0,
        TASK_NOTIFY_FRAME_HEADER_1,
        TASK_NOTIFY_PROTOCOL_VERSION,
        TASK_NOTIFY_MESSAGE_QUESTION6_DISTANCE,
        6U,
        (uint8)(encoded_distance >> 8U),
        (uint8)encoded_distance,
        0U,
    };
    uint8 repeat_index;

    frame[TASK_NOTIFY_DISTANCE_FRAME_LENGTH - 1U] =
        Task_Notify_CalculateCrc8(frame, TASK_NOTIFY_DISTANCE_CRC_DATA_LENGTH);

    for (repeat_index = 0U; repeat_index < TASK_NOTIFY_REPEAT_COUNT; ++repeat_index)
    {
        uart_write_buffer(TASK_NOTIFY_UART_INDEX,
                          frame,
                          TASK_NOTIFY_DISTANCE_FRAME_LENGTH);
    }
}
