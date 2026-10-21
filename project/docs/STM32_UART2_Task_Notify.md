# MSPM0G3507 to STM32 UART2 Task-Number Protocol

## Purpose

The MSPM0G3507 chassis controller sends the selected contest question number
when an operator starts a question from the menu. It sends only questions 2,
3, 4, 5, and 6. For Question 6 it then sends one signed distance parameter
after the operator confirms the value on the IPS200. The STM32 does not need
to send an acknowledgement.

Every valid frame is transmitted three times. Repeated valid frames are
normal: STM32 handling must update the same question or distance idempotently
and must not repeat any one-time action.

## Wiring and UART Parameters

| MSPM0G3507 | STM32 | Purpose |
| --- | --- | --- |
| UART2_TX_B15 | USARTx_RX | One-way UART data |
| UART2_RX_B16 | USARTx_TX | Optional return path; not required currently |
| GND | GND | Signal reference |

Both MCU sides must use 3.3 V TTL UART logic. Do not connect a 5 V TTL signal
to either MCU. The STM32 TX pin is not required for the current one-way link.
Do not connect both boards' VCC pins through the UART cable unless the power
design explicitly requires a common regulated supply.

```text
Baud rate:       115200
Data bits:       8
Parity:          none
Stop bits:       1
Hardware flow:   none
Format:          115200 8N1
```

## Frame Formats

All frames use the same two-byte header, protocol version, and CRC-8/ATM.

### Enter Question Frame (`type = 0x01`)

This is the existing six-byte frame. It is sent immediately when a menu
question is entered.

| Byte | Name | Value |
| --- | --- | --- |
| 0 | Header 0 | `0xA5` |
| 1 | Header 1 | `0x5A` |
| 2 | Protocol version | `0x01` |
| 3 | Message type | `0x01`, enter question |
| 4 | Question number | `0x02` through `0x06` |
| 5 | CRC-8 | CRC of bytes 0 through 4 |

### Question 6 Distance Frame (`type = 0x02`)

This eight-byte frame is sent only after the Question 6 operator completes
the two-step value entry. It follows the Question 6 enter frame; it does not
replace it.

| Byte | Name | Value |
| --- | --- | --- |
| 0 | Header 0 | `0xA5` |
| 1 | Header 1 | `0x5A` |
| 2 | Protocol version | `0x01` |
| 3 | Message type | `0x02`, Question 6 signed distance |
| 4 | Question number | `0x06` |
| 5 | Distance high byte | Signed distance bits 15 through 8 |
| 6 | Distance low byte | Signed distance bits 7 through 0 |
| 7 | CRC-8 | CRC of bytes 0 through 6 |

The distance is a signed `int16` in millimetres, transmitted in big-endian
two's-complement order. On the MSPM0, the value is calculated as:

```text
distance_mm = centimeters * 10 + millimetres
```

The menu limits the two signed inputs to `-15..+15 cm` and `-9..+9 mm`, so the
valid combined range is `-159..+159 mm`. STM32 firmware should reject a type
`0x02` frame with a question number other than `0x06` or a value outside this
range.

CRC is CRC-8/ATM with the following parameters:

```text
Polynomial:       0x07
Initial value:    0x00
Input reflected:  no
Output reflected: no
Final XOR:        0x00
```

## Test Vectors

```text
Question 2: A5 5A 01 01 02 0E
Question 3: A5 5A 01 01 03 09
Question 4: A5 5A 01 01 04 1C
Question 5: A5 5A 01 01 05 1B
Question 6: A5 5A 01 01 06 12

Question 6 distance -159 mm: A5 5A 01 02 06 FF 61 B0
Question 6 distance   -1 mm: A5 5A 01 02 06 FF FF 63
Question 6 distance    0 mm: A5 5A 01 02 06 00 00 47
Question 6 distance    1 mm: A5 5A 01 02 06 00 01 40
Question 6 distance  159 mm: A5 5A 01 02 06 00 9F 93
```

## STM32 Receiver Requirements

Use a non-blocking receiver. A byte-by-byte UART RX interrupt is the preferred
simple implementation. DMA circular buffering or DMA receive-to-idle is also
appropriate if it is already used in the STM32 project.

Implement a byte-stream state machine:

```text
Wait A5 -> wait 5A -> verify version -> read type -> collect the remaining
bytes required by that type -> verify CRC -> publish the message
```

For type `0x01`, collect `question + CRC` for a six-byte frame and publish the
question number. For type `0x02`, collect `question + distance_hi +
distance_lo + CRC` for an eight-byte frame, validate that `question == 6`, and
decode the signed value with:

```c
int16_t distance_mm = (int16_t)(((uint16_t)distance_hi << 8U) | distance_lo);
```

On header, version, type, question-range, distance-range, or CRC failure,
discard the partial frame and resume searching for `0xA5`. If the failed byte
is itself `0xA5`, it may be retained as the first byte of a potential new
frame.

On a valid frame, set a small state object such as `current_question`, or call
a lightweight notification function. Do not call delay, `printf`, display
refresh, motor control, or other long-running functions from a UART interrupt
or DMA callback.

## Prompt for the STM32-Side AI

Copy the following prompt, replace each angle-bracket field, and give it to the
AI responsible for the STM32 firmware.

```text
Modify this STM32 project so it receives question numbers from an MSPM0G3507
over a one-way UART link. Use the existing project conventions and preserve all
unrelated code.

STM32 hardware and project environment:
- MCU model: <for example STM32F103C8T6 or STM32F407VGT6>
- Firmware library/framework: <HAL, LL, StdPeriph, FreeRTOS, RT-Thread, etc.>
- Receive peripheral: <for example USART1>
- STM32 RX pin: <for example PA10>
- STM32 TX pin: <write "not connected" if unused>
- Required receive method: <byte RX interrupt / DMA receive-to-idle / DMA ring buffer>
- UART initialization file and function: <path and function name>
- UART IRQ, DMA callback, or RTOS receive task: <path and function name>
- Destination after a valid frame: <variable name or application function>

Electrical connection:
- MSPM0 UART2_TX_B15 -> STM32 USARTx_RX.
- MSPM0 UART2_RX_B16 <- STM32 USARTx_TX only if a future return path is needed.
- MSPM0 GND -> STM32 GND.
- Both sides are 3.3 V TTL UART.
- UART format is 115200 8N1 with no hardware flow control.

MSPM0 message protocol:
[A5] [5A] [01] [01] [question] [crc8]
[A5] [5A] [01] [02] [06] [distance_hi] [distance_lo] [crc8]

- Header: A5 5A
- Version: 01
- Type: 01 means "enter question".
- Type: 02 means "Question 6 signed distance in mm" and is always followed
  by question value 06.
- Question is only 2, 3, 4, 5, or 6.
- A type-02 distance is a big-endian two's-complement int16. Valid Question 6
  values are -159 through +159 mm.
- CRC is CRC-8/ATM over every preceding byte in its frame: polynomial 0x07, init 0x00,
  no input reflection, no output reflection, xorout 0x00.
- Valid test frames:
  Q2 = A5 5A 01 01 02 0E
  Q3 = A5 5A 01 01 03 09
  Q4 = A5 5A 01 01 04 1C
  Q5 = A5 5A 01 01 05 1B
  Q6 = A5 5A 01 01 06 12
- The MSPM0 sends each valid frame three times. Repeated valid question values
  and distance values must be accepted without triggering duplicate one-time actions.

Implement all of the following:
1. Configure the stated UART RX peripheral and pin for 115200 8N1.
2. Add a non-blocking receiver using the required method.
3. Implement a byte-stream frame parser with CRC-8/ATM that selects the
   six-byte or eight-byte frame length from the message type.
4. Recover from partial frames, noise, bad CRC, and invalid values, then rescan
   for the two-byte header.
5. On a valid type-01 frame, update the stated destination with the received
   question. On a valid type-02 frame, publish the signed Question 6 distance
   in millimetres to a separate stated destination.
6. Keep ISR/DMA callbacks short: no blocking delay, printf, display operation,
   motor control, or other slow work there.
7. Show every file and function that must be changed, then provide complete code
   changes in the style already used by this STM32 project.
8. Do not change unrelated UART, DMA, interrupt-priority, or RTOS behavior.
```

## Future Extension

Question 6 now sends an entry notification (`type = 0x01`, `question = 6`) and
one signed distance message (`type = 0x02`). Future status messages can use a
new message type while preserving this frame header, version, and CRC method.
Existing handling of message type `0x01` remains unchanged.
