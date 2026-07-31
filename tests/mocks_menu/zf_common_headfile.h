#ifndef MENU_TEST_ZF_COMMON_HEADFILE_H
#define MENU_TEST_ZF_COMMON_HEADFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

typedef enum
{
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_NUMBER,
} key_index_enum;

typedef enum
{
    KEY_RELEASE,
    KEY_SHORT_PRESS,
    KEY_LONG_PRESS,
} key_state_enum;

void key_init(uint32 period);
void key_scanner(void);
key_state_enum key_get_state(key_index_enum key_n);
void key_clear_state(key_index_enum key_n);
void key_clear_all_state(void);
void Motor_Stop(void);
void ips200_clear(void);
void ips200_show_string(uint16 x, uint16 y, const char dat[]);

#endif
