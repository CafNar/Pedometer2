#ifndef OLED_H
#define OLED_H

#include <stdint.h>

// OLED I2C Address
#define OLED_ADDR 0x3C

// OLED Register Addresses
#define OLED_CONTROL_BYTE_CMD  0x00
#define OLED_CONTROL_BYTE_DATA 0x40

// OLED Command Registers
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF
#define OLED_CMD_SET_PAGE_START_ADDRESS 0xB0
#define OLED_CMD_SET_LOW_COL_START      0x00
#define OLED_CMD_SET_HIGH_COL_START     0x10
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERSE        0xA7

// Function prototypes
void oled_init(void);
void oled_clear(void);
void oled_draw_char(uint8_t page, uint8_t col, char c);
void oled_draw_string(uint8_t page, uint8_t col, const char *str);
void oled_draw_number(uint8_t page, uint8_t col, int num);
void update_display(int steps, bool counting);

#endif // OLED_H