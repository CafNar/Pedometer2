#include <stdio.h>
#include "pico/stdlib.h"
#include "oled.h" 
#define I2C_PORT  0x40090000
#define OLED_ADDR 0x3C

// I2C Registers 
#define I2C_BASE        0x40090000
#define I2C_IC_CON      (I2C_BASE + 0x00)
#define I2C_IC_TAR      (I2C_BASE + 0x04)
#define I2C_IC_DATA_CMD (I2C_BASE + 0x10)
#define I2C_IC_RAW_INTR_STAT (I2C_BASE + 0x1C)
#define I2C_IC_CLR_INTR   (I2C_BASE + 0x40)
#define I2C_IC_ENABLE     (I2C_BASE + 0x6C)
#define I2C_IC_STATUS     (I2C_BASE + 0x70)
#define I2C_IC_TXFLR      (I2C_BASE + 0x74)
#define I2C_IC_RXFLR      (I2C_BASE + 0x78)

// Helper functions for register access
static inline void reg_write(uintptr_t addr, uint32_t value) {
    *(volatile uint32_t*)addr = value;
}

static inline uint32_t reg_read(uintptr_t addr) {
    return *(volatile uint32_t*)addr;
}

const uint8_t font_5x7[][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x49, 0x4F, 0x30}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x01, 0x71, 0x09, 0x07}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x26, 0x49, 0x49, 0x49, 0x3E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x26, 0x49, 0x49, 0x49, 0x32}, // S
    {0x38, 0x54, 0x54, 0x54, 0x48}, // e
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x48, 0x54, 0x54, 0x54, 0x24}, // s
    {0x00, 0x08, 0x3C, 0x48, 0x08}, // t
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
};

// Wait for I2C bus to be ready
static void i2c_wait_ready() {
    while (reg_read(I2C_IC_STATUS) & (1 << 1)); // Wait for not busy
}

// Send a byte of command to the OLED
void oled_send_command(uint8_t cmd) {
    i2c_wait_ready();
    reg_write(I2C_IC_TAR, OLED_ADDR);
    reg_write(I2C_IC_DATA_CMD, 0x00); // Control byte for command
    reg_write(I2C_IC_DATA_CMD, cmd);
}

// Send a byte of data to the OLED
void oled_send_data(uint8_t data) {
    i2c_wait_ready();
    reg_write(I2C_IC_TAR, OLED_ADDR);
    reg_write(I2C_IC_DATA_CMD, 0x40); // Control byte for data
    reg_write(I2C_IC_DATA_CMD, data);
}

// Initialize the OLED
void oled_init() {
    uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_send_command(init_cmds[i]);
    }
}

// Clear the OLED display
void oled_clear() {
    for (uint8_t page = 0; page < 8; page++) {
        oled_send_command(0xB0 + page);
        oled_send_command(0x00);
        oled_send_command(0x10);

        for (uint8_t col = 0; col < 128; col++) {
            oled_send_data(0x00);
        }
    }
}

// Draw a character on the OLED (5x7 font)
void oled_draw_char(uint8_t page, uint8_t col, char c) {
    if (c >= '0' && c <= '9') c -= '0'; // Ký tự số từ 0 đến 9
    else if (c == 'S') c = 11;
    else if (c == 'e') c = 12;
    else if (c == 'p') c = 13;
    else if (c == 's') c = 14;
    else if (c == 't') c = 15;
    else if (c == 'a') c = 16;
    else if (c == 'r') c = 17;
    else if (c == 'o') c = 18;
    else c = 10;

    oled_send_command(0xB0 + page);
    oled_send_command(0x00 + (col & 0x0F));
    oled_send_command(0x10 + (col >> 4));

    for (uint8_t i = 0; i < 5; i++) {
        oled_send_data(font_5x7[c][i]);
    }
    oled_send_data(0x00);
}

// Draw a string on the OLED
void oled_draw_string(uint8_t page, uint8_t col, const char *str) {
    while (*str) {
        oled_draw_char(page, col, *str++);
        col += 6;
        if (col >= 128) break;
    }
}

// Draw a number on the OLED
void oled_draw_number(uint8_t page, uint8_t col, int num) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%d", num);
    oled_draw_string(page, col, buffer);
}

// Update the OLED display
void update_display(int steps, bool counting) {
    oled_clear();
    if (counting) {
        oled_draw_string(0, 0, "Start");
    } else {
        oled_draw_string(0, 0, "Stop");
    }
    oled_draw_string(1, 0, "Steps:");
    oled_draw_number(2, 0, steps);
}
