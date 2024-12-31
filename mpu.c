#include "mpu.h"
#include "pico/stdlib.h"

// I2C Registers - Section 4.6.1 datasheet
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

// I2C Clock Divider
#define CLOCKS_BASE         0x40010000
#define CLK_PERI_CTRL       0x04
#define CLK_PERI_DIV        0x08

// I2C Pin Function Select
#define IO_BANK0_BASE       0x40028000
#define GPIO_FUNC_I2C       2

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13

// Helper functions for register access (same as in test2.c)
static inline void reg_write(uintptr_t addr, uint32_t value) {
    *(volatile uint32_t*)addr = value;
}

static inline uint32_t reg_read(uintptr_t addr) {
    return *(volatile uint32_t*)addr;
}

// Function to initialize I2C
void i2c_init_direct() {
    // Enable I2C clock
    reg_write(CLOCKS_BASE + CLK_PERI_CTRL, (1 << 11));
    // Set clock divider (adjust as needed for your desired I2C speed)
    reg_write(CLOCKS_BASE + CLK_PERI_DIV, (1 << 12));

    // Configure GPIO pins for I2C
    reg_write(IO_BANK0_BASE + (I2C_SDA_PIN * 4), GPIO_FUNC_I2C); // SDA
    reg_write(IO_BANK0_BASE + (I2C_SCL_PIN * 4), GPIO_FUNC_I2C); // SCL

    // Enable I2C
    reg_write(I2C_IC_ENABLE, 1);
}

// Function to write a single register
void mpu6050_write_register(uint8_t reg, uint8_t value) {
    // Set target address
    reg_write(I2C_IC_TAR, MPU6050_ADDR);

    // Write register address
    reg_write(I2C_IC_DATA_CMD, reg);

    // Write value
    reg_write(I2C_IC_DATA_CMD, value);

    // Wait for transfer to complete
    while(reg_read(I2C_IC_STATUS) & (1 << 1));
}

// Function to read a single register
uint8_t mpu6050_read_register(uint8_t reg) {
    uint8_t value;

    // Set target address
    reg_write(I2C_IC_TAR, MPU6050_ADDR);

    // Write register address (with read bit)
    reg_write(I2C_IC_DATA_CMD, reg);

    // Read value
    reg_write(I2C_IC_DATA_CMD, (1 << 9)); // Read command
    while(!(reg_read(I2C_IC_STATUS) & (1 << 2)));
    value = reg_read(I2C_IC_DATA_CMD);

    // Wait for transfer to complete
    while(reg_read(I2C_IC_STATUS) & (1 << 1));

    return value;
}

void mpu6050_init() {
    i2c_init_direct();
    // Configure Power Management Register (PWR_MGMT_1)
    // Bit 6: Reset device
    // Bit 3: Disable temperature sensor
    // Bits 2-0: Clock source (0b000 = Internal 8MHz oscillator)
    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);

    // Configure Accelerometer Configuration Register (ACCEL_CONFIG)
    // Bits 4-3: Full scale range (±2g)
    // Other bits set to 0 for default configuration
    mpu6050_write_register(MPU6050_ACCEL_CONFIG, 0x00);

    // Configure DLPF (Digital Low Pass Filter)
    // Bits 2-0: DLPF configuration for low pass filter
    mpu6050_write_register(MPU6050_CONFIG, 0x06);
}

void mpu6050_read_accel(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t buffer[6];
    uint8_t start_reg = MPU6050_ACCEL_XOUT_H;

    // Set target address
    reg_write(I2C_IC_TAR, MPU6050_ADDR);

    // Write register address (with read bit)
    reg_write(I2C_IC_DATA_CMD, start_reg);

    // Read 6 bytes of accelerometer data (2 bytes each for X, Y, Z)
    for (int i = 0; i < 6; i++) {
        reg_write(I2C_IC_DATA_CMD, (1 << 9)); // Read command
        while(!(reg_read(I2C_IC_STATUS) & (1 << 2)));
        buffer[i] = reg_read(I2C_IC_DATA_CMD);
    }

    // Combine high and low bytes (16-bit signed integers)
    // Shift high byte left by 8 and OR with low byte
    *x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *z = (int16_t)((buffer[4] << 8) | buffer[5]);
}
