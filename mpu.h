#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR 0x68

// Extended Register Definitions
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_CONFIG        0x1A

// Function Prototypes
void mpu6050_write_register(uint8_t reg, uint8_t value);
uint8_t mpu6050_read_register(uint8_t reg);
void mpu6050_init(void);
void mpu6050_read_accel(int16_t* x, int16_t* y, int16_t* z);

#endif // MPU6050_H