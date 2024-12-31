#include "hardware/regs/m33.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/i2c.h"
#include "hardware/regs/pads_bank0.h"

#include "oled.h"
#include "mpu.h"

#define RED_LED_PIN 14
#define GREEN_LED_PIN 15
#define RESET_BUTTON_PIN 17
#define STATE_BUTTON_PIN 16

#define I2C_PORT_BASE I2C0_BASE
#define I2C_SDA 12
#define I2C_SCL 13
#define HWREG(x) (*((volatile uint32_t *)(x)))
#define SYSTICK_RELOAD_VALUE 125000 // For 1ms interrupt at 125MHz clock
volatile uint32_t systick_count = 0;

volatile bool reset_button_flag = false;
volatile bool state_button_flag = false;

void init_gpio() {
    // Set LED pins as outputs
    HWREG(IO_BANK0_GPIO14_CTRL) = 0x05; // Output
    HWREG(IO_BANK0_GPIO15_CTRL) = 0x05; // Output

    // Set button pins as inputs
    HWREG(IO_BANK0_GPIO17_CTRL) = 0x05; // Input
    HWREG(IO_BANK0_GPIO16_CTRL) = 0x05; // Input

    // Enable pull-ups for buttons
    HWREG(IO_BANK0_GPIO17_PUE) = 1;
    HWREG(IO_BANK0_GPIO16_PUE) = 1;

    // Set initial LED states
    HWREG(IO_BANK0_GPIO14_OUT) = 1; // Red LED on
    HWREG(IO_BANK0_GPIO15_OUT) = 0; // Green LED off
}

void init_i2c() {
    // Enable I2C clock
    HWREG(RESETS_RESET) &= ~(RESETS_RESET_I2C0_MASK);
    while ((HWREG(RESETS_RESET_DONE) & RESETS_RESET_DONE_I2C0_MASK) == 0);

    // Configure GPIO for I2C
    HWREG(IO_BANK0_GPIO12_CTRL) = 0x03; // I2C function
    HWREG(IO_BANK0_GPIO13_CTRL) = 0x03; // I2C function

    // Enable pull-ups for I2C
    HWREG(IO_BANK0_GPIO12_PUE) = 1;
    HWREG(IO_BANK0_GPIO13_PUE) = 1;

    // Configure I2C speed (400kHz)
    uint32_t freq = 400000;
    uint32_t clk_sys_freq = 125000000; // Assuming 125MHz system clock
    uint32_t div = (clk_sys_freq / (freq * 10));
    HWREG(I2C0_IC_SS_SCL_HCNT) = div;
    HWREG(I2C0_IC_SS_SCL_LCNT) = div;
    HWREG(I2C0_IC_SDA_HOLD) = 0x10;
    HWREG(I2C0_IC_SDA_SETUP) = 0x10;
    HWREG(I2C0_IC_CON) = I2C_IC_CON_MASTER_MODE_MASK | I2C_IC_CON_SPEED_MASK;
    HWREG(I2C0_IC_ENABLE) = 1;
}

void systick_handler() {
    // Check if the SysTick interrupt is enabled
    if (HWREG(NVIC_ISER) & (1 << SysTick_IRQn)) {
        systick_count++;
    }
}

void init_systick() {
    // Disable SysTick during setup
    HWREG(NVIC_ST_CTRL) = 0;
    // Set reload value
    HWREG(NVIC_ST_RELOAD) = SYSTICK_RELOAD_VALUE;
    // Clear current value
    HWREG(NVIC_ST_CURRENT) = 0;
    // Enable SysTick interrupt and counter
    HWREG(NVIC_ST_CTRL) = NVIC_ST_CTRL_CLK_SRC_MASK | NVIC_ST_CTRL_TICKINT_MASK | NVIC_ST_CTRL_ENABLE_MASK;
    // Set interrupt priority
    HWREG(NVIC_IPR2) = (HWREG(NVIC_IPR2) & ~(0xFF << 24)) | (0x01 << 24); // Priority 1
    // Enable the interrupt
    HWREG(NVIC_ISER) = (1 << SysTick_IRQn);
}

void gpio_irq_handler() {
    // Check if the GPIO interrupt is enabled
    if (HWREG(NVIC_ISER) & (1 << IO_IRQ_BANK0)) {
        // Check which pin triggered the interrupt
        if (HWREG(IO_BANK0_INTR) & (1 << RESET_BUTTON_PIN)) {
            reset_button_flag = true;
            HWREG(IO_BANK0_INTR) |= (1 << RESET_BUTTON_PIN); // Clear interrupt flag
        }
        if (HWREG(IO_BANK0_INTR) & (1 << STATE_BUTTON_PIN)) {
            state_button_flag = true;
            HWREG(IO_BANK0_INTR) |= (1 << STATE_BUTTON_PIN); // Clear interrupt flag
        }
    }
}

void init_gpio_interrupts() {
    // Enable falling edge detection
    HWREG(IO_BANK0_GPIO17_IRQ_FALL) = 1;
    HWREG(IO_BANK0_GPIO16_IRQ_FALL) = 1;

    // Set interrupt priority
    HWREG(NVIC_IPR0) = (HWREG(NVIC_IPR0) & ~(0xFF << 8)) | (0x02 << 8); // Priority 2
    // Enable the interrupt
    HWREG(NVIC_ISER) = (1 << IO_IRQ_BANK0);

    // Set the interrupt handler
    HWREG(SIO_IRQ_CTRL) = (uint32_t)gpio_irq_handler;
}

int main() {
    // Initialize peripherals
    init_gpio();
    init_i2c();
    init_systick();
    init_gpio_interrupts();

    // Initialize OLED and MPU 
    oled_init();
    oled_clear();
    mpu6050_init();

    int16_t accel_x, accel_y, accel_z;
    int16_t last_x = 0, last_y = 0, last_z = 0;
    bool led_state = false;
    update_display(0, step_counting_enabled);

    while (1) {
        // Read accelerometer data 
        mpu6050_read_accel(&accel_x, &accel_y, &accel_z);

        // Handle button presses using flags
        if (state_button_flag) {
            state_button_flag = false; // Clear the flag
            step_counting_enabled = !step_counting_enabled;
            if (step_counting_enabled) {
                HWREG(IO_BANK0_GPIO15_OUT) = 1; // Green LED on
                HWREG(IO_BANK0_GPIO14_OUT) = 0; // Red LED off
                update_display(step_count, step_counting_enabled);
            } else {
                HWREG(IO_BANK0_GPIO15_OUT) = 0; // Green LED off
                HWREG(IO_BANK0_GPIO14_OUT) = 1; // Red LED on
                update_display(step_count, step_counting_enabled);
            }
        }

        if (reset_button_flag) {
            reset_button_flag = false; // Clear the flag
            step_count = 0;
            update_display(step_count, step_counting_enabled);
        }

        // Step counting logic 
        if (step_counting_enabled) {
            int16_t total_delta = abs(accel_x - last_x) +
                                  abs(accel_y - last_y) +
                                  abs(accel_z - last_z);

            if (total_delta > STEP_THRESHOLD) {
                step_count++;
                update_display(step_count, step_counting_enabled);
            }

            // LED blinking logic using systick_count
            if (systick_count % 500 == 0) { // Toggle every 500ms
                led_state = !led_state;
                HWREG(IO_BANK0_GPIO15_OUT) = led_state; // Toggle green LED
            }
        }

        last_x = accel_x;
        last_y = accel_y;
        last_z = accel_z;

        // sleep_ms(100); 
        for(volatile int i = 0; i < 100000; i++);
    }

    return 0;
}
