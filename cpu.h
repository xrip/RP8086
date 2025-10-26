#pragma once

/**
 * Start i8086 clock generation on CONFIG_CLOCK_PIN (GPIO29)
 * Frequency: CONFIG_I8086_CLOCK_HZ
 * Duty cycle: CONFIG_I8086_DUTY_CYCLE (33%)
 */
void start_cpu_clock(void);

/**
 * Perform RESET sequence for i8086
 * RESET is held LOW for minimum 4 clock cycles, then released
 * CPU starts execution from FFFF0h (reset vector)
 */
void reset_cpu(void);

// Включаем и выкключаем клок у цпу
void toggle_cpu();