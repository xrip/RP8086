#pragma once
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>
// ============================================================================
// System Configuration
// ============================================================================
#define PICO_CLOCK_SPEED     (400 * MHZ)  // 400 MHz

// ============================================================================
// i8086 Clock Configuration
// ============================================================================
#define I8086_CLOCK_PIN         29
#define I8086_CLOCK_SPEED       100     // 100 Hz for debug (change to 5000000 for 5 MHz)
#define CONFIG_I8086_DUTY_CYCLE 33      // 33% duty cycle (required for i8086)

#define RESET_PIN        27             // Reset output (active low)


// ============================================================================
// PIO Configuration
// ============================================================================
#define BUS_CTRL_PIO     pio0
#define BUS_CTRL_SM      0
#define WRITE_IRQ        PIO0_IRQ_0
#define READ_IRQ         PIO0_IRQ_1
