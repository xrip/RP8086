#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>

// ============================================================================
// Compiler hints for branch prediction
// ============================================================================
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

// ============================================================================
// System Configuration
// ============================================================================
#define PICO_CLOCK_SPEED     (400 * MHZ)  // Raspberry Pi Pico clock frequency

// ============================================================================
// i8086 Clock Configuration
// ============================================================================
#define I8086_CLOCK_SPEED    (4 * MHZ)  // i8086 clock frequency
#define CONFIG_I8086_DUTY_CYCLE 33      // 33% duty cycle (required for i8086)

// ============================================================================
// GPIO Pin Configuration
// ============================================================================
#define INTR_PIN         26             // Interrupt request output to i8086 (active HIGH)
#define RESET_PIN        28             // Reset output (active HIGH)
#define CLOCK_PIN        29             // Clock output to i8086

// ============================================================================
// PIO Configuration
// ============================================================================
#define BUS_CTRL_PIO     pio0
#define BUS_CTRL_SM      0
#define WRITE_IRQ        PIO0_IRQ_0
#define READ_IRQ         PIO0_IRQ_1

// ============================================================================
// Memory Configuration
// ============================================================================
#define RAM_SIZE               (192 * 1024)                       // 192KB RAM
#define BIOS_ROM_SIZE          8192                               // 8KB BIOS
#define BIOS_ROM_BASE          (0x100000 - BIOS_ROM_SIZE)         // 0xFE000-0xFFFFF

// ============================================================================
// Bus Control Signal Macros
// ============================================================================
#define MIO (1 << 24)  // Memory/IO bit in bus state
#define BHE (1 << 25)  // Bus High Enable bit in bus state

// ============================================================================
// Inline Helper Functions
// ============================================================================

// Универсальная функция записи с поддержкой BHE (8/16-bit operations)
__always_inline static void write_to(uint8_t *destination, const uint32_t address,
                                      const uint16_t data, const bool bhe) {
    const bool A0 = address & 1;
    if (likely(!bhe && !A0)) {
        // 16-bit aligned write
        *(uint16_t *)&destination[address] = data;
    } else if (bhe && !A0) {
        // Low byte only
        destination[address] = (uint8_t)(data & 0xFF);
    } else if (!bhe && A0) {
        // High byte only
        destination[address] = (uint8_t)(data >> 8);
    }
}
