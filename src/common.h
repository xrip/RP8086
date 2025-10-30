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


typedef struct {
    uint8_t interrupt_mask_register; //mask register
    uint8_t interrupt_request_register; //request register
    uint8_t in_service_register; //service register
    uint8_t initialization_command_word_step; //used during initialization to keep track of which ICW we're at
    uint8_t initialization_command_words_1; // ICW1
    uint8_t interrupt_vector_offset; //interrupt vector offset
    uint8_t register_read_mode; //remember what to return on read register from OCW3
} i8259_s;

typedef struct {
    uint16_t channel_reload_value[3]; // chandata -> channel reload values (what gets loaded into counters)
    uint8_t channel_access_mode[3]; // accessmode -> how each channel is accessed (lobyte/hibyte/toggle)
    uint8_t channel_byte_toggle[3]; // bytetoggle -> tracks which byte to read/write in toggle mode
    uint32_t channel_effective_count[3]; // effectivedata -> actual count value used by channel
    float channel_frequency[3]; // chanfreq -> calculated frequency for each channel
    uint8_t channel_active[3]; // active -> whether channel is actively counting
    uint16_t channel_current_count[3]; // counter -> current counter value for each channel
} i8253_s;