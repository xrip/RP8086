#include <stdio.h>

#include "cpu.h"
#include "cpu_bus.h"
#include "common.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "hardware/i8259.h"
#include "hardware/i8253.h"

// ============================================================================
// Global Memory Arrays
// ============================================================================
uint8_t RAM[RAM_SIZE] __attribute__((aligned(4)));
uint8_t VIDEORAM[4096] __attribute__((aligned(4)));

i8259_s i8259 __attribute__((aligned(4))) = {
    .interrupt_mask_register = 0xFF, // Все IRQ замаскированы по умолчанию
    .interrupt_vector_offset = 0x08, // Стандартный offset для IBM PC
};
uint32_t timer_interval = 54925;
bool speakerenabled = false;
i8253_s i8253 __attribute__((aligned(4))) = { 0 };

// ============================================================================
// IRQ System - Intel 8259A Compatible Controller
// ============================================================================

// ============================================================================
// Keyboard - Single Scancode (no buffer needed for human input)
// ============================================================================
volatile uint8_t current_scancode = 0; // 0 = нет данных

// ============================================================================
// ASCII to Scancode (IBM PC/XT Set 1) - Simplified
// ============================================================================
static uint8_t ascii_to_scancode(const int ascii) {
    // Letters
    if (ascii >= 'a' && ascii <= 'z') {
        // QWERTY layout mapping
        static const uint8_t qwerty_map[] = {
            0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, // a-h
            0x17, 0x24, 0x25, 0x26, 0x32, 0x31, 0x18, 0x19, // i-p
            0x10, 0x13, 0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D, // q-x
            0x15, 0x2C // y-z
        };
        return qwerty_map[ascii - 'a'];
    }
    if (ascii >= 'A' && ascii <= 'Z') {
        // Uppercase - same scancodes (shift handled separately)
        static const uint8_t qwerty_map[] = {
            0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23,
            0x17, 0x24, 0x25, 0x26, 0x32, 0x31, 0x18, 0x19,
            0x10, 0x13, 0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D,
            0x15, 0x2C
        };
        return qwerty_map[ascii - 'A'];
    }
    // Digits
    if (ascii >= '0' && ascii <= '9') {
        static const uint8_t digit_map[] = {
            0x0B, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A
        };
        return digit_map[ascii - '0'];
    }
    // Special keys
    switch (ascii) {
        case '!': return 0x41; // "
        case '@': return 0x3f; // "
        case '#': return 0x3d; // "
        case '$': return 0x3b; // "
        case '%': return 0x3c; // "
        case '^': return 0x58; // "
        case '&': return 0x64; // "
        case '(': return 0x42; // "
        case ')': return 0x4; // "
        case '"': return 0x68; // "

        case ' ': return 0x39; // Space
        case '\r':
        case '\n': return 0x1C; // Enter
        case '\b': return 0x0E; // Backspace
        case '\t': return 0x0F; // Tab
        case 27: return 0x01; // Escape

        // Symbols without Shift
        case '-': return 0x0C; // Minus
        case '=': return 0x0D; // Equals
        case '[': return 0x1A; // Left bracket
        case ']': return 0x1B; // Right bracket
        case ';': return 0x27; // Semicolon
        case '\'': return 0x28; // Single quote
        case '`': return 0x29; // Backtick
        case '\\': return 0x2B; // Backslash
        case ',': return 0x33; // Comma
        case '.': return 0x34; // Period
        case '/': return 0x35; // Slash
        case '*': return 0x37;
        default: return 0x00; // Unknown
    }
}

// ============================================================================
// Set scancode and trigger IRQ1 (keyboard interrupt)
// ============================================================================
static void push_scancode(const uint8_t scancode) {
    if (scancode == 0x00) return; // Ignore unknown keys

    current_scancode = scancode;
    i8259_interrupt(1); // IRQ1 - Keyboard interrupt через i8259
}

void pic_init(void) {
    // Настройка INTR как выход
    gpio_init(INTR_PIN);
    gpio_set_dir(INTR_PIN, GPIO_OUT);
    gpio_put(INTR_PIN, 0); // По умолчанию LOW
}

// ============================================================================
// Core1: Обработка i8086_bus
// ============================================================================
[[noreturn]] void bus_handler_core(void) {
    start_cpu_clock(); // Start i8086 clock generator
    pic_init(); // Initialize interrupt controller and start Core1 IRQ generator
    cpu_bus_init(); // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu(); // Now i8086 can safely start

    absolute_time_t next_irq0 = get_absolute_time();
    next_irq0 = delayed_by_us(next_irq0, timer_interval);

    while (true) {
        // ═══════════════════════════════════════════════════════
        // 1. Генерация таймерного прерывания IRQ0 (18.2 Hz)
        // ═══════════════════════════════════════════════════════
        if (absolute_time_diff_us(next_irq0, get_absolute_time()) >= 0) {
            i8259_interrupt(0);
            next_irq0 = delayed_by_us(next_irq0, timer_interval);
        }

        // ═══════════════════════════════════════════════════════
        // 2. Управление сигналом INTR (проверка pending IRQ в IRR)
        // ═══════════════════════════════════════════════════════
        gpio_put(INTR_PIN, i8259_get_pending_irqs());

        tight_loop_contents();
    }
}

[[noreturn]] int main() {
    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    busy_wait_at_least_cycles((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t) XOSC_HZ) / 1000000);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);
    // busy_wait_ms(250); // Даем время стабилизироваться напряжению
    stdio_usb_init();
    while (!stdio_usb_connected()) { tight_loop_contents(); }


    multicore_launch_core1(bus_handler_core);

    absolute_time_t next_frame = get_absolute_time();
    next_frame = delayed_by_us(next_frame, 16666);

    bool video_enabled = true;

    while (true) {
        // Отрисовка MDA фреймбуфера
        if (video_enabled && absolute_time_diff_us(next_frame, get_absolute_time()) >= 0) {
            next_frame = delayed_by_us(next_frame, 16666);

            printf("\033[2J\033[H");
            for (int y = 0; y < 25; y++) {
                for (int x = 0; x < 160; x += 2) {
                    printf("%c", VIDEORAM[__fast_mul(y, 160) + x]);
                }
                printf("\n");
            }
        }

        int c = getchar_timeout_us(0);

        // Special debug commands (uppercase variants)
        if (c == '`') {
            video_enabled = !video_enabled;
        } else if (c == 'R') {
            printf("\033[2JReseting cpu\n");
            gpio_put(INTR_PIN, 0); // По умолчанию LOW
            reset_cpu();
            // watchdog_enable(0, 0);
        } else if (c == 'B') {
            printf("===================== RESET");
            reset_usb_boot(0, 0);
        } else if (c == 'M') {
            printf("\nMemory dump (first 400 bytes):\n");
            for (int i = 0; i < 0x400; i += 16) {
                printf("%04X: ", i);
                for (int j = 0; j < 16; j++) {
                    uint8_t value = RAM[i + j];
                    printf("%02X ", value);
                }
                printf(" | ");
                for (int j = 0; j < 16; j++) {
                    uint8_t value = RAM[i + j];
                    printf("%c", value);
                }
                printf("\n");
            }
        } else if (c == 'V') {
            printf("\nVideo Memory dump \n");
            for (int i = 0; i < 160 * 5; i += 16) {
                printf("%04X: ", i);
                for (int j = 0; j < 16; j++) {
                    uint8_t value = VIDEORAM[i + j];
                    printf("%02X ", value);
                }
                printf(" | ");
                for (int j = 0; j < 16; j++) {
                    uint8_t value = VIDEORAM[i + j];
                    printf("%c", value);
                }
                printf("\n");
            }
        } else if (c != PICO_ERROR_TIMEOUT) {
            // Regular key - convert to scancode and send to i8086
            const uint8_t scancode = ascii_to_scancode(c);
            push_scancode(scancode);
        }

        //__wfi();
        tight_loop_contents();
    }
}
