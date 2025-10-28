#include <stdio.h>

#include "cpu.h"
#include "cpu_bus.h"
#include "config.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"


extern uint8_t RAM[] __attribute__((aligned(4)));
extern uint8_t PORTS[] __attribute__((aligned(4)));
extern uint8_t VIDEORAM[] __attribute__((aligned(4)));

// ============================================================================
// IRQ System - Simple Version
// ============================================================================
uint16_t current_irq_vector = 0; // 0=IRQ0(timer), 1=IRQ1(keyboard)

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
// Set scancode and trigger IRQ1
// ============================================================================
static void push_scancode(const uint8_t scancode) {
    if (scancode == 0x00) return; // Ignore unknown keys

    current_scancode = scancode;
    if (!current_irq_vector) {
        current_irq_vector = (0xFF00 | 8) + 1; // IRQ 1
    }
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
    constexpr uint32_t timer_interval = 54925; // 549ms на 500Khz, 54.925ms на 5Mhz

    start_cpu_clock(); // Start i8086 clock generator
    pic_init(); // Initialize interrupt controller and start Core1 IRQ generator
    cpu_bus_init(); // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu(); // Now i8086 can safely start

    absolute_time_t next_irq0 = get_absolute_time();
    next_irq0 = delayed_by_us(next_irq0, timer_interval);

    while (true) {
        // ═══════════════════════════════════════════════════════
        // 1. Генерация таймерного прерывания IRQ0
        // ═══════════════════════════════════════════════════════
        if (absolute_time_diff_us(next_irq0, get_absolute_time()) >= 0) {
            current_irq_vector = (0xFF00 | 8) + 0; // IRQ 0
            next_irq0 = delayed_by_us(next_irq0, timer_interval);
        }

        // ═══════════════════════════════════════════════════════
        // 2. Управление сигналом INTR (приоритет: IRQ0 > IRQ1)
        // ═══════════════════════════════════════════════════════
        gpio_put(INTR_PIN, current_irq_vector ? 1 : 0);

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
        } else if (c == 'P') {
            printf("\nPorts dump (first 400 bytes):\n");
            for (int i = 0; i < 0x3FF; i += 16) {
                printf("%04X: ", i);
                for (int j = 0; j < 16; j++) {
                    uint8_t value = PORTS[i + j];
                    printf("%02X ", value);
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
