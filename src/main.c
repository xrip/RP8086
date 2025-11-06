#include <stdio.h>

#include "cpu.h"
#include "cpu_bus.h"
#include "common.h"
#include "graphics.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "hardware/i8259.h"
#include "hardware/i8253.h"
#include "hardware/uart16550.h"
#if PICO_RP2350
#include <hardware/structs/qmi.h>
#endif


uint32_t timer_interval = 54925;
bool ctty_mode = false;  // false = keyboard mode, true = CTTY mode
uint8_t current_scancode = 0; // 0 = нет данных

// ============================================================================
// ASCII to Scancode (IBM PC/XT Set 1) - Simplified
// ============================================================================
__force_inline uint8_t ascii_to_scancode(const int ascii) {
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
        case '#': return 0x3c; // "
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
__force_inline static void push_scancode(const uint8_t scancode) {
    if (scancode == 0x00) return; // Ignore unknown keys

    current_scancode = scancode;
    i8259_interrupt(1); // IRQ1 - Keyboard interrupt через i8259
}

static void pic_init(void) {
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
#if PICO_RP2350
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_60);
    busy_wait_at_least_cycles((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t) XOSC_HZ) / 1000000);
    qmi_hw->m[0].timing = 0x60007304; // 4x FLASH divisor
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);
#else
    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    busy_wait_at_least_cycles((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t) XOSC_HZ) / 1000000);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);

    #endif
    // busy_wait_ms(250); // Даем время стабилизироваться напряжению
    stdio_usb_init();
    while (!stdio_usb_connected()) { tight_loop_contents(); }


    multicore_launch_core1(bus_handler_core);

    absolute_time_t next_frame = get_absolute_time();
    next_frame = delayed_by_us(next_frame, 16666);
 
    bool video_enabled = true;
#if PICO_RP2350
    graphics_init();
    graphics_set_buffer((uint8_t *)VIDEORAM, 80, 200);
    graphics_set_textbuffer((uint8_t *)VIDEORAM);
    graphics_set_bgcolor(0);
    graphics_set_offset(0, 0);
    graphics_set_flashmode(true, true);
    //graphics_set_mode(TEXTMODE_80x25_BW);
#endif
    while (true) {
        // Отрисовка MDA фреймбуфера
        if (video_enabled && absolute_time_diff_us(next_frame, get_absolute_time()) >= 0) {
            next_frame = delayed_by_us(next_frame, 16666 * 2);

            printf("\033[H");        // cursor home
            printf("\033[2J");       // clear screen
            printf("\033[3J");       // clear scrollback
            printf("\033[40m");      // black background
            printf("\033[?25l");     // hide cursor (reduce flicker)
            for (int y = 0; y < 25; y++) {
                const uint32_t *framebuffer_line = (uint32_t*) VIDEORAM + __fast_mul(y, 40);
                for (int x = 40; x--;) {
                    const uint32_t dword = *framebuffer_line++ & 0x00FF00FF;
                    putchar_raw(dword);
                    putchar_raw(dword >> 16);
                }
                if (y != 24) {
                    putchar_raw(0x0D);
                    putchar_raw(0x0A);
                }
            }
        }

        // DMA polling и передача данных (асинхронная эмуляция Intel 8237)

        int c = getchar_timeout_us(0);

        // Special debug commands (uppercase variants)
        if (c == '`') {
            video_enabled = !video_enabled;
        } else if (c == 'C') {
            // Переключение между keyboard и CTTY режимами
            ctty_mode = !ctty_mode;
            printf("\033[2J\033[H");
            if (ctty_mode) {
                video_enabled = false;
                printf("*** CTTY Mode Enabled ***\n");
                printf("Press 'C' again to return to keyboard mode.\n\n");
            } else {
                video_enabled = true;
            }
        } else if (c == 'R') {
            printf("\033[2JReseting cpu\n");
            gpio_put(INTR_PIN, 0); // По умолчанию LOW
            reset_cpu();
            // watchdog_enable(0, 0);
        } else if (c == 'B') {
            reset_usb_boot(0, 0);
        } else if (c == 'M') {
            printf("\nEnter base address (hex): ");
            uint32_t base = 0;
            while (1) {
                int k = getchar_timeout_us(0);
                if (k == PICO_ERROR_TIMEOUT) continue;

                if (k == '\r' || k == '\n') break;

                if ((k >= '0' && k <= '9') || (k >= 'a' && k <= 'f') || (k >= 'A' && k <= 'F')) {
                    k = (k >= 'a') ? k - 'a' + 10 : (k >= 'A') ? k - 'A' + 10 : k - '0';
                    base = (base << 4) | k;
                    printf("%X", k);
                }
            }
            printf("\nMemory dump from %04X:\n", base);

            for (int i = 0; i < 0x200 && i + base < RAM_SIZE; i += 16) {
                printf("%04X: ", base + i);
                for (int j = 0; j < 16; j++) {
                    uint8_t value = RAM[base + i + j];
                    printf("%02X ", value);
                }
                printf(" | ");
                for (int j = 0; j < 16; j++) {
                    uint8_t value = RAM[base + i + j];
                    printf("%c", (value >= 32 && value < 127) ? value : '.');
                }
                printf("\n");

                // Check for abort keys
                int k = getchar_timeout_us(0);
                if (k != PICO_ERROR_TIMEOUT && (k == 'M' || k == 'R' || k == 'B' || k == 'V'))
                    break;
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
            // ═══════════════════════════════════════════════════════
            // Обработка обычных символов в зависимости от режима
            // ═══════════════════════════════════════════════════════
            if (ctty_mode) {
                // CTTY Mode: USB → UART RBR → DOS
                uart.rbr = (uint8_t)c;
                uart.data_ready = true;
            } else {
                // Keyboard Mode: USB → Scancode → i8086
                const uint8_t scancode = ascii_to_scancode(c);
                push_scancode(scancode);
            }
        }

        //__wfi();
        tight_loop_contents();
    }
}
