#pragma once
#ifdef DEBUG
#include <pico/stdio.h>
bool handleScancode(uint8_t ps2scancode);

// ============================================================================
// ASCII to Scancode (IBM PC/XT Set 1) - Simplified
// ============================================================================
__force_inline uint8_t ascii_to_scancode(const int ascii) {
    const uint8_t qwerty_map[] = {
        0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, // a-h
        0x17, 0x24, 0x25, 0x26, 0x32, 0x31, 0x18, 0x19, // i-p
        0x10, 0x13, 0x1F, 0x14, 0x16, 0x2F, 0x11, 0x2D, // q-x
        0x15, 0x2C // y-z
    };
    // Letters
    if (ascii >= 'a' && ascii <= 'z') {
        return qwerty_map[ascii - 'a'];
    }
    if (ascii >= 'A' && ascii <= 'Z') {
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

__force_inline void debug_init() {
    stdio_init_all();
}

__force_inline void debug_console(const int videomode) {
    static bool video_enabled = true;
    static bool ctty_mode = false; // false = keyboard mode, true = CTTY mode
    if (video_enabled && videomode <= TEXTMODE_80x25_COLOR) {
        printf("\033[H"); // cursor home
        printf("\033[2J"); // clear screen
        printf("\033[3J"); // clear scrollback
        printf("\033[40m"); // black background
        printf("\033[?25l"); // hide cursor (reduce flicker)
        for (int y = 0; y < 25; y++) {
            const uint32_t *framebuffer_line = (uint32_t *) VIDEORAM + __fast_mul(y, 40);
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
        printf("\nVideo dump from %04X:\n", base);

        for (int i = 0; i < 0x200 && i + base < RAM_SIZE; i += 16) {
            printf("%04X: ", base + i);
            for (int j = 0; j < 16; j++) {
                uint8_t value = VIDEORAM[base + i + j];
                printf("%02X ", value);
            }
            printf(" | ");
            for (int j = 0; j < 16; j++) {
                uint8_t value = VIDEORAM[base + i + j];
                printf("%c", (value >= 32 && value < 127) ? value : '.');
            }
            printf("\n");

            // Check for abort keys
            int k = getchar_timeout_us(0);
            if (k != PICO_ERROR_TIMEOUT && (k == 'M' || k == 'R' || k == 'B' || k == 'V'))
                break;
        }
    } else if (c != PICO_ERROR_TIMEOUT) {
        // ═══════════════════════════════════════════════════════
        // Обработка обычных символов в зависимости от режима
        // ═══════════════════════════════════════════════════════
        if (ctty_mode) {
            // CTTY Mode: USB → UART RBR → DOS
            uart.rbr = (uint8_t) c;
            uart.data_ready = true;
        } else {
            // Keyboard Mode: USB → Scancode → i8086
            handleScancode(ascii_to_scancode(c));
        }
    }
}
#define tusb_init(...)
#define keyboard_init(...)
#define mouse_init(...)
#define keyboard_tick(...)

#else
#include "hid_app.h"
#include "tusb.h"
#define debug_init(...)
#define debug_console(...)
#endif
