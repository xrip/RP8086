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
    // Special keys with Shift (возвращаем базовый scancode, Shift добавится в send_ascii_as_scancode)
    switch (ascii) {
        case '!': return 0x02; // Shift+1
        case '@': return 0x03; // Shift+2
        case '#': return 0x04; // Shift+3
        case '$': return 0x05; // Shift+4
        case '%': return 0x06; // Shift+5
        case '^': return 0x07; // Shift+6
        case '&': return 0x08; // Shift+7
        case '*': return 0x09; // Shift+8
        case '(': return 0x0A; // Shift+9
        case ')': return 0x0B; // Shift+0
        case '_': return 0x0C; // Shift+-
        case '+': return 0x0D; // Shift+=
        case '{': return 0x1A; // Shift+[
        case '}': return 0x1B; // Shift+]
        case ':': return 0x27; // Shift+;
        case '"': return 0x28; // Shift+'
        case '|': return 0x2B; // Shift+\
        case '<': return 0x33; // Shift+,
        case '>': return 0x34; // Shift+.
        case '?': return 0x35; // Shift+/
        case '~': return 0x29; // Shift+`

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
        // Note: '*' обрабатывается выше как Shift+8
        default: return 0x00; // Unknown
    }
}

// ============================================================================
// Отправка обычного ASCII символа с автоматическим Shift и задержками
// ============================================================================
__force_inline void send_ascii_as_scancode(int ascii) {
    // Обработка Ctrl+буква (control characters 0x01-0x1F)
    if (ascii >= 0x01 && ascii <= 0x1F && ascii != 0x08 && ascii != 0x09 && ascii != 0x0D && ascii != 0x1B) {
        // Ctrl+A=0x01, Ctrl+B=0x02, ..., Ctrl+Z=0x1A
        // Ctrl+\=0x1C, Ctrl+]=0x1D, Ctrl+^=0x1E, Ctrl+_=0x1F
        // Исключаем Backspace(0x08), Tab(0x09), Enter(0x0D), ESC(0x1B) - они обрабатываются отдельно
        uint8_t letter_scancode;
        if (ascii <= 0x1A) {
            letter_scancode = ascii_to_scancode('a' + (ascii - 1)); // Ctrl+A-Z
        } else if (ascii == 0x1C) {
            letter_scancode = 0x2B; // Ctrl+\ (backslash)
        } else if (ascii == 0x1D) {
            letter_scancode = 0x1B; // Ctrl+] (right bracket)
        } else if (ascii == 0x1E) {
            letter_scancode = 0x07; // Ctrl+^ (6)
        } else { // 0x1F
            letter_scancode = 0x0C; // Ctrl+_ (minus)
        }

        handleScancode(0x1D); // Ctrl DOWN
        sleep_ms(1);
        handleScancode(letter_scancode);
        sleep_ms(1);
        handleScancode(0x9D); // Ctrl UP
        sleep_ms(1);
        return;
    }

    uint8_t scancode = ascii_to_scancode(ascii);
    if (scancode == 0x00) return; // Unknown character

    // Проверяем, нужен ли Shift для этого символа
    bool need_shift = false;

    if (ascii >= 'A' && ascii <= 'Z') {
        need_shift = true; // Uppercase letters
    } else if (ascii == '!' || ascii == '@' || ascii == '#' || ascii == '$' ||
               ascii == '%' || ascii == '^' || ascii == '&' || ascii == '*' ||
               ascii == '(' || ascii == ')' || ascii == '_' || ascii == '+' ||
               ascii == '{' || ascii == '}' || ascii == ':' || ascii == '"' ||
               ascii == '|' || ascii == '<' || ascii == '>' || ascii == '?' ||
               ascii == '~') {
        need_shift = true; // Shifted symbols
    }

    // Отправляем с задержками (нет буфера!)
    if (need_shift) {
        handleScancode(0x2A); // Left Shift DOWN
        sleep_ms(1);
    }

    handleScancode(scancode);
    sleep_ms(1);

    if (need_shift) {
        handleScancode(0xAA); // Left Shift UP
        sleep_ms(1);
    }
}

__force_inline void debug_init() {
    stdio_init_all();
}

// ============================================================================
// ANSI Escape Sequence Parser (для специальных клавиш терминала)
// ============================================================================
typedef enum {
    ANSI_STATE_NORMAL,      // Обычный ввод
    ANSI_STATE_ESC,         // Получен ESC (0x1B)
    ANSI_STATE_CSI,         // Получен ESC[ (начало CSI последовательности)
    ANSI_STATE_SS3          // Получен ESC O (SS3 последовательность)
} ansi_state_t;

// ============================================================================
// Отправка клавиши с модификаторами (Shift/Ctrl/Alt)
// ============================================================================
__force_inline void send_key_with_modifiers(uint8_t modifier, uint8_t scancode) {
    // ANSI модификаторы: 1=none, 2=Shift, 3=Alt, 4=Shift+Alt, 5=Ctrl, 6=Shift+Ctrl, 7=Alt+Ctrl, 8=All
    bool has_shift = (modifier == 2 || modifier == 4 || modifier == 6 || modifier == 8);
    bool has_alt = (modifier == 3 || modifier == 4 || modifier == 7 || modifier == 8);
    bool has_ctrl = (modifier >= 5 && modifier <= 8);

    // Отправляем make-коды модификаторов
    if (has_shift) {
        handleScancode(0x2A); // Left Shift DOWN
        sleep_ms(1);
    }
    if (has_alt) {
        handleScancode(0x38); // Left Alt DOWN
        sleep_ms(1);
    }
    if (has_ctrl) {
        handleScancode(0x1D); // Left Ctrl DOWN
        sleep_ms(1);
    }

    // Отправляем код клавиши
    handleScancode(scancode);
    sleep_ms(1); // Обязательная задержка - нет буфера!

    // Отправляем break-коды модификаторов (в обратном порядке)
    if (has_ctrl) {
        handleScancode(0x9D); // Left Ctrl UP
        sleep_ms(1);
    }
    if (has_alt) {
        handleScancode(0xB8); // Left Alt UP
        sleep_ms(1);
    }
    if (has_shift) {
        handleScancode(0xAA); // Left Shift UP
        sleep_ms(1);
    }
}

__force_inline void send_extended_key_with_modifiers(uint8_t modifier, uint8_t scancode) {
    // Для extended keys (стрелки, Home, End и т.д.) с префиксом 0xE0
    bool has_shift = (modifier == 2 || modifier == 4 || modifier == 6 || modifier == 8);
    bool has_alt = (modifier == 3 || modifier == 4 || modifier == 7 || modifier == 8);
    bool has_ctrl = (modifier >= 5 && modifier <= 8);

    // Отправляем make-коды модификаторов
    if (has_shift) {
        handleScancode(0x2A);
        sleep_ms(1);
    }
    if (has_alt) {
        handleScancode(0x38);
        sleep_ms(1);
    }
    if (has_ctrl) {
        handleScancode(0x1D);
        sleep_ms(1);
    }

    // Extended key с префиксом 0xE0
    handleScancode(0xE0);
    sleep_ms(1); // Задержка между префиксом и scancode!
    handleScancode(scancode);
    sleep_ms(1); // Обязательная задержка - нет буфера!

    // Отправляем break-коды модификаторов (в обратном порядке)
    if (has_ctrl) {
        handleScancode(0x9D);
        sleep_ms(1);
    }
    if (has_alt) {
        handleScancode(0xB8);
        sleep_ms(1);
    }
    if (has_shift) {
        handleScancode(0xAA);
        sleep_ms(1);
    }
}

__force_inline void process_ss3_sequence(char c) {
    // SS3 последовательности (ESC O X) - используются некоторыми терминалами для F1-F4
    switch (c) {
        case 'P': handleScancode(0x3B); break; // F1
        case 'Q': handleScancode(0x3C); break; // F2
        case 'R': handleScancode(0x3D); break; // F3
        case 'S': handleScancode(0x3E); break; // F4
        // Стрелки в некоторых режимах терминала
        case 'A': // Up
            handleScancode(0xE0);
            sleep_ms(1);
            handleScancode(0x48);
            break;
        case 'B': // Down
            handleScancode(0xE0);
            sleep_ms(1);
            handleScancode(0x50);
            break;
        case 'C': // Right
            handleScancode(0xE0);
            sleep_ms(1);
            handleScancode(0x4D);
            break;
        case 'D': // Left
            handleScancode(0xE0);
            sleep_ms(1);
            handleScancode(0x4B);
            break;
    }
}

__force_inline void process_ansi_sequence(const char *seq, int len) {
    if (len < 1) return;

    // Парсинг модификатора (формат: "1;2A" или "15;5~" где после ; идет модификатор)
    uint8_t modifier = 1; // По умолчанию без модификатора
    int num = 0;
    int semicolon_pos = -1;

    // Ищем точку с запятой
    for (int i = 0; i < len; i++) {
        if (seq[i] == ';') {
            semicolon_pos = i;
            break;
        }
    }

    if (semicolon_pos != -1) {
        // Парсим модификатор после точки с запятой
        modifier = 0; // Сбрасываем в 0 для корректного парсинга
        for (int i = semicolon_pos + 1; i < len; i++) {
            if (seq[i] >= '0' && seq[i] <= '9') {
                modifier = modifier * 10 + (seq[i] - '0');
            }
        }
        if (modifier == 0) modifier = 1; // Если ничего не нашли, без модификатора
    }

    // CSI последовательности (ESC[...)
    char terminator = seq[len - 1];

    if (terminator == 'A') { // Up arrow
        send_extended_key_with_modifiers(modifier, 0x48);
    } else if (terminator == 'B') { // Down arrow
        send_extended_key_with_modifiers(modifier, 0x50);
    } else if (terminator == 'C') { // Right arrow
        send_extended_key_with_modifiers(modifier, 0x4D);
    } else if (terminator == 'D') { // Left arrow
        send_extended_key_with_modifiers(modifier, 0x4B);
    } else if (terminator == 'H') { // Home
        send_extended_key_with_modifiers(modifier, 0x47);
    } else if (terminator == 'F') { // End
        send_extended_key_with_modifiers(modifier, 0x4F);
    } else if (terminator == 'P') { // F1 (некоторые терминалы: ESC[1;2P)
        send_key_with_modifiers(modifier, 0x3B);
    } else if (terminator == 'Q') { // F2
        send_key_with_modifiers(modifier, 0x3C);
    } else if (terminator == 'R') { // F3
        send_key_with_modifiers(modifier, 0x3D);
    } else if (terminator == 'S') { // F4
        send_key_with_modifiers(modifier, 0x3E);
    } else if (terminator == '~') {
        // Последовательности вида ESC[N~ или ESC[N;modifier~
        // Парсим число до точки с запятой (или до ~)
        int end_pos = (semicolon_pos != -1) ? semicolon_pos : len - 1;
        for (int i = 0; i < end_pos; i++) {
            if (seq[i] >= '0' && seq[i] <= '9') {
                num = num * 10 + (seq[i] - '0');
            }
        }

        switch (num) {
            case 1: send_extended_key_with_modifiers(modifier, 0x47); break; // Home
            case 2: send_extended_key_with_modifiers(modifier, 0x52); break; // Insert
            case 3: send_extended_key_with_modifiers(modifier, 0x53); break; // Delete
            case 4: send_extended_key_with_modifiers(modifier, 0x4F); break; // End
            case 5: send_extended_key_with_modifiers(modifier, 0x49); break; // Page Up
            case 6: send_extended_key_with_modifiers(modifier, 0x51); break; // Page Down
            case 11: send_key_with_modifiers(modifier, 0x3B); break; // F1
            case 12: send_key_with_modifiers(modifier, 0x3C); break; // F2
            case 13: send_key_with_modifiers(modifier, 0x3D); break; // F3
            case 14: send_key_with_modifiers(modifier, 0x3E); break; // F4
            case 15: send_key_with_modifiers(modifier, 0x3F); break; // F5
            case 17: send_key_with_modifiers(modifier, 0x40); break; // F6
            case 18: send_key_with_modifiers(modifier, 0x41); break; // F7
            case 19: send_key_with_modifiers(modifier, 0x42); break; // F8
            case 20: send_key_with_modifiers(modifier, 0x43); break; // F9
            case 21: send_key_with_modifiers(modifier, 0x44); break; // F10
            case 23: send_key_with_modifiers(modifier, 0x57); break; // F11
            case 24: send_key_with_modifiers(modifier, 0x58); break; // F12
        }
    }
}

__force_inline void debug_console(const int videomode) {
    static bool video_enabled = true;
    static bool ctty_mode = false; // false = keyboard mode, true = CTTY mode

    // ANSI escape-парсер state
    static ansi_state_t ansi_state = ANSI_STATE_NORMAL;
    static char ansi_buffer[16];
    static int ansi_buf_len = 0;
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

    // ═══════════════════════════════════════════════════════════════
    // Обработка ввода с учетом ANSI state machine
    // ═══════════════════════════════════════════════════════════════
    if (c == PICO_ERROR_TIMEOUT) {
        return; // Нет данных
    }

    // Если мы внутри ANSI последовательности - обрабатываем через state machine
    if (ansi_state != ANSI_STATE_NORMAL) {
        if (ctty_mode) {
            // В CTTY режиме отправляем все как есть
            uart.rbr = (uint8_t) c;
            uart.data_ready = true;
        } else {
            // Keyboard Mode: продолжаем парсинг ANSI
            switch (ansi_state) {
                case ANSI_STATE_ESC:
                    if (c == '[') {
                        ansi_state = ANSI_STATE_CSI;
                        ansi_buf_len = 0;
                    } else if (c == 'O') {
                        // SS3 последовательности (завершаются одним символом)
                        ansi_state = ANSI_STATE_SS3;
                    } else {
                        // Может быть: одиночный ESC, Alt+символ или двойной ESC
                        ansi_state = ANSI_STATE_NORMAL;

                        // Если это второй ESC подряд - отправляем только один ESC scancode
                        if (c == 0x1B) {
                            handleScancode(0x01); // Одиночный ESC
                            sleep_ms(1);
                        } else if (c >= 0x01 && c <= 0x1F && c != 0x08 && c != 0x09 && c != 0x0D && c != 0x1B) {
                            // Alt+Ctrl+буква (ESC + control character 0x01-0x1F)
                            // Alt+Ctrl+A-Z, Alt+Ctrl+\, Alt+Ctrl+], etc.
                            // Исключаем Backspace(0x08), Tab(0x09), Enter(0x0D), ESC(0x1B)
                            uint8_t letter_scancode;
                            if (c <= 0x1A) {
                                letter_scancode = ascii_to_scancode('a' + (c - 1));
                            } else if (c == 0x1C) {
                                letter_scancode = 0x2B; // backslash
                            } else if (c == 0x1D) {
                                letter_scancode = 0x1B; // ]
                            } else if (c == 0x1E) {
                                letter_scancode = 0x07; // ^
                            } else { // 0x1F
                                letter_scancode = 0x0C; // _
                            }
                            handleScancode(0x38); // Alt DOWN
                            sleep_ms(1);
                            handleScancode(0x1D); // Ctrl DOWN
                            sleep_ms(1);
                            handleScancode(letter_scancode);
                            sleep_ms(1);
                            handleScancode(0x9D); // Ctrl UP
                            sleep_ms(1);
                            handleScancode(0xB8); // Alt UP
                            sleep_ms(1);
                        } else {
                            // Обычный Alt+символ
                            uint8_t scancode = ascii_to_scancode(c);
                            if (scancode != 0x00) {
                                // Проверяем: Alt+Shift+буква? (ESC + uppercase)
                                bool need_shift = false;
                                if (c >= 'A' && c <= 'Z') {
                                    need_shift = true;
                                } else if (c == '!' || c == '@' || c == '#' || c == '$' ||
                                           c == '%' || c == '^' || c == '&' || c == '*' ||
                                           c == '(' || c == ')' || c == '_' || c == '+' ||
                                           c == '{' || c == '}' || c == ':' || c == '"' ||
                                           c == '|' || c == '<' || c == '>' || c == '?' ||
                                           c == '~') {
                                    need_shift = true;
                                }

                                handleScancode(0x38); // Alt DOWN
                                sleep_ms(1);
                                if (need_shift) {
                                    handleScancode(0x2A); // Shift DOWN
                                    sleep_ms(1);
                                }
                                handleScancode(scancode);
                                sleep_ms(1);
                                if (need_shift) {
                                    handleScancode(0xAA); // Shift UP
                                    sleep_ms(1);
                                }
                                handleScancode(0xB8); // Alt UP
                                sleep_ms(1);
                            }
                        }
                    }
                    break;

                case ANSI_STATE_SS3:
                    // SS3 последовательность завершается сразу одним символом
                    process_ss3_sequence((char)c);
                    ansi_state = ANSI_STATE_NORMAL;
                    break;

                case ANSI_STATE_CSI:
                    if (ansi_buf_len < (int)sizeof(ansi_buffer) - 1) {
                        ansi_buffer[ansi_buf_len++] = (char)c;
                    }

                    // Завершающие символы
                    if ((c >= 'A' && c <= 'Z') || c == '~') {
                        ansi_buffer[ansi_buf_len] = '\0';
                        process_ansi_sequence(ansi_buffer, ansi_buf_len);
                        ansi_state = ANSI_STATE_NORMAL;
                        ansi_buf_len = 0;
                    } else if (ansi_buf_len >= (int)sizeof(ansi_buffer) - 1) {
                        // Переполнение
                        ansi_state = ANSI_STATE_NORMAL;
                        ansi_buf_len = 0;
                    }
                    break;

                default:
                    break;
            }
        }
        return; // Выходим, не проверяя спецкоманды
    }

    // ═══════════════════════════════════════════════════════════════
    // Теперь мы точно в ANSI_STATE_NORMAL - проверяем спецкоманды
    // ═══════════════════════════════════════════════════════════════
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
    } else {
        // ═══════════════════════════════════════════════════════
        // Обработка обычных символов (не спецкоманды)
        // ═══════════════════════════════════════════════════════
        if (ctty_mode) {
            // CTTY Mode: USB → UART RBR → DOS (без ANSI парсинга)
            uart.rbr = (uint8_t) c;
            uart.data_ready = true;
        } else {
            // Keyboard Mode: проверяем начало ANSI последовательности
            if (c == 0x1B) { // ESC - начало ANSI последовательности
                ansi_state = ANSI_STATE_ESC;
                ansi_buf_len = 0;
            } else {
                // Обычный ASCII символ → scancode (с автоматическим Shift и задержками)
                send_ascii_as_scancode(c);
            }
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
