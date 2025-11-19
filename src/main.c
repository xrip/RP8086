// RP2350B Related libs
#include <hardware/pwm.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>
#include <hardware/structs/qmi.h>
#include <hardware/structs/xip.h>


// 8086 related libs
#include "cpu.h"
#include "cpu_bus.h"
#include "common.h"
#include "graphics.h"

#include "hardware/i8237.h"
#include "hardware/i8259.h"
#include "hardware/i8253.h"
#include "hardware/uart16550.h"

#ifndef DEBUG
#include "hid_app.h"
#include "tusb.h"
#else
#include "pico/stdio.h"
#endif

extern cga_s cga;
extern mc6845_s mc6845;
uint8_t videomode = 0;
repeating_timer_t irq0_timer;

bool ctty_mode = false; // false = keyboard mode, true = CTTY mode
uint8_t current_scancode = 0; // 0 = нет данных
pwm_config pwm;

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

    while (true) {
        // Управление сигналом INTR
        gpio_put(INTR_PIN, i8259_get_pending_irqs());

        __wfe();
        tight_loop_contents();
    }
}

void psram_init(const int cs_pin) {
    gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);

    // Enable direct mode, PSRAM CS, clkdiv of 10
    qmi_hw->direct_csr = 10 << QMI_DIRECT_CSR_CLKDIV_LSB |
                         QMI_DIRECT_CSR_EN_BITS |
                         QMI_DIRECT_CSR_AUTO_CS1N_BITS;

    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        tight_loop_contents();
    }

    // Enable QPI mode on the PSRAM
    constexpr uint CMD_QPI_EN = 0x35;
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS | CMD_QPI_EN;

    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        tight_loop_contents();
    }

    // Set PSRAM timing
    constexpr int max_psram_freq = PSRAM_FREQ_MHZ;
    const int clock_hz = clock_get_hz(clk_sys);
    int divisor = (clock_hz + max_psram_freq - 1) / max_psram_freq;

    if (divisor == 1 && clock_hz > 100000000) {
        divisor = 2;
    }

    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) {
        rxdelay += 1;
    }

    // Calculate timing parameters
    const int clock_period_fs = 1000000000000000ll / clock_hz;
    const int max_select = (125 * 1000000) / clock_period_fs; // 125 = 8000ns / 64
    const int min_deselect = (18 * 1000000 + (clock_period_fs - 1)) / clock_period_fs - (divisor + 1) / 2;

    qmi_hw->m[1].timing = 1 << QMI_M1_TIMING_COOLDOWN_LSB |
                          QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB |
                          max_select << QMI_M1_TIMING_MAX_SELECT_LSB |
                          min_deselect << QMI_M1_TIMING_MIN_DESELECT_LSB |
                          rxdelay << QMI_M1_TIMING_RXDELAY_LSB |
                          divisor << QMI_M1_TIMING_CLKDIV_LSB;

    // Set PSRAM read format
    qmi_hw->m[1].rfmt = QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB |
                        QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB |
                        QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |
                        QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB |
                        QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB |
                        QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB |
                        6 << QMI_M0_RFMT_DUMMY_LEN_LSB;

    qmi_hw->m[1].rcmd = 0xEB;

    // Set PSRAM write format
    qmi_hw->m[1].wfmt = QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB |
                        QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_WFMT_ADDR_WIDTH_LSB |
                        QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB |
                        QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_WFMT_DUMMY_WIDTH_LSB |
                        QMI_M0_WFMT_DATA_WIDTH_VALUE_Q << QMI_M0_WFMT_DATA_WIDTH_LSB |
                        QMI_M0_WFMT_PREFIX_LEN_VALUE_8 << QMI_M0_WFMT_PREFIX_LEN_LSB;

    qmi_hw->m[1].wcmd = 0x38;

    // Disable direct mode
    qmi_hw->direct_csr = 0;

    // Enable writes to PSRAM
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);
    // detect a chip size
}

// Corrected CGA palette from https://int10h.org/blog/2022/06/ibm-5153-color-true-cga-palette/
constexpr uint32_t cga_palette[16] = {
    //R, G, B
    0x000000, // 0 black
    0x0000C4, // 1 blue
    0x00C400, // 2 green
    0x00C4C4, // 3 cyan
    0xC40000, // 4 red
    0xC400C4, // 5 magenta
    0xC47E00, // 6 brown
    0xC4C4C4, // 7 light gray
    0x4E4E4E, // 8 dark gray
    0x4E4EDC, // 9 light blue
    0x4EDC4E, // 10 light green
    0x4EF3F3, // 11 light cyan
    0xDC4E4E, // 12 light red
    0xF34EF3, // 13 light magenta
    0xF3F34E, // 14 yellow
    0xFFFFFF, // 15 white
};

// Pallete, intensity, color_index from cga_palette
constexpr uint8_t cga_gfxpal[3][2][4] = {
    //palettes for 320x200 graphics mode
    {
        {0, 2, 4, 6}, //normal palettes
        {0, 10, 12, 14}, //intense palettes
    },
    {
        {0, 3, 5, 7},
        {0, 11, 13, 15},
    },
    {
        // the unofficial Mode 5 palette, accessed by disabling ColorBurst
        {0, 3, 4, 7},
        {0, 11, 12, 15},
    },
};


bool handleScancode(const uint32_t ps2scancode) {
    current_scancode = ps2scancode;
    i8259_interrupt(1); // IRQ1 - Keyboard interrupt через i8259
    return true;
}


[[noreturn]] int main() {
    // IMPORTANT! Dont remove, hack to create .flashdata section for linker
    extern uint32_t PICO_CLOCK_SPEED_MHZ;
    assert(PICO_CLOCK_SPEED_MHZ == PICO_CLOCK_SPEED);
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_60);
    busy_wait_at_least_cycles((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t) XOSC_HZ) / 1000000);
    qmi_hw->m[0].timing = 0x60007305; // 4x FLASH divisor
    set_sys_clock_hz(PICO_CLOCK_SPEED_MHZ, true);
    psram_init(47);
    // busy_wait_ms(250); // Даем время стабилизироваться напряжению


#ifndef DEBUG
    tusb_init();
    keyboard_init();
    mouse_init();  // Инициализация поддержки Microsoft Serial Mouse

#else
    stdio_init_all();
    // while (!stdio_usb_connected()) { tight_loop_contents(); }
#endif

    pwm = pwm_get_default_config();
    gpio_set_function(BEEPER_PIN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&pwm, 127);
    pwm_init(pwm_gpio_to_slice_num(BEEPER_PIN), &pwm, true);

    multicore_launch_core1(bus_handler_core);

    absolute_time_t next_frame = get_absolute_time();
    next_frame = delayed_by_us(next_frame, 16666);


    bool video_enabled = true;

    graphics_init();
    graphics_set_buffer((uint8_t *) VIDEORAM, 320, 200);
    graphics_set_textbuffer((uint8_t *) VIDEORAM);
    graphics_set_bgcolor(0);
    graphics_set_offset(0, 0);
    graphics_set_flashmode(true, true);
    graphics_set_mode(CGA_320x200x4);
    busy_wait_us(33);
    graphics_set_mode(TEXTMODE_80x25_BW);
    for (int i = 0; i < 16; i++) {
        graphics_set_palette(i, cga_palette[i]);
    }

    uint32_t frame_counter = 0;
    uint8_t old_videomode = 0;
    while (true) {
        for (dma_channel_s *channel = dma_channels; channel < dma_channels + DMA_CHANNELS; channel++) {
            if (channel->dreq && !channel->masked) {
                // Вычисляем физический адрес назначения
                const uint32_t dest_addr = channel->page + channel->address;
                const size_t size = (uint32_t) channel->count + 1;

                memcpy(&RAM[dest_addr], channel->data_source, size);

                // Обновляем счётчики
                update_count(channel, size);

                // Генерируем IRQ если назначен (после завершения передачи!)
                if (channel->finished) {
                    channel->dreq = 0;

                    if (channel->irq)
                        i8259_interrupt(channel->irq);
                    // printf("DMA CH%i transfer compete from %x to %x size %x, irq %d\n", dma_channels-channel, channel->data_source, dest_addr, size, channel->irq);
                }
            }
        }

        // Отрисовка MDA фреймбуфера
        if (absolute_time_diff_us(next_frame, get_absolute_time()) >= 0) {
#ifndef DEBUG
            keyboard_tick();
#endif
            next_frame = delayed_by_us(next_frame, 16666);
            mc6845.cursor_blink_state = frame_counter++ >> 4 & 1;


            if (cga.updated) {
                if (unlikely(cga.port3D8 & 0b10)) {
                    // Bit 1: Graphics/Text Select
                    if (unlikely(cga.port3D8 & 0b10000)) {
                        {
                            videomode = CGA_640x200x2;

                            graphics_set_palette(0, 0);
                            graphics_set_palette(1, cga_palette[cga.port3D9 & 0b1111]);
                        }
                    } else {
                        videomode = CGA_320x200x4;
                        // If colorburst set -- 3rd palette, else from palette register
                        const uint8_t palette = (cga.port3D8 & 4) ? 2 : ((cga.port3D9 >> 5) & 1);
                        const uint8_t intensity = (cga.port3D9 >> 4) & 1;

                        graphics_set_palette(0, cga_palette[cga.port3D9 & 0b1111]);

                        for (int i = 1; i < 4; i++) {
                            graphics_set_palette(i, cga_palette[cga_gfxpal[palette][intensity][i]]);
                        }
                    }
                } else {
                    videomode = cga.port3D8 & 1 ? TEXTMODE_80x25_COLOR : TEXTMODE_40x25_COLOR;
                    for (int i = 0; i < 16; i++) {
                        graphics_set_palette(i, cga_palette[i]);
                    }
                }

                if (unlikely(cga.port3DA_tandy /* == 0x20 */)) {
                    // printf("Tandy hack detected: %i\n", videomode);
                    // videomode = (cga.port3D8 & 0b10000) ? TGA_320x200x16 : TGA_160x200x16;
                    videomode = videomode == CGA_640x200x2 ? TGA_320x200x16 : TGA_160x200x16;
                    for (int i = 0; i < 16; i++) {
                        graphics_set_palette(i, cga_palette[i]);
                    }
                }

                if (videomode != old_videomode) {
                    // printf("Videomode %i\n", videomode);
                    graphics_set_mode(videomode);
                    old_videomode = videomode;
                }

                cga.updated = false;
            }
#if DEBUG
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
#endif
        }
#if DEBUG
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
                const uint8_t scancode = ascii_to_scancode(c);
                push_scancode(scancode);
            }
        }
#endif
        //__wfi();
        tight_loop_contents();
    }
}
