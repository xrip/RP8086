#include "graphics.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include <string.h>
#include <arm_acle.h>
#include <common.h>

#ifdef VGA_CSYNC
constexpr uint8_t VGA_PINS = 7;
#else
constexpr uint8_t VGA_PINS = 8;
#endif

// --- external hardware / memory references ---
extern uint8_t port3DA;
extern uint8_t VIDEORAM[];
extern mc6845_s mc6845;

// --- PIO program (1 instruction) ---
uint16_t pio_program_VGA_instructions[] = {
    //     .wrap_target
    0x6008, //  0: out    pins, 8
    //     .wrap
};

const struct pio_program pio_program_VGA = {
    .instructions = pio_program_VGA_instructions,
    .length = 1,
    .origin = -1,
};

// --- Scanline buffer storage (aligned) ---
// scanline_buffers: [0]=blank, [1]=vsync, [2]=odd image, [3]=even image
static uint32_t *scanline_buffers[4] __attribute__((aligned(4))) = {0};
static uint32_t scanline_buffer_mem[(400 * 2 * 4) / 4] __attribute__((aligned(32)));

static int pio_sm_vga = -1;

// --- Display geometry / timing (renamed) ---
static int total_scanlines = 525;        // previously N_lines_total
static int visible_scanlines = 480;      // previously N_lines_visible
static int vsync_start_line = 490;       // previously line_VS_begin
static int vsync_end_line = 491;         // previously line_VS_end
static int picture_hshift_pixels = 0;    // previously shift_picture

// --- DMA / PIO channels ---
static int dma_ctrl_chan;
static int dma_data_chan;

// --- Framebuffer / palette ---
static uint8_t *framebuffer_ptr = nullptr;  // previously graphics_framebuffer

static uint16_t palette[256] __attribute__((aligned(4)));

static constexpr uint16_t textmode_palette[16] __attribute__((aligned(4))) = {
    0b000000 & 0x3f | 0xc0,
    0b000001 & 0x3f | 0xc0,
    0b000100 & 0x3f | 0xc0,
    0b000101 & 0x3f | 0xc0,
    0b010000 & 0x3f | 0xc0,
    0b010001 & 0x3f | 0xc0,
    0b010100 & 0x3f | 0xc0,
    0b010101 & 0x3f | 0xc0,
    0b101010 & 0x3f | 0xc0,
    0b000011 & 0x3f | 0xc0,
    0b001100 & 0x3f | 0xc0,
    0b001111 & 0x3f | 0xc0,
    0b110000 & 0x3f | 0xc0,
    0b110011 & 0x3f | 0xc0,
    0b111100 & 0x3f | 0xc0,
    0b111111 & 0x3f | 0xc0,
};

// 2K text palette expanded for fast lookup: 256 * 4 entries
static uint16_t textmode_palette_lut[256 * 4] __attribute__((aligned(4)));

enum graphics_mode_t graphics_mode;

void __time_critical_func() vga_scanline_dma() {
    // acknowledge interrupt for control channel
    dma_hw->ints0 = 1u << dma_ctrl_chan;

    static uint32_t frame_counter = 0;     // previously frame_number
    static uint32_t current_scanline = 0;  // previously screen_line

    // advance scanline/frame counters
    current_scanline++;
    if (current_scanline == (uint32_t)total_scanlines) {
        current_scanline = 0;
        frame_counter++;
    }

    // If outside visible area - mark output as finished
    if (unlikely(current_scanline >= (uint32_t)visible_scanlines)) {
        port3DA = 8;
        const int idx = (current_scanline >= (uint32_t)vsync_start_line && current_scanline <= (uint32_t)vsync_end_line) ? 1 : 0;
        // write pointer in one op (avoid extra branching later)
        dma_channel_set_read_addr(dma_ctrl_chan, &scanline_buffers[idx], false);
        port3DA |= 1;
        return;
    }

    // activate output for visible lines
    port3DA = 0;

    // choose odd/even image buffer pointer
    const unsigned int odd_even = current_scanline & 1;

    uint32_t **scanline_output_ptr = &scanline_buffers[2 + odd_even];
    uint16_t *__restrict scanline_output_16 = (uint16_t *)(*scanline_output_ptr) + picture_hshift_pixels / 2;
    uint32_t *__restrict scanline_output_32 = (uint32_t *)scanline_output_16;

    // If line index beyond prepared image area — fall back to blank
    if (unlikely(current_scanline >= 400)) {
        dma_channel_set_read_addr(dma_ctrl_chan, &scanline_buffers[0], false);
        return;
    }

    uint32_t y = current_scanline;

    // Non-interlace: skip odd sublines and fold y
    if (likely((mc6845.r.interlace_mode & 1) == 0)) {
        if (odd_even) {
            return;
        }
        y >>= 1; // 200 logical lines
    }

    switch (graphics_mode) {
        case TEXTMODE_40x25_COLOR:
        case TEXTMODE_40x25_BW: {
            // "слой" символа
            uint8_t char_scanlines = mc6845.r.max_scanline_addr;
            const uint8_t glyph_line = y & char_scanlines;
            char_scanlines++;
            const uint8_t screen_y = y / char_scanlines;

            //указатель откуда начать считывать символы
            const uint32_t *__restrict text_buffer_line = (uint32_t *) VIDEORAM + mc6845.vram_offset + __fast_mul(screen_y, mc6845.r.h_displayed / 2);
            __builtin_prefetch(text_buffer_line);

            const bool is_cursor_line_active =
                    (screen_y == mc6845.cursor_y) &&
                    (likely(mc6845.r.cursor_start <= mc6845.r.cursor_end)
                         ? (glyph_line >= mc6845.r.cursor_start && glyph_line <= mc6845.r.cursor_end)
                         : (glyph_line >= mc6845.r.cursor_start || glyph_line <= mc6845.r.cursor_end));

            // Предвычисление позиции курсора
            const int cursor_char_x = is_cursor_line_active ? mc6845.cursor_x : -1;

            for (int char_x = 0; char_x < mc6845.r.h_displayed; char_x += 2) {
                uint32_t dword = *text_buffer_line++;

                // Первый символ из пачки
                uint8_t glyph_pixels = font_8x8[(dword & 0xFF) * char_scanlines + glyph_line];
                dword >>= 8;
                uint8_t color = dword;
                if (unlikely(mc6845.cursor_blink_state && (char_x == cursor_char_x))) {
                    glyph_pixels = 0xff; // Инвертируем все 2-битные пиксели разом
                } else if (unlikely(mc6845.cursor_blink_state && mc6845.text_blinking_mask == 0x7F && color & 0x80)) {
                    glyph_pixels = 0x00;
                }
                const uint16_t *palette_color = &textmode_palette_lut[4 * (color & mc6845.text_blinking_mask)];

                // генерируем 4 блока по 2-битным пикселям (удвоение по горизонтали для 40-колоночного режима)
                for (int k = 0; k < 4; ++k) {
                    const uint16_t palette = palette_color[glyph_pixels & 3];
                    const uint16_t lo = palette & 0xFF;
                    const uint16_t hi = palette >> 8;
                    const uint32_t out32 = (uint32_t) (lo << 8 | lo) | ((uint32_t) (hi << 8 | hi) << 16);
                    *scanline_output_32++ = out32;
                    glyph_pixels >>= 2;
                }

                // Второй символ из пачки
                dword >>= 8;
                glyph_pixels = font_8x8[(dword & 0xFF) * char_scanlines + glyph_line];
                dword >>= 8;
                color = dword;
                if (unlikely(mc6845.cursor_blink_state && ((char_x + 1) == cursor_char_x))) {
                    glyph_pixels = 0xff; // Инвертируем все 2-битные пиксели разом
                } else if (unlikely(mc6845.cursor_blink_state && mc6845.text_blinking_mask == 0x7F && color & 0x80)) {
                    glyph_pixels = 0x00;
                }
                palette_color = &textmode_palette_lut[4 * (color & mc6845.text_blinking_mask)];

                // генерируем 4 блока по 2-битным пикселям (удвоение по горизонтали для 40-колоночного режима)
                for (int k = 0; k < 4; ++k) {
                    const uint16_t palette = palette_color[glyph_pixels & 3];
                    const uint16_t lo = palette & 0xFF;
                    const uint16_t hi = palette >> 8;
                    const uint32_t out32 = (uint32_t) (lo << 8 | lo) | ((uint32_t) (hi << 8 | hi) << 16);
                    *scanline_output_32++ = out32;
                    glyph_pixels >>= 2;
                }
            }

            dma_channel_set_read_addr(dma_ctrl_chan, scanline_output_ptr, false);
            port3DA |= 1; // no more data shown
            return;
        }
        case TEXTMODE_80x25_COLOR:
        case TEXTMODE_80x25_BW: {
            // "слой" символа
            uint8_t char_scanlines = mc6845.r.max_scanline_addr;
            const uint8_t glyph_line = y & char_scanlines;
            char_scanlines++;
            const uint8_t screen_y = y / char_scanlines;

            //указатель откуда начать считывать символы
            const uint32_t *__restrict text_buffer_line = (uint32_t *) VIDEORAM + mc6845.vram_offset + __fast_mul(screen_y, mc6845.r.h_displayed / 2);
            const bool is_cursor_line_active =
                    unlikely(mc6845.cursor_blink_state) &&
                    (screen_y == mc6845.cursor_y) &&
                    (likely(mc6845.r.cursor_start <= mc6845.r.cursor_end)
                         ? (glyph_line >= mc6845.r.cursor_start && glyph_line <= mc6845.r.cursor_end)
                         : (glyph_line >= mc6845.r.cursor_start || glyph_line <= mc6845.r.cursor_end));

            // Предвычисление позиции курсора
            const int cursor_char_x = is_cursor_line_active ? mc6845.cursor_x : -1;

            for (int char_x = 0; char_x < mc6845.r.h_displayed; char_x += 2) {
                uint32_t dword = *text_buffer_line++;

                // Первый символ из пачки
                uint8_t glyph_pixels = font_8x8[(dword & 0xFF) * char_scanlines + glyph_line];
                dword >>= 8;
                uint8_t color = dword;
                if (unlikely(mc6845.cursor_blink_state && (char_x == cursor_char_x))) {
                    glyph_pixels = 0xff; // Инвертируем все 2-битные пиксели разом
                } else if (unlikely(mc6845.cursor_blink_state && mc6845.text_blinking_mask == 0x7F && color & 0x80)) {
                    glyph_pixels = 0x00;
                }
                const uint16_t *palette_color = &textmode_palette_lut[4 * (color & mc6845.text_blinking_mask)];

                // 32-битная запись (2 пикселя за раз)
                *scanline_output_32++ = palette_color[glyph_pixels & 3] | ((uint32_t) palette_color[glyph_pixels >> 2 & 3] << 16);
                *scanline_output_32++ = palette_color[glyph_pixels >> 4 & 3] | ((uint32_t) palette_color[glyph_pixels >> 6] << 16);

                // Второй символ из пачки
                dword >>= 8;
                glyph_pixels = font_8x8[(dword & 0xFF) * char_scanlines + glyph_line];
                dword >>= 8;
                color = dword;

                if (unlikely(mc6845.cursor_blink_state && ((char_x+1) == cursor_char_x))) {
                    glyph_pixels = 0xff; // Инвертируем все 2-битные пиксели разом
                } else if (unlikely(mc6845.cursor_blink_state && mc6845.text_blinking_mask == 0x7F && color & 0x80)) {
                    glyph_pixels = 0x00;
                }

                palette_color = &textmode_palette_lut[4 * (color & mc6845.text_blinking_mask)];

                // 32-битная запись (2 пикселя за раз)
                *scanline_output_32++ = palette_color[glyph_pixels & 3] | ((uint32_t) palette_color[glyph_pixels >> 2 & 3] << 16);
                *scanline_output_32++ = palette_color[glyph_pixels >> 4 & 3] | ((uint32_t) palette_color[glyph_pixels >> 6] << 16);
            }

            dma_channel_set_read_addr(dma_ctrl_chan, scanline_output_ptr, false);
            port3DA |= 1; // no more data shown
            return;
        }
    }
    // const uint16_t *current_palette = palette[(y & is_flash_line) + (frame_number & is_flash_frame) & 1];
    //const uint16_t *current_palette = palette;

    switch (graphics_mode) {
        case CGA_320x200x4:
        case CGA_320x200x4_BW: {
            const uint32_t *__restrict cga_row = (uint32_t *) (VIDEORAM + ((mc6845.vram_offset + __fast_mul(y >> 1, 80) + ((y & 1) << 13)) & 0x3FFF));
            __builtin_prefetch(cga_row);

            // 2bit buf, 16 pixels at once, 32-bit writes
            for (int x = 20; x--;) {
                const uint32_t dword = *cga_row++; // Fetch 16 pixels from CGA memory

                // младший байт (4 пикселя, 2 записи по 32 бита)
                *scanline_output_32++ = palette[(dword >> 6) & 3] | ((uint32_t) palette[(dword >> 4) & 3] << 16);
                *scanline_output_32++ = palette[(dword >> 2) & 3] | ((uint32_t) palette[dword & 3] << 16);

                // следующий байт (4 пикселя, 2 записи по 32 бита)
                *scanline_output_32++ = palette[(dword >> 14) & 3] | ((uint32_t) palette[(dword >> 12) & 3] << 16);
                *scanline_output_32++ = palette[(dword >> 10) & 3] | ((uint32_t) palette[(dword >> 8) & 3] << 16);

                // следующий байт (4 пикселя, 2 записи по 32 бита)
                *scanline_output_32++ = palette[(dword >> 22) & 3] | ((uint32_t) palette[(dword >> 20) & 3] << 16);
                *scanline_output_32++ = palette[(dword >> 18) & 3] | ((uint32_t) palette[(dword >> 16) & 3] << 16);

                // старший байт (4 пикселя, 2 записи по 32 бита)
                *scanline_output_32++ = palette[(dword >> 30)] | ((uint32_t) palette[(dword >> 28) & 3] << 16);
                *scanline_output_32++ = palette[(dword >> 26) & 3] | ((uint32_t) palette[(dword >> 24) & 3] << 16);
            }
            break;
        }
        case CGA_640x200x2: {
            const uint32_t *__restrict cga_row = (uint32_t *) (VIDEORAM + ((mc6845.vram_offset + __fast_mul(y >> 1, 80) + ((y & 1) << 13)) & 0x3FFF));
            __builtin_prefetch(cga_row);
            auto output_buffer_8bit = (uint8_t *) scanline_output_16;
            //1bit buf, 32 pixels at once
            for (int x = 20; x--;) {
                uint32_t dword = __rbit(__rev(*cga_row++)); // Fetch 32 pixels from CGA memory

#pragma GCC unroll(32)
                for (int i = 32; i--;) {
                    *output_buffer_8bit++ = palette[dword & 1];
                    dword >>= 1;
                }
            }
            break;
        }
        case TGA_160x200x16: {
            const uint32_t *__restrict tga_row = (uint32_t *) (VIDEORAM + (y & 1) * 8192 + __fast_mul(y >> 1, 80));
            __builtin_prefetch(tga_row);
            for (int x = 20; x--;) {
                const uint32_t dword = *tga_row++; // Fetch 8 pixels from TGA memory

                // Обработка первого байта (2 пикселя, удвоенные по горизонтали = 4 выходных пикселя)
                uint8_t pixel1 = (dword >> 4) & 15;
                uint8_t pixel2 = dword & 15;
                *scanline_output_32++ = palette[pixel1] << 16 | palette[pixel1];
                *scanline_output_32++ = palette[pixel2] << 16 | palette[pixel2];

                // Обработка второго байта
                pixel1 = (dword >> 12) & 15;
                pixel2 = (dword >> 8) & 15;
                *scanline_output_32++ = palette[pixel1] << 16 | palette[pixel1];
                *scanline_output_32++ = palette[pixel2] << 16 | palette[pixel2];

                // Обработка третьего байта
                pixel1 = (dword >> 20) & 15;
                pixel2 = (dword >> 16) & 15;
                *scanline_output_32++ = palette[pixel1] << 16 | palette[pixel1];
                *scanline_output_32++ = palette[pixel2] << 16 | palette[pixel2];

                // Обработка четвертого байта
                pixel1 = (dword >> 28);
                pixel2 = (dword >> 24) & 15;
                *scanline_output_32++ = palette[pixel1] << 16 | palette[pixel1];
                *scanline_output_32++ = palette[pixel2] << 16 | palette[pixel2];
            }
            break;
        }
        case TGA_320x200x16: {
            //4bit buf, 32-bit reads
            const uint32_t *__restrict tga_row = (uint32_t *) (VIDEORAM + (y & 3) * 8192 + __fast_mul(y >> 2, 160));
            __builtin_prefetch(tga_row);
            for (int x = 40; x--;) {
                const uint32_t dword = *tga_row++; // Fetch 8 pixels from TGA memory

                // Обработка 4 байтов (8 пикселей)
                *scanline_output_32++ = palette[dword & 15] << 16 | palette[(dword >> 4) & 15];
                *scanline_output_32++ = palette[(dword >> 8) & 15] << 16 | palette[(dword >> 12) & 15];
                *scanline_output_32++ = palette[(dword >> 16) & 15] << 16 | palette[(dword >> 20) & 15];
                *scanline_output_32++ = palette[(dword >> 24) & 15] << 16 | palette[(dword >> 28)];
            }
            break;
        }
        default:
            const uint8_t *vga_row = &VIDEORAM[__fast_mul(y, 160)];
            for (int x = 160; x--;) {
                *scanline_output_16++ = palette[*vga_row++];
            }
            break;
    }
    dma_channel_set_read_addr(dma_ctrl_chan, scanline_output_ptr, false);
    port3DA |= 1; // no more data shown
}

// -----------------------------------------------------------------------------
void graphics_set_buffer(uint8_t *buffer, const uint16_t width, const uint16_t height) {
    (void)width;
    (void)height;
    framebuffer_ptr = buffer;
}

// -----------------------------------------------------------------------------
void graphics_set_palette(const uint8_t index, const uint32_t color) {
    // color is RGB888, convert to 2-bit-per-component (0..3) and pack into 6-bit value
    const uint8_t r = ((color >> 16) & 0xff) >> 6;
    const uint8_t g = ((color >> 8) & 0xff) >> 6;
    const uint8_t b = (color & 0xff) >> 6;

    const uint8_t rgb = (r << 4) | (g << 2) | b;

    palette[index] = (rgb << 8 | rgb) & 0x3f3f | 0xc0c0;
}

// -----------------------------------------------------------------------------
void graphics_init() {
    // --- initialize PIO ---
    pio_set_gpio_base(PIO_VGA, 16);
    const uint offset = pio_add_program(PIO_VGA, &pio_program_VGA);
    pio_sm_vga = pio_claim_unused_sm(PIO_VGA, true);
    const uint sm = pio_sm_vga;

    for (int i = 0; i < VGA_PINS; i++) {
        gpio_init(VGA_BASE_PIN + i);
        gpio_set_dir(VGA_BASE_PIN + i, GPIO_OUT);
        pio_gpio_init(PIO_VGA, VGA_BASE_PIN + i);
    }

    pio_sm_set_consecutive_pindirs(PIO_VGA, sm, VGA_BASE_PIN, VGA_PINS, true);

    pio_sm_config cfg = pio_get_default_sm_config();
    sm_config_set_out_pin_base(&cfg, VGA_BASE_PIN);
    sm_config_set_wrap(&cfg, offset + 0, offset + (pio_program_VGA.length - 1));
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_out_pins(&cfg, VGA_BASE_PIN, VGA_PINS);

    pio_sm_init(PIO_VGA, sm, offset, &cfg);
    pio_sm_set_enabled(PIO_VGA, sm, true);

    // --- initialize DMA channels ---
    dma_ctrl_chan = dma_claim_unused_channel(true);
    dma_data_chan = dma_claim_unused_channel(true);

    // main data channel config
    dma_channel_config data_cfg = dma_channel_get_default_config(dma_data_chan);
    channel_config_set_transfer_data_size(&data_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&data_cfg, true);
    channel_config_set_write_increment(&data_cfg, false);

    uint dreq = DREQ_PIO1_TX0 + sm;
    if (PIO_VGA == pio0) {
        dreq = DREQ_PIO0_TX0 + sm;
    }

    channel_config_set_dreq(&data_cfg, dreq);
    channel_config_set_chain_to(&data_cfg, dma_ctrl_chan);

    dma_channel_configure(
        dma_data_chan,
        &data_cfg,
        &PIO_VGA->txf[sm],   // write address (PIO TX FIFO)
        scanline_buffers[0], // read address (will be updated)
        600 / 4,
        false);

    // control channel config
    dma_channel_config ctrl_cfg = dma_channel_get_default_config(dma_ctrl_chan);
    channel_config_set_transfer_data_size(&ctrl_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&ctrl_cfg, false);
    channel_config_set_write_increment(&ctrl_cfg, false);
    channel_config_set_chain_to(&ctrl_cfg, dma_data_chan);

    dma_channel_configure(
        dma_ctrl_chan,
        &ctrl_cfg,
        &dma_hw->ch[dma_data_chan].read_addr, // write address (point to read_addr of data channel)
        &scanline_buffers[0],                 // read address (pattern pointer)
        1,
        false);

    // default graphics mode
    graphics_set_mode(CGA_320x200x4);

    // IRQ setup
    irq_set_exclusive_handler(VGA_DMA_IRQ, vga_scanline_dma);
    dma_channel_set_irq0_enabled(dma_ctrl_chan, true);
    irq_set_enabled(VGA_DMA_IRQ, true);

    dma_start_channel_mask(1u << dma_data_chan);

    // --- prepare text palette expanded table ---
    for (int i = 0; i < 256; i++) {
        const uint8_t c1 = textmode_palette[i & 0xF];
        const uint8_t c0 = textmode_palette[i >> 4];

        textmode_palette_lut[i * 4 + 0] = c0 | (c0 << 8);
        textmode_palette_lut[i * 4 + 1] = c1 | (c0 << 8);
        textmode_palette_lut[i * 4 + 2] = c0 | (c1 << 8);
        textmode_palette_lut[i * 4 + 3] = c1 | (c1 << 8);
    }

    // --- timing/template constants and buffer setup ---
    constexpr uint8_t tmpl_active_video = 0b11000000; // TMPL_LINE8
    constexpr int hsync_offset_pixels = 328 * 2;      // HS_SHIFT
    constexpr int hsync_pulse_width = 48 * 2;         // HS_SIZE

    constexpr int scanline_bytes = 400 * 2;
#ifdef VGA_CSYNC
    picture_hshift_pixels = scanline_bytes - hsync_offset_pixels + 12;
#else
    picture_hshift_pixels = scanline_bytes - hsync_offset_pixels;
#endif
    // set PIO clock divider approximately for 25.175MHz pixel clock
    const uint32_t div32 = (uint32_t)(clock_get_hz(clk_sys) / 25175000 * (1 << 16) + 0.0);
    PIO_VGA->sm[pio_sm_vga].clkdiv = div32 & 0xfffff000;

    // tell data DMA how many 32-bit words per scanline
    dma_channel_set_trans_count(dma_data_chan, scanline_bytes / 4, false);

    // assign buffer pointers into single large array
    for (int i = 0; i < 4; i++) {
        scanline_buffers[i] = &scanline_buffer_mem[i * (scanline_bytes / 4)];
    }

    // prepare templates
#ifdef VGA_CSYNC
    // --- CSYNC MODE (VHBBGGRR) ---
    // HSync (bit 6) несет композитный сигнал.
    // VSync (bit 7) всегда 1 (отключен/неактивен).
    // Логика: XNOR (стандартная композитная синхра для VGA входов типа GBS-C/Scart).

    const uint8_t tmpl_hsync         = 0b10000000; // Обычная строка: импульс HSync = 0 (Bit6=0, Bit7=1)
    const uint8_t tmpl_video_sync    = 0b10000000; // VSync строка: фон = 0 (Bit6=0, Bit7=1)
    const uint8_t tmpl_video_hv_sync = 0b11000000; // VSync строка: импульс (serration) = 1 (Bit6=1, Bit7=1)
    // tmpl_active_video остается 0b11000000 (Bit6=1, Bit7=1)
#else
    // --- STANDARD VGA MODE (VHBBGGRR) ---
    const uint8_t tmpl_video_hv_sync = tmpl_active_video ^ 0b11000000; // 00... (V=0, H=0)
    const uint8_t tmpl_video_sync    = tmpl_active_video ^ 0b10000000; // 01... (V=0, H=1)
    const uint8_t tmpl_hsync         = tmpl_active_video ^ 0b01000000; // 10... (V=1, H=0)
#endif

    // base pointer to buffer memory as bytes
    auto base_ptr = (uint8_t *)scanline_buffers[0];

    // пустая строка (active video background)
    memset(base_ptr, tmpl_active_video, scanline_bytes);

    // выровненная синхра вначале
    memset(base_ptr, tmpl_hsync, hsync_pulse_width);

    // кадровая синхра (vsync)
    base_ptr = (uint8_t *)scanline_buffers[1];
    memset(base_ptr, tmpl_video_sync, scanline_bytes);
    memset(base_ptr, tmpl_video_hv_sync, hsync_pulse_width);

    // заготовки для строк с изображением (copy blank template)
    base_ptr = (uint8_t *)scanline_buffers[2];
    memcpy(base_ptr, scanline_buffers[0], scanline_bytes);
    base_ptr = (uint8_t *)scanline_buffers[3];
    memcpy(base_ptr, scanline_buffers[0], scanline_bytes);
}

// -----------------------------------------------------------------------------
void graphics_set_mode(const enum graphics_mode_t mode) {
    graphics_mode = mode;
}