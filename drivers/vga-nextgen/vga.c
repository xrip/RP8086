#include "graphics.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include <string.h>
#include <arm_acle.h>
#include <common.h>

uint16_t pio_program_VGA_instructions[] = {
    //     .wrap_target
    0x6008, //  0: out    pins, 8
    //     .wrap
};

extern uint8_t port3DA;
extern uint8_t VIDEORAM[];
extern mc6845_s mc6845;

const struct pio_program pio_program_VGA = {
    .instructions = pio_program_VGA_instructions,
    .length = 1,
    .origin = -1,
};

static uint32_t *lines_pattern[4] __attribute__((aligned(4))) = {0};
static uint32_t lines_pattern_data[400 * 2 * 4 / 4] __attribute__((aligned(4)));
static int _SM_VGA = -1;


static int N_lines_total = 525;
static int N_lines_visible = 480;
static int line_VS_begin = 490;
static int line_VS_end = 491;
static int shift_picture = 0;

static int visible_line_size = 320;


static int dma_channel_control;
static int dma_channel_data;

static uint8_t *graphics_framebuffer;

static uint16_t __aligned(4) palette[256];

static constexpr uint16_t __aligned(4) txt_palette[16] = {
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

//буфер 2К текстовой палитры для быстрой работы
//static uint16_t *txt_palette_fast = NULL;
static uint16_t __aligned(4) txt_palette_fast[256 * 4];

enum graphics_mode_t graphics_mode;

void __time_critical_func() dma_handler_VGA() {
    dma_hw->ints0 = 1u << dma_channel_control;
    static uint32_t frame_number = 0;
    static uint32_t screen_line = 0;
    screen_line++;

    if (screen_line == N_lines_total) {
        screen_line = 0;
        frame_number++;
    }

    if (screen_line >= N_lines_visible) {
        port3DA = 8; // useful frame is finished
        /*//заполнение цветом фона
        if (screen_line == N_lines_visible | screen_line == N_lines_visible + 3) {
            uint32_t *output_buffer_32bit = lines_pattern[2 + (screen_line & 1)];
            output_buffer_32bit += shift_picture / 4;
            uint32_t p_i = (screen_line & is_flash_line) & 1;
            uint32_t color32 = bg_color[p_i];
            for (int i = visible_line_size / 2; i--;) {
                *output_buffer_32bit++ = color32;
            }
        }*/

        //синхросигналы
        if (screen_line >= line_VS_begin && screen_line <= line_VS_end)
            dma_channel_set_read_addr(dma_channel_control, &lines_pattern[1], false); //VS SYNC
        else
            dma_channel_set_read_addr(dma_channel_control, &lines_pattern[0], false);
        port3DA |= 1; // no more data shown
        return;
    }

    port3DA = 0; // activated output

    uint32_t * *output_buffer = &lines_pattern[2 + (screen_line & 1)];
    uint16_t *__restrict output_buffer_16bit = (uint16_t *) (*output_buffer) + shift_picture / 2;
    auto output_buffer_32bit = (uint32_t *) output_buffer_16bit;


    if (screen_line >= 400) {
        dma_channel_set_read_addr(dma_channel_control, &lines_pattern[0], false); // TODO: ensue it is required
        return;
    }

    uint32_t y = screen_line;

    // Non-interlace: удвоение каждой строки (пропуск нечётных)
    if (likely((mc6845.r.interlace_mode & 1) == 0)) {
        if (screen_line & 1)
            return;
        y >>= 1; // 200 логических строк
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
                const uint16_t *palette_color = &txt_palette_fast[4 * (color & mc6845.text_blinking_mask)];

                // генерируем 4 блока по 2-битным пикселям (удвоение по горизонтали для 40-колоночного режима)
                for (int k = 0; k < 4; ++k) {
                    const uint16_t palette = palette_color[glyph_pixels & 3];
                    const uint16_t lo = palette & 0xFF;
                    const uint16_t hi = palette >> 8;
                    const uint32_t out32 = (uint32_t) (lo << 8 | lo) | ((uint32_t) (hi << 8 | hi) << 16);
                    *output_buffer_32bit++ = out32;
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
                palette_color = &txt_palette_fast[4 * (color & mc6845.text_blinking_mask)];

                // генерируем 4 блока по 2-битным пикселям (удвоение по горизонтали для 40-колоночного режима)
                for (int k = 0; k < 4; ++k) {
                    const uint16_t palette = palette_color[glyph_pixels & 3];
                    const uint16_t lo = palette & 0xFF;
                    const uint16_t hi = palette >> 8;
                    const uint32_t out32 = (uint32_t) (lo << 8 | lo) | ((uint32_t) (hi << 8 | hi) << 16);
                    *output_buffer_32bit++ = out32;
                    glyph_pixels >>= 2;
                }
            }

            dma_channel_set_read_addr(dma_channel_control, output_buffer, false);
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
                const uint16_t *palette_color = &txt_palette_fast[4 * (color & mc6845.text_blinking_mask)];

                // 32-битная запись (2 пикселя за раз)
                *output_buffer_32bit++ = palette_color[glyph_pixels & 3] | ((uint32_t) palette_color[glyph_pixels >> 2 & 3] << 16);
                *output_buffer_32bit++ = palette_color[glyph_pixels >> 4 & 3] | ((uint32_t) palette_color[glyph_pixels >> 6] << 16);

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

                palette_color = &txt_palette_fast[4 * (color & mc6845.text_blinking_mask)];

                // 32-битная запись (2 пикселя за раз)
                *output_buffer_32bit++ = palette_color[glyph_pixels & 3] | ((uint32_t) palette_color[glyph_pixels >> 2 & 3] << 16);
                *output_buffer_32bit++ = palette_color[glyph_pixels >> 4 & 3] | ((uint32_t) palette_color[glyph_pixels >> 6] << 16);
            }

            dma_channel_set_read_addr(dma_channel_control, output_buffer, false);
            port3DA |= 1; // no more data shown
            return;
        }
    }
    // const uint16_t *current_palette = palette[(y & is_flash_line) + (frame_number & is_flash_frame) & 1];
    const uint16_t *current_palette = palette;

    switch (graphics_mode) {
        case CGA_320x200x4:
        case CGA_320x200x4_BW: {
            const uint32_t *__restrict cga_row = (uint32_t *) (VIDEORAM + ((mc6845.vram_offset + __fast_mul(y >> 1, 80) + ((y & 1) << 13)) & 0x3FFF));

            // 2bit buf, 16 pixels at once, 32-bit writes
            for (int x = 20; x--;) {
                const uint32_t dword = *cga_row++; // Fetch 16 pixels from CGA memory

                // младший байт (4 пикселя, 2 записи по 32 бита)
                *output_buffer_32bit++ = current_palette[(dword >> 6) & 3] | ((uint32_t) current_palette[(dword >> 4) & 3] << 16);
                *output_buffer_32bit++ = current_palette[(dword >> 2) & 3] | ((uint32_t) current_palette[dword & 3] << 16);

                // следующий байт (4 пикселя, 2 записи по 32 бита)
                *output_buffer_32bit++ = current_palette[(dword >> 14) & 3] | ((uint32_t) current_palette[(dword >> 12) & 3] << 16);
                *output_buffer_32bit++ = current_palette[(dword >> 10) & 3] | ((uint32_t) current_palette[(dword >> 8) & 3] << 16);

                // следующий байт (4 пикселя, 2 записи по 32 бита)
                *output_buffer_32bit++ = current_palette[(dword >> 22) & 3] | ((uint32_t) current_palette[(dword >> 20) & 3] << 16);
                *output_buffer_32bit++ = current_palette[(dword >> 18) & 3] | ((uint32_t) current_palette[(dword >> 16) & 3] << 16);

                // старший байт (4 пикселя, 2 записи по 32 бита)
                *output_buffer_32bit++ = current_palette[(dword >> 30)] | ((uint32_t) current_palette[(dword >> 28) & 3] << 16);
                *output_buffer_32bit++ = current_palette[(dword >> 26) & 3] | ((uint32_t) current_palette[(dword >> 24) & 3] << 16);
            }
            break;
        }
        case CGA_640x200x2: {
            const uint32_t *__restrict cga_row = (uint32_t *) (VIDEORAM + ((mc6845.vram_offset + __fast_mul(y >> 1, 80) + ((y & 1) << 13)) & 0x3FFF));
            auto output_buffer_8bit = (uint8_t *) output_buffer_16bit;
            //1bit buf, 32 pixels at once
            for (int x = 20; x--;) {
                uint32_t dword = __rbit(__rev(*cga_row++)); // Fetch 32 pixels from CGA memory

#pragma GCC unroll(32)
                for (int i = 32; i--;) {
                    *output_buffer_8bit++ = current_palette[dword & 1];
                    dword >>= 1;
                }
            }
            break;
        }
        case TGA_160x200x16: {
            const uint32_t *__restrict tga_row = (uint32_t *) (VIDEORAM + (y & 1) * 8192 + __fast_mul(y >> 1, 80));
            for (int x = 20; x--;) {
                const uint32_t dword = *tga_row++; // Fetch 8 pixels from TGA memory

                // Обработка первого байта (2 пикселя, удвоенные по горизонтали = 4 выходных пикселя)
                uint8_t pixel1 = (dword >> 4) & 15;
                uint8_t pixel2 = dword & 15;
                *output_buffer_32bit++ = current_palette[pixel1] << 16 | current_palette[pixel1];
                *output_buffer_32bit++ = current_palette[pixel2] << 16 | current_palette[pixel2];

                // Обработка второго байта
                pixel1 = (dword >> 12) & 15;
                pixel2 = (dword >> 8) & 15;
                *output_buffer_32bit++ = current_palette[pixel1] << 16 | current_palette[pixel1];
                *output_buffer_32bit++ = current_palette[pixel2] << 16 | current_palette[pixel2];

                // Обработка третьего байта
                pixel1 = (dword >> 20) & 15;
                pixel2 = (dword >> 16) & 15;
                *output_buffer_32bit++ = current_palette[pixel1] << 16 | current_palette[pixel1];
                *output_buffer_32bit++ = current_palette[pixel2] << 16 | current_palette[pixel2];

                // Обработка четвертого байта
                pixel1 = (dword >> 28);
                pixel2 = (dword >> 24) & 15;
                *output_buffer_32bit++ = current_palette[pixel1] << 16 | current_palette[pixel1];
                *output_buffer_32bit++ = current_palette[pixel2] << 16 | current_palette[pixel2];
            }
            break;
        }
        case TGA_320x200x16: {
            //4bit buf, 32-bit reads
            const uint32_t *__restrict tga_row = (uint32_t *) (VIDEORAM + (y & 3) * 8192 + __fast_mul(y >> 2, 160));
            for (int x = 40; x--;) {
                const uint32_t dword = *tga_row++; // Fetch 8 pixels from TGA memory

                // Обработка 4 байтов (8 пикселей)
                *output_buffer_32bit++ = current_palette[dword & 15] << 16 | current_palette[(dword >> 4) & 15];
                *output_buffer_32bit++ = current_palette[(dword >> 8) & 15] << 16 | current_palette[(dword >> 12) & 15];
                *output_buffer_32bit++ = current_palette[(dword >> 16) & 15] << 16 | current_palette[(dword >> 20) & 15];
                *output_buffer_32bit++ = current_palette[(dword >> 24) & 15] << 16 | current_palette[(dword >> 28)];
            }
            break;
        }
        default:
            const uint8_t *vga_row = &VIDEORAM[__fast_mul(y, 160)];
            for (int x = 160; x--;) {
                *output_buffer_16bit++ = current_palette[*vga_row++];
            }
            break;
    }
    dma_channel_set_read_addr(dma_channel_control, output_buffer, false);
    port3DA |= 1; // no more data shown
}

void graphics_set_buffer(uint8_t *buffer, const uint16_t width, const uint16_t height) {
    graphics_framebuffer = buffer;
}

void graphics_set_palette(const uint8_t index, const uint32_t color) {
    const uint8_t r = (color >> 16 & 0xff) >> 6;
    const uint8_t g = (color >> 8 & 0xff) >> 6;
    const uint8_t b = (color & 0xff) >> 6;

    const uint8_t rgb = r << 4 | g << 2 | b;

    palette[index] = (rgb << 8 | rgb) & 0x3f3f | 0xc0c0;
}

void graphics_init() {
    //инициализация PIO
    //загрузка программы в один из PIO
    pio_set_gpio_base(PIO_VGA, 16);
    const uint offset = pio_add_program(PIO_VGA, &pio_program_VGA);
    _SM_VGA = pio_claim_unused_sm(PIO_VGA, true);
    const uint sm = _SM_VGA;

    for (int i = 0; i < 8; i++) {
        gpio_init(VGA_BASE_PIN + i);
        gpio_set_dir(VGA_BASE_PIN + i, GPIO_OUT);
        pio_gpio_init(PIO_VGA, VGA_BASE_PIN + i);
    }; //резервируем под выход PIO

    //pio_sm_config c = pio_vga_program_get_default_config(offset);

    pio_sm_set_consecutive_pindirs(PIO_VGA, sm, VGA_BASE_PIN, 8, true); //конфигурация пинов на выход

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pin_base(&c, VGA_BASE_PIN);
    sm_config_set_wrap(&c, offset + 0, offset + (pio_program_VGA.length - 1));

    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX); //увеличение буфера TX за счёт RX до 8-ми
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_out_pins(&c, VGA_BASE_PIN, 8);
    pio_sm_init(PIO_VGA, sm, offset, &c);

    pio_sm_set_enabled(PIO_VGA, sm, true);

    //инициализация DMA
    dma_channel_control = dma_claim_unused_channel(true);
    dma_channel_data = dma_claim_unused_channel(true);
    //основной ДМА канал для данных
    dma_channel_config c0 = dma_channel_get_default_config(dma_channel_data);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);

    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);

    uint dreq = DREQ_PIO1_TX0 + sm;
    if (PIO_VGA == pio0) dreq = DREQ_PIO0_TX0 + sm;

    channel_config_set_dreq(&c0, dreq);
    channel_config_set_chain_to(&c0, dma_channel_control); // chain to other channel

    dma_channel_configure(
        dma_channel_data,
        &c0,
        &PIO_VGA->txf[sm], // Write address
        lines_pattern[0], // read address
        600 / 4, //
        false // Don't start yet
    );
    //канал DMA для контроля основного канала
    dma_channel_config c1 = dma_channel_get_default_config(dma_channel_control);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);

    channel_config_set_read_increment(&c1, false);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_chain_to(&c1, dma_channel_data); // chain to other channel
    //channel_config_set_dreq(&c1, DREQ_PIO0_TX0);

    dma_channel_configure(
        dma_channel_control,
        &c1,
        &dma_hw->ch[dma_channel_data].read_addr, // Write address
        &lines_pattern[0], // read address
        1, //
        false // Don't start yet
    );
    //dma_channel_set_read_addr(dma_chan, &DMA_BUF_ADDR[0], false);

    graphics_set_mode(CGA_320x200x4);

    irq_set_exclusive_handler(VGA_DMA_IRQ, dma_handler_VGA);

    dma_channel_set_irq0_enabled(dma_channel_control, true);

    irq_set_enabled(VGA_DMA_IRQ, true);
    dma_start_channel_mask(1u << dma_channel_data);


    uint8_t TMPL_VHS8 = 0;
    uint8_t TMPL_VS8 = 0;
    uint8_t TMPL_HS8 = 0;
    uint8_t TMPL_LINE8 = 0;

    int HS_SIZE = 4;
    int HS_SHIFT = 100;

    //txt_palette_fast = (uint16_t *) calloc(256 * 4, sizeof(uint16_t));
    for (int i = 0; i < 256; i++) {
        const uint8_t c1 = txt_palette[i & 0xf];
        const uint8_t c0 = txt_palette[i >> 4];

        txt_palette_fast[i * 4 + 0] = c0 | c0 << 8;
        txt_palette_fast[i * 4 + 1] = c1 | c0 << 8;
        txt_palette_fast[i * 4 + 2] = c0 | c1 << 8;
        txt_palette_fast[i * 4 + 3] = c1 | c1 << 8;
    }

    TMPL_LINE8 = 0b11000000;
    HS_SHIFT = 328 * 2;
    HS_SIZE = 48 * 2;

    constexpr int line_size = 400 * 2;

    shift_picture = line_size - HS_SHIFT;

    //инициализация шаблонов строк и синхросигнала
    const uint32_t div32 = (uint32_t) (clock_get_hz(clk_sys) / 25175000 * (1 << 16) + 0.0);
    PIO_VGA->sm[_SM_VGA].clkdiv = div32 & 0xfffff000; //делитель для конкретной sm
    dma_channel_set_trans_count(dma_channel_data, line_size / 4, false);

    for (int i = 0; i < 4; i++) {
        lines_pattern[i] = &lines_pattern_data[i * (line_size / 4)];
    }
    // memset(lines_pattern_data,N_TMPLS*1200,0);
    TMPL_VHS8 = TMPL_LINE8 ^ 0b11000000;
    TMPL_VS8 = TMPL_LINE8 ^ 0b10000000;
    TMPL_HS8 = TMPL_LINE8 ^ 0b01000000;

    auto base_ptr = (uint8_t *) lines_pattern[0];
    //пустая строка
    memset(base_ptr, TMPL_LINE8, line_size);
    //memset(base_ptr+HS_SHIFT,TMPL_HS8,HS_SIZE);
    //выровненная синхра вначале
    memset(base_ptr, TMPL_HS8, HS_SIZE);

    // кадровая синхра
    base_ptr = (uint8_t *) lines_pattern[1];
    memset(base_ptr, TMPL_VS8, line_size);
    //memset(base_ptr+HS_SHIFT,TMPL_VHS8,HS_SIZE);
    //выровненная синхра вначале
    memset(base_ptr, TMPL_VHS8, HS_SIZE);

    //заготовки для строк с изображением
    base_ptr = (uint8_t *) lines_pattern[2];
    memcpy(base_ptr, lines_pattern[0], line_size);
    base_ptr = (uint8_t *) lines_pattern[3];
    memcpy(base_ptr, lines_pattern[0], line_size);
}


void graphics_set_mode(const enum graphics_mode_t mode) {
    graphics_mode = mode;
}