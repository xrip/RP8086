#include "common.h"
#include "hardware/i8237.h"
#include "hardware/i8272.h"

IMPORT_BIN("./binary/GLABIOS.ROM", BIOS);
IMPORT_BIN("./binary/ide_xt.bin", IDE);
// IMPORT_BIN("./binary/landmark.bin", BIOS);
// IMPORT_BIN("./binary/ruuds_diagnostic_rom_v5.4_8kb.bin", BIOS);
// IMPORT_BIN("./binary/checkit.img", FLOPPY);
IMPORT_BIN("./binary/os.img", FLOPPY);


uint8_t RAM[RAM_SIZE] __attribute__((aligned(4), section(".psram")));
uint8_t UMB[UMB_SIZE] __attribute__((aligned(4), section(".psram")));
uint8_t VIDEORAM[VIDEORAM_SIZE] __attribute__((aligned(4)));

i8259_s i8259 __attribute__((aligned(4))) = {
    .interrupt_mask_register = 0xFF, // Все IRQ замаскированы
    .interrupt_vector_offset = 0x08, // Стандартный offset для IBM PC
};
i8253_s i8253 __attribute__((aligned(4))) = {
    {
        {.latched_value = 0xFFFF},
        {.latched_value = 0xFFFF},
        {.latched_value = 0xFFFF},
    }
};
i8272_s i8272 __attribute__((aligned(4)));

dma_channel_s dma_channels[DMA_CHANNELS] = {
    { .masked =  1},
    { .masked =  1},
    { .masked =  1},
    { .masked =  1},
};

uart_16550_s uart __attribute__((aligned(4))) = {
    .data_ready = false,
    .lcr = 0x03,  // 8 data bits, 1 stop bit, no parity (стандарт)
    .mcr = 0x00,
    .msr = 0xB0,  // CTS, DSR, DCD активны
    .rx_head = 0,  // FIFO: индекс записи
    .rx_tail = 0,  // FIFO: индекс чтения
};

mc6845_s mc6845 __attribute__((aligned(4)));
cga_s cga __attribute__((aligned(4)));
ide_s ide __attribute__((aligned(4)));

// IMPORTANT! Dont remove, hack to create .flashdata section for linker
const uint64_t __in_flash("DUMMY") PICO_CLOCK_SPEED_MHZ = PICO_CLOCK_SPEED;

// Инициализация MC6845 стандартными значениями для текстового режима 80x25
void mc6845_init_text_mode(void) {
    mc6845.r.h_total = 113;           // R0: Horizontal Total
    mc6845.r.h_displayed = 80;        // R1: Horizontal Displayed (80 columns)
    mc6845.r.h_sync_pos = 90;         // R2: HSync Position
    mc6845.r.h_sync_width = 10;       // R3: HSync Width
    mc6845.r.v_total = 31;            // R4: Vertical Total
    mc6845.r.v_total_adjust = 6;      // R5: VTotal Adjust
    mc6845.r.v_displayed = 30;        // R6: Vertical Displayed (25 rows)
    mc6845.r.v_sync_pos = 28;         // R7: VSync Position
    mc6845.r.interlace_mode = 2;      // R8: Interlace Mode
    mc6845.r.max_scanline_addr = 7;   // R9: Max Scanline (8 lines per char)
    mc6845.r.cursor_start = 6;        // R10: Cursor Start Line
    mc6845.r.cursor_end = 7;          // R11: Cursor End Line
    mc6845.r.start_addr_h = 0;        // R12: Start Addr (H)
    mc6845.r.start_addr_l = 0;        // R13: Start Addr (L)
    mc6845.r.cursor_addr_h = 0;       // R14: Cursor Addr (H)
    mc6845.r.cursor_addr_l = 0;       // R15: Cursor Addr (L)

    mc6845.vram_offset = 0;
    mc6845.cursor_x = 0;
    mc6845.cursor_y = 0;
    mc6845.cursor_blink_state = false;
    mc6845.text_blinking_mask = 0xFF; // Blinking disabled by default
}

