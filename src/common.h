#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <pico/time.h>
#include <hardware/structs/qmi.h>
#include <hardware/structs/xip.h>


// ============================================================================
// Compiler hints for branch prediction
// ============================================================================
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#define PICO_CLOCK_SPEED     (504 * MHZ)  // Raspberry Pi Pico clock frequency
#define PSRAM_FREQ_MHZ       (166 * MHZ)

#define I8086_CLOCK_SPEED    (6000 * KHZ)  // i8086 clock frequency
#define I8086_DUTY_CYCLE     (33)          // 33% duty cycle required for i8086

#define VIDEORAM_SIZE        (32 * 1024)   // Tandy/PC Jr compat
// #define RAM_SIZE             (640 * 1024)
#define RAM_SIZE             (736 * 1024)
#define UMB_SIZE             (128 * 1024)

#define BIOS_ROM_SIZE        (8 * 1024)                         // 8KB BIOS
#define BIOS_ROM_BASE        (0x100000 - BIOS_ROM_SIZE)         // 0xFE000-0xFFFFF


// ============================================================================
// GPIO Pin Configuration
// ============================================================================
#define INTR_PIN         26             // Interrupt request output to i8086 (active HIGH)
#define RESET_PIN        28             // Reset output (active HIGH)
#define CLOCK_PIN        29             // Clock output to i8086

#define BEEPER_PIN        46            // PC Speaker pin]


// ============================================================================
// Bus Control Signal Macros
// ============================================================================
#define MIO (1 << 24)  // Memory/IO bit in bus state
#define BHE (1 << 25)  // Bus High Enable bit in bus state


// ============================================================================
// PIO Configuration
// ============================================================================
#define BUS_CTRL_PIO     pio0
#define BUS_CTRL_SM      0
#define WRITE_IRQ        PIO0_IRQ_0
#define READ_IRQ         PIO0_IRQ_1

#define IMPORT_BIN(file, sym) asm (\
".section .flashdata."#sym"\n"                  /* Change section */\
".balign 4\n"                           /* Word alignment */\
".global " #sym "\n"                    /* Export the object address */\
#sym ":\n"                              /* Define the object label */\
".incbin \"" file "\"\n"                /* Import the file */\
".global _sizeof_" #sym "\n"            /* Export the object size */\
".set _sizeof_" #sym ", . - " #sym "\n" /* Define the object size */\
".balign 4\n"                           /* Word alignment */\
".section \".text\"\n");                 /* Restore section */


// Общедоступные массивы и структуры
extern uint8_t UMB[UMB_SIZE] __attribute__((aligned(4)));
extern uint8_t RAM[RAM_SIZE] __attribute__((aligned(4)));
extern uint8_t VIDEORAM[VIDEORAM_SIZE] __attribute__((aligned(4)));

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
    uint16_t reload_value;       // Значение для загрузки в счётчик
    uint8_t access_mode;         // Режим доступа: LOBYTE/HIBYTE/TOGGLE
    uint8_t byte_toggle;         // Отслеживание байта в режиме TOGGLE
    uint8_t active;              // Канал активно считает (bool)
    uint8_t latch_mode;          // Режим latch: 0=нет, 1=lobyte, 2=hibyte, 3=toggle
    uint16_t latched_value;      // Защёлкнутое значение для LATCHCOUNT
    uint64_t start_timestamp_us; // Временная метка старта
    uint8_t operating_mode;
} i8253_channel_s;

// Intel 8253 Programmable Interval Timer (3 независимых канала)
typedef struct {
    i8253_channel_s channels[3];
} i8253_s;
typedef struct {
    uint32_t page;
    uint32_t address;
    uint32_t reload_address;
    uint32_t address_increase;
    uint16_t count;
    uint16_t reload_count;
    uint8_t auto_init;
    uint8_t mode;
    uint8_t enable;
    uint8_t masked;
    uint8_t dreq;
    uint8_t finished;
    uint8_t transfer_type;

    // Асинхронная передача данных (для polling на Core0)
    uint8_t data_source_type;
    const void *data_source;  // Источник данных (для device→memory)
    uint32_t data_offset;  // Смещениен в источнике
    uint8_t irq;               // IRQ для генерации при завершении (0 = нет IRQ)
    bool transfer_active;             // Флаг активной передачи
} dma_channel_s;

typedef struct {
    uint8_t rbr;           // Receive Buffer Register (текущий байт для чтения)
    uint8_t thr;           // Transmit Holding Register (для отправки)
    uint8_t ier;           // Interrupt Enable Register
    uint8_t iir;           // Interrupt Identification Register
    uint8_t lcr;           // Line Control Register (бит 7 = DLAB)
    uint8_t mcr;           // Modem Control Register
    uint8_t lsr;           // Line Status Register
    uint8_t msr;           // Modem Status Register
    uint16_t divisor;      // Divisor latch (для baud rate, игнорируем)
    bool data_ready;       // Флаг: есть данные в RBR для чтения

    // FIFO буфер для приема данных (для Microsoft Serial Mouse)
    uint8_t rx_fifo[16];   // Приемный FIFO (достаточно для нескольких mouse packets)
    uint8_t rx_head;       // Индекс головы (куда писать)
    uint8_t rx_tail;       // Индекс хвоста (откуда читать)
} uart_16550_s;

// Consolidated controller state for tighter locality; align to cache line for fast access
typedef struct {
    uint8_t DOR;
    uint8_t response[4];
    uint8_t command[9];
    uint8_t result[7];
    uint8_t presentCylinder[4];
    uint8_t command_length;
    uint8_t result_count;
    uint8_t command_index;
    uint8_t result_index;
    uint8_t check_drives_mask;
} i8272_s;

typedef struct {
    union {
        // Член для доступа к регистрам как к массиву
        uint8_t registers[16];

        // Член для доступа к регистрам по их именам
        struct {
            uint8_t h_total;            // R0: Horizontal Total
            uint8_t h_displayed;        // R1: Horizontal Displayed
            uint8_t h_sync_pos;         // R2: HSync Position
            uint8_t h_sync_width;       // R3: HSync Width
            uint8_t v_total;            // R4: Vertical Total
            uint8_t v_total_adjust;     // R5: VTotal Adjust
            uint8_t v_displayed;        // R6: Vertical Displayed
            uint8_t v_sync_pos;         // R7: VSync Position
            uint8_t interlace_mode;     // R8: Interlace Mode
            uint8_t max_scanline_addr;  // R9: Max Scanline Address
            uint8_t cursor_start;       // R10: Cursor Start Line
            uint8_t cursor_end;         // R11: Cursor End Line
            uint8_t start_addr_h;       // R12: Start Addr (H)
            uint8_t start_addr_l;       // R13: Start Addr (L)
            uint8_t cursor_addr_h;      // R14: Cursor Addr (H)
            uint8_t cursor_addr_l;      // R15: Cursor Addr (L)
        } r;
    };
    uint16_t vram_offset;

    bool cursor_blink_state;
    uint8_t text_blinking_mask;  // 0x80 = enabled, 0x00 = disabled (маска для бита атрибута)
    uint8_t cursor_x;
    uint8_t cursor_y;
} mc6845_s;

    typedef struct {
    uint8_t port3D8;
    uint8_t port3D9;
    uint8_t port3DA;
    uint8_t port3DA_tandy;
    bool updated;
} cga_s;


// Corrected CGA palette from https://int10h.org/blog/2022/06/ibm-5153-color-true-cga-palette/
constexpr uint32_t cga_palette[16] = {
    //R, G, B
    0x000000, // 0: black
    0x0000AA, // 1: blue
    0x00AA00, // 2: green
    0x00AAAA, // 3: cyan
    0xAA0000, // 4: red
    0xAA00AA, // 5: magenta
    0xAAAA00, // 6: brown
    0xAAAAAA, // 7: light gray

    0x555555, // 8: dark gray
    0x0000FF, // 9: bright blue
    0x00FF00, // 10: bright green
    0x00FFFF, // 11: bright cyan
    0xFF0000, // 12: bright red
    0xFF00FF, // 13: bright magenta
    0xFFFF00, // 14: bright yellow
    0xFFFFFF  // 15: white
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



static void psram_init(const int cs_pin) {
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

void cpu_bus_init();
void start_cpu_clock(void);
void reset_cpu(void);
