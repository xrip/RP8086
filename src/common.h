#pragma once
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>

// ============================================================================
// Compiler hints for branch prediction
// ============================================================================
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)
#if PICO_RP2350
#define PICO_CLOCK_SPEED     (500 * MHZ)  // Raspberry Pi Pico clock frequency
#define I8086_CLOCK_SPEED    (4700 * KHZ)  // i8086 clock frequency
#else
#define PICO_CLOCK_SPEED     (400 * MHZ)  // Raspberry Pi Pico clock frequency
#define I8086_CLOCK_SPEED    (3250 * KHZ)  // i8086 clock frequency
#endif
#define CONFIG_I8086_DUTY_CYCLE 33      // 33% duty cycle (required for i8086)

// ============================================================================
// GPIO Pin Configuration
// ============================================================================
#define INTR_PIN         26             // Interrupt request output to i8086 (active HIGH)
#define RESET_PIN        28             // Reset output (active HIGH)
#define CLOCK_PIN        29             // Clock output to i8086

// ============================================================================
// PIO Configuration
// ============================================================================
#define BUS_CTRL_PIO     pio0
#define BUS_CTRL_SM      0
#define WRITE_IRQ        PIO0_IRQ_0
#define READ_IRQ         PIO0_IRQ_1

// ============================================================================
// Memory Configuration
// ============================================================================
#define RAM_SIZE               (192 * 1024)                       // 192KB RAM (maximum that fits)
#define BIOS_ROM_SIZE          8192                               // 8KB BIOS
#define BIOS_ROM_BASE          (0x100000 - BIOS_ROM_SIZE)         // 0xFE000-0xFFFFF

// ============================================================================
// Bus Control Signal Macros
// ============================================================================
#define MIO (1 << 24)  // Memory/IO bit in bus state
#define BHE (1 << 25)  // Bus High Enable bit in bus state

// ============================================================================
// Inline Helper Functions
// ============================================================================
// Универсальная функция записи с поддержкой BHE (8/16-bit operations)
__always_inline static void write_to(uint8_t *destination, const uint32_t address,
                                       const uint16_t data, const bool bhe) {
    const uint32_t A0 = address & 1;

    // Fast path: aligned 16-bit write (90% случаев)
    if (likely(!(bhe | A0))) {
        *(uint16_t *)&destination[address] = data;
        return;
    }

    // Slow path: byte write
    const uint8_t byte_val = A0 ? data >> 8 : data & 0xFF;
    destination[address] = byte_val;
}

// Общедоступные массивы и структу

extern uint8_t RAM[RAM_SIZE] __attribute__((aligned(4)));
extern uint8_t VIDEORAM[4096] __attribute__((aligned(4)));

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
    uint16_t channel_reload_value[3]; // chandata -> channel reload values (what gets loaded into counters)
    uint8_t channel_access_mode[3]; // accessmode -> how each channel is accessed (lobyte/hibyte/toggle)
    uint8_t channel_byte_toggle[3]; // bytetoggle -> tracks which byte to read/write in toggle mode
    uint32_t channel_effective_count[3]; // effectivedata -> actual count value used by channel
    float channel_frequency[3]; // chanfreq -> calculated frequency for each channel
    uint8_t channel_active[3]; // active -> whether channel is actively counting
    uint16_t channel_current_count[3]; // counter -> current counter value for each channel
    uint16_t channel_latched_value[3]; // latched value for LATCHCOUNT mode
    uint8_t channel_latch_mode[3]; // latch mode: 0=not latched, 1=lobyte, 2=hibyte, 3=toggle
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
    const uint8_t *transfer_source;  // Источник данных (для device→memory)
    uint8_t irq_number;               // IRQ для генерации при завершении (0 = нет IRQ)
    bool transfer_active;             // Флаг активной передачи
} dma_channel_s;

typedef struct {
    uint8_t rbr;           // Receive Buffer Register (один байт входящих данных)
    uint8_t thr;           // Transmit Holding Register (для отправки)
    uint8_t ier;           // Interrupt Enable Register
    uint8_t iir;           // Interrupt Identification Register
    uint8_t lcr;           // Line Control Register (бит 7 = DLAB)
    uint8_t mcr;           // Modem Control Register
    uint8_t lsr;           // Line Status Register
    uint8_t msr;           // Modem Status Register
    uint16_t divisor;      // Divisor latch (для baud rate, игнорируем)
    bool data_ready;       // Флаг: есть данные в RBR для чтения
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