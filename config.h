#pragma once
#include <hardware/vreg.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>
// ============================================================================
// System Configuration
// ============================================================================
#define PICO_CLOCK_SPEED     (400 * MHZ)  // 400 MHz

// ============================================================================
// i8086 Clock Configuration
// ============================================================================

#define I8086_CLOCK_SPEED       500 * KHZ     // 100 Hz for debug (change to 5000000 for 5 MHz)
#define CONFIG_I8086_DUTY_CYCLE 33      // 33% duty cycle (required for i8086)


// Пины, доступные в GPIO
#define INTR_PIN         26             // Interrupt request output to i8086 (active HIGH)
#define RESET_PIN        28             // Reset output (active high)
#define CLOCK_PIN        29
// ============================================================================
// PIO Configuration
// ============================================================================
#define BUS_CTRL_PIO     pio0
#define BUS_CTRL_SM      0
#define WRITE_IRQ        PIO0_IRQ_0
#define READ_IRQ         PIO0_IRQ_1



//


// Структуры остаются те же, но спин-блок в них больше не нужен.
typedef enum { LOG_READ, LOG_WRITE, LOG_INTA } log_type_t;

typedef struct {
    uint64_t timestamp;   // Метка времени для отладки таймингов
    log_type_t type;      // Тип операции
    uint32_t address;     // 20-битный адрес
    uint16_t data;        // 16-битные данные
    bool bhe;             // Состояние BHE
    bool mio;             // Состояние MIO
} log_entry_t;

#define LOG_BUFFER_SIZE 256
typedef struct {
    log_entry_t buffer[LOG_BUFFER_SIZE];
    volatile uint32_t head;
    // tail больше не нужен здесь, Core 1 будет сам следить за указателями, полученными из FIFO
} shared_log_buffer_t;

// Глобальный буфер в общей памяти
extern shared_log_buffer_t log_buffer;