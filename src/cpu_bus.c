#include "i8086_bus.pio.h"
#include "cpu_bus.h"
#include "config.h"
#include "bios.h"
#include "basic.h"

// IRQ management (from main.c)
extern uint16_t current_irq_vector;

// Keyboard scancode (from main.c)
extern volatile uint8_t current_scancode;

// ROM configuration
#define BIOS_ROM_SIZE          sizeof(BIOS) // 8K BIOS
#define BIOS_ROM_BASE          (0x100000 - BIOS_ROM_SIZE)   // 0xFE000-0xFFFFF
#define RAM_SIZE               (192 * 1024)                       // KB

// Удобные макросы для вычисления пинов
#define MIO (1 << 24)
#define BHE (1 << 25)

uint8_t RAM[RAM_SIZE] __attribute__((aligned(4)));
uint8_t VIDEORAM[4096] __attribute__((aligned(4)));
uint8_t PORTS[0xfff] __attribute__((aligned(4))) = {0xFF};

// VGA status port state (вынесено из функции для оптимизации доступа)
static uint8_t port3DA = 0;

// INTA pending IRQ vector (вынесено для минимизации memory access overhead)
static uint16_t irq_pending_vector = 0;

__always_inline void write_to(uint8_t *destination, const uint32_t address, const uint16_t data, const bool bhe) {
    const bool A0 = address & 1;
    if (likely(!bhe && !A0)) {
        *(uint16_t *) &destination[address] = data;
    } else if (bhe && !A0) {
        destination[address] = (uint8_t) (data & 0xFF);
    } else if (!bhe && A0) {
        destination[address] = (uint8_t) (data >> 8);
    }
}

__force_inline static uint16_t i8086_read(const uint32_t address, const bool is_memory_access) {
    if (is_memory_access) {
        if (address < RAM_SIZE) {
            return *(uint16_t *) &RAM[address];
        }
        // Оптимизация: одна проверка через вычитание вместо двух сравнений
        // (address - 0xB0000) < 0x8000 эквивалентно: address >= 0xB0000 && address < 0xB8000
        if ((address - 0xB0000) < 0x8000) {
            return *(uint16_t *) &VIDEORAM[address & 4094];
        }
        // BASIC ROM: 0xF6000-0xFDFFF (используем вычитание для одной проверки)
        if ((address - 0xF6000) < (BIOS_ROM_BASE - 0xF6000)) {
            return *(uint16_t *) &BASIC[address - 0xF6000];
        }
        if (address >= BIOS_ROM_BASE) {
            return *(uint16_t *) &BIOS[address - BIOS_ROM_BASE];
        }
        // Unmapped memory
        return 0xFFFF;
    } else {
        // I/O access
        switch (address & 0xFFE) {
            case 0x3BA: { // MDA status port

                return port3DA ^= 9;
            }
            case 0x60: { // Keyboard Data Port - читаем скан-код и сбрасываем
                const uint8_t scancode = current_scancode;
                current_scancode = 0;
                return scancode;
            }
            case 0x64: { // Keyboard Status Port - bit 0 = данные доступны
                return current_scancode != 0;  // Упрощено: компилятор генерирует оптимальный код
            }
            default:
                return 0xFFFF;
        }
    }
}

__force_inline static void i8086_write(const uint32_t address, const uint16_t data, const bool is_memory_access, const bool bhe) {
    if (is_memory_access) {
        // Memory write
        if (address < RAM_SIZE) {
            write_to(RAM, address, data, bhe);
        }
        // Оптимизация: одна проверка вместо двух для видеопамяти
        else if ((address - 0xB0000) < 0x8000) {
            write_to(VIDEORAM, address & 0xFFF, data, bhe);
        }
    } else {
        // I/O write
        write_to(PORTS, address & 0x3FF, data, bhe);
    }
}

void __time_critical_func(bus_write_handler)() {
    // Упрощено: обрабатываем ровно одну транзакцию.
    // PIO генерирует IRQ на каждую запись, поэтому цикл избыточен.
    // Это уменьшает register pressure и упрощает IRQ prologue/epilogue.
    const uint32_t bus_state = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];
    const uint16_t data = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];

    i8086_write(bus_state & 0xFFFFF, data, bus_state & MIO, bus_state & BHE);

    pio_interrupt_clear(BUS_CTRL_PIO, 0);
}

void __time_critical_func(bus_read_handler)() {
    if (pio_interrupt_get(BUS_CTRL_PIO, 1)) {
        // Прямой доступ к FIFO. pio_interrupt_get() уже подтвердил что данные есть,
        // повторная проверка в pio_sm_get_blocking() избыточна.
        const uint32_t bus_state = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];

        // Read data and send back to PIO
        // Прямая запись в TX FIFO без проверки заполненности.
        // PIO заблокирован на 'pull block' и активно ждёт наши данные, TX FIFO точно не полон.
        if (unlikely(irq_pending_vector)) {
            // INTA: возвращаем вектор прерывания (0x08 для IRQ0, 0x09 для IRQ1)
            BUS_CTRL_PIO->txf[BUS_CTRL_SM] = irq_pending_vector << 16 | 0x00FF;
            irq_pending_vector = 0;
        } else {
            // Обычное чтение памяти/портов.
            BUS_CTRL_PIO->txf[BUS_CTRL_SM] = i8086_read(bus_state & 0xFFFFE, bus_state & MIO) << 16 | 0xFFFF;
        }

        // TODO если мы не можем обработать адрес, вместо того чтобы _НЕ_ перключать пины на выход - можно выполнить следующий код, чтобы отпустить шину
        // pio_sm_exec(BUS_CTRL_PIO, BUS_CTRL_SM, pio_encode_jmp(i8086_bus_wrap_target) | pio_encode_sideset_opt(1,1)); // jmp .wrap_target side 1
        // pio_sm_put_blocking(BUS_CTRL_PIO, BUS_CTRL_SM, 0xDEADBEEF); // Совершенно не важно что отправится в буфер, следующая команда JMP в начало

        // log_event(LOG_READ, bus_state & 0xFFFFE, data, bus_state & (1 << 25), bus_state & (1 << 24));

        pio_interrupt_clear(BUS_CTRL_PIO, 1);
    } else if (pio_interrupt_get(BUS_CTRL_PIO, 3)) {
        // INTA cycle
        pio_interrupt_clear(BUS_CTRL_PIO, 3);

        irq_pending_vector = current_irq_vector;
        current_irq_vector = 0;
    }
}

void cpu_bus_init() {
    const uint pio_offset = pio_add_program(BUS_CTRL_PIO, &i8086_bus_program);
    i8086_bus_program_init(BUS_CTRL_PIO, BUS_CTRL_SM, pio_offset);

    pio_set_irq0_source_enabled(BUS_CTRL_PIO, pis_interrupt0, true);
    irq_set_exclusive_handler(WRITE_IRQ, bus_write_handler);
    irq_set_priority(WRITE_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(WRITE_IRQ, true);

    pio_set_irq1_source_enabled(BUS_CTRL_PIO, pis_interrupt1, true);
    pio_set_irq1_source_enabled(BUS_CTRL_PIO, pis_interrupt3, true);
    irq_set_exclusive_handler(READ_IRQ, bus_read_handler);
    irq_set_priority(READ_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(READ_IRQ, true);

    pio_sm_set_enabled(BUS_CTRL_PIO, BUS_CTRL_SM, true);
}
