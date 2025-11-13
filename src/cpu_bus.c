#include "i8086_bus.pio.h"
#include "cpu_bus.h"
#include "common.h"
#include "memory.h"
#include "ports.h"
#include "hardware/i8259.h"

// INTA pending IRQ vector (вынесено для минимизации memory access overhead)
static uint16_t irq_pending_vector = 0;

// ============================================================================
// Bus Read/Write Routing (маршрутизация между memory и ports)
// ============================================================================

__force_inline static uint16_t i8086_read(const uint32_t address, const bool is_memory_access, const bool bhe) {
    return is_memory_access ? memory_read(address  & 0xFFFFE) : port_read(address & 0xFFF, bhe);
}

__force_inline static void i8086_write(const uint32_t address, const uint16_t data,
                                        const bool is_memory_access, const bool bhe) {
    if (is_memory_access) {
        memory_write(address, data, bhe);
    } else {
        port_write(address, data, bhe);
    }
}

void __time_critical_func(bus_write_handler)() {
    const uint32_t bus_state = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];
    const uint16_t data = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];

    i8086_write(bus_state & 0xFFFFF, data, bus_state & MIO, bus_state & BHE);

    pio_interrupt_clear(BUS_CTRL_PIO, 0);
}

void __time_critical_func(bus_read_handler)() {
    // INTA cycle проверяем первым (более редкий, но высокоприоритетный)
    if (unlikely(pio_interrupt_get(BUS_CTRL_PIO, 3))) {
        pio_interrupt_clear(BUS_CTRL_PIO, 3);
        const uint8_t vector = i8259_nextirq();
        if (vector) {
            irq_pending_vector = 0xFF00 | vector;
        }
        return; // ← ВАЖНО: INTA не требует чтения данных
    }

    // IRQ1 - обычное чтение (без дополнительной проверки)
    const uint32_t bus_state = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];

    if (unlikely(irq_pending_vector)) {
        BUS_CTRL_PIO->txf[BUS_CTRL_SM] = irq_pending_vector << 16 | 0x00FF;
        irq_pending_vector = 0;
    } else {
        BUS_CTRL_PIO->txf[BUS_CTRL_SM] = i8086_read(bus_state, bus_state & MIO, bus_state & BHE) << 16 | 0xFFFF;
    }

    pio_interrupt_clear(BUS_CTRL_PIO, 1);
}

void __time_critical_func(bus_read_handler1)() {
    if (pio_interrupt_get(BUS_CTRL_PIO, 1)) {
        // Прямой доступ к FIFO. pio_interrupt_get() уже подтвердил что данные есть,
        // повторная проверка в pio_sm_get_blocking() избыточна.
        const uint32_t bus_state = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];

        // Read data and send back to PIO
        // Прямая запись в TX FIFO без проверки заполненности.
        // PIO заблокирован на 'pull block' и активно ждёт наши данные, TX FIFO точно не полон.
        if (unlikely(irq_pending_vector)) {
            // INTA: возвращаем вектор прерывания (0x08 для IRQ0, 0x09 для IRQ1)
            BUS_CTRL_PIO->txf[BUS_CTRL_SM] = irq_pending_vector;
            irq_pending_vector = 0;
        } else {
            // Обычное чтение памяти/портов.
            const uint16_t data = i8086_read(bus_state, bus_state & MIO, bus_state & BHE);
            // if (data == 0xFFFF) {
                // BUS_CTRL_PIO->txf[BUS_CTRL_SM] = 0;
            // } else {
                BUS_CTRL_PIO->txf[BUS_CTRL_SM] = data << 16 | 0xFFFF;
            // }
        }

        // TODO если мы не можем обработать адрес, вместо того чтобы _НЕ_ перключать пины на выход - можно выполнить следующий код, чтобы отпустить шину
        // pio_sm_exec(BUS_CTRL_PIO, BUS_CTRL_SM, pio_encode_jmp(i8086_bus_wrap_target) | pio_encode_sideset_opt(1,1)); // jmp .wrap_target side 1
        // pio_sm_put_blocking(BUS_CTRL_PIO, BUS_CTRL_SM, 0xDEADBEEF); // Совершенно не важно что отправится в буфер, следующая команда JMP в начало

        // log_event(LOG_READ, bus_state & 0xFFFFE, data, bus_state & (1 << 25), bus_state & (1 << 24));

        pio_interrupt_clear(BUS_CTRL_PIO, 1);
    } else if (pio_interrupt_get(BUS_CTRL_PIO, 3)) {
        // INTA cycle (первый INTA pulse от CPU)
        pio_interrupt_clear(BUS_CTRL_PIO, 3);

        // Получаем вектор прерывания от i8259
        const uint8_t vector = i8259_nextirq();
        if (vector) {
            irq_pending_vector = vector << 16 | 0x00FF;  // Формат: 0xFF00 | вектор
        }

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
