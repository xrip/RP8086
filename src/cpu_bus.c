#include "i8086_bus.pio.h"
#include "cpu_bus.h"
#include "config.h"
#include "bios.h"

// IRQ management (from main.c)
extern uint16_t current_irq_vector;

// Keyboard buffer (from main.c)
extern uint8_t keyboard_buffer[16];
extern volatile uint8_t kb_head;
extern volatile uint8_t kb_tail;

// ROM configuration
#define BIOS_ROM_SIZE          sizeof(BIOS) // 8K BIOS
#define BIOS_ROM_BASE          (0x100000 - BIOS_ROM_SIZE)   // 0xFE000-0xFFFFF
#define BIOS_ROM_MASK          (BIOS_ROM_SIZE - 1)          // 0x1FFF
#define BIOS_ROM_MASK_16BIT    (BIOS_ROM_SIZE - 2)          // 0x1FFE

#define RAM_SIZE         (192 * 1024)                       // KB
#define RAM_MASK         (RAM_SIZE - 1)                     // 0xFFFF
#define RAM_MASK_16BIT   (RAM_SIZE - 2)                     // 0xFFFE

// Удобные макросы для вычисления пинов
#define MIO (1 << 24)
#define BHE (1 << 25)

uint8_t RAM[RAM_SIZE] __attribute__((aligned(4)));
uint8_t VIDEORAM[4096] __attribute__((aligned(4)));
uint8_t PORTS[0xfff] __attribute__((aligned(2))) = {0xFF};


__force_inline static uint16_t i8086_read(const uint32_t address, const bool is_memory_access) {
    if (is_memory_access) {
        if (address < RAM_SIZE) {
            return *(uint16_t *) &RAM[address];
        } else if (address >= 0xB0000 && address < 0xB8000) {
            return *(uint16_t *) &VIDEORAM[address & 4094];
        } else if (address >= BIOS_ROM_BASE) {
            return *(uint16_t *) &BIOS[address - BIOS_ROM_BASE];
        }
        // Unmapped memory
        return 0xFFFF;
    } else {
        // I/O access
        const uint16_t port = address & 0xFFE;
        static uint8_t port3DA = 0;

        // VGA Status Port - самый частый, проверяем первым
        if (port == 0x3BA) {
            port3DA ^= 1;
            if (!(port3DA & 1)) port3DA ^= 8;
            return port3DA;
        }

        // Keyboard ports (редкие) - объединяем в одну диапазонную проверку
        // Проверяем диапазон 0x60-0x6F одним сравнением (экономим 2-3 такта)
        if ((port & 0xFF0) == 0x60) {
            // Локальные копии volatile переменных для минимизации обращений к памяти
            const uint8_t head = kb_head;
            const uint8_t tail = kb_tail;

            if (port == 0x60) {
                // Keyboard Data Port - читаем скан-код
                if (head != tail) {
                    const uint8_t scancode = keyboard_buffer[tail];
                    kb_tail = (tail + 1) & 15;
                    return scancode;
                }
                return 0;
            }
            if (port == 0x64) {
                // Keyboard Status Port - bit 0 = данные доступны
                return (head != tail) ? 0x01 : 0x00;
            }
        }
        return 0xFFFF;
    }
}


__force_inline static void i8086_write(uint32_t address, const uint16_t data, const bool is_memory_access, const bool bhe) {
    if (is_memory_access) {
        // Memory write
        if (address < RAM_SIZE) {
            const bool a0 = address & 1;

            // Определяем тип операции по BHE и A0
            if (!bhe && !a0) {
                // 16-bit transfer: BHE=0, A0=0
                *(uint16_t *) &RAM[address] = data;
            } else if (bhe && !a0) {
                // Byte transfer на четном адресе: BHE=1, A0=0 (только младший байт D7-D0)
                RAM[address] = data & 0xFF;
            } else if (!bhe && a0) {
                // Byte transfer на нечетном адресе: BHE=0, A0=1 (только старший байт D15-D8)
                RAM[address] = data >> 8;
            }
            // BHE=1, A0=1 - недопустимая комбинация, игнорируем
        } else if (address >= 0xB0000 && address < 0xB8000) {
            // printf ("VIDEO RAM write at %05x %x %d\n", address, data, bhe);
            const bool a0 = address & 1;
            address &= 4095;

            // Определяем тип операции по BHE и A0
            if (!bhe && !a0) {
                // 16-bit transfer: BHE=0, A0=0
                *(uint16_t *) &VIDEORAM[address] = data;
            } else if (bhe && !a0) {
                // Byte transfer на четном адресе: BHE=1, A0=0 (только младший байт D7-D0)
                VIDEORAM[address] = data & 0xFF;
            } else if (!bhe && a0) {
                // Byte transfer на нечетном адресе: BHE=0, A0=1 (только старший байт D15-D8)
                VIDEORAM[address] = data >> 8;
            }
        }
    } else {
        // I/O write

        const bool a0 = address & 1;
        // Определяем тип операции по BHE и A0
        if (!bhe && !a0) {
            *(uint16_t *) &PORTS[address & 0xFFE] = data;
        } else if (bhe && !a0) {
            PORTS[address & 0xFFF] = data & 0xFF;
        } else if (!bhe && a0) {
            PORTS[address & 0xFFF] = (data >> 8) & 0xFF;
        }
    }
}

void __time_critical_func(bus_write_handler)() {
    while (pio_sm_get_rx_fifo_level(BUS_CTRL_PIO, BUS_CTRL_SM) >= 2) {
        // First FIFO entry: address + control signals
        const uint32_t bus_state = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM);

        // Second FIFO entry: data from AD0-AD15
        const uint16_t data = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM);
        i8086_write(bus_state & 0xFFFFF, data, bus_state & MIO, bus_state & BHE);
        // log_event(LOG_WRITE, bus_state & 0xFFFFF, data, bus_state & (1 << 25), bus_state & (1 << 24));
    }
    pio_interrupt_clear(BUS_CTRL_PIO, 0);
}

void __time_critical_func(bus_read_handler)() {
    static bool irq_pending = false;
    if (pio_interrupt_get(BUS_CTRL_PIO, 1)) {
        // while (pio_sm_get_rx_fifo_level(BUS_CTRL_PIO, BUS_CTRL_SM) > 0)
        {
            // FIFO entry: address + control signals
            const uint32_t bus_state = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM);

            // Read data and send back to PIO
            // Разделяем пути выполнения для оптимизации компилятора
            uint16_t data;
            if (unlikely(irq_pending)) {
                // INTA: возвращаем вектор прерывания (0x08 для IRQ0, 0x09 для IRQ1)
                data = current_irq_vector;
                irq_pending = current_irq_vector = 0;
            } else {
                // Обычное чтение памяти/портов. Хак для 16 битного чтения с помощью 0xFFFFE
                data = i8086_read(bus_state & 0xFFFFE, bus_state & MIO);
            }
            pio_sm_put_blocking(BUS_CTRL_PIO, BUS_CTRL_SM, data << 16 | 0xFFFF);
            // log_event(LOG_READ, bus_state & 0xFFFFE, data, bus_state & (1 << 25), bus_state & (1 << 24));
        }

        pio_interrupt_clear(BUS_CTRL_PIO, 1);
    } else if (pio_interrupt_get(BUS_CTRL_PIO, 3)) {
        // INTA cycle
        // log_event(LOG_INTA, 0, 0, 0, 0);
        pio_interrupt_clear(BUS_CTRL_PIO, 3);
        gpio_put(INTR_PIN, 0); // Опускаем INTR
        irq_pending = true;
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
